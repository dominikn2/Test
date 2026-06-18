//! Per-client protocol session: dispatches rosbridge ops to backend operations,
//! manages publishers/subscriptions/services/actions, and produces outgoing
//! frames. This is the Rust equivalent of `RosbridgeProtocol` plus the
//! capability classes.

use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use rosbridge_protocol::incoming::*;
use rosbridge_protocol::outgoing::StatusLevel;
use rosbridge_protocol::IncomingMessage;
use serde_json::{json, Map, Value};
use tokio::sync::{mpsc, oneshot};
use tokio::task::JoinHandle;

use crate::backend::{
    Durability, PublisherId, QosSpec, Reliability, Sample, ServiceServerId, SharedBackend,
    SubscriptionId,
};
use crate::compression::{self, Compression};
use crate::config::SharedConfig;
use crate::fragment::{self, Defragmenter, FragmentOutcome};
use crate::glob;
use crate::subscription::{coalesce, SubParams};

/// An outgoing WebSocket frame.
#[derive(Debug, Clone)]
pub enum OutFrame {
    Text(String),
    Binary(Vec<u8>),
    Ping(Vec<u8>),
    Pong(Vec<u8>),
    Close,
}

/// State shared across all client sessions of one server.
pub struct Shared {
    pub cfg: SharedConfig,
    pub backend: SharedBackend,
    /// Best-effort topic -> type map, learned from advertise/publish, used to
    /// infer types for `subscribe` when the client omits `type`.
    pub topic_types: Mutex<HashMap<String, String>>,
}

impl Shared {
    pub fn new(cfg: SharedConfig, backend: SharedBackend) -> Arc<Self> {
        Arc::new(Shared {
            cfg,
            backend,
            topic_types: Mutex::new(HashMap::new()),
        })
    }

    fn learn_type(&self, topic: &str, ty: &str) {
        self.topic_types
            .lock()
            .entry(topic.to_string())
            .or_insert_with(|| ty.to_string());
    }

    fn infer_type(&self, topic: &str) -> Option<String> {
        // Prefer a type learned locally from advertise/publish; otherwise ask
        // the backend to discover it from the ROS graph.
        self.topic_types
            .lock()
            .get(topic)
            .cloned()
            .or_else(|| self.backend.discover_type(topic))
    }
}

struct PubEntry {
    id: PublisherId,
    type_name: String,
    adv_ids: HashSet<String>,
}

struct SubEntry {
    id: SubscriptionId,
    #[allow(dead_code)]
    type_name: String,
    sids: HashMap<String, SubParams>,
    effective: Arc<Mutex<SubParams>>,
    task: JoinHandle<()>,
}

struct SvcEntry {
    id: ServiceServerId,
    task: JoinHandle<()>,
}

struct ActEntry {
    id: crate::backend::ActionServerId,
    #[allow(dead_code)]
    type_name: String,
    task: JoinHandle<()>,
}

/// Channels for a goal currently being served by this client (client-hosted
/// action server). Feedback/result are JSON; the backend serializes for ROS.
struct HostedGoal {
    feedback_tx: mpsc::UnboundedSender<Value>,
    result_tx: Option<oneshot::Sender<(Option<Value>, i8)>>,
}

#[derive(Default)]
struct State {
    publishers: HashMap<String, PubEntry>,
    subscriptions: HashMap<String, SubEntry>,
    services: HashMap<String, SvcEntry>,
    actions: HashMap<String, ActEntry>,
    /// request id -> responder for client-hosted services.
    pending_service_responses: HashMap<String, oneshot::Sender<Option<Value>>>,
    /// goal id (as string) -> action name, for cancellation of issued goals.
    issued_goals: HashMap<String, String>,
    /// goal id (as string) -> channels, for client-hosted action servers.
    hosted_goals: HashMap<String, HostedGoal>,
}

const DUMMY_ADV_ID: &str = "dummy_adv_id";

/// A single connected client.
pub struct ClientSession {
    pub id: u64,
    shared: Arc<Shared>,
    out_tx: mpsc::UnboundedSender<OutFrame>,
    state: Mutex<State>,
    defrag: Mutex<Defragmenter>,
    frag_seed: AtomicU64,
    service_req_seq: AtomicU64,
    /// Threshold controlling which `status` messages are forwarded to the
    /// client (set via the `set_level` op). Messages are always logged.
    status_level: Mutex<StatusLevel>,
}

impl ClientSession {
    pub fn new(
        id: u64,
        shared: Arc<Shared>,
        out_tx: mpsc::UnboundedSender<OutFrame>,
    ) -> Arc<Self> {
        let timeout = shared.cfg.fragment_timeout;
        Arc::new(ClientSession {
            id,
            shared,
            out_tx,
            state: Mutex::new(State::default()),
            defrag: Mutex::new(Defragmenter::new(timeout)),
            frag_seed: AtomicU64::new(0),
            service_req_seq: AtomicU64::new(0),
            status_level: Mutex::new(StatusLevel::Info),
        })
    }

    // ---- entry points ---------------------------------------------------

    /// Handle a text frame (JSON). May reassemble fragments.
    pub fn handle_text(self: &Arc<Self>, text: &str) {
        match rosbridge_protocol::parse(text) {
            Ok(msg) => self.dispatch(msg),
            Err(e) => self.send_status(StatusLevel::Error, format!("invalid message: {e}"), None),
        }
    }

    /// Handle a binary frame (CBOR-encoded op).
    pub fn handle_binary(self: &Arc<Self>, data: &[u8]) {
        match ciborium::from_reader::<Value, _>(data) {
            Ok(v) => match rosbridge_protocol::parse_value(v) {
                Ok(msg) => self.dispatch(msg),
                Err(e) => {
                    self.send_status(StatusLevel::Error, format!("invalid CBOR message: {e}"), None)
                }
            },
            Err(e) => self.send_status(StatusLevel::Error, format!("invalid CBOR: {e}"), None),
        }
    }

    fn dispatch(self: &Arc<Self>, msg: IncomingMessage) {
        match msg {
            IncomingMessage::Advertise(m) => self.op_advertise(m),
            IncomingMessage::Unadvertise(m) => self.op_unadvertise(m),
            IncomingMessage::Publish(m) => self.op_publish(m),
            IncomingMessage::Subscribe(m) => self.op_subscribe(m),
            IncomingMessage::Unsubscribe(m) => self.op_unsubscribe(m),
            IncomingMessage::CallService(m) => self.op_call_service(m),
            IncomingMessage::AdvertiseService(m) => self.op_advertise_service(m),
            IncomingMessage::UnadvertiseService(m) => self.op_unadvertise_service(m),
            IncomingMessage::ServiceResponse(m) => self.op_service_response(m),
            IncomingMessage::AdvertiseAction(m) => self.op_advertise_action(m),
            IncomingMessage::UnadvertiseAction(m) => self.op_unadvertise_action(m),
            IncomingMessage::SendActionGoal(m) => self.op_send_action_goal(m),
            IncomingMessage::CancelActionGoal(m) => self.op_cancel_action_goal(m),
            IncomingMessage::ActionFeedback(m) => self.op_action_feedback(m),
            IncomingMessage::ActionResult(m) => self.op_action_result(m),
            IncomingMessage::SetLevel(m) => {
                if let Some(level) = StatusLevel::parse(&m.level) {
                    *self.status_level.lock() = level;
                } else {
                    self.send_status(
                        StatusLevel::Error,
                        format!("invalid set_level value: {}", m.level),
                        m.id,
                    );
                }
            }
            IncomingMessage::Status(_) | IncomingMessage::Auth(_) => {
                /* status echoes and auth are accepted and ignored */
            }
            IncomingMessage::Fragment(m) => self.op_fragment(m),
            IncomingMessage::Unknown => {
                self.send_status(StatusLevel::Error, "unknown operation", None)
            }
        }
    }

    // ---- helpers --------------------------------------------------------

    fn send_frame(&self, frame: OutFrame) {
        let _ = self.out_tx.send(frame);
    }

    fn send_value(&self, v: &Value) {
        if let Ok(s) = serde_json::to_string(v) {
            self.send_frame(OutFrame::Text(s));
        }
    }

    /// Send a status message to the client and log it (a superset of the ros2
    /// branch, which logs only — strictly more informative for clients).
    fn send_status(&self, level: StatusLevel, msg: impl Into<String>, id: Option<String>) {
        let msg = msg.into();
        match level {
            StatusLevel::Error => tracing::warn!(client = self.id, "{msg}"),
            StatusLevel::Warning => tracing::warn!(client = self.id, "{msg}"),
            _ => tracing::info!(client = self.id, "{msg}"),
        }
        // Forward to the client only if within the configured verbosity.
        if level.rank() <= self.status_level.lock().rank() {
            let v = json!({"op":"status","level":level.as_str(),"msg":msg,"id":id});
            self.send_value(&v);
        }
    }

    fn next_frag_id(&self) -> String {
        format!("f{}:{}", self.id, self.frag_seed.fetch_add(1, Ordering::Relaxed))
    }

    fn now(&self) -> (i32, u32) {
        self.shared.backend.now()
    }

    // ---- advertise / unadvertise / publish ------------------------------

    fn op_advertise(&self, m: Advertise) {
        if !glob::allowed(&self.shared.cfg.topics_pub_glob, &m.topic) {
            self.send_status(
                StatusLevel::Warning,
                format!("advertise to {} blocked by topics_pub_glob", m.topic),
                m.id.clone(),
            );
            return;
        }
        let adv_id = m.id.clone().unwrap_or_else(|| DUMMY_ADV_ID.to_string());
        let mut state = self.state.lock();
        if let Some(entry) = state.publishers.get_mut(&m.topic) {
            if entry.type_name != m.msg_type {
                let existing = entry.type_name.clone();
                drop(state);
                self.send_status(
                    StatusLevel::Error,
                    format!(
                        "topic {} already advertised as {}, not {}",
                        m.topic, existing, m.msg_type
                    ),
                    m.id,
                );
                return;
            }
            entry.adv_ids.insert(adv_id);
            return;
        }
        let qos = resolve_qos(
            m.qos.as_ref(),
            m.latch,
            m.queue_size,
            QosSpec::default_publisher(),
        );
        match self.shared.backend.advertise(&m.topic, &m.msg_type, &qos) {
            Ok(id) => {
                let mut adv_ids = HashSet::new();
                adv_ids.insert(adv_id);
                state.publishers.insert(
                    m.topic.clone(),
                    PubEntry {
                        id,
                        type_name: m.msg_type.clone(),
                        adv_ids,
                    },
                );
                drop(state);
                self.shared.learn_type(&m.topic, &m.msg_type);
            }
            Err(e) => {
                drop(state);
                self.send_status(StatusLevel::Error, format!("advertise failed: {e}"), m.id);
            }
        }
    }

    fn op_unadvertise(&self, m: Unadvertise) {
        if !glob::allowed(&self.shared.cfg.topics_pub_glob, &m.topic) {
            return;
        }
        let adv_id = m.id.clone().unwrap_or_else(|| DUMMY_ADV_ID.to_string());
        let mut state = self.state.lock();
        let remove = if let Some(entry) = state.publishers.get_mut(&m.topic) {
            entry.adv_ids.remove(&adv_id);
            entry.adv_ids.is_empty()
        } else {
            self.send_status(
                StatusLevel::Error,
                format!("topic {} was not advertised", m.topic),
                m.id.clone(),
            );
            return;
        };
        if remove {
            if let Some(entry) = state.publishers.remove(&m.topic) {
                self.shared.backend.unadvertise(entry.id);
            }
        }
    }

    fn op_publish(&self, m: Publish) {
        if !glob::allowed(&self.shared.cfg.topics_pub_glob, &m.topic) {
            self.send_status(
                StatusLevel::Warning,
                format!("publish to {} blocked by topics_pub_glob", m.topic),
                m.id.clone(),
            );
            return;
        }
        // Determine publisher: existing, or auto-create (rosbridge Publish does).
        let (pub_id, type_name) = {
            let state = self.state.lock();
            match state.publishers.get(&m.topic) {
                Some(e) => (Some(e.id), e.type_name.clone()),
                None => (None, String::new()),
            }
        };
        let type_name = if !type_name.is_empty() {
            type_name
        } else if let Some(t) = m.msg_type.clone() {
            t
        } else if let Some(t) = self.shared.infer_type(&m.topic) {
            t
        } else {
            self.send_status(
                StatusLevel::Error,
                format!("cannot publish to {}: unknown message type", m.topic),
                m.id,
            );
            return;
        };

        let pub_id = match pub_id {
            Some(id) => id,
            None => {
                let qos = resolve_qos(
                    m.qos.as_ref(),
                    m.latch,
                    None,
                    QosSpec::default_publisher(),
                );
                match self.shared.backend.advertise(&m.topic, &type_name, &qos) {
                    Ok(id) => {
                        let mut adv = HashSet::new();
                        adv.insert(DUMMY_ADV_ID.to_string());
                        self.state.lock().publishers.insert(
                            m.topic.clone(),
                            PubEntry {
                                id,
                                type_name: type_name.clone(),
                                adv_ids: adv,
                            },
                        );
                        self.shared.learn_type(&m.topic, &type_name);
                        id
                    }
                    Err(e) => {
                        self.send_status(
                            StatusLevel::Error,
                            format!("publish auto-advertise failed: {e}"),
                            m.id,
                        );
                        return;
                    }
                }
            }
        };

        // Auto-fill header.stamp with the current time when omitted (the ROS
        // backend serializes the JSON itself), then hand the message off.
        let _ = type_name;
        let mut msg = m.msg;
        fill_header_stamp(&mut msg, self.now());
        if let Err(e) = self.shared.backend.publish(pub_id, &msg) {
            self.send_status(StatusLevel::Error, format!("publish failed: {e}"), m.id);
        }
    }

    // ---- subscribe / unsubscribe ----------------------------------------

    fn op_subscribe(self: &Arc<Self>, m: Subscribe) {
        if !glob::allowed(&self.shared.cfg.topics_sub_glob, &m.topic) {
            self.send_status(
                StatusLevel::Warning,
                format!("subscribe to {} blocked by topics_sub_glob", m.topic),
                m.id.clone(),
            );
            return;
        }
        let sid = m.id.clone().unwrap_or_else(|| m.topic.clone());
        let params = SubParams {
            throttle_rate: m.throttle_rate.unwrap_or(0),
            queue_length: m.queue_length.unwrap_or(0),
            fragment_size: m.fragment_size,
            compression: m
                .compression
                .as_deref()
                .map(Compression::parse)
                .unwrap_or(Compression::None),
        };

        let mut state = self.state.lock();
        if let Some(entry) = state.subscriptions.get_mut(&m.topic) {
            entry.sids.insert(sid, params);
            let eff = coalesce(entry.sids.values());
            *entry.effective.lock() = eff;
            return;
        }

        // New subscription: need a type to decode.
        let type_name = match m
            .msg_type
            .clone()
            .or_else(|| self.shared.infer_type(&m.topic))
        {
            Some(t) => t,
            None => {
                drop(state);
                self.send_status(
                    StatusLevel::Error,
                    format!("subscribe to {}: unknown message type (provide 'type')", m.topic),
                    m.id,
                );
                return;
            }
        };
        let qos = resolve_qos(m.qos.as_ref(), None, None, QosSpec::default_subscriber());
        let (sub_id, rx) = match self.shared.backend.subscribe(&m.topic, &type_name, &qos) {
            Ok(v) => v,
            Err(e) => {
                drop(state);
                self.send_status(StatusLevel::Error, format!("subscribe failed: {e}"), m.id);
                return;
            }
        };
        self.shared.learn_type(&m.topic, &type_name);

        let effective = Arc::new(Mutex::new(params));
        let mut sids = HashMap::new();
        sids.insert(sid, params);

        let task = self
            .clone()
            .spawn_forwarder(m.topic.clone(), rx, effective.clone());

        state.subscriptions.insert(
            m.topic.clone(),
            SubEntry {
                id: sub_id,
                type_name,
                sids,
                effective,
                task,
            },
        );
    }

    fn op_unsubscribe(&self, m: Unsubscribe) {
        let mut state = self.state.lock();
        let remove = if let Some(entry) = state.subscriptions.get_mut(&m.topic) {
            match &m.id {
                Some(sid) => {
                    entry.sids.remove(sid);
                }
                None => entry.sids.clear(),
            }
            if entry.sids.is_empty() {
                true
            } else {
                let eff = coalesce(entry.sids.values());
                *entry.effective.lock() = eff;
                false
            }
        } else {
            return;
        };
        if remove {
            if let Some(entry) = state.subscriptions.remove(&m.topic) {
                entry.task.abort();
                self.shared.backend.unsubscribe(entry.id);
            }
        }
    }

    fn spawn_forwarder(
        self: Arc<Self>,
        topic: String,
        mut rx: mpsc::UnboundedReceiver<Sample>,
        effective: Arc<Mutex<SubParams>>,
    ) -> JoinHandle<()> {
        tokio::spawn(async move {
            let mut queue: std::collections::VecDeque<Sample> = std::collections::VecDeque::new();
            let mut last_sent: Option<Instant> = None;
            loop {
                let p = *effective.lock();
                let throttle = Duration::from_millis(p.throttle_rate);
                if p.queue_length > 0 && !queue.is_empty() {
                    let next_at = last_sent
                        .map(|t| t + throttle)
                        .unwrap_or_else(Instant::now);
                    tokio::select! {
                        maybe = rx.recv() => match maybe {
                            Some(s) => push_bounded(&mut queue, s, p.queue_length),
                            None => break,
                        },
                        _ = tokio::time::sleep_until(next_at.into()) => {
                            if let Some(s) = queue.pop_front() {
                                self.emit_sample(&topic, s.value, &p);
                                last_sent = Some(Instant::now());
                            }
                        }
                    }
                } else {
                    match rx.recv().await {
                        None => break,
                        Some(s) => {
                            if p.throttle_rate == 0 {
                                self.emit_sample(&topic, s.value, &p);
                                last_sent = Some(Instant::now());
                            } else if p.queue_length > 0 {
                                push_bounded(&mut queue, s, p.queue_length);
                            } else {
                                let ready = last_sent
                                    .map(|t| Instant::now().duration_since(t) >= throttle)
                                    .unwrap_or(true);
                                if ready {
                                    self.emit_sample(&topic, s.value, &p);
                                    last_sent = Some(Instant::now());
                                }
                            }
                        }
                    }
                }
            }
        })
    }

    /// Convert a received JSON sample into outgoing frame(s) and send them.
    fn emit_sample(&self, topic: &str, value: Value, p: &SubParams) {
        for frame in self.build_publish_frames(topic, value, p) {
            self.send_frame(frame);
        }
    }

    fn build_publish_frames(&self, topic: &str, msg: Value, p: &SubParams) -> Vec<OutFrame> {
        let v = json!({"op":"publish","topic":topic,"msg":msg});
        match p.compression {
            // `cbor-raw` requires the raw serialized CDR, which this backend
            // boundary no longer carries (ROS owns serialization); it therefore
            // behaves as `cbor` here.
            Compression::Cbor | Compression::CborRaw => vec![OutFrame::Binary(compression::to_cbor(&v))],
            Compression::Png => {
                let s = serde_json::to_string(&v).unwrap_or_default();
                match compression::png_encode(&s) {
                    Ok(b64) => {
                        let frame = json!({"op":"png","data":b64});
                        self.text_frames(serde_json::to_string(&frame).unwrap(), p.fragment_size)
                    }
                    Err(_) => Vec::new(),
                }
            }
            Compression::None => {
                self.text_frames(serde_json::to_string(&v).unwrap(), p.fragment_size)
            }
        }
    }

    /// Apply central fragmentation to a text payload if needed.
    fn text_frames(&self, serialized: String, fragment_size: Option<usize>) -> Vec<OutFrame> {
        let cap = fragment_size
            .map(|f| f.min(self.shared.cfg.max_message_size))
            .filter(|f| *f > 0);
        if let Some(size) = cap {
            if let Some(frags) = fragment::fragment(&serialized, size, &self.next_frag_id()) {
                return frags.into_iter().map(OutFrame::Text).collect();
            }
        }
        vec![OutFrame::Text(serialized)]
    }

    // ---- call_service (client as caller) --------------------------------

    fn op_call_service(self: &Arc<Self>, m: CallService) {
        let (service, inline_id) = split_name_id(&m.service);
        let id = m.id.clone().or(inline_id);
        if !glob_allowed_service(&self.shared.cfg.services_glob, &service) {
            self.send_status(
                StatusLevel::Warning,
                format!("call to service {service} blocked by services_glob"),
                id,
            );
            return;
        }
        let type_name = match m.srv_type.clone() {
            Some(t) => t,
            None => {
                self.send_status(
                    StatusLevel::Error,
                    format!("call_service {service}: missing service type"),
                    id,
                );
                return;
            }
        };
        let request = args_or_empty(m.args.clone());
        let timeout = m.timeout.unwrap_or(self.shared.cfg.default_call_service_timeout);
        let fragment_size = m.fragment_size;
        let me = self.clone();
        let service_cl = service.clone();
        tokio::spawn(async move {
            let result = me
                .shared
                .backend
                .call_service(&service_cl, &type_name, request, timeout)
                .await;
            match result {
                Ok(values) => {
                    let v = json!({
                        "op":"service_response","service":service_cl,
                        "values":values,"result":true,"id":id
                    });
                    for f in me.text_frames(serde_json::to_string(&v).unwrap(), fragment_size) {
                        me.send_frame(f);
                    }
                }
                Err(e) => {
                    let v = json!({
                        "op":"service_response","service":service_cl,
                        "values":e.to_string(),"result":false,"id":id
                    });
                    me.send_value(&v);
                }
            }
        });
    }

    // ---- advertise_service (client as server) ---------------------------

    fn op_advertise_service(self: &Arc<Self>, m: AdvertiseService) {
        if !glob_allowed_service(&self.shared.cfg.services_glob, &m.service) {
            return;
        }
        let (sid, mut rx) = match self
            .shared
            .backend
            .advertise_service(&m.service, &m.srv_type)
        {
            Ok(v) => v,
            Err(e) => {
                self.send_status(StatusLevel::Error, format!("advertise_service: {e}"), None);
                return;
            }
        };
        // Replace any existing.
        if let Some(old) = self.state.lock().services.remove(&m.service) {
            old.task.abort();
            self.shared.backend.unadvertise_service(old.id);
        }
        let me = self.clone();
        let service = m.service.clone();
        let task = tokio::spawn(async move {
            while let Some(req) = rx.recv().await {
                let req_id = format!(
                    "service_request:{}:{}",
                    service,
                    me.service_req_seq.fetch_add(1, Ordering::Relaxed)
                );
                let args = req.request;
                me.state
                    .lock()
                    .pending_service_responses
                    .insert(req_id.clone(), req.responder);
                let v = json!({
                    "op":"call_service","service":service,"id":req_id,"args":args
                });
                me.send_value(&v);
            }
        });
        self.state.lock().services.insert(
            m.service.clone(),
            SvcEntry { id: sid, task },
        );
    }

    fn op_unadvertise_service(&self, m: UnadvertiseService) {
        if !glob_allowed_service(&self.shared.cfg.services_glob, &m.service) {
            return;
        }
        if let Some(entry) = self.state.lock().services.remove(&m.service) {
            entry.task.abort();
            self.shared.backend.unadvertise_service(entry.id);
        } else {
            self.send_status(
                StatusLevel::Error,
                format!("service {} was not advertised", m.service),
                None,
            );
        }
    }

    fn op_service_response(&self, m: ServiceResponse) {
        let id = match &m.id {
            Some(i) => i.clone(),
            None => {
                self.send_status(StatusLevel::Error, "service_response missing id", None);
                return;
            }
        };
        let responder = self.state.lock().pending_service_responses.remove(&id);
        let responder = match responder {
            Some(r) => r,
            None => {
                self.send_status(
                    StatusLevel::Error,
                    format!("no pending service request for id {id}"),
                    Some(id),
                );
                return;
            }
        };
        if !m.result {
            let _ = responder.send(None);
            return;
        }
        // The ROS backend serializes the response; forward the JSON values.
        let values = args_or_empty(m.values.clone());
        let _ = responder.send(Some(values));
    }

    // ---- actions: send_action_goal (client as caller) -------------------

    fn op_send_action_goal(self: &Arc<Self>, m: SendActionGoal) {
        if !glob_allowed_action(&self.shared.cfg.actions_glob, &m.action) {
            self.send_status(
                StatusLevel::Warning,
                format!("send_action_goal to {} blocked by actions_glob", m.action),
                m.id.clone(),
            );
            return;
        }
        let request = args_or_empty(m.args.clone());
        let id = m.id.clone();
        let action = m.action.clone();
        let action_type = m.action_type.clone();
        let want_feedback = m.feedback;
        let me = self.clone();
        tokio::spawn(async move {
            let stream = match me
                .shared
                .backend
                .send_action_goal(&action, &action_type, request)
                .await
            {
                Ok(s) => s,
                Err(e) => {
                    let v = json!({
                        "op":"action_result","action":action,
                        "values":e.to_string(),"status":0,"result":false,"id":id
                    });
                    me.send_value(&v);
                    return;
                }
            };
            let goal_key = uuid_to_string(&stream.goal_id);
            me.state
                .lock()
                .issued_goals
                .insert(goal_key.clone(), action.clone());

            let mut feedback = stream.feedback;
            let result = stream.result;

            // Feedback pump.
            if want_feedback {
                let me_fb = me.clone();
                let action_fb = action.clone();
                let id_fb = id.clone();
                tokio::spawn(async move {
                    while let Some(values) = feedback.recv().await {
                        let v = json!({
                            "op":"action_feedback","action":action_fb,
                            "values":values,"id":id_fb
                        });
                        me_fb.send_value(&v);
                    }
                });
            }

            match result.await {
                Ok(Ok((values, status))) => {
                    let v = json!({
                        "op":"action_result","action":action,
                        "values":values,"status":status,"result":true,"id":id
                    });
                    me.send_value(&v);
                }
                Ok(Err(err)) => {
                    let v = json!({
                        "op":"action_result","action":action,
                        "values":err,"status":0,"result":false,"id":id
                    });
                    me.send_value(&v);
                }
                Err(_) => {
                    let v = json!({
                        "op":"action_result","action":action,
                        "values":"action channel closed","status":0,"result":false,"id":id
                    });
                    me.send_value(&v);
                }
            }
            me.state.lock().issued_goals.remove(&goal_key);
        });
    }

    fn op_cancel_action_goal(&self, m: CancelActionGoal) {
        // Try to interpret id as a uuid string we issued.
        let action = self
            .state
            .lock()
            .issued_goals
            .get(&m.id)
            .cloned()
            .unwrap_or_else(|| m.action.clone());
        if let Some(uuid) = string_to_uuid(&m.id) {
            self.shared.backend.cancel_action_goal(&action, &uuid);
        }
    }

    // ---- actions: advertise_action (client as server) -------------------

    fn op_advertise_action(self: &Arc<Self>, m: AdvertiseAction) {
        if !glob_allowed_action(&self.shared.cfg.actions_glob, &m.action) {
            return;
        }
        let (aid, mut rx) = match self
            .shared
            .backend
            .advertise_action(&m.action, &m.action_type)
        {
            Ok(v) => v,
            Err(e) => {
                self.send_status(StatusLevel::Error, format!("advertise_action: {e}"), None);
                return;
            }
        };
        if let Some(old) = self.state.lock().actions.remove(&m.action) {
            old.task.abort();
            self.shared.backend.unadvertise_action(old.id);
        }
        let me = self.clone();
        let action = m.action.clone();
        let action_type = m.action_type.clone();
        let task = tokio::spawn(async move {
            while let Some(goal) = rx.recv().await {
                let goal_key = uuid_to_string(&goal.goal_id);
                let args = goal.goal.clone();
                let crate::backend::ActionGoal {
                    feedback_tx,
                    result_tx,
                    cancel_rx,
                    ..
                } = goal;
                // Register feedback/result senders so incoming ActionFeedback/
                // ActionResult ops can be routed back to ROS.
                me.state.lock().hosted_goals.insert(
                    goal_key.clone(),
                    HostedGoal {
                        feedback_tx,
                        result_tx: Some(result_tx),
                    },
                );
                // Forward a ROS cancellation request to the client.
                {
                    let me_c = me.clone();
                    let action_c = action.clone();
                    let key_c = goal_key.clone();
                    tokio::spawn(async move {
                        if cancel_rx.await.is_ok() {
                            let v = json!({
                                "op":"cancel_action_goal","action":action_c,"id":key_c
                            });
                            me_c.send_value(&v);
                        }
                    });
                }
                let v = json!({
                    "op":"send_action_goal","action":action,"action_type":action_type,
                    "id":goal_key,"args":args,"feedback":true
                });
                me.send_value(&v);
            }
        });
        self.state.lock().actions.insert(
            m.action.clone(),
            ActEntry {
                id: aid,
                type_name: m.action_type,
                task,
            },
        );
    }

    fn op_unadvertise_action(&self, m: UnadvertiseAction) {
        if !glob_allowed_action(&self.shared.cfg.actions_glob, &m.action) {
            return;
        }
        if let Some(entry) = self.state.lock().actions.remove(&m.action) {
            entry.task.abort();
            self.shared.backend.unadvertise_action(entry.id);
        } else {
            self.send_status(
                StatusLevel::Error,
                format!("action {} was not advertised", m.action),
                None,
            );
        }
    }

    /// Feedback from a client-hosted action server, routed to ROS.
    fn op_action_feedback(&self, m: ActionFeedback) {
        let feedback_tx = {
            let state = self.state.lock();
            match state.hosted_goals.get(&m.id) {
                Some(g) => g.feedback_tx.clone(),
                None => {
                    self.send_status(
                        StatusLevel::Error,
                        format!("action_feedback for unknown goal {}", m.id),
                        Some(m.id),
                    );
                    return;
                }
            }
        };
        // The ROS backend serializes; forward the JSON feedback values.
        let _ = feedback_tx.send(m.values);
    }

    /// Final result from a client-hosted action server, routed to ROS.
    fn op_action_result(&self, m: ActionResult) {
        let status = m.status.unwrap_or(if m.result { 4 } else { 6 }) as i8;
        let hosted = match self.state.lock().hosted_goals.remove(&m.id) {
            Some(h) => h,
            None => {
                self.send_status(
                    StatusLevel::Error,
                    format!("action_result for unknown goal {}", m.id),
                    Some(m.id),
                );
                return;
            }
        };
        let values = if m.result {
            Some(args_or_empty(m.values))
        } else {
            None
        };
        if let Some(tx) = hosted.result_tx {
            let _ = tx.send((values, status));
        }
    }

    // ---- fragment (inbound) ---------------------------------------------

    fn op_fragment(self: &Arc<Self>, m: Fragment) {
        let outcome = self.defrag.lock().push(&m.id, m.num, m.total, m.data);
        match outcome {
            FragmentOutcome::Complete(s) => self.handle_text(&s),
            FragmentOutcome::Incomplete => {}
            FragmentOutcome::Invalid(reason) => {
                self.send_status(StatusLevel::Error, format!("fragment: {reason}"), Some(m.id))
            }
        }
    }

    /// Clean up all backend resources for this client on disconnect.
    pub fn shutdown(&self) {
        let mut state = self.state.lock();
        for (_, e) in state.publishers.drain() {
            self.shared.backend.unadvertise(e.id);
        }
        for (_, e) in state.subscriptions.drain() {
            e.task.abort();
            self.shared.backend.unsubscribe(e.id);
        }
        for (_, e) in state.services.drain() {
            e.task.abort();
            self.shared.backend.unadvertise_service(e.id);
        }
        for (_, e) in state.actions.drain() {
            e.task.abort();
            self.shared.backend.unadvertise_action(e.id);
        }
    }
}

// ---- free helpers --------------------------------------------------------

fn push_bounded(queue: &mut std::collections::VecDeque<Sample>, s: Sample, maxlen: usize) {
    if queue.len() >= maxlen {
        queue.pop_front();
    }
    queue.push_back(s);
}

/// Map the protocol QoS plus deprecated latch/queue_size to a [`QosSpec`].
fn resolve_qos(
    qos: Option<&rosbridge_protocol::QosProfile>,
    latch: Option<bool>,
    queue_size: Option<usize>,
    mut base: QosSpec,
) -> QosSpec {
    if let Some(q) = qos {
        if let Some(h) = &q.history {
            base.history_keep_all = h == "keep_all";
        }
        if let Some(d) = q.depth {
            base.depth = d;
        }
        if let Some(r) = &q.reliability {
            base.reliability = match r.as_str() {
                "reliable" => Reliability::Reliable,
                "best_effort" => Reliability::BestEffort,
                "best_available" => Reliability::BestAvailable,
                _ => Reliability::SystemDefault,
            };
        }
        if let Some(d) = &q.durability {
            base.durability = match d.as_str() {
                "transient_local" => Durability::TransientLocal,
                "volatile" => Durability::Volatile,
                "best_available" => Durability::BestAvailable,
                _ => Durability::SystemDefault,
            };
        }
    }
    if let Some(true) = latch {
        base.durability = Durability::TransientLocal;
    }
    if let Some(qs) = queue_size {
        base.depth = qs;
    }
    base
}

/// Normalize an optional `args`/`values` payload, defaulting missing/null to an
/// empty object. (ROS serialization expects a structured object; positional
/// array args are passed through and validated by the backend.)
fn args_or_empty(args: Option<Value>) -> Value {
    match args {
        Some(Value::Null) | None => Value::Object(Map::new()),
        Some(v) => v,
    }
}

/// Auto-fill `header.stamp` with the current time when a message carries a
/// `header` object whose `stamp` is absent, null, or the string `"now"`. This
/// is the schema-free equivalent of rosbridge's header auto-stamping.
fn fill_header_stamp(value: &mut Value, now: (i32, u32)) {
    if let Some(Value::Object(header)) = value.as_object_mut().and_then(|o| o.get_mut("header")) {
        let needs = match header.get("stamp") {
            None | Some(Value::Null) => true,
            Some(Value::String(s)) => s == "now",
            _ => false,
        };
        if needs {
            let mut stamp = Map::new();
            stamp.insert("sec".into(), Value::from(now.0));
            stamp.insert("nanosec".into(), Value::from(now.1));
            header.insert("stamp".into(), Value::Object(stamp));
        }
    }
}

/// Split a deprecated `name#id` form into `(name, Some(id))`.
fn split_name_id(s: &str) -> (String, Option<String>) {
    match s.split_once('#') {
        Some((name, id)) => (name.to_string(), Some(id.to_string())),
        None => (s.to_string(), None),
    }
}

fn glob_allowed_service(globs: &crate::config::GlobList, name: &str) -> bool {
    glob::allowed(globs, name)
}

fn glob_allowed_action(globs: &crate::config::GlobList, name: &str) -> bool {
    glob::allowed(globs, name)
}

fn uuid_to_string(uuid: &[u8; 16]) -> String {
    uuid.iter().map(|b| format!("{b:02x}")).collect()
}

fn string_to_uuid(s: &str) -> Option<[u8; 16]> {
    let clean: String = s.chars().filter(|c| c.is_ascii_hexdigit()).collect();
    if clean.len() != 32 {
        return None;
    }
    let mut out = [0u8; 16];
    for i in 0..16 {
        out[i] = u8::from_str_radix(&clean[i * 2..i * 2 + 2], 16).ok()?;
    }
    Some(out)
}
