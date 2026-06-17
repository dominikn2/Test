//! Per-client protocol session: dispatches rosbridge ops to backend operations,
//! manages publishers/subscriptions/services/actions, and produces outgoing
//! frames. This is the Rust equivalent of `RosbridgeProtocol` plus the
//! capability classes.

use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use parking_lot::Mutex;
use ros_message::{Codec, Registry};
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
    pub registry: Arc<Registry>,
    pub backend: SharedBackend,
    /// Best-effort topic -> type map, learned from advertise/publish, used to
    /// infer types for `subscribe` when the client omits `type`.
    pub topic_types: Mutex<HashMap<String, String>>,
}

impl Shared {
    pub fn new(cfg: SharedConfig, registry: Arc<Registry>, backend: SharedBackend) -> Arc<Self> {
        Arc::new(Shared {
            cfg,
            registry,
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
        self.topic_types.lock().get(topic).cloned()
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
    type_name: String,
    task: JoinHandle<()>,
}

struct ActEntry {
    id: crate::backend::ActionServerId,
    #[allow(dead_code)]
    type_name: String,
    task: JoinHandle<()>,
}

/// Channels for a goal currently being served by this client (client-hosted
/// action server).
struct HostedGoal {
    feedback_tx: mpsc::UnboundedSender<Vec<u8>>,
    result_tx: Option<oneshot::Sender<(Option<Vec<u8>>, i8)>>,
    action_type: String,
}

#[derive(Default)]
struct State {
    publishers: HashMap<String, PubEntry>,
    subscriptions: HashMap<String, SubEntry>,
    services: HashMap<String, SvcEntry>,
    actions: HashMap<String, ActEntry>,
    /// request id -> responder for client-hosted services.
    pending_service_responses: HashMap<String, oneshot::Sender<Option<Vec<u8>>>>,
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
            IncomingMessage::Status(_) | IncomingMessage::SetLevel(_) | IncomingMessage::Auth(_) => {
                /* status/set_level echoes and auth are accepted and ignored */
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
        let v = json!({"op":"status","level":level.as_str(),"msg":msg,"id":id});
        self.send_value(&v);
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

        // Encode JSON -> CDR.
        let spec = match self.shared.registry.message(&type_name) {
            Ok(s) => s,
            Err(e) => {
                self.send_status(StatusLevel::Error, format!("publish: {e}"), m.id);
                return;
            }
        };
        let codec = Codec::with_now(&self.shared.registry, Some(self.now()));
        match codec.encode(spec, &m.msg) {
            Ok(cdr) => {
                if let Err(e) = self.shared.backend.publish(pub_id, &cdr) {
                    self.send_status(StatusLevel::Error, format!("publish failed: {e}"), m.id);
                }
            }
            Err(e) => self.send_status(StatusLevel::Error, format!("publish encode: {e}"), m.id),
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

        let task = self.clone().spawn_forwarder(
            m.topic.clone(),
            type_name.clone(),
            rx,
            effective.clone(),
        );

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
        type_name: String,
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
                                self.emit_sample(&topic, &type_name, &s.cdr, &p);
                                last_sent = Some(Instant::now());
                            }
                        }
                    }
                } else {
                    match rx.recv().await {
                        None => break,
                        Some(s) => {
                            if p.throttle_rate == 0 {
                                self.emit_sample(&topic, &type_name, &s.cdr, &p);
                                last_sent = Some(Instant::now());
                            } else if p.queue_length > 0 {
                                push_bounded(&mut queue, s, p.queue_length);
                            } else {
                                let ready = last_sent
                                    .map(|t| Instant::now().duration_since(t) >= throttle)
                                    .unwrap_or(true);
                                if ready {
                                    self.emit_sample(&topic, &type_name, &s.cdr, &p);
                                    last_sent = Some(Instant::now());
                                }
                            }
                        }
                    }
                }
            }
        })
    }

    /// Convert a received CDR sample into outgoing frame(s) and send them.
    fn emit_sample(&self, topic: &str, type_name: &str, cdr: &[u8], p: &SubParams) {
        for frame in self.build_publish_frames(topic, type_name, cdr, p) {
            self.send_frame(frame);
        }
    }

    fn build_publish_frames(
        &self,
        topic: &str,
        type_name: &str,
        cdr: &[u8],
        p: &SubParams,
    ) -> Vec<OutFrame> {
        if p.compression == Compression::CborRaw {
            let (secs, nsecs) = wall_clock();
            return vec![OutFrame::Binary(compression::cbor_raw_frame(
                topic, cdr, secs, nsecs,
            ))];
        }
        let spec = match self.shared.registry.message(type_name) {
            Ok(s) => s,
            Err(e) => {
                tracing::warn!("cannot decode {type_name} on {topic}: {e}");
                return Vec::new();
            }
        };
        let codec = Codec::new(&self.shared.registry);
        let msg = match codec.decode(spec, cdr) {
            Ok(v) => v,
            Err(e) => {
                tracing::warn!("decode error on {topic}: {e}");
                return Vec::new();
            }
        };
        match p.compression {
            Compression::Cbor => {
                let v = json!({"op":"publish","topic":topic,"msg":msg});
                vec![OutFrame::Binary(compression::to_cbor(&v))]
            }
            Compression::Png => {
                let v = json!({"op":"publish","topic":topic,"msg":msg});
                let s = serde_json::to_string(&v).unwrap_or_default();
                match compression::png_encode(&s) {
                    Ok(b64) => {
                        let frame = json!({"op":"png","data":b64});
                        self.text_frames(serde_json::to_string(&frame).unwrap(), p.fragment_size)
                    }
                    Err(_) => Vec::new(),
                }
            }
            _ => {
                let v = json!({"op":"publish","topic":topic,"msg":msg});
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
        let spec = match self.shared.registry.service(&type_name) {
            Ok(s) => s.clone(),
            Err(e) => {
                self.send_status(StatusLevel::Error, format!("call_service: {e}"), id);
                return;
            }
        };
        let args = m.args.clone().unwrap_or(Value::Object(Map::new()));
        let codec = Codec::with_now(&self.shared.registry, Some(self.now()));
        let req_obj = args_to_object(&spec.request.fields, &args);
        let request_cdr = match codec.encode(&spec.request, &req_obj) {
            Ok(c) => c,
            Err(e) => {
                self.send_status(StatusLevel::Error, format!("call_service encode: {e}"), id);
                return;
            }
        };
        let timeout = m.timeout.unwrap_or(self.shared.cfg.default_call_service_timeout);
        let fragment_size = m.fragment_size;
        let me = self.clone();
        let service_cl = service.clone();
        let resp_type = type_name.clone();
        tokio::spawn(async move {
            let result = me
                .shared
                .backend
                .call_service(&service_cl, &resp_type, request_cdr, timeout)
                .await;
            match result {
                Ok(resp_cdr) => {
                    let resp_spec = me.shared.registry.service(&resp_type);
                    let values = match resp_spec {
                        Ok(s) => Codec::new(&me.shared.registry)
                            .decode(&s.response, &resp_cdr)
                            .unwrap_or(Value::Object(Map::new())),
                        Err(_) => Value::Object(Map::new()),
                    };
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
        // Validate type resolvable.
        if self.shared.registry.service(&m.srv_type).is_err() {
            self.send_status(
                StatusLevel::Error,
                format!("advertise_service: unknown type {}", m.srv_type),
                None,
            );
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
        let type_name = m.srv_type.clone();
        let task = tokio::spawn(async move {
            while let Some(req) = rx.recv().await {
                let req_id = format!(
                    "service_request:{}:{}",
                    service,
                    me.service_req_seq.fetch_add(1, Ordering::Relaxed)
                );
                let spec = match me.shared.registry.service(&type_name) {
                    Ok(s) => s.clone(),
                    Err(_) => {
                        let _ = req.responder.send(None);
                        continue;
                    }
                };
                let args = Codec::new(&me.shared.registry)
                    .decode(&spec.request, &req.request_cdr)
                    .unwrap_or(Value::Object(Map::new()));
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
            SvcEntry {
                id: sid,
                type_name: m.srv_type,
                task,
            },
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
        // Encode the response values.
        let type_name = self
            .state
            .lock()
            .services
            .get(&m.service)
            .map(|e| e.type_name.clone());
        let cdr = match type_name.and_then(|t| self.shared.registry.service(&t).ok().cloned()) {
            Some(spec) => {
                let values = m.values.clone().unwrap_or(Value::Object(Map::new()));
                let obj = args_to_object(&spec.response.fields, &values);
                Codec::with_now(&self.shared.registry, Some(self.now()))
                    .encode(&spec.response, &obj)
                    .ok()
            }
            None => None,
        };
        let _ = responder.send(cdr);
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
        let spec = match self.shared.registry.action(&m.action_type) {
            Ok(s) => s.clone(),
            Err(e) => {
                self.send_status(StatusLevel::Error, format!("send_action_goal: {e}"), m.id);
                return;
            }
        };
        let args = m.args.clone().unwrap_or(Value::Object(Map::new()));
        let obj = args_to_object(&spec.goal.fields, &args);
        let goal_cdr = match Codec::with_now(&self.shared.registry, Some(self.now()))
            .encode(&spec.goal, &obj)
        {
            Ok(c) => c,
            Err(e) => {
                self.send_status(StatusLevel::Error, format!("goal encode: {e}"), m.id);
                return;
            }
        };
        let id = m.id.clone();
        let action = m.action.clone();
        let action_type = m.action_type.clone();
        let want_feedback = m.feedback;
        let me = self.clone();
        tokio::spawn(async move {
            let stream = match me
                .shared
                .backend
                .send_action_goal(&action, &action_type, goal_cdr)
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
            let result_spec = spec.result.clone();
            let feedback_spec = spec.feedback.clone();

            // Feedback pump.
            if want_feedback {
                let me_fb = me.clone();
                let action_fb = action.clone();
                let id_fb = id.clone();
                tokio::spawn(async move {
                    while let Some(cdr) = feedback.recv().await {
                        let values = Codec::new(&me_fb.shared.registry)
                            .decode(&feedback_spec, &cdr)
                            .unwrap_or(Value::Object(Map::new()));
                        let v = json!({
                            "op":"action_feedback","action":action_fb,
                            "values":values,"id":id_fb
                        });
                        me_fb.send_value(&v);
                    }
                });
            }

            match result.await {
                Ok(Ok((cdr, status))) => {
                    let values = Codec::new(&me.shared.registry)
                        .decode(&result_spec, &cdr)
                        .unwrap_or(Value::Object(Map::new()));
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
        if self.shared.registry.action(&m.action_type).is_err() {
            self.send_status(
                StatusLevel::Error,
                format!("advertise_action: unknown type {}", m.action_type),
                None,
            );
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
                let spec = match me.shared.registry.action(&action_type) {
                    Ok(s) => s.clone(),
                    Err(_) => continue,
                };
                let args = Codec::new(&me.shared.registry)
                    .decode(&spec.goal, &goal.goal_cdr)
                    .unwrap_or(Value::Object(Map::new()));
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
                        action_type: action_type.clone(),
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
        let (action_type, feedback_tx) = {
            let state = self.state.lock();
            match state.hosted_goals.get(&m.id) {
                Some(g) => (g.action_type.clone(), g.feedback_tx.clone()),
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
        let spec = match self.shared.registry.action(&action_type) {
            Ok(s) => s.clone(),
            Err(_) => return,
        };
        let obj = args_to_object(&spec.feedback.fields, &m.values);
        if let Ok(cdr) =
            Codec::with_now(&self.shared.registry, Some(self.now())).encode(&spec.feedback, &obj)
        {
            let _ = feedback_tx.send(cdr);
        }
    }

    /// Final result from a client-hosted action server, routed to ROS.
    fn op_action_result(&self, m: ActionResult) {
        let status = m.status.unwrap_or(if m.result { 4 } else { 6 }) as i8;
        let mut state = self.state.lock();
        let hosted = match state.hosted_goals.remove(&m.id) {
            Some(h) => h,
            None => {
                drop(state);
                self.send_status(
                    StatusLevel::Error,
                    format!("action_result for unknown goal {}", m.id),
                    Some(m.id),
                );
                return;
            }
        };
        drop(state);
        let cdr = if m.result {
            self.shared
                .registry
                .action(&hosted.action_type)
                .ok()
                .cloned()
                .and_then(|spec| {
                    let obj =
                        args_to_object(&spec.result.fields, m.values.as_ref().unwrap_or(&Value::Null));
                    Codec::with_now(&self.shared.registry, Some(self.now()))
                        .encode(&spec.result, &obj)
                        .ok()
                })
        } else {
            None
        };
        if let Some(tx) = hosted.result_tx {
            let _ = tx.send((cdr, status));
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

fn wall_clock() -> (i64, u32) {
    let d = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
    (d.as_secs() as i64, d.subsec_nanos())
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

/// Build a request/response object from `args`, accepting both object and
/// positional-array forms.
fn args_to_object(fields: &[ros_message::Field], args: &Value) -> Value {
    match args {
        Value::Object(_) => args.clone(),
        Value::Array(arr) => {
            let mut m = Map::new();
            for (f, v) in fields.iter().zip(arr.iter()) {
                m.insert(f.name.clone(), v.clone());
            }
            Value::Object(m)
        }
        Value::Null => Value::Object(Map::new()),
        other => other.clone(),
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
