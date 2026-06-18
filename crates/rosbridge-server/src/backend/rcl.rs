//! RMW-agnostic ROS 2 backend built on [`r2r`] (the `rcl`/`rmw` layer).
//!
//! Because this rides ROS 2's own middleware abstraction, the **same binary
//! works with any RMW** — CycloneDDS, Fast-DDS, or Zenoh — selected at runtime
//! by the `RMW_IMPLEMENTATION` environment variable. This is the only way to be
//! universal across middlewares (`rmw_zenoh` is not RTPS, so a pure-DDS stack
//! cannot reach it) and it gives full services support via ROS's own type
//! introspection.
//!
//! ## Build requirements
//! This module compiles **only inside a sourced ROS 2 environment** with
//! `libclang` available (r2r generates bindings from the installed interface
//! packages at build time). Build it with the `rcl` feature inside the
//! container described in `docker/`. It is excluded from the default build.
//!
//! ## Representation
//! The [`RosBackend`] trait exchanges raw CDR. Topic subscriptions use r2r's
//! `subscribe_raw`, so received bytes flow through untouched (preserving the
//! zero-copy `cbor-raw` path). The publish and service paths convert at the r2r
//! boundary using the shared [`Registry`] codec, since r2r's untyped API speaks
//! `serde_json::Value`.
//!
//! ## Scope
//! Topics and services are implemented. Actions are not yet supported here
//! (r2r exposes only statically-typed actions, so dynamic/runtime-typed action
//! bridging needs a manual `rcl_action` implementation); action ops return
//! [`BackendError::Unsupported`].
//!
//! Some r2r call signatures vary slightly across point releases; any mismatch
//! surfaces on the first in-container build and is a mechanical fix.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use async_trait::async_trait;
use futures::StreamExt;
use parking_lot::Mutex;
use ros_message::{Codec, Registry};
use serde_json::{Map, Value};
use tokio::sync::{mpsc, oneshot};

use super::*;

/// A ROS 2 backend over `rcl`/`rmw` via r2r.
pub struct RclBackend {
    node: Arc<Mutex<r2r::Node>>,
    registry: Arc<Registry>,
    publishers: Mutex<HashMap<PublisherId, r2r::PublisherUntyped>>,
    sub_tasks: Mutex<HashMap<SubscriptionId, tokio::task::JoinHandle<()>>>,
    svc_tasks: Mutex<HashMap<ServiceServerId, tokio::task::JoinHandle<()>>>,
    counter: AtomicU64,
}

impl RclBackend {
    /// Create the backend, start the ROS spin thread, and return it.
    ///
    /// `registry` must already contain the interface definitions of the types
    /// being bridged (load them from the install with
    /// [`Registry::load_ament_prefix`] over `$AMENT_PREFIX_PATH`).
    pub fn new(registry: Arc<Registry>) -> Result<Self, BackendError> {
        Self::with_node_name(registry, "rosbridge_rs")
    }

    pub fn with_node_name(registry: Arc<Registry>, name: &str) -> Result<Self, BackendError> {
        let ctx = r2r::Context::create()
            .map_err(|e| BackendError::Failed(format!("rcl context: {e}")))?;
        let node = r2r::Node::create(ctx, name, "")
            .map_err(|e| BackendError::Failed(format!("rcl node: {e}")))?;
        let node = Arc::new(Mutex::new(node));

        // Drive rcl callbacks on a dedicated thread; entities are created and
        // spun under the same lock (spin_once is brief, so contention is low).
        let spin = node.clone();
        std::thread::Builder::new()
            .name("rcl-spin".into())
            .spawn(move || loop {
                spin.lock().spin_once(Duration::from_millis(50));
            })
            .map_err(|e| BackendError::Failed(format!("spin thread: {e}")))?;

        Ok(RclBackend {
            node,
            registry,
            publishers: Mutex::new(HashMap::new()),
            sub_tasks: Mutex::new(HashMap::new()),
            svc_tasks: Mutex::new(HashMap::new()),
            counter: AtomicU64::new(0),
        })
    }

    pub fn shared(registry: Arc<Registry>) -> Result<SharedBackend, BackendError> {
        Ok(Arc::new(Self::new(registry)?))
    }

    fn next_id(&self) -> u64 {
        self.counter.fetch_add(1, Ordering::Relaxed)
    }
}

#[async_trait]
impl RosBackend for RclBackend {
    fn advertise(
        &self,
        topic: &str,
        type_name: &str,
        qos: &QosSpec,
    ) -> Result<PublisherId, BackendError> {
        let publisher = self
            .node
            .lock()
            .create_publisher_untyped(topic, &ros_type_string(type_name), to_qos(qos))
            .map_err(|e| BackendError::Failed(format!("create_publisher: {e}")))?;
        let id = PublisherId(self.next_id());
        self.publishers.lock().insert(id, publisher);
        Ok(id)
    }

    fn publish(&self, id: PublisherId, cdr: &[u8]) -> Result<(), BackendError> {
        // The trait delivers CDR; r2r's untyped publisher takes JSON, so decode
        // against the schema first. (A future r2r `publish_raw` would let us
        // skip this round-trip.)
        let value = {
            let guard = self.publishers.lock();
            let publisher = guard
                .get(&id)
                .ok_or_else(|| BackendError::Failed("unknown publisher".into()))?;
            let type_name = publisher_type(publisher);
            let spec = self
                .registry
                .message(&type_name)
                .map_err(|e| BackendError::Failed(format!("publish type: {e}")))?;
            let v = Codec::new(&self.registry)
                .decode(spec, cdr)
                .map_err(|e| BackendError::Failed(format!("publish decode: {e}")))?;
            publisher
                .publish(v.clone())
                .map_err(|e| BackendError::Failed(format!("publish: {e}")))?;
            v
        };
        let _ = value;
        Ok(())
    }

    fn unadvertise(&self, id: PublisherId) {
        self.publishers.lock().remove(&id);
    }

    fn subscribe(
        &self,
        topic: &str,
        type_name: &str,
        qos: &QosSpec,
    ) -> Result<(SubscriptionId, mpsc::UnboundedReceiver<Sample>), BackendError> {
        // Raw subscription: bytes pass through untouched (zero-copy cbor-raw).
        let stream = self
            .node
            .lock()
            .subscribe_raw(topic, &ros_type_string(type_name), to_qos(qos))
            .map_err(|e| BackendError::Failed(format!("subscribe: {e}")))?;
        let (tx, rx) = mpsc::unbounded_channel();
        let id = SubscriptionId(self.next_id());
        let task = tokio::spawn(async move {
            let mut stream = stream;
            while let Some(bytes) = stream.next().await {
                if tx.send(Sample { cdr: bytes }).is_err() {
                    break;
                }
            }
        });
        self.sub_tasks.lock().insert(id, task);
        Ok((id, rx))
    }

    fn unsubscribe(&self, id: SubscriptionId) {
        if let Some(task) = self.sub_tasks.lock().remove(&id) {
            task.abort();
        }
    }

    async fn call_service(
        &self,
        service: &str,
        type_name: &str,
        request_cdr: Cdr,
        timeout_secs: f64,
    ) -> Result<Cdr, BackendError> {
        let svc = self
            .registry
            .service(type_name)
            .map_err(|e| BackendError::Failed(format!("service type: {e}")))?
            .clone();
        let request = Codec::new(&self.registry)
            .decode(&svc.request, &request_cdr)
            .map_err(|e| BackendError::Failed(format!("request decode: {e}")))?;

        let client = self
            .node
            .lock()
            .create_client_untyped(service, &ros_type_string(type_name), to_qos(&QosSpec::default_publisher()))
            .map_err(|e| BackendError::Failed(format!("create_client: {e}")))?;

        let fut = client
            .request(request)
            .map_err(|e| BackendError::Failed(format!("request: {e}")))?;
        let resp_value = tokio::time::timeout(Duration::from_secs_f64(timeout_secs.max(0.0)), fut)
            .await
            .map_err(|_| BackendError::Timeout)?
            .map_err(|e| BackendError::Failed(format!("service error: {e}")))?
            .map_err(|e| BackendError::Failed(format!("service response: {e}")))?;

        Codec::with_now(&self.registry, Some(self.now()))
            .encode(&svc.response, &resp_value)
            .map_err(|e| BackendError::Failed(format!("response encode: {e}")))
    }

    fn advertise_service(
        &self,
        service: &str,
        type_name: &str,
    ) -> Result<(ServiceServerId, mpsc::UnboundedReceiver<ServiceRequest>), BackendError> {
        let svc = self
            .registry
            .service(type_name)
            .map_err(|e| BackendError::Failed(format!("service type: {e}")))?
            .clone();
        let server = self
            .node
            .lock()
            .create_service_untyped(service, &ros_type_string(type_name), to_qos(&QosSpec::default_publisher()))
            .map_err(|e| BackendError::Failed(format!("create_service: {e}")))?;

        let (tx, rx) = mpsc::unbounded_channel();
        let id = ServiceServerId(self.next_id());
        let registry = self.registry.clone();
        let now = self.now();
        let task = tokio::spawn(async move {
            let mut server = server;
            while let Some(req) = server.next().await {
                // Encode the ROS request (JSON) to CDR for the ws client.
                let request_cdr = match Codec::new(&registry).encode(&svc.request, req.message()) {
                    Ok(c) => c,
                    Err(_) => continue,
                };
                let (resp_tx, resp_rx) = oneshot::channel();
                if tx.send(ServiceRequest { request_cdr, responder: resp_tx }).is_err() {
                    break;
                }
                // Await the ws client's response, convert back to JSON, reply.
                let reply = match resp_rx.await {
                    Ok(Some(cdr)) => Codec::with_now(&registry, Some(now))
                        .decode(&svc.response, &cdr)
                        .unwrap_or_else(|_| Value::Object(Map::new())),
                    _ => Value::Object(Map::new()),
                };
                let _ = req.respond(reply);
            }
        });
        self.svc_tasks.lock().insert(id, task);
        Ok((id, rx))
    }

    fn unadvertise_service(&self, id: ServiceServerId) {
        if let Some(task) = self.svc_tasks.lock().remove(&id) {
            task.abort();
        }
    }

    async fn send_action_goal(
        &self,
        _action: &str,
        _type_name: &str,
        _goal_cdr: Cdr,
    ) -> Result<GoalStream, BackendError> {
        Err(BackendError::Unsupported(
            "actions over rcl (r2r exposes only statically-typed actions)",
        ))
    }

    fn cancel_action_goal(&self, _action: &str, _goal_id: &[u8; 16]) {}

    fn advertise_action(
        &self,
        _action: &str,
        _type_name: &str,
    ) -> Result<(ActionServerId, mpsc::UnboundedReceiver<ActionGoal>), BackendError> {
        Err(BackendError::Unsupported("actions over rcl"))
    }

    fn unadvertise_action(&self, _id: ActionServerId) {}

    fn now(&self) -> (i32, u32) {
        let d = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
        (d.as_secs() as i32, d.subsec_nanos())
    }
}

/// ROS 2 type strings for r2r use the `pkg/msg/Type` form already produced by
/// the protocol layer; normalize category-omitted names just in case.
fn ros_type_string(type_name: &str) -> String {
    let parts: Vec<&str> = type_name.split('/').collect();
    match parts.as_slice() {
        [pkg, ty] => format!("{pkg}/msg/{ty}"),
        _ => type_name.to_string(),
    }
}

/// The message type a publisher was created with, recovered from r2r.
fn publisher_type(p: &r2r::PublisherUntyped) -> String {
    p.type_name().to_string()
}

/// Map a normalized [`QosSpec`] to an r2r [`r2r::QosProfile`].
fn to_qos(qos: &QosSpec) -> r2r::QosProfile {
    let mut p = r2r::QosProfile::default();
    p = if qos.history_keep_all {
        p.keep_all()
    } else {
        p.keep_last(qos.depth as i32)
    };
    p = match qos.reliability {
        Reliability::BestEffort => p.best_effort(),
        _ => p.reliable(),
    };
    p = match qos.durability {
        Durability::TransientLocal => p.transient_local(),
        _ => p.volatile(),
    };
    p
}
