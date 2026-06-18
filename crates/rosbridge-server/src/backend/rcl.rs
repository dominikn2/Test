//! RMW-agnostic ROS 2 backend built on [`r2r`] (the `rcl`/`rmw` layer).
//!
//! Because this rides ROS 2's own middleware abstraction, the **same binary
//! works with any RMW** — CycloneDDS, Fast-DDS, or Zenoh — selected at runtime
//! by the `RMW_IMPLEMENTATION` environment variable. This is the only way to be
//! universal across middlewares (`rmw_zenoh` is not RTPS, so a pure-DDS stack
//! cannot reach it).
//!
//! All (de)serialization is delegated to ROS 2's own type introspection via
//! r2r's **untyped** API: messages cross the backend boundary as
//! `serde_json::Value` and r2r/`rmw` produces and parses the CDR. There is no
//! custom codec on this path.
//!
//! ## Build requirements
//! This module compiles **only inside a sourced ROS 2 environment** with
//! `libclang` available (r2r generates bindings from the installed interface
//! packages at build time). Build it with the `rcl` feature inside the
//! container described in `docker/`. It is excluded from the default build.
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
use serde_json::{Map, Value};
use tokio::sync::{mpsc, oneshot};

use super::*;

/// A ROS 2 backend over `rcl`/`rmw` via r2r.
pub struct RclBackend {
    node: Arc<Mutex<r2r::Node>>,
    publishers: Mutex<HashMap<PublisherId, r2r::PublisherUntyped>>,
    sub_tasks: Mutex<HashMap<SubscriptionId, tokio::task::JoinHandle<()>>>,
    svc_tasks: Mutex<HashMap<ServiceServerId, tokio::task::JoinHandle<()>>>,
    counter: AtomicU64,
}

impl RclBackend {
    /// Create the backend and start the ROS spin thread.
    pub fn new() -> Result<Self, BackendError> {
        Self::with_node_name("rosbridge_rs")
    }

    pub fn with_node_name(name: &str) -> Result<Self, BackendError> {
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
            publishers: Mutex::new(HashMap::new()),
            sub_tasks: Mutex::new(HashMap::new()),
            svc_tasks: Mutex::new(HashMap::new()),
            counter: AtomicU64::new(0),
        })
    }

    pub fn shared() -> Result<SharedBackend, BackendError> {
        Ok(Arc::new(Self::new()?))
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

    fn publish(&self, id: PublisherId, msg: &Value) -> Result<(), BackendError> {
        let guard = self.publishers.lock();
        let publisher = guard
            .get(&id)
            .ok_or_else(|| BackendError::Failed("unknown publisher".into()))?;
        // r2r's untyped publisher serializes the JSON via ROS type introspection.
        publisher
            .publish(msg.clone())
            .map_err(|e| BackendError::Failed(format!("publish: {e}")))
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
        // Untyped subscription: r2r deserializes CDR to JSON via introspection.
        let stream = self
            .node
            .lock()
            .subscribe_untyped(topic, &ros_type_string(type_name), to_qos(qos))
            .map_err(|e| BackendError::Failed(format!("subscribe: {e}")))?;
        let (tx, rx) = mpsc::unbounded_channel();
        let id = SubscriptionId(self.next_id());
        let task = tokio::spawn(async move {
            let mut stream = stream;
            while let Some(result) = stream.next().await {
                match result {
                    Ok(value) => {
                        if tx.send(Sample { value }).is_err() {
                            break;
                        }
                    }
                    Err(e) => tracing::debug!("rcl subscription error: {e}"),
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
        request: Value,
        timeout_secs: f64,
    ) -> Result<Value, BackendError> {
        let client = self
            .node
            .lock()
            .create_client_untyped(service, &ros_type_string(type_name), to_qos(&QosSpec::default_publisher()))
            .map_err(|e| BackendError::Failed(format!("create_client: {e}")))?;

        let fut = client
            .request(request)
            .map_err(|e| BackendError::Failed(format!("request: {e}")))?;
        let resp = tokio::time::timeout(Duration::from_secs_f64(timeout_secs.max(0.0)), fut)
            .await
            .map_err(|_| BackendError::Timeout)?
            .map_err(|e| BackendError::Failed(format!("service error: {e}")))?;
        // r2r untyped responses are Result<Value, _>; surface either form.
        resp.map_err(|e| BackendError::Failed(format!("service response: {e}")))
    }

    fn advertise_service(
        &self,
        service: &str,
        type_name: &str,
    ) -> Result<(ServiceServerId, mpsc::UnboundedReceiver<ServiceRequest>), BackendError> {
        let server = self
            .node
            .lock()
            .create_service_untyped(service, &ros_type_string(type_name), to_qos(&QosSpec::default_publisher()))
            .map_err(|e| BackendError::Failed(format!("create_service: {e}")))?;

        let (tx, rx) = mpsc::unbounded_channel();
        let id = ServiceServerId(self.next_id());
        let task = tokio::spawn(async move {
            let mut server = server;
            while let Some(req) = server.next().await {
                let (resp_tx, resp_rx) = oneshot::channel();
                // r2r untyped request carries the JSON message; respond with JSON.
                let request: Value = req.message.clone();
                if tx.send(ServiceRequest { request, responder: resp_tx }).is_err() {
                    break;
                }
                let reply = match resp_rx.await {
                    Ok(Some(value)) => value,
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
        _goal: Value,
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

    fn discover_type(&self, topic: &str) -> Option<String> {
        // Query the ROS graph for the topic's type(s).
        let names = self.node.lock().get_topic_names_and_types().ok()?;
        names
            .get(topic)
            .and_then(|types| types.first().cloned())
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
