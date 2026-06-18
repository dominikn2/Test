//! The ROS backend abstraction.
//!
//! The boundary is `serde_json::Value` (the rosbridge-native representation):
//! the protocol layer hands messages as JSON and the backend is responsible for
//! any ROS serialization. The `rcl` backend delegates that entirely to ROS 2's
//! own type introspection (via r2r's untyped API), so there is no custom CDR
//! codec on the runtime path.
//!
//! Two implementations exist:
//! * [`loopback::LoopbackBackend`] — an in-process bus, used for tests and for
//!   browser↔browser bridging without any ROS 2 middleware.
//! * `rcl::RclBackend` — an RMW-agnostic ROS 2 backend built on `r2r` (the
//!   `rcl`/`rmw` layer), compiled in with the `rcl` feature. Works with any
//!   middleware (CycloneDDS, Fast-DDS, Zenoh) via `RMW_IMPLEMENTATION`.

pub mod loopback;
#[cfg(feature = "rcl")]
pub mod rcl;

use std::sync::Arc;

use async_trait::async_trait;
use serde_json::Value;
use tokio::sync::{mpsc, oneshot};

/// Normalized QoS, derived from the protocol `qos` object plus deprecated
/// `latch`/`queue_size` shortcuts.
#[derive(Debug, Clone, PartialEq)]
pub struct QosSpec {
    pub history_keep_all: bool,
    pub depth: usize,
    pub reliability: Reliability,
    pub durability: Durability,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reliability {
    Reliable,
    BestEffort,
    SystemDefault,
    BestAvailable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Durability {
    Volatile,
    TransientLocal,
    SystemDefault,
    BestAvailable,
}

impl QosSpec {
    /// Default publisher QoS per the protocol: reliable, transient-local,
    /// keep-last depth 100.
    pub fn default_publisher() -> Self {
        QosSpec {
            history_keep_all: false,
            depth: 100,
            reliability: Reliability::Reliable,
            durability: Durability::TransientLocal,
        }
    }

    /// Default subscriber QoS per the protocol: best-effort, volatile,
    /// keep-last depth 10.
    pub fn default_subscriber() -> Self {
        QosSpec {
            history_keep_all: false,
            depth: 10,
            reliability: Reliability::BestEffort,
            durability: Durability::Volatile,
        }
    }
}

/// An opaque handle identifying a created publisher.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PublisherId(pub u64);

/// An opaque handle identifying a created subscription.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SubscriptionId(pub u64);

/// An opaque handle identifying a hosted (advertised) service server.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ServiceServerId(pub u64);

/// An opaque handle identifying a hosted (advertised) action server.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ActionServerId(pub u64);

/// A received topic sample (the message as JSON).
#[derive(Debug, Clone)]
pub struct Sample {
    pub value: Value,
}

/// An inbound service request routed to a client-hosted service server.
pub struct ServiceRequest {
    pub request: Value,
    /// Channel to deliver the response (or `None` on failure/abort).
    pub responder: oneshot::Sender<Option<Value>>,
}

/// An inbound action goal routed to a client-hosted action server. The hosting
/// client emits feedback and a final result through the provided senders.
pub struct ActionGoal {
    pub goal: Value,
    pub goal_id: [u8; 16],
    /// The hosting client publishes feedback (JSON) here.
    pub feedback_tx: mpsc::UnboundedSender<Value>,
    /// The hosting client delivers the final `(result, status)` here;
    /// `None` result indicates failure/abort.
    pub result_tx: oneshot::Sender<(Option<Value>, i8)>,
    /// Fires when the goal-issuing side requests cancellation.
    pub cancel_rx: oneshot::Receiver<()>,
}

/// Outcome of issuing an action goal as a client.
pub struct GoalStream {
    pub feedback: mpsc::UnboundedReceiver<Value>,
    pub result: oneshot::Receiver<Result<(Value, i8), String>>,
    pub goal_id: [u8; 16],
}

/// Backend errors.
#[derive(Debug, thiserror::Error)]
pub enum BackendError {
    #[error("backend operation failed: {0}")]
    Failed(String),
    #[error("service call timed out")]
    Timeout,
    #[error("unsupported by this backend: {0}")]
    Unsupported(&'static str),
}

/// The transport-agnostic interface the protocol layer drives.
#[async_trait]
pub trait RosBackend: Send + Sync {
    /// Create (or reuse) a publisher for `topic` of `type_name` with `qos`.
    fn advertise(
        &self,
        topic: &str,
        type_name: &str,
        qos: &QosSpec,
    ) -> Result<PublisherId, BackendError>;

    /// Publish a message (JSON) through a previously created publisher.
    fn publish(&self, id: PublisherId, msg: &Value) -> Result<(), BackendError>;

    /// Destroy a publisher.
    fn unadvertise(&self, id: PublisherId);

    /// Subscribe to `topic`. Samples are delivered on the returned channel.
    fn subscribe(
        &self,
        topic: &str,
        type_name: &str,
        qos: &QosSpec,
    ) -> Result<(SubscriptionId, mpsc::UnboundedReceiver<Sample>), BackendError>;

    /// Destroy a subscription.
    fn unsubscribe(&self, id: SubscriptionId);

    /// Call a ROS service as a client and await the response (JSON).
    async fn call_service(
        &self,
        service: &str,
        type_name: &str,
        request: Value,
        timeout_secs: f64,
    ) -> Result<Value, BackendError>;

    /// Advertise a service to ROS; inbound requests are delivered on the
    /// returned channel for the client to answer.
    fn advertise_service(
        &self,
        service: &str,
        type_name: &str,
    ) -> Result<(ServiceServerId, mpsc::UnboundedReceiver<ServiceRequest>), BackendError>;

    /// Stop hosting a service.
    fn unadvertise_service(&self, id: ServiceServerId);

    /// Send an action goal as a client; returns feedback/result streams.
    async fn send_action_goal(
        &self,
        action: &str,
        type_name: &str,
        goal: Value,
    ) -> Result<GoalStream, BackendError>;

    /// Cancel a previously sent goal.
    fn cancel_action_goal(&self, action: &str, goal_id: &[u8; 16]);

    /// Advertise an action server to ROS; inbound goals are delivered on the
    /// returned channel.
    fn advertise_action(
        &self,
        action: &str,
        type_name: &str,
    ) -> Result<(ActionServerId, mpsc::UnboundedReceiver<ActionGoal>), BackendError>;

    /// Stop hosting an action server.
    fn unadvertise_action(&self, id: ActionServerId);

    /// Current ROS time as `(sec, nanosec)` for `"now"` substitution.
    fn now(&self) -> (i32, u32);

    /// Best-effort discovery of a topic's message type from the ROS graph,
    /// used to infer the type for `subscribe` when the client omits it.
    /// Backends without discovery return `None`.
    fn discover_type(&self, _topic: &str) -> Option<String> {
        None
    }
}

/// A shared backend handle.
pub type SharedBackend = Arc<dyn RosBackend>;
