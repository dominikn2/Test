//! The ROS backend abstraction.
//!
//! The protocol layer converts between rosbridge JSON and CDR bytes using the
//! [`ros_message`] codec, then hands raw CDR to a [`RosBackend`]. This mirrors
//! the foxglove-bridge "move raw bytes" design: the bridge does minimal CDR
//! work and the backend only transports opaque buffers over its middleware.
//!
//! Two implementations exist:
//! * [`loopback::LoopbackBackend`] — an in-process bus, used for tests and for
//!   browser↔browser bridging without any ROS2 middleware.
//! * `dds::DdsBackend` — a real ROS2/DDS backend built on `ros2-client`
//!   (compiled in when the `dds` feature is enabled).

#[cfg(feature = "dds")]
pub mod dds;
pub mod loopback;

use std::sync::Arc;

use async_trait::async_trait;
use tokio::sync::{mpsc, oneshot};

/// Raw CDR-encoded payload (including the 4-byte encapsulation header).
pub type Cdr = Vec<u8>;

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

/// A received topic sample.
#[derive(Debug, Clone)]
pub struct Sample {
    pub cdr: Cdr,
}

/// An inbound service request routed to a client-hosted service server.
pub struct ServiceRequest {
    pub request_cdr: Cdr,
    /// Channel to deliver the response CDR (or `None` on failure/abort).
    pub responder: oneshot::Sender<Option<Cdr>>,
}

/// An inbound action goal routed to a client-hosted action server. The hosting
/// client emits feedback and a final result through the provided senders.
pub struct ActionGoal {
    pub goal_cdr: Cdr,
    pub goal_id: [u8; 16],
    /// The hosting client publishes feedback CDR here.
    pub feedback_tx: mpsc::UnboundedSender<Cdr>,
    /// The hosting client delivers the final `(result_cdr, status)` here;
    /// `None` result indicates failure/abort.
    pub result_tx: oneshot::Sender<(Option<Cdr>, i8)>,
    /// Fires when the goal-issuing side requests cancellation.
    pub cancel_rx: oneshot::Receiver<()>,
}

/// Outcome of issuing an action goal as a client.
pub struct GoalStream {
    pub feedback: mpsc::UnboundedReceiver<Cdr>,
    pub result: oneshot::Receiver<Result<(Cdr, i8), String>>,
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

    /// Publish a CDR payload through a previously created publisher.
    fn publish(&self, id: PublisherId, cdr: &[u8]) -> Result<(), BackendError>;

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

    /// Call a ROS service as a client and await the response CDR.
    async fn call_service(
        &self,
        service: &str,
        type_name: &str,
        request_cdr: Cdr,
        timeout_secs: f64,
    ) -> Result<Cdr, BackendError>;

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
        goal_cdr: Cdr,
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
}

/// A shared backend handle.
pub type SharedBackend = Arc<dyn RosBackend>;
