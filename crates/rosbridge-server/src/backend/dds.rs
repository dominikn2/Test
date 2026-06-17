//! Real ROS2/DDS backend built on the pure-Rust `ros2-client` + `rustdds`
//! stack (no ROS2 installation required).
//!
//! Dynamic types are moved as opaque CDR (the foxglove-bridge design); the
//! bridge never re-serializes message bodies.
//!
//! * **Publish** uses ros2-client's `Publisher` with a pass-through serde
//!   newtype that emits the CDR *body* verbatim (RustDDS prepends the 4-byte
//!   encapsulation header). The bytes on the wire are byte-identical to what a
//!   native ROS2 publisher emits.
//! * **Subscribe** uses a raw RustDDS `DataReader` with a custom
//!   passthrough [`DeserializerAdapter`] that hands back the full CDR buffer
//!   untouched. (ros2-client's built-in CDR reader would misinterpret a raw
//!   body as a length-prefixed sequence, so we bypass it on the read path.)
//!
//! Services and actions over DDS are not yet implemented here (dynamic
//! raw-service request-id carriage differs between Fast-DDS and Cyclone and
//! needs validation against a live ROS2 peer); they return
//! [`BackendError::Unsupported`]. The loopback backend implements them in full.

use std::collections::HashMap;
use std::fmt;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use async_trait::async_trait;
use futures::StreamExt;
use parking_lot::Mutex;
use rustdds::no_key::{Decode, DefaultDecoder, DeserializerAdapter};
use rustdds::{policy, QosPolicies, QosPolicyBuilder, RepresentationIdentifier, Subscriber, TopicKind};
use serde::ser::{SerializeTuple, Serializer};
use serde::Serialize;
use tokio::sync::mpsc;

use ros2_client::{Context, MessageTypeName, Name, Node, NodeName, NodeOptions};

use super::*;

/// CDR body (no encapsulation header) emitted verbatim on the wire.
struct RawCdr(Vec<u8>);

impl Serialize for RawCdr {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        // A fixed-size tuple of bytes: CDR writes each u8 1:1 with no framing.
        let mut t = s.serialize_tuple(self.0.len())?;
        for b in &self.0 {
            t.serialize_element(b)?;
        }
        t.end()
    }
}

/// Full CDR buffer (with reconstructed 4-byte header) received from the wire.
struct RawBytes(Vec<u8>);

#[derive(Debug)]
struct RawError;
impl fmt::Display for RawError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("raw cdr decode error")
    }
}
impl std::error::Error for RawError {}

const SUPPORTED_ENCODINGS: [RepresentationIdentifier; 4] = [
    RepresentationIdentifier::CDR_BE,
    RepresentationIdentifier::CDR_LE,
    RepresentationIdentifier::PL_CDR_BE,
    RepresentationIdentifier::PL_CDR_LE,
];

/// Decoder that reconstructs the full encapsulated CDR buffer from the body
/// bytes plus the negotiated representation identifier.
#[derive(Clone)]
struct RawDecoder;

impl Decode<Vec<u8>> for RawDecoder {
    type Error = RawError;
    fn decode_bytes(
        self,
        input_bytes: &[u8],
        encoding: RepresentationIdentifier,
    ) -> Result<Vec<u8>, RawError> {
        let mut buf = Vec::with_capacity(input_bytes.len() + 4);
        buf.extend_from_slice(&encoding.to_bytes()); // 2-byte representation id
        buf.extend_from_slice(&[0x00, 0x00]); // 2-byte options
        buf.extend_from_slice(input_bytes);
        Ok(buf)
    }
}

/// Passthrough deserializer adapter producing a full CDR buffer.
struct RawAdapter;

impl DeserializerAdapter<RawBytes> for RawAdapter {
    type Error = RawError;
    type Decoded = Vec<u8>;
    fn supported_encodings() -> &'static [RepresentationIdentifier] {
        &SUPPORTED_ENCODINGS
    }
    fn transform_decoded(decoded: Vec<u8>) -> RawBytes {
        RawBytes(decoded)
    }
}

impl DefaultDecoder<RawBytes> for RawAdapter {
    type Decoder = RawDecoder;
    const DECODER: RawDecoder = RawDecoder;
}

/// A ROS2/DDS [`RosBackend`].
pub struct DdsBackend {
    context: Context,
    node: Mutex<Node>,
    node_name: NodeName,
    subscriber: Subscriber,
    publishers: Mutex<HashMap<PublisherId, ros2_client::Publisher<RawCdr>>>,
    sub_tasks: Mutex<HashMap<SubscriptionId, tokio::task::JoinHandle<()>>>,
    counter: AtomicU64,
}

impl DdsBackend {
    /// Create a DDS backend on the default domain and start its spinner.
    pub fn new() -> Result<Self, BackendError> {
        Self::with_node_name("rosbridge_rs")
    }

    pub fn with_node_name(name: &str) -> Result<Self, BackendError> {
        let context =
            Context::new().map_err(|e| BackendError::Failed(format!("dds context: {e}")))?;
        let node_name =
            NodeName::new("/", name).map_err(|e| BackendError::Failed(format!("node name: {e}")))?;
        let mut node = context
            .new_node(node_name.clone(), NodeOptions::new().enable_rosout(false))
            .map_err(|e| BackendError::Failed(format!("create node: {e}")))?;

        let subscriber = context
            .domain_participant()
            .create_subscriber(&QosPolicyBuilder::new().build())
            .map_err(|e| BackendError::Failed(format!("create_subscriber: {e}")))?;

        // Drive ROS discovery in the background.
        if let Ok(spinner) = node.spinner() {
            tokio::spawn(async move {
                if let Err(e) = spinner.spin().await {
                    tracing::warn!("dds spinner stopped: {e}");
                }
            });
        }

        Ok(DdsBackend {
            context,
            node: Mutex::new(node),
            node_name,
            subscriber,
            publishers: Mutex::new(HashMap::new()),
            sub_tasks: Mutex::new(HashMap::new()),
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
impl RosBackend for DdsBackend {
    fn advertise(
        &self,
        topic: &str,
        type_name: &str,
        qos: &QosSpec,
    ) -> Result<PublisherId, BackendError> {
        let (name, ty) = ros_names(topic, type_name)?;
        let policies = to_qos(qos);
        let mut node = self.node.lock();
        let dds_topic = node
            .create_topic(&name, ty, &policies)
            .map_err(|e| BackendError::Failed(format!("create_topic: {e}")))?;
        let publisher = node
            .create_publisher::<RawCdr>(&dds_topic, Some(policies))
            .map_err(|e| BackendError::Failed(format!("create_publisher: {e}")))?;
        drop(node);
        let id = PublisherId(self.next_id());
        self.publishers.lock().insert(id, publisher);
        Ok(id)
    }

    fn publish(&self, id: PublisherId, cdr: &[u8]) -> Result<(), BackendError> {
        if cdr.len() < 4 {
            return Err(BackendError::Failed("cdr too short".into()));
        }
        let body = cdr[4..].to_vec();
        let guard = self.publishers.lock();
        let publisher = guard
            .get(&id)
            .ok_or_else(|| BackendError::Failed("unknown publisher".into()))?;
        publisher
            .publish(RawCdr(body))
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
        let (name, ty) = ros_names(topic, type_name)?;
        let policies = to_qos(qos);
        let dds_topic_name = name.to_dds_name("rt", &self.node_name, "");
        let dds_type_name = ty.dds_msg_type();
        let dp = self.context.domain_participant();
        let dds_topic = dp
            .create_topic(dds_topic_name, dds_type_name, &policies, TopicKind::NoKey)
            .map_err(|e| BackendError::Failed(format!("create_topic: {e}")))?;
        let reader = self
            .subscriber
            .create_datareader_no_key::<RawBytes, RawAdapter>(&dds_topic, Some(policies))
            .map_err(|e| BackendError::Failed(format!("create_datareader: {e}")))?;

        let (tx, rx) = mpsc::unbounded_channel();
        let id = SubscriptionId(self.next_id());
        let task = tokio::spawn(async move {
            let mut stream = reader.async_sample_stream();
            while let Some(result) = stream.next().await {
                match result {
                    Ok(sample) => {
                        let RawBytes(cdr) = sample.into_value();
                        if tx.send(Sample { cdr }).is_err() {
                            break;
                        }
                    }
                    Err(e) => tracing::debug!("dds read error: {e}"),
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
        _service: &str,
        _type_name: &str,
        _request_cdr: Cdr,
        _timeout_secs: f64,
    ) -> Result<Cdr, BackendError> {
        Err(BackendError::Unsupported("call_service over DDS"))
    }

    fn advertise_service(
        &self,
        _service: &str,
        _type_name: &str,
    ) -> Result<(ServiceServerId, mpsc::UnboundedReceiver<ServiceRequest>), BackendError> {
        Err(BackendError::Unsupported("advertise_service over DDS"))
    }

    fn unadvertise_service(&self, _id: ServiceServerId) {}

    async fn send_action_goal(
        &self,
        _action: &str,
        _type_name: &str,
        _goal_cdr: Cdr,
    ) -> Result<GoalStream, BackendError> {
        Err(BackendError::Unsupported("send_action_goal over DDS"))
    }

    fn cancel_action_goal(&self, _action: &str, _goal_id: &[u8; 16]) {}

    fn advertise_action(
        &self,
        _action: &str,
        _type_name: &str,
    ) -> Result<(ActionServerId, mpsc::UnboundedReceiver<ActionGoal>), BackendError> {
        Err(BackendError::Unsupported("advertise_action over DDS"))
    }

    fn unadvertise_action(&self, _id: ActionServerId) {}

    fn now(&self) -> (i32, u32) {
        let d = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
        (d.as_secs() as i32, d.subsec_nanos())
    }

    fn discover_type(&self, topic: &str) -> Option<String> {
        for t in self.context.discovered_topics() {
            if unmangle_topic(t.topic_name()).as_deref() == Some(topic) {
                if let Some(ty) = unmangle_type(t.type_name()) {
                    return Some(ty);
                }
            }
        }
        None
    }
}

/// `rt/foo/bar` -> `/foo/bar` (ROS2 topic DDS-name prefix is `rt`).
fn unmangle_topic(dds_name: &str) -> Option<String> {
    dds_name.strip_prefix("rt").map(|s| s.to_string())
}

/// `std_msgs::msg::dds_::String_` -> `std_msgs/msg/String`.
fn unmangle_type(dds_type: &str) -> Option<String> {
    let parts: Vec<&str> = dds_type.split("::").filter(|p| *p != "dds_").collect();
    if parts.len() < 2 {
        return None;
    }
    let mut out = parts.join("/");
    if out.ends_with('_') {
        out.pop();
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unmangle_roundtrips_ros_names() {
        assert_eq!(unmangle_topic("rt/chatter").as_deref(), Some("/chatter"));
        assert_eq!(unmangle_topic("rt/ns/topic").as_deref(), Some("/ns/topic"));
        assert_eq!(unmangle_topic("not_a_topic"), None);
        assert_eq!(
            unmangle_type("std_msgs::msg::dds_::String_").as_deref(),
            Some("std_msgs/msg/String")
        );
        assert_eq!(
            unmangle_type("geometry_msgs::msg::dds_::Twist_").as_deref(),
            Some("geometry_msgs/msg/Twist")
        );
    }

    #[test]
    fn ros_names_split_correctly() {
        let (name, ty) = ros_names("/foo/bar", "std_msgs/msg/String").unwrap();
        assert_eq!(name.to_dds_name("rt", &NodeName::new("/", "n").unwrap(), ""), "rt/foo/bar");
        assert_eq!(ty.dds_msg_type(), "std_msgs::msg::dds_::String_");
    }
}

/// Convert a rosbridge topic + `pkg/msg/Type` name into ros2-client names.
fn ros_names(topic: &str, type_name: &str) -> Result<(Name, MessageTypeName), BackendError> {
    let (ns, base) = match topic.rfind('/') {
        Some(0) => ("/", &topic[1..]),
        Some(i) => (&topic[..i], &topic[i + 1..]),
        None => ("/", topic),
    };
    let name =
        Name::new(ns, base).map_err(|e| BackendError::Failed(format!("bad topic '{topic}': {e}")))?;

    let parts: Vec<&str> = type_name.split('/').collect();
    let (pkg, ty) = match parts.as_slice() {
        [pkg, _kind, ty] => (*pkg, *ty),
        [pkg, ty] => (*pkg, *ty),
        _ => return Err(BackendError::Failed(format!("bad type '{type_name}'"))),
    };
    Ok((name, MessageTypeName::new(pkg, ty)))
}

/// Map normalized [`QosSpec`] to a rustdds [`QosPolicies`].
fn to_qos(qos: &QosSpec) -> QosPolicies {
    let mut b = QosPolicyBuilder::new();
    b = b.history(if qos.history_keep_all {
        policy::History::KeepAll
    } else {
        policy::History::KeepLast {
            depth: qos.depth as i32,
        }
    });
    b = b.reliability(match qos.reliability {
        Reliability::BestEffort => policy::Reliability::BestEffort,
        _ => policy::Reliability::Reliable {
            max_blocking_time: rustdds::Duration::from_millis(100),
        },
    });
    b = b.durability(match qos.durability {
        Durability::TransientLocal => policy::Durability::TransientLocal,
        _ => policy::Durability::Volatile,
    });
    b.build()
}
