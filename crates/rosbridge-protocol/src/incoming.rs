//! Client → Server operations, as defined by the rosbridge v2.1 protocol.

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::qos::QosProfile;

/// Any operation a client may send to the server.
///
/// Decoded with serde's internal tagging on the `op` field. Unknown ops are
/// captured by [`IncomingMessage::Unknown`] so the server can emit a
/// protocol-level error instead of dropping the connection.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum IncomingMessage {
    Advertise(Advertise),
    Unadvertise(Unadvertise),
    Publish(Publish),
    Subscribe(Subscribe),
    Unsubscribe(Unsubscribe),
    CallService(CallService),
    AdvertiseService(AdvertiseService),
    UnadvertiseService(UnadvertiseService),
    ServiceResponse(ServiceResponse),
    AdvertiseAction(AdvertiseAction),
    UnadvertiseAction(UnadvertiseAction),
    SendActionGoal(SendActionGoal),
    CancelActionGoal(CancelActionGoal),
    ActionFeedback(ActionFeedback),
    ActionResult(ActionResult),
    Fragment(Fragment),
    SetLevel(SetLevel),
    Status(StatusIn),
    Auth(Auth),
    #[serde(other)]
    Unknown,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Advertise {
    pub topic: String,
    #[serde(rename = "type")]
    pub msg_type: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub qos: Option<QosProfile>,
    /// Deprecated alias for `qos.durability == transient_local`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub latch: Option<bool>,
    /// Deprecated alias for `qos.depth`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub queue_size: Option<usize>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Unadvertise {
    pub topic: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Publish {
    pub topic: String,
    pub msg: Value,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    #[serde(rename = "type", default, skip_serializing_if = "Option::is_none")]
    pub msg_type: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub qos: Option<QosProfile>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub latch: Option<bool>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Subscribe {
    pub topic: String,
    #[serde(rename = "type", default, skip_serializing_if = "Option::is_none")]
    pub msg_type: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    /// Minimum milliseconds between forwarded messages (default 0 = no throttle).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub throttle_rate: Option<u64>,
    /// Buffer length used together with `throttle_rate` (default 0).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub queue_length: Option<usize>,
    /// Maximum bytes before fragmenting outgoing messages.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fragment_size: Option<usize>,
    /// One of `none`, `png`, `cbor`, `cbor-raw`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub compression: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub qos: Option<QosProfile>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Unsubscribe {
    pub topic: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CallService {
    pub service: String,
    #[serde(rename = "type", default, skip_serializing_if = "Option::is_none")]
    pub srv_type: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub args: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fragment_size: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub compression: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub timeout: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvertiseService {
    pub service: String,
    #[serde(rename = "type")]
    pub srv_type: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UnadvertiseService {
    pub service: String,
}

/// Client's response to a server-forwarded `call_service` for a client-hosted
/// service (i.e. the client is acting as service server).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServiceResponse {
    pub service: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    pub result: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub values: Option<Value>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvertiseAction {
    pub action: String,
    #[serde(rename = "type")]
    pub action_type: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UnadvertiseAction {
    pub action: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SendActionGoal {
    pub action: String,
    pub action_type: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub args: Option<Value>,
    #[serde(default)]
    pub feedback: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fragment_size: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub compression: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CancelActionGoal {
    pub action: String,
    pub id: String,
}

/// Feedback emitted by a client-hosted action server.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionFeedback {
    pub action: String,
    pub id: String,
    pub values: Value,
}

/// Result emitted by a client-hosted action server.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionResult {
    pub action: String,
    pub id: String,
    pub result: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub values: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub status: Option<i32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Fragment {
    pub id: String,
    pub data: String,
    pub num: usize,
    pub total: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SetLevel {
    pub level: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
}

/// Clients may echo `status` messages; the server accepts and ignores them.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StatusIn {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub level: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub msg: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Auth {
    pub mac: String,
    pub client: String,
    pub dest: String,
    pub rand: String,
    pub t: serde_json::Number,
    pub level: String,
    pub end: serde_json::Number,
}
