//! Server → Client operations.

use serde::{Deserialize, Serialize};
use serde_json::Value;

/// Status level used in `status` messages and accepted by `set_level`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum StatusLevel {
    None,
    Error,
    Warning,
    Info,
}

impl StatusLevel {
    /// Parse a level string as used by `set_level`. Unknown values are
    /// treated as the most permissive (`None`) to match rosbridge behavior of
    /// only filtering when an explicit threshold is set.
    pub fn parse(s: &str) -> Option<StatusLevel> {
        match s.to_ascii_lowercase().as_str() {
            "none" => Some(StatusLevel::None),
            "error" => Some(StatusLevel::Error),
            "warning" | "warn" => Some(StatusLevel::Warning),
            "info" => Some(StatusLevel::Info),
            _ => None,
        }
    }

    /// Severity rank: higher means more verbose. A configured threshold emits
    /// messages whose rank is `<=` the threshold rank.
    pub fn rank(self) -> u8 {
        match self {
            StatusLevel::None => 0,
            StatusLevel::Error => 1,
            StatusLevel::Warning => 2,
            StatusLevel::Info => 3,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            StatusLevel::None => "none",
            StatusLevel::Error => "error",
            StatusLevel::Warning => "warning",
            StatusLevel::Info => "info",
        }
    }
}

/// A `status` message reporting protocol-level info/warnings/errors.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Status {
    pub op: &'static str,
    pub level: StatusLevel,
    pub msg: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
}

impl Status {
    pub fn new(level: StatusLevel, msg: impl Into<String>, id: Option<String>) -> Status {
        Status {
            op: "status",
            level,
            msg: msg.into(),
            id,
        }
    }

    pub fn error(msg: impl Into<String>, id: Option<String>) -> Status {
        Status::new(StatusLevel::Error, msg, id)
    }

    pub fn warning(msg: impl Into<String>, id: Option<String>) -> Status {
        Status::new(StatusLevel::Warning, msg, id)
    }

    pub fn info(msg: impl Into<String>, id: Option<String>) -> Status {
        Status::new(StatusLevel::Info, msg, id)
    }
}

/// A `publish` message forwarded to a subscribed client.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PublishOut {
    pub op: &'static str,
    pub topic: String,
    pub msg: Value,
}

impl PublishOut {
    pub fn new(topic: impl Into<String>, msg: Value) -> PublishOut {
        PublishOut {
            op: "publish",
            topic: topic.into(),
            msg,
        }
    }
}

/// A server→client `call_service` forwarded to a client acting as service
/// server.
#[derive(Debug, Clone, Serialize)]
pub struct CallServiceOut {
    pub op: &'static str,
    pub service: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    pub args: Value,
}

/// A `service_response` returned to the client that issued `call_service`.
#[derive(Debug, Clone, Serialize)]
pub struct ServiceResponseOut {
    pub op: &'static str,
    pub service: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    pub values: Value,
    pub result: bool,
}

/// A server→client `send_action_goal` forwarded to a client action server.
#[derive(Debug, Clone, Serialize)]
pub struct SendActionGoalOut {
    pub op: &'static str,
    pub action: String,
    pub action_type: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    pub args: Value,
}

/// An `action_feedback` forwarded to the goal-issuing client.
#[derive(Debug, Clone, Serialize)]
pub struct ActionFeedbackOut {
    pub op: &'static str,
    pub action: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    pub values: Value,
}

/// An `action_result` forwarded to the goal-issuing client.
#[derive(Debug, Clone, Serialize)]
pub struct ActionResultOut {
    pub op: &'static str,
    pub action: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    pub values: Value,
    pub result: bool,
    pub status: i32,
}

/// A `fragment` chunk of a larger message.
#[derive(Debug, Clone, Serialize)]
pub struct FragmentOut {
    pub op: &'static str,
    pub id: String,
    pub data: String,
    pub num: usize,
    pub total: usize,
}
