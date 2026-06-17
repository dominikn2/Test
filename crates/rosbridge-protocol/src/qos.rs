//! QoS profile representation matching the rosbridge protocol `qos` object.
//!
//! Mirrors the fields accepted by rosbridge_server's `qos_extraction`:
//! `history`, `depth`, `reliability`, `durability`, `deadline`, `lifespan`,
//! `liveliness`, `liveliness_lease_duration`.

use serde::{Deserialize, Serialize};

/// A duration value as it may appear in a QoS field. The protocol accepts a
/// float number of seconds, an object `{secs/sec, nsecs/nanosec}`, or the
/// strings `"infinite"` / `"best_available"`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum QosDuration {
    /// Seconds as a float.
    Seconds(f64),
    /// Structured `{secs, nsecs}` form (ROS1 or ROS2 field names).
    Parts(DurationParts),
    /// `"infinite"` or `"best_available"`.
    Keyword(String),
}

/// Structured duration accepting both ROS1 (`secs`/`nsecs`) and
/// ROS2 (`sec`/`nanosec`) field spellings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct DurationParts {
    #[serde(default, alias = "secs")]
    pub sec: i64,
    #[serde(default, alias = "nsecs")]
    pub nanosec: u64,
}

impl QosDuration {
    /// Convert to `(seconds, nanoseconds)`. `infinite`/`best_available`
    /// resolve to the conventional "infinite" sentinel used by DDS.
    pub fn to_parts(&self) -> (i64, u64) {
        match self {
            QosDuration::Seconds(s) => {
                let sec = s.trunc() as i64;
                let nsec = ((s.fract()) * 1e9).round().abs() as u64;
                (sec, nsec)
            }
            QosDuration::Parts(p) => (p.sec, p.nanosec),
            QosDuration::Keyword(_) => (i64::MAX, 0),
        }
    }

    pub fn is_infinite(&self) -> bool {
        matches!(self, QosDuration::Keyword(k) if k == "infinite")
    }
}

/// The full QoS profile object accepted on advertise/subscribe/publish.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct QosProfile {
    /// `"keep_last"` or `"keep_all"`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub history: Option<String>,
    /// Queue depth for `keep_last`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub depth: Option<usize>,
    /// `"reliable"`, `"best_effort"`, `"best_available"`, or `"system_default"`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reliability: Option<String>,
    /// `"transient_local"`, `"volatile"`, `"best_available"`, or `"system_default"`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub durability: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deadline: Option<QosDuration>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lifespan: Option<QosDuration>,
    /// `"automatic"`, `"manual_by_topic"`, `"system_default"`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub liveliness: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub liveliness_lease_duration: Option<QosDuration>,
}
