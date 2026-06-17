//! Pure data-model and (de)serialization for the rosbridge v2.1 protocol.
//!
//! This crate is transport- and ROS-agnostic: it only knows how to parse the
//! JSON operations a client sends and to build the operations a server sends
//! back. It is the foundation the server crate builds upon.

pub mod incoming;
pub mod outgoing;
pub mod qos;

pub use incoming::IncomingMessage;
pub use outgoing::{Status, StatusLevel};
pub use qos::QosProfile;

/// Errors arising while decoding an incoming protocol message.
#[derive(Debug, thiserror::Error)]
pub enum ProtocolError {
    #[error("invalid JSON: {0}")]
    Json(#[from] serde_json::Error),
    #[error("message is missing required 'op' field")]
    MissingOp,
    #[error("unsupported operation: {0}")]
    UnsupportedOp(String),
}

/// Parse a single rosbridge message from a JSON string.
pub fn parse(input: &str) -> Result<IncomingMessage, ProtocolError> {
    Ok(serde_json::from_str(input)?)
}

/// Parse a single rosbridge message from an already-decoded JSON value.
pub fn parse_value(value: serde_json::Value) -> Result<IncomingMessage, ProtocolError> {
    Ok(serde_json::from_value(value)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use incoming::IncomingMessage;

    #[test]
    fn parse_subscribe_minimal() {
        let m = parse(r#"{"op":"subscribe","topic":"/chatter"}"#).unwrap();
        match m {
            IncomingMessage::Subscribe(s) => {
                assert_eq!(s.topic, "/chatter");
                assert!(s.msg_type.is_none());
                assert!(s.throttle_rate.is_none());
            }
            other => panic!("expected subscribe, got {other:?}"),
        }
    }

    #[test]
    fn parse_advertise_with_qos() {
        let m = parse(
            r#"{"op":"advertise","topic":"/t","type":"std_msgs/msg/String",
                "qos":{"reliability":"best_effort","depth":5}}"#,
        )
        .unwrap();
        match m {
            IncomingMessage::Advertise(a) => {
                assert_eq!(a.msg_type, "std_msgs/msg/String");
                let qos = a.qos.unwrap();
                assert_eq!(qos.reliability.as_deref(), Some("best_effort"));
                assert_eq!(qos.depth, Some(5));
            }
            other => panic!("expected advertise, got {other:?}"),
        }
    }

    #[test]
    fn parse_publish_payload_preserved() {
        let m = parse(r#"{"op":"publish","topic":"/t","msg":{"data":"hi"}}"#).unwrap();
        match m {
            IncomingMessage::Publish(p) => {
                assert_eq!(p.msg["data"], "hi");
            }
            other => panic!("expected publish, got {other:?}"),
        }
    }

    #[test]
    fn unknown_op_maps_to_unknown() {
        let m = parse(r#"{"op":"does_not_exist"}"#).unwrap();
        assert!(matches!(m, IncomingMessage::Unknown));
    }

    #[test]
    fn status_level_ordering() {
        assert!(StatusLevel::Info.rank() > StatusLevel::Error.rank());
        assert_eq!(StatusLevel::parse("warn"), Some(StatusLevel::Warning));
    }
}
