//! Pure-Rust dynamic ROS2 message model.
//!
//! Parses `.msg`/`.srv`/`.action` interface definitions at runtime and provides
//! a schema-driven CDR <-> `serde_json::Value` codec, so a rosbridge server can
//! bridge arbitrary ROS2 types to JSON without a ROS2 installation and without
//! compile-time code generation.

pub mod cdr;
pub mod codec;
pub mod datatype;
pub mod defs;
pub mod parser;
pub mod registry;
pub mod spec;

pub use codec::{Codec, CodecError, RosTime};
pub use registry::{Registry, RegistryError};
pub use spec::{ActionSpec, Constant, Field, MessageSpec, ServiceSpec};

#[cfg(test)]
mod roundtrip_tests {
    use super::*;
    use serde_json::json;

    fn reg() -> Registry {
        Registry::with_standard_types()
    }

    #[test]
    fn roundtrip_string() {
        let r = reg();
        let spec = r.message("std_msgs/msg/String").unwrap();
        let codec = Codec::new(&r);
        let v = json!({"data": "hello world"});
        let bytes = codec.encode(spec, &v).unwrap();
        let back = codec.decode(spec, &bytes).unwrap();
        assert_eq!(back["data"], "hello world");
    }

    #[test]
    fn roundtrip_nested_pose() {
        let r = reg();
        let spec = r.message("geometry_msgs/msg/Pose").unwrap();
        let codec = Codec::new(&r);
        let v = json!({
            "position": {"x": 1.0, "y": 2.0, "z": 3.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        });
        let bytes = codec.encode(spec, &v).unwrap();
        let back = codec.decode(spec, &bytes).unwrap();
        assert_eq!(back["position"]["x"], 1.0);
        assert_eq!(back["orientation"]["w"], 1.0);
    }

    #[test]
    fn partial_message_uses_defaults() {
        let r = reg();
        let spec = r.message("geometry_msgs/msg/Vector3").unwrap();
        let codec = Codec::new(&r);
        let v = json!({"x": 5.0}); // y, z omitted
        let bytes = codec.encode(spec, &v).unwrap();
        let back = codec.decode(spec, &bytes).unwrap();
        assert_eq!(back["x"], 5.0);
        assert_eq!(back["y"], 0.0);
        assert_eq!(back["z"], 0.0);
    }

    #[test]
    fn byte_array_base64() {
        let r = reg();
        // sensor_msgs/CompressedImage has uint8[] data.
        let spec = r.message("sensor_msgs/msg/CompressedImage").unwrap();
        let codec = Codec::new(&r);
        let v = json!({
            "header": {"frame_id": "cam"},
            "format": "jpeg",
            "data": base64::Engine::encode(&base64::engine::general_purpose::STANDARD, [1u8,2,3,4])
        });
        let bytes = codec.encode(spec, &v).unwrap();
        let back = codec.decode(spec, &bytes).unwrap();
        // round-trips back to the same base64 string
        assert_eq!(back["data"], v["data"]);
        assert_eq!(back["format"], "jpeg");
    }

    #[test]
    fn header_stamp_now_autofill() {
        let r = reg();
        let spec = r.message("std_msgs/msg/Header").unwrap();
        let codec = Codec::with_now(&r, Some((123, 456)));
        let v = json!({"frame_id": "base"}); // stamp omitted
        let bytes = codec.encode(spec, &v).unwrap();
        let back = codec.decode(spec, &bytes).unwrap();
        assert_eq!(back["stamp"]["sec"], 123);
        assert_eq!(back["stamp"]["nanosec"], 456);
        assert_eq!(back["frame_id"], "base");
    }

    #[test]
    fn float_array_roundtrip() {
        let r = reg();
        let spec = r.message("std_msgs/msg/Float64MultiArray").unwrap();
        let codec = Codec::new(&r);
        let v = json!({"layout": {"dim": [], "data_offset": 0}, "data": [1.5, 2.5, 3.5]});
        let bytes = codec.encode(spec, &v).unwrap();
        let back = codec.decode(spec, &bytes).unwrap();
        assert_eq!(back["data"], json!([1.5, 2.5, 3.5]));
    }
}
