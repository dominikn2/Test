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
    use serde_json::{json, Value};

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

    /// Golden test: exact CDR bytes for `std_msgs/String{data:"hello"}` as a
    /// real ROS2 (CycloneDDS/FastDDS) endpoint would emit them.
    #[test]
    fn golden_string_cdr() {
        let r = reg();
        let spec = r.message("std_msgs/msg/String").unwrap();
        let bytes = Codec::new(&r).encode(spec, &json!({"data":"hello"})).unwrap();
        let expected = [
            0x00, 0x01, 0x00, 0x00, // CDR_LE encapsulation
            0x06, 0x00, 0x00, 0x00, // string length = 6 (incl NUL)
            b'h', b'e', b'l', b'l', b'o', 0x00,
        ];
        assert_eq!(bytes, expected);
    }

    /// Golden test: `geometry_msgs/Point{1.0, 2.0, 3.0}` — three 8-aligned f64.
    #[test]
    fn golden_point_cdr() {
        let r = reg();
        let spec = r.message("geometry_msgs/msg/Point").unwrap();
        let bytes = Codec::new(&r)
            .encode(spec, &json!({"x":1.0,"y":2.0,"z":3.0}))
            .unwrap();
        let mut expected = vec![0x00, 0x01, 0x00, 0x00];
        expected.extend_from_slice(&1.0f64.to_le_bytes());
        expected.extend_from_slice(&2.0f64.to_le_bytes());
        expected.extend_from_slice(&3.0f64.to_le_bytes());
        assert_eq!(bytes, expected);
    }

    /// Golden test exercising alignment: `std_msgs/Header` has a Time
    /// (int32+uint32 = 8 bytes) then a 4-aligned string.
    #[test]
    fn golden_header_alignment() {
        let r = reg();
        let spec = r.message("std_msgs/msg/Header").unwrap();
        let bytes = Codec::with_now(&r, Some((0, 0)))
            .encode(spec, &json!({"stamp":{"sec":1,"nanosec":2},"frame_id":"m"}))
            .unwrap();
        let expected = [
            0x00, 0x01, 0x00, 0x00, // encapsulation
            0x01, 0x00, 0x00, 0x00, // sec = 1 (int32)
            0x02, 0x00, 0x00, 0x00, // nanosec = 2 (uint32)
            0x02, 0x00, 0x00, 0x00, // frame_id length = 2
            b'm', 0x00, // "m" + NUL
        ];
        assert_eq!(bytes, expected);
    }

    /// Decoding the golden Point bytes must reproduce the values.
    #[test]
    fn golden_point_decode() {
        let r = reg();
        let spec = r.message("geometry_msgs/msg/Point").unwrap();
        let mut bytes = vec![0x00, 0x01, 0x00, 0x00];
        bytes.extend_from_slice(&4.5f64.to_le_bytes());
        bytes.extend_from_slice(&(-6.0f64).to_le_bytes());
        bytes.extend_from_slice(&0.0f64.to_le_bytes());
        let v = Codec::new(&r).decode(spec, &bytes).unwrap();
        assert_eq!(v["x"], 4.5);
        assert_eq!(v["y"], -6.0);
        assert_eq!(v["z"], 0.0);
    }

    /// Big-endian CDR must decode identically.
    #[test]
    fn decode_big_endian() {
        let r = reg();
        let spec = r.message("std_msgs/msg/Int32").unwrap();
        // CDR_BE header (scheme byte 0x00), then int32 = 258 big-endian.
        let bytes = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02];
        let v = Codec::new(&r).decode(spec, &bytes).unwrap();
        assert_eq!(v["data"], 258);
    }

    #[test]
    fn array_of_messages_roundtrip() {
        let r = reg();
        let spec = r.message("nav_msgs/msg/Path").unwrap();
        let codec = Codec::with_now(&r, Some((0, 0)));
        let v = json!({
            "header": {"frame_id": "map"},
            "poses": [
                {"header":{"frame_id":"a"}, "pose":{"position":{"x":1.0,"y":0.0,"z":0.0},
                    "orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}},
                {"header":{"frame_id":"b"}, "pose":{"position":{"x":2.0,"y":0.0,"z":0.0},
                    "orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}}
            ]
        });
        let bytes = codec.encode(spec, &v).unwrap();
        let back = codec.decode(spec, &bytes).unwrap();
        assert_eq!(back["poses"].as_array().unwrap().len(), 2);
        assert_eq!(back["poses"][1]["pose"]["position"]["x"], 2.0);
        assert_eq!(back["poses"][0]["header"]["frame_id"], "a");
    }

    #[test]
    fn fixed_array_covariance() {
        let r = reg();
        let spec = r.message("geometry_msgs/msg/PoseWithCovariance").unwrap();
        let codec = Codec::new(&r);
        let cov: Vec<f64> = (0..36).map(|i| i as f64).collect();
        let v = json!({
            "pose": {"position":{"x":0.0,"y":0.0,"z":0.0},
                     "orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}},
            "covariance": cov
        });
        let bytes = codec.encode(spec, &v).unwrap();
        let back = codec.decode(spec, &bytes).unwrap();
        assert_eq!(back["covariance"].as_array().unwrap().len(), 36);
        assert_eq!(back["covariance"][35], 35.0);
    }

    #[test]
    fn nonfinite_float_becomes_null() {
        let r = reg();
        let spec = r.message("std_msgs/msg/Float64").unwrap();
        let codec = Codec::new(&r);
        // Encode NaN via null, decode back to null.
        let bytes = codec.encode(spec, &json!({"data": null})).unwrap();
        let back = codec.decode(spec, &bytes).unwrap();
        assert_eq!(back["data"], Value::Null);
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

    #[test]
    fn integer_extremes_roundtrip() {
        let r = reg();
        let codec = Codec::new(&r);
        for (ty, val) in [
            ("std_msgs/msg/Int64", json!(i64::MIN)),
            ("std_msgs/msg/UInt64", json!(u64::MAX)),
            ("std_msgs/msg/Int8", json!(-128)),
            ("std_msgs/msg/UInt8", json!(255)),
            ("std_msgs/msg/UInt32", json!(u32::MAX)),
        ] {
            let spec = r.message(ty).unwrap();
            let bytes = codec.encode(spec, &json!({"data": val})).unwrap();
            let back = codec.decode(spec, &bytes).unwrap();
            assert_eq!(back["data"], val, "type {ty}");
        }
    }

    #[test]
    fn out_of_range_int_is_rejected() {
        let r = reg();
        let spec = r.message("std_msgs/msg/UInt8").unwrap();
        let err = Codec::new(&r).encode(spec, &json!({"data": 256}));
        assert!(err.is_err());
    }

    #[test]
    fn string_sequence_roundtrip() {
        let r = reg();
        let spec = r.message("sensor_msgs/msg/JointState").unwrap();
        let codec = Codec::with_now(&r, Some((0, 0)));
        let v = json!({
            "header": {"frame_id": ""},
            "name": ["j1", "j2", "j3"],
            "position": [0.1, 0.2, 0.3],
            "velocity": [],
            "effort": []
        });
        let bytes = codec.encode(spec, &v).unwrap();
        let back = codec.decode(spec, &bytes).unwrap();
        assert_eq!(back["name"], json!(["j1", "j2", "j3"]));
        assert_eq!(back["position"][2], 0.3);
        assert_eq!(back["velocity"], json!([]));
    }

    #[test]
    fn empty_message_roundtrip() {
        let r = reg();
        let spec = r.message("std_msgs/msg/Empty").unwrap();
        let codec = Codec::new(&r);
        let bytes = codec.encode(spec, &json!({})).unwrap();
        let back = codec.decode(spec, &bytes).unwrap();
        assert_eq!(back, json!({}));
    }

    #[test]
    fn service_request_response_roundtrip() {
        let r = reg();
        let svc = r.service("example_interfaces/srv/AddTwoInts").unwrap();
        let codec = Codec::new(&r);
        let req = codec.encode(&svc.request, &json!({"a": 40, "b": 2})).unwrap();
        assert_eq!(codec.decode(&svc.request, &req).unwrap()["b"], 2);
        let resp = codec.encode(&svc.response, &json!({"sum": 42})).unwrap();
        assert_eq!(codec.decode(&svc.response, &resp).unwrap()["sum"], 42);
    }

    #[test]
    fn bool_field_accepts_int() {
        let r = reg();
        let spec = r.message("std_msgs/msg/Bool").unwrap();
        let codec = Codec::new(&r);
        let bytes = codec.encode(spec, &json!({"data": 1})).unwrap();
        assert_eq!(codec.decode(spec, &bytes).unwrap()["data"], true);
    }

    #[test]
    fn malicious_sequence_length_is_rejected_not_panicking() {
        let r = reg();
        let spec = r.message("std_msgs/msg/String").unwrap();
        // A String's length prefix claims ~4 billion bytes in a tiny buffer.
        let bytes = [0x00, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00];
        let res = Codec::new(&r).decode(spec, &bytes);
        assert!(res.is_err(), "huge length must error, not panic/allocate");
    }

    #[test]
    fn truncated_buffer_errors_gracefully() {
        let r = reg();
        let spec = r.message("geometry_msgs/msg/Point").unwrap();
        // Only 4 of the 24 required payload bytes are present.
        let bytes = [0x00, 0x01, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04];
        assert!(Codec::new(&r).decode(spec, &bytes).is_err());
    }

    #[test]
    fn time_accepts_ros1_aliases() {
        let r = reg();
        let spec = r.message("std_msgs/msg/Header").unwrap();
        let codec = Codec::with_now(&r, Some((0, 0)));
        // ROS1-style secs/nsecs must be accepted.
        let v = json!({"stamp": {"secs": 7, "nsecs": 8}, "frame_id": "x"});
        let bytes = codec.encode(spec, &v).unwrap();
        let back = codec.decode(spec, &bytes).unwrap();
        assert_eq!(back["stamp"]["sec"], 7);
        assert_eq!(back["stamp"]["nanosec"], 8);
    }
}
