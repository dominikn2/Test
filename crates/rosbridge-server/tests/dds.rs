//! Real RTPS round-trip test for the DDS backend. Two independent DDS
//! participants in one process discover each other over the loopback network
//! and exchange a message — exercising the actual ros2-client/rustdds wire path.
//!
//! Requires the `dds` feature and a network stack that permits RTPS discovery
//! (UDP, typically multicast). Run with: `cargo test --features dds --test dds`.
#![cfg(feature = "dds")]

use std::sync::Arc;
use std::time::Duration;

use ros_message::{Codec, Registry};
use rosbridge_server::backend::dds::DdsBackend;
use rosbridge_server::backend::{QosSpec, RosBackend};
use serde_json::json;

#[tokio::test]
async fn dds_pubsub_roundtrip() {
    let registry = Arc::new(Registry::with_standard_types());

    let pubr = DdsBackend::with_node_name("rb_test_pub").expect("pub backend");
    let subr = DdsBackend::with_node_name("rb_test_sub").expect("sub backend");

    // Reliable + transient-local so a late-joining subscriber still receives.
    let qos = QosSpec {
        history_keep_all: false,
        depth: 10,
        reliability: rosbridge_server::backend::Reliability::Reliable,
        durability: rosbridge_server::backend::Durability::TransientLocal,
    };

    let topic = "/rb_dds_test";
    let ty = "std_msgs/msg/String";
    let pub_id = pubr.advertise(topic, ty, &qos).expect("advertise");
    let (_sub_id, mut rx) = subr.subscribe(topic, ty, &qos).expect("subscribe");

    // Allow RTPS discovery to match the endpoints.
    tokio::time::sleep(Duration::from_secs(2)).await;

    let spec = registry.message(ty).unwrap();
    let cdr = Codec::new(&registry)
        .encode(spec, &json!({"data": "over the wire"}))
        .unwrap();

    // Publish repeatedly; transient-local + retries tolerate discovery timing.
    let recv = tokio::time::timeout(Duration::from_secs(10), async {
        loop {
            pubr.publish(pub_id, &cdr).expect("publish");
            tokio::select! {
                s = rx.recv() => {
                    if let Some(sample) = s { return sample; }
                }
                _ = tokio::time::sleep(Duration::from_millis(300)) => {}
            }
        }
    })
    .await;

    let sample = recv.expect("did not receive sample over RTPS within timeout");
    let decoded = Codec::new(&registry).decode(spec, &sample.cdr).unwrap();
    assert_eq!(decoded["data"], "over the wire");
}
