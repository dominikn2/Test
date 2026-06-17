//! Micro-benchmark for the dynamic CDR<->JSON codec.
//!
//! Run with: `cargo run --release -p ros-message --example bench_codec`
//!
//! Reports throughput for the three hot paths a rosbridge subscription takes:
//! full CDR->JSON decode, JSON->CDR encode, and the `cbor-raw` passthrough
//! (which does zero deserialization — the key efficiency win).

use std::hint::black_box;
use std::time::Instant;

use ros_message::{Codec, Registry};
use serde_json::json;

fn bench<F: FnMut()>(name: &str, iters: u32, payload_bytes: usize, mut f: F) {
    // Warm up.
    for _ in 0..(iters / 10).max(1) {
        f();
    }
    let start = Instant::now();
    for _ in 0..iters {
        f();
    }
    let elapsed = start.elapsed();
    let per = elapsed / iters;
    let per_s = iters as f64 / elapsed.as_secs_f64();
    let mb_s = (payload_bytes as f64 * iters as f64) / elapsed.as_secs_f64() / 1e6;
    println!(
        "{name:<28} {per:>10.2?}/op   {per_s:>12.0} ops/s   {mb_s:>8.1} MB/s",
        per = per,
    );
}

fn main() {
    let reg = Registry::with_standard_types();
    let codec = Codec::new(&reg);

    // A realistic mid-size sensor message: 360-beam LaserScan.
    let scan_spec = reg.message("sensor_msgs/msg/LaserScan").unwrap();
    let ranges: Vec<f64> = (0..360).map(|i| i as f64 * 0.01).collect();
    let scan = json!({
        "header": {"frame_id": "laser"},
        "angle_min": -3.1, "angle_max": 3.1, "angle_increment": 0.0175,
        "time_increment": 0.0, "scan_time": 0.1,
        "range_min": 0.1, "range_max": 30.0,
        "ranges": ranges, "intensities": []
    });
    let scan_cdr = Codec::new(&reg).encode(scan_spec, &scan).unwrap();
    println!("LaserScan(360) CDR size: {} bytes\n", scan_cdr.len());

    let iters = 200_000;
    bench("LaserScan encode JSON->CDR", iters, scan_cdr.len(), || {
        black_box(codec.encode(scan_spec, &scan).unwrap());
    });
    bench("LaserScan decode CDR->JSON", iters, scan_cdr.len(), || {
        black_box(codec.decode(scan_spec, &scan_cdr).unwrap());
    });

    // A large PointCloud2-style byte blob to show passthrough vs decode.
    let img_spec = reg.message("sensor_msgs/msg/Image").unwrap();
    let data: Vec<u8> = (0..(640 * 480 * 3)).map(|i| (i % 251) as u8).collect();
    let b64 = base64::Engine::encode(&base64::engine::general_purpose::STANDARD, &data);
    let img = json!({
        "header": {"frame_id": "cam"}, "height": 480, "width": 640,
        "encoding": "rgb8", "is_bigendian": 0, "step": 640 * 3, "data": b64
    });
    let img_cdr = Codec::new(&reg).encode(img_spec, &img).unwrap();
    println!("\nImage(640x480 rgb8) CDR size: {} bytes\n", img_cdr.len());

    let img_iters = 5_000;
    bench("Image decode CDR->JSON(b64)", img_iters, img_cdr.len(), || {
        black_box(codec.decode(img_spec, &img_cdr).unwrap());
    });
    bench("Image cbor-raw passthrough", img_iters, img_cdr.len(), || {
        // cbor-raw forwards the raw CDR verbatim: a single copy, no decode.
        black_box(rosbridge_cbor_raw(&img_cdr));
    });
}

/// Mimics the `cbor-raw` hot path cost: wrap raw CDR with no deserialization.
#[inline]
fn rosbridge_cbor_raw(cdr: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(cdr.len() + 16);
    out.extend_from_slice(b"\xa3"); // map(3) — representative envelope cost
    out.extend_from_slice(cdr);
    out
}
