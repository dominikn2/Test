# rosbridge-rs

A pure-Rust, runtime-efficient reimplementation of
[`rosbridge_server`](https://github.com/RobotWebTools/rosbridge_suite/tree/ros2/rosbridge_server)
(ROS 2 branch), targeting **1:1 protocol and API parity** — a drop-in
replacement that aims to be leaner than `rosbridge_server` and
`foxglove_bridge`.

It speaks the [rosbridge v2.1 protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md)
over WebSockets and bridges to ROS 2 over DDS using the pure-Rust
[`ros2-client`](https://crates.io/crates/ros2-client) / [`rustdds`](https://crates.io/crates/rustdds)
stack — **no ROS 2 installation required**.

## Why it's fast

* **Move bytes, not objects.** Like `foxglove_bridge`, the bridge transports
  opaque CDR buffers and only pays the CDR↔JSON cost at the rosbridge JSON
  boundary, which is inherent to the protocol. The `cbor-raw` path forwards the
  raw CDR with **zero** deserialization.
* **Schema-driven dynamic codec.** A pure-Rust `.msg`/`.srv`/`.action` parser
  builds a cached type model; each message is a table-driven CDR walk, not a
  re-parse.
* **Async, lock-light core.** Tokio end-to-end; per-client state behind tiny
  `parking_lot` critical sections; subscriptions coalesce to the lowest common
  throttle/queue and fan out without copies where possible.
* **Aggressive release profile.** fat LTO, one codegen unit, panic=abort.

## Workspace layout

| Crate | Responsibility |
|-------|----------------|
| `rosbridge-protocol` | The rosbridge v2.1 message data-model and (de)serialization (transport- and ROS-agnostic). |
| `ros-message` | Pure-Rust dynamic ROS 2 message model: `.msg`/`.srv`/`.action` parser, type registry (bundled standard interfaces + ament-prefix runtime loading), and a schema-driven, alignment-aware CDR ↔ `serde_json::Value` codec. |
| `rosbridge-server` | The WebSocket server: per-client protocol sessions, all capabilities, compression, fragmentation, glob security, and the pluggable ROS backend (in-process `loopback` + ROS 2 `dds`). |

## Feature parity

All rosbridge ops are implemented: `advertise` / `unadvertise` / `publish`,
`subscribe` / `unsubscribe` (with `throttle_rate`, `queue_length`,
`fragment_size`, `compression`, `qos`, and multi-subscription coalescing),
`call_service` / `advertise_service` / `unadvertise_service` /
`service_response` (client as caller **and** as server),
`advertise_action` / `unadvertise_action` / `send_action_goal` /
`cancel_action_goal` / `action_feedback` / `action_result` (both directions),
and `fragment` (defragmentation + central outgoing fragmentation).

Compression: `none`, `cbor`, `cbor-raw`, `png` (precedence
`cbor-raw > cbor > png > none`). Binary frames for CBOR; PNG as
`{"op":"png",...}` text.

Configuration mirrors the ROS 2 node parameters exactly (names, types,
defaults): `port`, `address`, `url_path`, `retry_startup_delay`,
`certfile`/`keyfile`, `websocket_ping_interval`/`websocket_ping_timeout`,
`use_compression`, `fragment_timeout`, `delay_between_messages`,
`max_message_size`, `unregister_timeout`, the `*_glob` security filters
(with legacy `topics_glob` merge and `/rosapi/*` auto-append), and the
service/action threading + timeout knobs.

> Enhancement over the ROS 2 branch: protocol errors are also surfaced to the
> client as `{"op":"status"}` messages (the upstream ros2 branch logs only).

## Running

```bash
# Browser↔browser / testing, no ROS 2 needed:
cargo run --release -- --port 9090

# Real ROS 2 / DDS bridge (pure Rust, still no ROS 2 install required):
cargo run --release --features dds -- --port 9090 --backend dds
```

Interface definitions beyond the bundled standard set are loaded from
`$AMENT_PREFIX_PATH` (or `--interface-paths a:b:c`) by scanning
`<prefix>/share/<pkg>/{msg,srv,action}/*`.

## Benchmarks

The dynamic codec is the hot path. Measured on the dev container
(`cargo run --release -p ros-message --example bench_codec`):

| Operation | Throughput |
|-----------|-----------:|
| `LaserScan` (360 beams, 1.5 KB) decode CDR→JSON | ~177k msg/s · 265 MB/s |
| `LaserScan` encode JSON→CDR | ~46k msg/s · 69 MB/s |
| `Image` (640×480 rgb8, 900 KB) decode CDR→JSON+base64 | ~2.6k msg/s · 2.4 GB/s |
| `Image` **`cbor-raw` passthrough** (no decode) | ~29k msg/s · **27 GB/s** |

The `cbor-raw` path is ~11× faster for large payloads because it never
deserializes the message body — exactly the win this design targets for
high-bandwidth topics like images and point clouds.

## Testing

```bash
cargo test                         # unit + end-to-end (real WebSocket) tests
cargo test --features dds --test dds   # real RTPS round-trip (two DDS peers)
cargo clippy --workspace --all-targets
```

The repository ships 67 default tests (protocol, codec with golden
wire-format vectors, fragmentation, glob, compression, end-to-end over real
WebSockets) plus a real-RTPS DDS round-trip test behind the `dds` feature.
