# rosbridge-rs

A pure-Rust, runtime-efficient reimplementation of
[`rosbridge_server`](https://github.com/RobotWebTools/rosbridge_suite/tree/ros2/rosbridge_server)
(ROS 2 branch), targeting **1:1 protocol and API parity** — a drop-in
replacement that aims to be leaner than `rosbridge_server` and
`foxglove_bridge`.

It speaks the [rosbridge v2.1 protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md)
over WebSockets and bridges to ROS 2 through the `rcl`/`rmw` layer via
[`r2r`](https://github.com/sequenceplanner/r2r), so it is **RMW-agnostic**: the
same binary works with **CycloneDDS, Fast-DDS, or Zenoh**, selected at runtime
by `RMW_IMPLEMENTATION`. A self-contained `loopback` backend additionally
enables browser↔browser bridging and the full test-suite with no ROS install.

> Going through `rcl`/`rmw` (rather than a single DDS vendor) is what makes the
> bridge universal — `rmw_zenoh` is not RTPS, so a pure-DDS stack cannot reach
> it — and it gives full services support via ROS's own type introspection.

## Design

* **ROS owns serialization.** The bridge moves messages as `serde_json::Value`
  and lets ROS 2's own type introspection (via r2r's untyped API) produce and
  parse the CDR. There is no custom serializer on the runtime path — any
  installed message/service type is handled, with no codegen of our own.
* **RMW-agnostic.** Riding `rcl`/`rmw` means the one binary speaks whatever
  middleware `RMW_IMPLEMENTATION` selects (CycloneDDS, Fast-DDS, Zenoh).
* **Async, lock-light core.** Tokio end-to-end; per-client state behind tiny
  `parking_lot` critical sections; subscriptions coalesce to the lowest common
  throttle/queue.
* **Aggressive release profile.** fat LTO, one codegen unit, panic=abort.

## Workspace layout

| Crate | Responsibility |
|-------|----------------|
| `rosbridge-protocol` | The rosbridge v2.1 message data-model and (de)serialization (transport- and ROS-agnostic). |
| `rosbridge-server` | The WebSocket server: per-client protocol sessions, all capabilities, compression, fragmentation, glob security, and the pluggable ROS backend (in-process `loopback` + RMW-agnostic `rcl` via r2r). |
| `ros-message` | A **standalone** pure-Rust dynamic ROS 2 message library (`.msg`/`.srv`/`.action` parser + alignment-aware CDR↔JSON codec). It is **not** used by the bridge runtime (ROS does serialization); kept as an independent, tested library. |

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

# Real ROS 2 bridge, any RMW (built & run inside the ROS container — see docker/):
cargo run --release --features rcl -- --port 9090 --backend rcl

# WSS / TLS termination:
cargo run --release --features tls -- --port 9090 \
    --certfile cert.pem --keyfile key.pem
```

The `rcl` backend is RMW-agnostic — pick the middleware with the environment:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  cargo run --features rcl -- --backend rcl
RMW_IMPLEMENTATION=rmw_zenoh_cpp       cargo run --features rcl -- --backend rcl
```

Optional Cargo features: `rcl` (RMW-agnostic ROS 2 backend via r2r; builds only
inside a sourced ROS 2 env with `libclang` — use the `docker/` setup) and `tls`
(WSS via rustls). Both are off by default to keep the core build lean. The
easiest way to build/run the `rcl` backend across the RMW matrix is the
[Docker dev environment](docker/README.md): `make up-cyclone` / `make up-zenoh`.

Message types come from the ROS 2 install: r2r generates bindings for the
installed interface packages at build time, and the untyped API uses ROS's
runtime introspection — so any installed type works with no configuration.

## Testing

```bash
cargo test                         # unit + end-to-end (real WebSocket) tests
cargo clippy --workspace --all-targets
make test                          # in-container: full suite + rcl-feature compile
```

The repository ships 67 default tests (protocol, codec with golden
wire-format vectors, fragmentation, glob, compression, end-to-end over real
WebSockets) plus a TLS acceptor test behind the `tls` feature. The compiled
binary is also verified end-to-end (two WebSocket clients exchanging a message
through it). The `rcl` backend is built and exercised against live ROS 2 nodes
inside the Docker RMW matrix (`make up-cyclone` / `up-fastdds` / `up-zenoh`).

## Status & limitations

Implemented and tested:

* Full rosbridge v2.1 protocol over WebSockets, all ops, both service/action
  directions, all compression modes, fragmentation, throttling/queueing,
  glob security, `set_level`.
* Serialization delegated entirely to ROS 2 introspection (no custom codec on
  the path); any installed message/service type works out of the box.
* Loopback backend (full feature set, incl. services & actions).
* `rcl` backend (via r2r): RMW-agnostic topics and services across CycloneDDS,
  Fast-DDS, and Zenoh.
* TLS (`tls` feature), input hardening.

Known gaps (honest scope for the prototype):

* **Actions over the `rcl` backend** are not yet wired (they return
  `Unsupported`; the loopback backend implements them fully). r2r exposes only
  statically-typed actions, so dynamic/runtime-typed action bridging needs a
  manual `rcl_action` implementation.
* The `rcl` backend module compiles only inside a sourced ROS 2 environment
  with `libclang` (use the `docker/` setup); it is excluded from the default
  build.
* `cbor-raw` compression degrades to `cbor` on these backends: the raw
  serialized CDR is no longer carried at the JSON boundary now that ROS owns
  serialization.
* `use_compression` (WebSocket permessage-deflate) is accepted but not yet
  applied.

The ROS 2 `rosbridge_server` branch removes the client-facing `status`/
`set_level` ops (it logs only); this implementation keeps them as a
backward-compatible superset (errors are both logged and sent to the client,
subject to `set_level`).
