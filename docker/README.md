# ROS 2 (Jazzy) Docker development setup

A containerized dev environment based on `ros:jazzy-ros-base` for building and
running rosbridge-rs with the **`rcl` backend** (via [`r2r`](https://github.com/sequenceplanner/r2r)).
Because the bridge rides ROS 2's `rcl`/`rmw` layer, it is **RMW-agnostic**: the
same binary bridges CycloneDDS, Fast-DDS, and **Zenoh** nodes — you just change
`RMW_IMPLEMENTATION`.

> Why a real ROS install? `rmw_zenoh` is not RTPS, so a pure-Rust DDS stack
> cannot reach it. Going through `rcl`/`rmw` is the only way to be universal
> across middlewares, and it also gives full services/actions support via ROS's
> own type introspection.

## What's here

| File | Purpose |
|------|---------|
| `../.devcontainer/Dockerfile` | Dev image: ROS 2 Jazzy + Rust + `libclang` + all three RMWs + sample nodes. |
| `../.devcontainer/devcontainer.json` | VS Code Dev Container (rust-analyzer with the `rcl` feature). |
| `docker-compose.yml` | The bridge + sample talker/service/action nodes + Zenoh router, profiled per RMW. |
| `../Makefile` | `up-cyclone` / `up-fastdds` / `up-zenoh`, `shell`, `build`, `test`. |
| `config/cyclonedds.xml` | Optional CycloneDDS config for bridged (non-host) networks. |

## Quick start

```bash
# Build the dev image once.
make image

# Bring up the bridge + sample nodes under a given middleware:
make up-cyclone     # CycloneDDS
make up-fastdds     # Fast-DDS
make up-zenoh       # Zenoh (also starts the rmw_zenoh router)
```

The bridge listens on `ws://localhost:9090`. Point any rosbridge client
(roslibjs, Foxglove's rosbridge connection, etc.) at it and you'll see the
sample `/chatter` topic, `/add_two_ints` service, and `/fibonacci` action —
regardless of which RMW you selected.

Smoke-test from the host without a browser:

```bash
# subscribe to the sample topic through the bridge
websocat ws://localhost:9090 <<'EOF'
{"op":"subscribe","topic":"/chatter","type":"std_msgs/msg/String"}
EOF
```

## Developing

```bash
make shell     # interactive ROS + Rust shell in the image
make build     # cargo build --features rcl (inside the container)
make test      # full test suite + rcl-feature compile
```

Or open the folder in VS Code and "Reopen in Container" (uses
`.devcontainer/`). `cargo`'s `target/` and the cargo registry are cached in
named volumes so rebuilds are fast.

## How the RMW switch works

* **CycloneDDS / Fast-DDS** — pure RTPS; discovery is automatic over host
  networking. Nothing else to run.
* **Zenoh** — `rmw_zenoh_cpp` needs a router. The `zenoh` compose profile
  starts `rmw_zenohd`; nodes connect to it on `localhost:7447` (host
  networking). The bridge inherits `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and works
  unchanged.

All services use `network_mode: host` so DDS multicast and the Zenoh router are
reachable without extra network plumbing. For a bridged network instead, drop
host networking and set `CYCLONEDDS_URI` to `config/cyclonedds.xml` (and add
explicit peers if multicast is unavailable).

## Build-time notes for the `rcl` backend

`r2r` generates Rust bindings from the **installed** ROS interface packages at
build time (it needs a sourced ROS env and `libclang`). To add message types,
install the corresponding `ros-jazzy-<pkg>` package in the Dockerfile and add
its name to `IDL_PACKAGE_FILTER` (which we set to keep codegen fast); unset that
variable to generate bindings for every installed package.
