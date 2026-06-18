#!/usr/bin/env bash
# Build (if needed) and run the rosbridge server with the rcl backend.
# Honors RMW_IMPLEMENTATION from the environment so the same binary bridges
# CycloneDDS, Fast-DDS, or Zenoh nodes.
set -euo pipefail

source /opt/ros/jazzy/setup.bash

PORT="${ROSBRIDGE_PORT:-9090}"

echo "rosbridge-rs starting on :${PORT} via ${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
cargo build --release --features rcl
exec ./target/release/rosbridge_websocket --backend rcl --port "${PORT}" "$@"
