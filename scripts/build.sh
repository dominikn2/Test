#!/usr/bin/env bash
# Build the bridge with the rcl (r2r/ROS 2) backend. Must run inside the ROS
# container (r2r generates bindings against the sourced ROS install at build
# time and needs libclang).
set -euo pipefail

source /opt/ros/jazzy/setup.bash

# Limit r2r codegen to the packages we actually use (much faster builds).
# Override IDL_PACKAGE_FILTER in the environment to add more, or unset for all.
echo "Building with RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
echo "IDL_PACKAGE_FILTER=${IDL_PACKAGE_FILTER:-<all packages>}"

exec cargo build --features rcl "$@"
