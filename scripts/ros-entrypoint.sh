#!/usr/bin/env bash
# Entrypoint: source the ROS 2 overlay (and the Zenoh router config when using
# rmw_zenoh) before executing the container command.
set -e

source /opt/ros/jazzy/setup.bash

# When using Zenoh, point sessions at the router service if one is configured.
if [ "${RMW_IMPLEMENTATION:-}" = "rmw_zenoh_cpp" ] && [ -n "${ZENOH_ROUTER_CONFIG_URI:-}" ]; then
    export ZENOH_ROUTER_CONFIG_URI
fi

exec "$@"
