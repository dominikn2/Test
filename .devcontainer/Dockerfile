# Development image for rosbridge-rs on ROS 2 Jazzy.
#
# Provides everything needed to build the r2r-based `rcl` backend (which links
# against a sourced ROS 2 install and needs libclang at build time) and to run
# an RMW interop matrix (CycloneDDS, Fast-DDS, Zenoh) against sample ROS nodes.
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# ---- System & ROS build/runtime dependencies -------------------------------
# * clang/libclang-dev: required by r2r's bindgen-based code generation.
# * rmw_*: the middlewares we want the bridge to be universal across.
# * demo/example/action packages: sample nodes for the interop tests.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        clang \
        libclang-dev \
        cmake \
        curl \
        git \
        pkg-config \
        python3-pip \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-rmw-fastrtps-cpp \
        ros-jazzy-rmw-zenoh-cpp \
        ros-jazzy-demo-nodes-cpp \
        ros-jazzy-example-interfaces \
        ros-jazzy-action-tutorials-cpp \
        ros-jazzy-action-tutorials-interfaces \
    && rm -rf /var/lib/apt/lists/*

# ---- Rust toolchain --------------------------------------------------------
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
        | sh -s -- -y --default-toolchain stable --profile minimal \
    && rustup component add clippy rustfmt

# Speed up r2r codegen: only generate bindings for the packages we use. Extend
# this list (or remove it for "all packages") as your interfaces grow.
ENV IDL_PACKAGE_FILTER="std_msgs;builtin_interfaces;geometry_msgs;sensor_msgs;std_srvs;example_interfaces;action_tutorials_interfaces;nav_msgs;diagnostic_msgs;rosgraph_msgs;unique_identifier_msgs;action_msgs"

# Source ROS for all interactive shells.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc

WORKDIR /workspace
COPY scripts/ros-entrypoint.sh /usr/local/bin/ros-entrypoint.sh
RUN chmod +x /usr/local/bin/ros-entrypoint.sh
ENTRYPOINT ["/usr/local/bin/ros-entrypoint.sh"]
CMD ["bash"]
