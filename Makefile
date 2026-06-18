# Developer entrypoints for the ROS 2 (Jazzy) Docker setup.
# These run on a host with Docker; they are not used by the pure-Rust build.

COMPOSE := docker compose -f docker/docker-compose.yml

.PHONY: image up-cyclone up-fastdds up-zenoh down logs shell build test clean-volumes

## Build the dev image (ros:jazzy-ros-base + Rust + r2r deps + all RMWs).
image:
	$(COMPOSE) build

## Bring up the bridge + sample nodes under CycloneDDS.
up-cyclone:
	RMW_IMPLEMENTATION=rmw_cyclonedds_cpp $(COMPOSE) --profile cyclone up

## Bring up the bridge + sample nodes under Fast-DDS.
up-fastdds:
	RMW_IMPLEMENTATION=rmw_fastrtps_cpp $(COMPOSE) --profile fastdds up

## Bring up the bridge + sample nodes + router under Zenoh.
up-zenoh:
	RMW_IMPLEMENTATION=rmw_zenoh_cpp $(COMPOSE) --profile zenoh up

## Stop everything.
down:
	$(COMPOSE) --profile cyclone --profile fastdds --profile zenoh down

logs:
	$(COMPOSE) logs -f bridge

## Interactive ROS+Rust dev shell in the image.
shell:
	$(COMPOSE) run --rm --no-deps bridge bash

## Build the rcl backend inside the container.
build:
	$(COMPOSE) run --rm --no-deps bridge bash scripts/build.sh

## Run the full test suite inside the container (incl. rcl feature compile).
test:
	$(COMPOSE) run --rm --no-deps bridge bash -lc \
		"source /opt/ros/jazzy/setup.bash && cargo test && cargo build --features rcl"

## Drop the cached cargo target/registry volumes.
clean-volumes:
	docker volume rm rosbridge-rs_rosbridge-cargo-target rosbridge-rs_rosbridge-cargo-registry || true
