#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
KIND="$1"
export RBNX_PACKAGE_ROOT="$ROOT/primitives/go2_sim_$KIND"
source /opt/ros/humble/setup.bash
source "$ROOT/primitives/go2_sim_chassis/rbnx-build/codegen/ros2_idl/install/setup.bash"
export ROS_DOMAIN_ID=141 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI ROS_DISCOVERY_SERVER
export ROBONIX_PROVIDER_BIND_HOST=127.0.0.1 ROBONIX_ADVERTISE_HOST=127.0.0.1
API="$(rbnx path robonix-api)"
export PYTHONPATH="$ROOT:$API:$RBNX_PACKAGE_ROOT/rbnx-build/codegen/proto_gen${PYTHONPATH:+:$PYTHONPATH}"
exec "${GO2_PROVIDER_PYTHON:-python3}" -m go2_sim.provider "$KIND"
