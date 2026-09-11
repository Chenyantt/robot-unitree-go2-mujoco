#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=141 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI ROS_DISCOVERY_SERVER
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
PYTHON="${GO2_SIM_PYTHON:-$ROOT/.venv/bin/python}"
SIM_PID= BRIDGE_PID=
cleanup() {
  trap - EXIT INT TERM
  [ -z "$BRIDGE_PID" ] || kill -TERM "$BRIDGE_PID" 2>/dev/null || true
  [ -z "$SIM_PID" ] || kill -TERM "$SIM_PID" 2>/dev/null || true
  [ -z "$BRIDGE_PID" ] || wait "$BRIDGE_PID" 2>/dev/null || true
  [ -z "$SIM_PID" ] || wait "$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
"$PYTHON" -m go2_sim.runtime --port "${GO2_SIM_PORT:-18765}" "$@" & SIM_PID=$!
GO2_SIM_EXPECTED_PID="$SIM_PID" "$PYTHON" "$ROOT/scripts/wait_ready.py"
"$PYTHON" -m go2_sim.bridge & BRIDGE_PID=$!
wait -n "$SIM_PID" "$BRIDGE_PID"
