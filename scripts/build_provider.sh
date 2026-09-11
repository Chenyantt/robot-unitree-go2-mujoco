#!/usr/bin/env bash
set -eo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$PWD}"
source /opt/ros/humble/setup.bash
if [ "$(basename "$PKG")" != go2_sim_chassis ]; then
  rbnx codegen -p "$PKG"
  exit 0
fi
rbnx codegen -p "$PKG" --ros2
cd "$PKG/rbnx-build/codegen/ros2_idl"
colcon build --executor sequential --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
