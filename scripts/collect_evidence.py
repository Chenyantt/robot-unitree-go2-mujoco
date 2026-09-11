# SPDX-License-Identifier: Apache-2.0
"""Collect measured, credential-free acceptance reports into release documentation."""
import json
from pathlib import Path
import shutil

root = Path(__file__).resolve().parents[1]
docs = root/"docs"
docs.mkdir(exist_ok=True)
for name in ("locomotion.json", "ros-acceptance.json", "robonix-acceptance.json",
             "walking-demo.json", "walking-demo.mp4", "walking-demo.png"):
    shutil.copyfile(root/".runtime"/name, docs/name)
loco = json.loads((docs/"locomotion.json").read_text())
ros = json.loads((docs/"ros-acceptance.json").read_text())
rbnx = json.loads((docs/"robonix-acceptance.json").read_text())
assert loco["passed"] and ros["passed"] and rbnx["passed"]
lines = ["# Native Go2 simulation acceptance — 2026-09-11 / 2026-09-12", "",
         "Measured locally on Ubuntu 22.04 / ROS 2 Humble. MuJoCo 3.3.6, CPU torch 2.8.0.",
         "Robonix CLI build: 178fd2a+; API/codegen source checkout: 2548fe8ba549d39d1b95f98f552372c6e2134769.",
         "This is simulator acceptance, not physical Go2, Jetson or Nav2 performance.", "",
         "## Physics-only cases", "",
         "Each case: settle 2 s, command 5 s, stop 2 s, all in simulation time.", "",
         "| Case | X displacement (m) | Y displacement (m) | Yaw change (rad) | Speed after stop (m/s) | Pass |",
         "| --- | ---: | ---: | ---: | ---: | --- |"]
for row in loco["cases"]:
    lines.append(f'| {row["case"]} | {row["displacement_m"][0]:.4f} | {row["displacement_m"][1]:.4f} | {row["yaw_change_rad"]:.4f} | {row["speed_after_stop_m_s"]:.5f} | {row["passed"]} |')
lines += ["", "## ROS closed loop", "",
          f'- Forward displacement: {ros["forward_distance_m"]:.4f} m from a 3 s, 0.3 m/s ROS command.',
          f'- Speed after command expiry: {ros["watchdog_stop_speed_m_s"]:.6f} m/s.',
          '- Advancing clock, TF, IMU, 12 joints, 180-ray scan, RGB and float metric depth received.',
          '- Explicit Trigger stop and reset succeeded; reset incremented the epoch and restored origin.',
          "", "## Robonix closed loop", "",
          f'- Four ACTIVE primitives; {rbnx["capability_count"]} capabilities.',
          f'- Timed velocity displacement including settling: {rbnx["velocity_distance_m"]:.4f} m.',
          f'- Relative 0.5 m command measured at return: {rbnx["relative_distance_m"]:.4f} m.',
          f'- Relative 45-degree command measured at return: {rbnx["angle_rad"]:.4f} rad.',
          f'- Speed after lifecycle cancellation: {rbnx["stop_speed_m_s"]:.6f} m/s.',
          '- Concurrent RPC returned busy; lifecycle deactivation cancelled the active move.',
          '- Non-finite command rejected; lifecycle activation restored availability.',
          "", "## Evidence and limits", "",
          "The adjacent JSON files are test-produced measurements. The MP4 is rendered from actual",
          "MuJoCo stepping and motor torques, in a separate 18-second deterministic dynamics run.",
          "It is not a screen recording of the gRPC test. No base teleporting is used to create walking.",
          "15 unit tests and six physics cases form the CI template; all reported tests ran locally.",
          "GitHub Actions activation is pending workflow-write permission; no cloud CI pass is claimed.",
          "The Sept 12 repeat includes a minimum 0.12 m/s relative-distance command outside tolerance",
          "to avoid the policy's near-zero-velocity standing deadband observed during the Sept 11 repeat.",
          "Complex terrain, calibrated physical sensor noise, autonomous mapping/navigation,",
          "language planning and Web WASM are not included in this acceptance.", ""]
(docs/"acceptance.md").write_text("\n".join(lines))
