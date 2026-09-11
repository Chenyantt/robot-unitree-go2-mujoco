#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Deterministic headless dynamic acceptance; never imports ROS or a Unitree SDK."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from go2_sim.scene import load_scene
from go2_sim.controller import Go2Controller


def run():
    """Measure real simulated displacement and tilt for representative commands."""
    assets = ROOT / ".runtime/assets"
    model = load_scene(assets, room=False)
    data = mujoco.MjData(model)
    controller = Go2Controller(model, data, assets)
    results = []
    for name, command in (("stand", [0, 0, 0]), ("forward", [.4, 0, 0]),
                          ("backward", [-.3, 0, 0]), ("left", [0, .3, 0]),
                          ("turn_left", [0, 0, .5]), ("turn_right", [0, 0, -.5])):
        controller.reset()
        for _ in range(400):
            controller.step([0, 0, 0])
        start = controller.state()
        min_height, min_upright = 10., 1.
        for _ in range(1000):
            controller.step(command)
            state = controller.state()
            min_height = min(min_height, state["position"][2])
            min_upright = min(min_upright, state["upright"])
        end = controller.state()
        delta = np.asarray(end["position"]) - start["position"]
        yaw = np.arctan2(np.sin(end["yaw"]-start["yaw"]), np.cos(end["yaw"]-start["yaw"]))
        for _ in range(400):
            controller.step([0, 0, 0])
        stopped = controller.state()
        result = {"case": name, "command": command, "duration_s": 5,
                  "displacement_m": delta.tolist(), "yaw_change_rad": float(yaw),
                  "minimum_height_m": min_height, "minimum_upright": min_upright,
                  "yaw_rate_after_stop_rad_s": float(stopped["angular_velocity"][2]),
                  "speed_after_stop_m_s": float(np.linalg.norm(stopped["linear_velocity"][:2]))}
        checks = [min_height > .20, min_upright > .85, result["speed_after_stop_m_s"] < .12,
                  abs(result["yaw_rate_after_stop_rad_s"]) < .12]
        if name == "stand":
            checks.extend([np.linalg.norm(delta[:2]) < .1, abs(yaw) < .12])
        if name == "forward":
            checks.extend([delta[0] > 1., abs(delta[1]) < .3, abs(yaw) < .2])
        if name == "backward":
            checks.extend([delta[0] < -.7, abs(delta[1]) < .3, abs(yaw) < .2])
        if name == "left":
            checks.append(delta[1] > .7)
        if name.startswith("turn_"):
            checks.append(yaw * command[2] > .5)
        result["passed"] = bool(all(checks))
        results.append(result)
        print(json.dumps(result), flush=True)
    return {"mujoco": mujoco.__version__, "cases": results, "passed": all(r["passed"] for r in results)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    result = run()
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2) + "\n")
    raise SystemExit(0 if result["passed"] else 1)
