#!/usr/bin/env python3
"""Run all configured yard actions through genuine Native MuJoCo physics."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402

from sim.native.controller import Go2Controller  # noqa: E402
from sim.native.scene_builder import NativeSceneBuilder  # noqa: E402


def advance(controller: Go2Controller, seconds: float) -> None:
    for _ in range(round(seconds/controller.model.opt.timestep)):
        controller.step()
        mujoco.mj_step(controller.model, controller.data)


def run(name: str, model: mujoco.MjModel, environment: dict) -> dict:
    data = mujoco.MjData(model)
    controller = Go2Controller(
        model, data, ROOT / "assets/robots/go2/policy/moe_rough",
        clock=lambda: data.time,
        action_profile=ROOT / environment["namedActionProfile"], environment_id="yard",
        action_assets=ROOT / ".runtime/stunt-assets")
    if name == "jump":
        data.qpos[controller.base_qpos:controller.base_qpos+2] = [3.2, 0.0]
        data.qpos[controller.base_qpos+3:controller.base_qpos+7] = [1, 0, 0, 0]
        mujoco.mj_forward(model, data)
    advance(controller, 2.0)
    token = f"acceptance-{name}"
    if not controller.command({"type": "named_action", "operation": "start",
                               "name": name, "actionId": token}):
        raise RuntimeError(f"{name} rejected: {controller.command_error}")
    if controller.command({"type": "cmd_vel", "linearX": .1}):
        raise RuntimeError(f"{name} did not acquire exclusive motor control")
    timeout = float(controller.named_actions.profile.data["actions"][name]["durationS"])+8
    for _ in range(round(timeout/model.opt.timestep)):
        controller.step()
        mujoco.mj_step(model, data)
        if controller.named_actions.phase == "HOLD":
            break
    result = controller.named_actions.status() or {}
    if result.get("state") != "SUCCEEDED":
        raise RuntimeError(f"{name} failed: {result}")
    if not controller.command({"type": "named_action", "operation": "release",
                               "actionId": token}):
        raise RuntimeError(f"{name} release failed: {controller.command_error}")
    if controller.state()["controllerMode"] != "BASE_GAIT":
        raise RuntimeError(f"{name} did not restore base gait")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("actions", nargs="*")
    args = parser.parse_args()
    builder = NativeSceneBuilder(ROOT)
    scene, environment, _ = builder.build("yard", "go2")
    try:
        model = mujoco.MjModel.from_xml_path(str(scene))
        names = args.actions or list(json.loads(
            (ROOT / environment["namedActionProfile"]).read_text())["actions"])
        results = {name: run(name, model, environment) for name in names}
        print(json.dumps(results, indent=2, sort_keys=True))
    finally:
        builder.cleanup(scene)


if __name__ == "__main__":
    main()
