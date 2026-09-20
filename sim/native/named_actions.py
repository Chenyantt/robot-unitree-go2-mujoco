"""Exclusive, profile-gated named actions for the native Go2 controller."""
from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


SCRIPTED_ACTIONS = frozenset({"crouch", "bow", "dance", "sway", "stretch", "jump"})
LEARNED_ACTIONS = frozenset({"backflip", "handstand_walk"})
TERMINAL_STATES = frozenset({"SUCCEEDED", "FAILED", "CANCELED"})


def smooth(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def yaw_from_quaternion(quaternion: np.ndarray) -> float:
    w, x, y, z = quaternion
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def wrapped_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


class NamedActionProfile:
    """Validated scene-owned action and safety-zone configuration."""

    def __init__(self, path: Path | None, environment_id: str) -> None:
        self.path = path
        self.environment_id = environment_id
        self.data: dict[str, Any] = {}
        if path is None:
            return
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("schemaVersion") != 1 or data.get("environmentId") != environment_id:
            raise ValueError("named action profile does not match the active environment")
        if not isinstance(data.get("actions"), dict) or not isinstance(data.get("zones"), dict):
            raise ValueError("named action profile requires actions and zones")
        unknown = set(data["actions"]) - SCRIPTED_ACTIONS - LEARNED_ACTIONS
        if unknown:
            raise ValueError(f"named action profile contains unsupported actions: {sorted(unknown)}")
        for name, action in data["actions"].items():
            if action.get("zone") not in data["zones"]:
                raise ValueError(f"named action {name} references an unknown zone")
            if action.get("executionZone", action.get("zone")) not in data["zones"]:
                raise ValueError(f"named action {name} references an unknown execution zone")
            duration = action.get("durationS")
            if type(duration) not in (int, float) or not math.isfinite(duration) or duration <= 0:
                raise ValueError(f"named action {name} duration must be finite and positive")
        self.data = data

    @property
    def configured(self) -> bool:
        return bool(self.data)

    def available(self, assets: Path) -> list[str]:
        result = []
        have_ort = importlib.util.find_spec("onnxruntime") is not None
        for name, spec in self.data.get("actions", {}).items():
            required = list(spec.get("assets", []))
            if spec.get("asset"):
                required.append(spec["asset"])
            if required and (not have_ort or not all((assets / item).is_file() for item in required)):
                continue
            result.append(name)
        return sorted(result)

    def check_zone(self, name: str, position: np.ndarray, yaw: float,
                   execution: bool = False) -> str | None:
        action = self.data["actions"][name]
        zone_name = action.get("executionZone", action["zone"]) if execution else action["zone"]
        zone = self.data["zones"][zone_name]
        point = np.asarray(position[:2], dtype=float)
        if zone.get("shape") == "circle":
            inside = np.linalg.norm(point - np.asarray(zone["center"], dtype=float)) <= float(zone["radius"])
        elif zone.get("shape") == "box":
            inside = np.all(point >= np.asarray(zone["minimum"], dtype=float)) and np.all(
                point <= np.asarray(zone["maximum"], dtype=float))
        else:
            return f"zone {zone_name} has an unsupported shape"
        if not inside:
            return f"robot is outside safe zone {zone_name}"
        if not execution and "headingRad" in zone:
            error = abs(wrapped_angle(yaw - float(zone["headingRad"])))
            if error > float(zone["headingToleranceRad"]):
                return f"heading error {error:.3f} rad exceeds safe-zone tolerance"
        return None


class BackflipRunner:
    """Torque-only adapter for the pinned public phase-conditioned policy."""

    def __init__(self, owner: "NamedActionManager") -> None:
        import onnxruntime as ort

        self.owner = owner
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            str(owner.assets / "backflip/policy.onnx"), options,
            providers=["CPUExecutionProvider"])
        self.reset()

    def reset(self) -> None:
        self.current = np.zeros(12)
        self.last = np.zeros(12)
        self.slew = np.zeros(12)
        self.applied = np.zeros(12)
        self.next_inference = 2.0
        self.start_joints = self.owner.joint_position.copy()

    def torque(self, elapsed: float) -> np.ndarray:
        owner = self.owner
        if elapsed < 2.0:
            blend = smooth(elapsed / .8)
            target = self.start_joints*(1-blend) + owner.default_pose*blend
            return 40*(target-owner.joint_position) - owner.joint_velocity
        if elapsed + 1e-9 >= self.next_inference:
            phase = np.pi * min(elapsed-2.0, 2.0) / 2.0
            features = [np.sin(phase), np.cos(phase), np.sin(phase/2),
                        np.cos(phase/2), np.sin(phase/4), np.cos(phase/4)]
            observation = np.concatenate((
                owner.base_angular_velocity*.25,
                owner.base_rotation.T @ np.array([0.0, 0.0, -1.0]),
                owner.joint_position-owner.default_pose,
                owner.joint_velocity*.05, self.current, self.last, features,
            )).astype(np.float32)
            output = np.asarray(self.session.run(None, {
                self.session.get_inputs()[0].name: np.clip(observation, -100, 100)[None]
            })[0]).reshape(-1)
            if output.shape != (12,) or not np.isfinite(output).all():
                raise RuntimeError("backflip policy produced an invalid action")
            self.applied = self.current.copy()
            self.last = self.current.copy()
            self.current = np.clip(output, -100, 100)
            self.next_inference += .02
        self.slew += np.clip(self.applied-self.slew, -.054, .054)
        velocity = owner.joint_velocity
        torque = 40*(owner.default_pose+.5*self.slew-owner.joint_position)-velocity
        limit = np.where(velocity*torque > 0, 20.2, 23.4) * np.clip(
            (30-np.abs(velocity))/(30-13.5), 0, 1)
        return np.clip(torque, -limit, limit)


class HandstandRunner:
    """Torque-only adapter for the pinned quad-to-hand public policies."""

    def __init__(self, owner: "NamedActionManager") -> None:
        import onnxruntime as ort

        self.owner = owner
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        folder = owner.assets / "quad2hand"
        self.estimator = ort.InferenceSession(str(folder / "go2_estimator.onnx"), options,
                                              providers=["CPUExecutionProvider"])
        self.actor = ort.InferenceSession(str(folder / "go2_actor.onnx"), options,
                                          providers=["CPUExecutionProvider"])
        self.reset()

    def reset(self) -> None:
        self.action = np.zeros(12, dtype=np.float32)
        self.history = np.zeros((4, 55), dtype=np.float32)
        self.first = True
        self.next_inference = 0.0

    def torque(self, elapsed: float) -> np.ndarray:
        mode = int(3.0 <= elapsed < 20.0)
        command = np.array([.20 if 9.0 < elapsed < 16.0 else 0.0, 0.0, 0.0])
        if elapsed + 1e-9 >= self.next_inference:
            period = .4 if mode else .5
            phase = (elapsed % period) / period
            offsets = np.array([0, .5, 0, 0] if mode else [0, .5, .5, 0])
            legs = (phase + offsets) % 1
            owner = self.owner
            observation = np.concatenate((
                owner.base_angular_velocity*.25,
                owner.base_rotation.T @ np.array([0.0, 0.0, -1.0]),
                command*np.array([2, 2, .25]), owner.joint_position-owner.default_pose,
                owner.joint_velocity*.05, self.action,
                np.sin(2*np.pi*legs), np.cos(2*np.pi*legs), [1-mode, mode],
            )).astype(np.float32)
            estimator_input = np.concatenate((observation, self.history.ravel())).astype(np.float32)
            if self.first:
                self.history[:] = observation
                self.first = False
            else:
                self.history[1:] = self.history[:-1].copy()
                self.history[0] = observation
            latent = np.asarray(self.estimator.run(None, {"input": estimator_input[None]})[0]).ravel()
            actor_input = np.concatenate((estimator_input, latent)).astype(np.float32)
            self.action = np.asarray(self.actor.run(None, {"input": actor_input[None]})[0]).ravel()
            if self.action.shape != (12,) or not np.isfinite(self.action).all():
                raise RuntimeError("handstand policy produced an invalid action")
            self.next_inference += .02
        owner = self.owner
        return 30*(owner.default_pose+.25*self.action-owner.joint_position)-.75*owner.joint_velocity


class NamedActionManager:
    """Own action admission, torque generation, recovery and terminal hold."""

    def __init__(self, controller: Any, profile_path: Path | None,
                 environment_id: str, assets: Path) -> None:
        self.controller = controller
        self.model, self.data = controller.model, controller.data
        self.assets = assets
        self.profile = NamedActionProfile(profile_path, environment_id)
        self.default_pose = np.array([0, .8, -1.5, 0, .8, -1.5,
                                      0, 1.0, -1.5, 0, 1.0, -1.5], dtype=float)
        self.runners: dict[str, Any] = {}
        self.reset()

    @property
    def joint_position(self) -> np.ndarray:
        return self.data.qpos[self.controller.qpos]

    @property
    def joint_velocity(self) -> np.ndarray:
        return self.data.qvel[self.controller.dof]

    @property
    def base_rotation(self) -> np.ndarray:
        return self.data.xmat[self.controller.base].reshape(3, 3)

    @property
    def base_angular_velocity(self) -> np.ndarray:
        start = self.controller.base_dof
        return self.data.qvel[start+3:start+6]

    @property
    def active(self) -> bool:
        return self.phase != "IDLE"

    def reset(self) -> None:
        self.phase = "IDLE"
        self.action: dict[str, Any] | None = None
        self.started = 0.0
        self.phase_started = 0.0
        self.stable_since: float | None = None
        self.recovery_start = self.default_pose.copy()
        self.terminal_state = ""
        self.cancel_requested = False
        self.airborne_run = 0.0

    def available(self) -> list[str]:
        return self.profile.available(self.assets)

    def _foot_contacts(self) -> set[int]:
        contacts: set[int] = set()
        foot_geoms = set(self.controller.foot_geoms)
        for contact in self.data.contact:
            pair = {contact.geom1, contact.geom2}
            contacts.update(pair & foot_geoms)
        return contacts

    def _precondition_error(self, name: str) -> str | None:
        if self.controller.estopped:
            return "emergency stop is active; reset before starting a named action"
        if name not in self.profile.data.get("actions", {}):
            return f"action {name!r} is not configured for this environment"
        if name not in self.available():
            return f"action {name!r} is unavailable; install its pinned assets"
        q = self.controller.base_qpos
        zone_error = self.profile.check_zone(
            name, self.data.qpos[q:q+3], yaw_from_quaternion(self.data.qpos[q+3:q+7]))
        if zone_error:
            return zone_error
        requirements = self.profile.data["preconditions"]
        upright = float(self.data.xmat[self.controller.base, 8])
        height = float(self.data.qpos[q+2])
        linear = float(np.linalg.norm(self.data.qvel[self.controller.base_dof:self.controller.base_dof+3]))
        angular = float(np.linalg.norm(self.base_angular_velocity))
        joint_speed = float(np.max(np.abs(self.joint_velocity)))
        checks = (
            (upright >= requirements["minimumUprightCosine"], "robot is not upright"),
            (height >= requirements["minimumBaseHeightM"], "base is too low"),
            (height <= requirements["maximumBaseHeightM"], "base is too high"),
            (linear <= requirements["maximumLinearSpeedMps"], "robot is still translating"),
            (angular <= requirements["maximumAngularSpeedRadS"], "robot is still rotating"),
            (joint_speed <= requirements["maximumJointSpeedRadS"], "leg joints are still moving"),
            (len(self._foot_contacts()) >= requirements["requiredFootContacts"],
             "required foot contacts are not established"),
        )
        for passed, detail in checks:
            if not passed:
                return detail
        if self.stable_since is None or float(self.data.time)-self.stable_since < requirements["settleTimeS"]:
            return "robot has not remained stable for the configured settle time"
        return None

    def observe_idle(self) -> None:
        if not self.profile.configured or self.active:
            return
        requirements = self.profile.data["preconditions"]
        speed = np.linalg.norm(self.data.qvel[self.controller.base_dof:self.controller.base_dof+3])
        angular = np.linalg.norm(self.base_angular_velocity)
        joint_speed = np.max(np.abs(self.joint_velocity))
        stable = (self.data.xmat[self.controller.base, 8] >= requirements["minimumUprightCosine"]
                  and speed <= requirements["maximumLinearSpeedMps"]
                  and angular <= requirements["maximumAngularSpeedRadS"]
                  and joint_speed <= requirements["maximumJointSpeedRadS"]
                  and len(self._foot_contacts()) >= requirements["requiredFootContacts"])
        if stable and self.stable_since is None:
            self.stable_since = float(self.data.time)
        elif not stable:
            self.stable_since = None

    def start(self, name: str, action_id: str) -> tuple[bool, str | None]:
        if self.active:
            return False, "another named action owns the controller"
        if not isinstance(action_id, str) or not 1 <= len(action_id) <= 128:
            return False, "action id must contain 1 to 128 characters"
        error = self._precondition_error(name)
        if error:
            return False, error
        if name == "backflip":
            if name not in self.runners:
                self.runners[name] = BackflipRunner(self)
            runner = self.runners[name]
            runner.reset()
        elif name == "handstand_walk":
            if name not in self.runners:
                self.runners[name] = HandstandRunner(self)
            runner = self.runners[name]
            runner.reset()
        now = float(self.data.time)
        self.controller.twist[:] = 0
        self.controller.last_twist_time = -math.inf
        self.controller.session = None
        self.controller.policy_status.update(loaded=False, error=None)
        self.phase = "RUNNING"
        self.started = self.phase_started = now
        self.cancel_requested = False
        self.terminal_state = ""
        self.airborne_run = 0.0
        self.action = {
            "id": action_id, "name": name, "state": "RUNNING", "phase": "EXECUTE",
            "detail": "action owns motor control", "elapsedS": 0.0,
            "maxHeightM": float(self.data.xpos[self.controller.base, 2]),
            "airborneS": 0.0, "obstacleContacts": 0,
            "nonfootGroundContacts": 0, "pitchRotationRad": 0.0,
            "handstandS": 0.0,
        }
        self.origin = self.data.xpos[self.controller.base].copy()
        self.action_start_joints = self.joint_position.copy()
        self.initial_foot_x = [float(self.data.xpos[body, 0]) for body in self.controller.foot_bodies]
        return True, None

    def cancel(self, action_id: str) -> tuple[bool, str | None]:
        if not self.active or self.action is None or self.action["id"] != action_id:
            return False, "action id does not match the active action"
        if self.phase == "HOLD":
            return False, "terminal action must be released"
        self.cancel_requested = True
        self._begin_recovery("CANCELED", "cancellation requested")
        return True, None

    def release(self, action_id: str) -> tuple[bool, str | None]:
        if self.phase != "HOLD" or self.action is None or self.action["id"] != action_id:
            return False, "only the exact terminal action can be released"
        self.phase = "IDLE"
        self.stable_since = None
        self.action.update(phase="RELEASED", detail=f"{self.action['state'].lower()}; normal control restored")
        self.controller.targets[:] = self.joint_position
        self.controller.kp[:] = 55.0
        self.controller.kd[:] = 2.0
        self.controller.feedforward[:] = 0.0
        self.controller._begin_transition()
        return True, None

    def emergency_stop(self) -> None:
        if self.phase in ("RUNNING", "RECOVERING"):
            self._begin_recovery("CANCELED", "emergency stop requested")

    def _scripted_torque(self, name: str, elapsed: float) -> np.ndarray:
        target = self.default_pose.copy()
        kp, kd = 65.0, 2.0
        if elapsed < 1.0:
            blend = smooth(elapsed/.6)
            target = self.action_start_joints*(1-blend)+self.default_pose*blend
        elif name == "jump":
            local = elapsed-1.0
            squat = self.default_pose + np.tile([0.0, .35, -.70], 4)
            thrust = self.default_pose.copy()
            thrust[[1, 4]] -= .85
            thrust[[7, 10]] -= .10
            thrust[2::3] += .65
            tuck = self.default_pose + np.tile([0.0, .20, -.40], 4)
            if local < .5:
                target = self.default_pose*(1-smooth(local/.5))+squat*smooth(local/.5)
            elif local < .7:
                target = squat
            elif local < .90:
                target = thrust
                kp, kd = 90.0, 1.5
            elif local < 1.15:
                target = tuck
        elif name in ("crouch", "bow"):
            blend = smooth((elapsed-1)/.6)*(1-smooth((elapsed-2.2)/.6))
            pose = np.tile([0.0, 1.27, -2.54], 4) if name == "crouch" else self.default_pose.copy()
            if name == "bow":
                pose[[1, 4]] += .34
                pose[[2, 5]] -= .68
                pose[[7, 10]] -= .08
                pose[[8, 11]] += .16
            target = self.default_pose*(1-blend)+pose*blend
        elif name == "dance":
            envelope = smooth((elapsed-1)/.6)*(1-smooth((elapsed-5.8)/.6))
            phase = (elapsed-1)*2*np.pi*.65
            target[::3] += .20*np.sin(phase)*envelope
            target[1::3] += .18*np.sin(phase*2)*envelope
            target[2::3] -= .36*np.sin(phase*2)*envelope
        elif name in ("sway", "stretch"):
            envelope = smooth((elapsed-1)/.6)*(1-smooth((elapsed-4.8)/.6))
            wave = np.sin((elapsed-1)*2*np.pi*.5)*envelope
            if name == "sway":
                target[::3] += .26*wave
            else:
                target[[1, 4]] += .10*wave
                target[[2, 5]] -= .20*wave
                target[[7, 10]] -= .10*wave
                target[[8, 11]] += .20*wave
        return kp*(target-self.joint_position)-kd*self.joint_velocity

    def _update_metrics(self, elapsed: float) -> None:
        assert self.action is not None
        dt = self.model.opt.timestep
        self.action["elapsedS"] = elapsed
        self.action["pitchRotationRad"] += float(self.data.qvel[self.controller.base_dof+4])*dt
        self.action["maxHeightM"] = max(self.action["maxHeightM"],
                                        float(self.data.xpos[self.controller.base, 2]))
        ground_contact = False
        foot_geoms = set(self.controller.foot_geoms)
        for contact in self.data.contact:
            pair = {contact.geom1, contact.geom2}
            names = {self.model.geom(index).name for index in pair}
            environment = any(self.model.geom_group[index] == 3 for index in pair)
            if environment:
                ground_contact = ground_contact or bool(pair & foot_geoms)
                if not pair & foot_geoms:
                    self.action["nonfootGroundContacts"] += 1
            if "yard_jump_hurdle_collision" in names:
                self.action["obstacleContacts"] += 1
        self.airborne_run = 0.0 if ground_contact else self.airborne_run+dt
        self.action["airborneS"] = max(self.action["airborneS"], self.airborne_run)
        if self.action["name"] == "handstand_walk":
            rear_height = min(float(self.data.xpos[body, 2]) for body in self.controller.foot_bodies[2:])
            if -self.base_rotation[2, 0] > .90 and rear_height > .4:
                self.action["handstandS"] += dt
            if elapsed >= 9.0 and "handstandOrigin" not in self.action:
                self.action["handstandOrigin"] = self.data.xpos[self.controller.base, :2].tolist()
            if elapsed >= 16.0 and "handstandTravelM" not in self.action and "handstandOrigin" in self.action:
                origin = np.asarray(self.action["handstandOrigin"])
                self.action["handstandTravelM"] = float(np.linalg.norm(
                    self.data.xpos[self.controller.base, :2]-origin))

    def _begin_recovery(self, terminal_state: str, detail: str) -> None:
        if self.phase == "RECOVERING":
            return
        self.phase = "RECOVERING"
        self.phase_started = float(self.data.time)
        self.recovery_start = self.joint_position.copy()
        self.terminal_state = terminal_state
        if self.action is not None:
            self.action.update(state="RECOVERING", phase="RECOVERY", detail=detail)

    def _complete_recovery(self, succeeded: bool, detail: str) -> None:
        assert self.action is not None
        if self.terminal_state == "SUCCEEDED":
            succeeded = (succeeded and self.action["nonfootGroundContacts"] == 0
                         and self.action["obstacleContacts"] == 0
                         and self.data.xmat[self.controller.base, 8] > .82
                         and self.data.qpos[self.controller.base_qpos+2] > .18)
            if self.action["name"] == "jump":
                succeeded = succeeded and self.action["airborneS"] > .05
            elif self.action["name"] == "backflip":
                succeeded = succeeded and abs(self.action["pitchRotationRad"]) > 5.8
            elif self.action["name"] == "handstand_walk":
                succeeded = (succeeded and self.action["handstandS"] > 4.0
                             and self.action.get("handstandTravelM", 0.0) > .4)
        state = self.terminal_state or ("SUCCEEDED" if succeeded else "FAILED")
        if state == "SUCCEEDED" and not succeeded:
            state = "FAILED"
            detail = "action result checks failed; controller remains held"
        elif state != "SUCCEEDED":
            detail = f"{self.action.get('detail', state.lower())}; {detail}"
        self.phase = "HOLD"
        self.action.update(state=state, phase="HOLD", detail=detail,
                           displacementM=(self.data.xpos[self.controller.base]-self.origin).tolist(),
                           upright=float(self.data.xmat[self.controller.base, 8]))
        feet = [float(self.data.xpos[body, 0]) for body in self.controller.foot_bodies]
        hurdle = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM,
                                   "yard_jump_hurdle_collision")
        if hurdle >= 0:
            x = float(self.model.geom_pos[hurdle, 0])
            half = float(self.model.geom_size[hurdle, 0])
            self.action["crossedHurdle"] = bool(
                max(self.initial_foot_x) < x-half and min(feet) > x+half
                and self.action["obstacleContacts"] == 0 and self.action["airborneS"] > .05)

    def step(self) -> None:
        assert self.action is not None
        now = float(self.data.time)
        elapsed = now-self.started
        try:
            if self.phase == "RUNNING":
                name = self.action["name"]
                q = self.controller.base_qpos
                envelope_error = self.profile.check_zone(
                    name, self.data.qpos[q:q+3],
                    yaw_from_quaternion(self.data.qpos[q+3:q+7]), execution=True)
                if envelope_error:
                    self._begin_recovery("FAILED", envelope_error)
                else:
                    if name in SCRIPTED_ACTIONS:
                        torque = self._scripted_torque(name, elapsed)
                    else:
                        torque = self.runners[name].torque(elapsed)
                    self._update_metrics(elapsed)
                    duration = float(self.profile.data["actions"][name]["durationS"])
                    if elapsed >= duration:
                        self.terminal_state = "SUCCEEDED"
                        self._begin_recovery("SUCCEEDED", "action complete; returning to stand")
            if self.phase in ("RECOVERING", "HOLD"):
                recovery = self.profile.data["recovery"]
                if self.phase == "RECOVERING":
                    phase_elapsed = now-self.phase_started
                    blend = smooth(phase_elapsed/float(recovery["blendTimeS"]))
                    target = self.recovery_start*(1-blend)+self.default_pose*blend
                else:
                    phase_elapsed = 0.0
                    target = self.default_pose
                torque = 55*(target-self.joint_position)-2*self.joint_velocity
                if self.phase == "RECOVERING":
                    stable = (self.data.xmat[self.controller.base, 8] >= recovery["minimumUprightCosine"]
                              and self.data.qpos[self.controller.base_qpos+2] >= recovery["minimumBaseHeightM"]
                              and np.max(np.abs(self.joint_velocity)) <= recovery["maximumJointSpeedRadS"])
                    if stable and phase_elapsed >= recovery["blendTimeS"]+recovery["settleTimeS"]:
                        self._complete_recovery(True, "recovery complete; release required")
                    elif phase_elapsed >= recovery["timeoutS"]:
                        self._complete_recovery(False, "recovery timeout; controller remains held")
            limits = self.model.actuator_ctrlrange[self.controller.actuators]
            self.data.ctrl[self.controller.actuators] = np.clip(torque, limits[:, 0], limits[:, 1])
            if not np.isfinite(self.data.ctrl[self.controller.actuators]).all():
                raise RuntimeError("named action produced nonfinite motor torque")
        except Exception as error:
            self.terminal_state = "FAILED"
            if self.phase != "RECOVERING":
                self._begin_recovery("FAILED", str(error))
            limits = self.model.actuator_ctrlrange[self.controller.actuators]
            torque = 45*(self.default_pose-self.joint_position)-2*self.joint_velocity
            self.data.ctrl[self.controller.actuators] = np.clip(torque, limits[:, 0], limits[:, 1])

    def status(self) -> dict | None:
        return None if self.action is None else dict(self.action)
