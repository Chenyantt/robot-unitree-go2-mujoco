# SPDX-License-Identifier: Apache-2.0
# Observation/action semantics adapted from Ziqi Fan's rl_sar (Apache-2.0).
"""Named-joint MuJoCo adapter for the pinned rl_sar Go2 robot_lab policy."""

from pathlib import Path
import math
import mujoco
import numpy as np
import torch
import yaml


class Go2Controller:
    """Own all 12 simulated motors; advance motion through physics only."""

    def __init__(self, model, data, assets):
        self.model, self.data = model, data
        assets = Path(assets)
        self.config = yaml.safe_load((assets / "policy/config.yaml").read_text())["go2/robot_lab"]
        self.base_config = yaml.safe_load((assets / "policy/base.yaml").read_text())["go2"]
        torch.set_num_threads(1)
        self.policy = torch.jit.load(str(assets / "policy/policy.pt"), map_location="cpu").eval()
        self.names = self.base_config["joint_names"]
        self.jids = [self._id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.names]
        self.aids = [self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, name.removesuffix("_joint")) for name in self.names]
        self.qadr = model.jnt_qposadr[self.jids]
        self.vadr = model.jnt_dofadr[self.jids]
        self.bid = self._id(mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if model.nu != 12 or len(set(self.aids)) != 12:
            raise ValueError("Expected exactly 12 Go2 motors")
        if not np.array_equal(model.actuator_trnid[self.aids, 0], self.jids):
            raise ValueError("Joint/actuator transmission mismatch")
        self.default = np.asarray(self.config["default_dof_pos"])
        self.kp, self.kd = np.asarray(self.config["rl_kp"]), np.asarray(self.config["rl_kd"])
        self.scale = np.asarray(self.config["action_scale"])
        self.limits = np.minimum(self.config["torque_limits"], model.actuator_ctrlrange[self.aids, 1])
        model.opt.timestep = self.base_config["dt"]
        self.decimation = self.base_config["decimation"]
        self.reset()

    def _id(self, kind, name):
        value = mujoco.mj_name2id(self.model, kind, name)
        if value < 0:
            raise ValueError(f"Missing MuJoCo name: {name}")
        return value

    def reset(self):
        """Reset physics and policy history, the only direct pose initialization."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:7] = [0, 0, 0.34, 1, 0, 0, 0]
        self.data.qpos[self.qadr] = self.default
        self.data.ctrl[:] = 0
        self.actions = np.zeros(12)
        self.target = self.default.copy()
        self.steps = 0
        self.yaw_target = 0.
        mujoco.mj_forward(self.model, self.data)

    def observation(self, command):
        """Follow the upstream 45-element body-frame observation ordering."""
        rot = self.data.xmat[self.bid].reshape(3, 3)
        gyro = self.data.sensor("imu_gyro").data.copy()
        obs = np.concatenate((gyro * self.config["ang_vel_scale"],
                              rot.T @ np.array([0., 0., -1.]),
                              np.asarray(command) * self.config["commands_scale"],
                              (self.data.qpos[self.qadr] - self.default) * self.config["dof_pos_scale"],
                              self.data.qvel[self.vadr] * self.config["dof_vel_scale"],
                              self.actions))
        if obs.shape != (45,) or not np.isfinite(obs).all():
            raise ValueError("Invalid policy observation")
        return np.clip(obs, -self.config["clip_obs"], self.config["clip_obs"]).astype(np.float32)

    def step(self, command):
        """Infer at 50 Hz; apply bounded PD torque at the 200 Hz physics rate."""
        command = np.asarray(command, dtype=float)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError("Expected finite x/y/yaw velocity")
        self.yaw_target += float(command[2]) * self.model.opt.timestep
        if self.steps % self.decimation == 0:
            # This generic Go2 MJCF differs from the policy training plant.
            # Close yaw around measured heading to remove observed idle drift;
            # never alter qpos or the upstream policy weights to enforce motion.
            yaw = self.state()["yaw"]
            error = math.atan2(math.sin(self.yaw_target-yaw), math.cos(self.yaw_target-yaw))
            policy_command = command.copy()
            policy_command[2] = np.clip(command[2] + 2.0 * error, -1., 1.)
            with torch.inference_mode():
                raw = self.policy(torch.from_numpy(self.observation(policy_command)).unsqueeze(0))
            raw = raw.squeeze(0).numpy()
            if raw.shape != (12,) or not np.isfinite(raw).all():
                raise ValueError("Invalid policy action")
            self.actions = np.clip(raw, self.config["clip_actions_lower"], self.config["clip_actions_upper"])
            self.target = self.default + self.scale * self.actions
        torque = self.kp * (self.target - self.data.qpos[self.qadr]) - self.kd * self.data.qvel[self.vadr]
        self.data.ctrl[self.aids] = np.clip(torque, -self.limits, self.limits)
        mujoco.mj_step(self.model, self.data)
        self.steps += 1
        if not np.isfinite(self.data.qpos).all() or any(w.number for w in self.data.warning):
            raise RuntimeError("MuJoCo numerical failure")

    def state(self):
        """Return root-body pose and body-frame velocity, not the offset IMU pose."""
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self.bid, velocity, 1)
        q = self.data.xquat[self.bid].copy()
        yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
        transforms = []
        for bid in range(1, self.model.nbody):
            parent = int(self.model.body_parentid[bid])
            if parent == 0:
                continue
            parent_rot = self.data.xmat[parent].reshape(3, 3)
            rotation = parent_rot.T @ self.data.xmat[bid].reshape(3, 3)
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, rotation.flatten())
            transforms.append({"parent": self.model.body(parent).name, "child": self.model.body(bid).name,
                               "position": (parent_rot.T @ (self.data.xpos[bid]-self.data.xpos[parent])).tolist(),
                               "quaternion_wxyz": quat.tolist()})
        return {"time": float(self.data.time), "position": self.data.xpos[self.bid].tolist(),
                "quaternion_wxyz": q.tolist(), "yaw": yaw,
                "linear_velocity": velocity[3:].tolist(), "angular_velocity": velocity[:3].tolist(),
                "joint_names": self.names, "joint_positions": self.data.qpos[self.qadr].tolist(),
                "joint_velocities": self.data.qvel[self.vadr].tolist(),
                "torques": self.data.ctrl[self.aids].tolist(),
                "upright": float(self.data.xmat[self.bid].reshape(3, 3)[2, 2]), "transforms": transforms}
