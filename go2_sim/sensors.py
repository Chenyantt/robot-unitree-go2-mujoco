# SPDX-License-Identifier: Apache-2.0
"""Actual MuJoCo ray/RGB-D sensors with explicit virtual mounting frames."""
import base64
import math
import mujoco
import numpy as np


class Sensors:
    def __init__(self, model, data, camera=True):
        self.model, self.data = model, data
        self.lidar = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "sim_lidar")
        self.width, self.height = 320, 240
        self.renderer = mujoco.Renderer(model, self.height, self.width) if camera else None
        self.mask = np.array([1, 1, 0, 0, 1, 1], dtype=np.uint8)

    def scan(self):
        """Exclude the vendor robot visual/collision groups, not the room."""
        origin = self.data.site_xpos[self.lidar]
        rot = self.data.site_xmat[self.lidar].reshape(3, 3)
        ranges = []
        geom = np.zeros(1, dtype=np.int32)
        for angle in np.linspace(-math.pi, math.pi, 180, endpoint=False):
            direction = rot @ np.array([math.cos(angle), math.sin(angle), 0.])
            distance = mujoco.mj_ray(self.model, self.data, origin, direction, self.mask, 1, -1, geom)
            ranges.append(float(distance) if .05 <= distance < 10. else 10.)
        return {"angle_min": -math.pi, "angle_increment": 2*math.pi/180,
                "range_min": .05, "range_max": 10., "ranges": ranges,
                "frame": "sim_lidar", "time": float(self.data.time)}

    def camera(self):
        """Render RGB and metric float depth at the same frozen physics state."""
        if self.renderer is None:
            return None
        self.renderer.disable_depth_rendering()
        self.renderer.update_scene(self.data, camera="front_rgbd")
        rgb = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render().copy().astype("<f4")
        focal = self.height / (2 * math.tan(math.radians(60)/2))
        return {"width": self.width, "height": self.height, "fx": focal, "fy": focal,
                "cx": self.width/2, "cy": self.height/2, "frame": "sim_camera_optical",
                "time": float(self.data.time), "rgb": base64.b64encode(rgb.tobytes()).decode(),
                "depth": base64.b64encode(depth.tobytes()).decode()}

    def close(self):
        if self.renderer:
            self.renderer.close()
