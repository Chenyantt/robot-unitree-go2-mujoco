#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Exercise simulation ROS feedback, movement, lease expiry, stop and reset."""
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from go2_sim.bridge import request


def main():
    assert os.environ.get("ROS_DOMAIN_ID") == "141"
    assert os.environ.get("ROS_LOCALHOST_ONLY") == "1"
    rclpy.init()
    node = rclpy.create_node("go2_sim_acceptance")
    samples, counts = {}, {}
    types = {"/clock": Clock, "/go2_sim/odom": Odometry, "/go2_sim/joint_states": JointState,
             "/go2_sim/imu": Imu, "/go2_sim/scan": LaserScan, "/tf": TFMessage,
             "/go2_sim/camera/color/image_raw": Image, "/go2_sim/camera/depth/image_raw": Image,
             "/go2_sim/camera/camera_info": CameraInfo}
    def callback(topic, message):
        samples[topic] = message
        counts[topic] = counts.get(topic, 0)+1
    for topic, typ in types.items():
        node.create_subscription(typ, topic, lambda m, t=topic: callback(t, m), qos_profile_sensor_data)
    publisher = node.create_publisher(Twist, "/go2_sim/cmd_vel", 10)
    def spin(seconds, velocity=None):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if velocity is not None:
                msg = Twist()
                msg.linear.x, msg.linear.y, msg.angular.z = map(float, velocity)
                publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=.04)
    def service(name):
        client = node.create_client(Trigger, "/go2_sim/"+name)
        assert client.wait_for_service(timeout_sec=3.), name
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
        assert future.done() and future.result().success, name
        node.destroy_client(client)
    try:
        spin(3.)
        assert set(samples) == set(types), set(types)-set(samples)
        assert all(counts[k] >= 3 for k in types), counts
        rgb, depth = (samples["/go2_sim/camera/"+k+"/image_raw"] for k in ("color", "depth"))
        assert rgb.encoding == "rgb8" and len(rgb.data) == rgb.width*rgb.height*3
        assert depth.encoding == "32FC1" and len(depth.data) == depth.width*depth.height*4
        assert rgb.header.frame_id == depth.header.frame_id == "sim_camera_optical"
        assert len(samples["/go2_sim/joint_states"].name) == 12
        assert len(samples["/go2_sim/scan"].ranges) == 180
        assert all(math.isfinite(v) and .05 <= v <= 10 for v in samples["/go2_sim/scan"].ranges)
        before = request("/state")
        spin(3., [.3, 0, 0])
        moving = request("/state")
        distance = math.dist(before["position"][:2], moving["position"][:2])
        assert distance > .35, distance
        # No more commands: native lease must expire without asking for a stop.
        spin(2.)
        stopped = request("/state")
        speed = math.hypot(*stopped["linear_velocity"][:2])
        assert stopped["command"] == [0., 0., 0.] and speed < .08, stopped
        service("stop")
        epoch = stopped["epoch"]
        service("reset")
        spin(2.)
        reset = request("/state")
        assert reset["epoch"] == epoch+1 and math.hypot(*reset["position"][:2]) < .10
        result = {"passed": True, "sample_counts": counts, "forward_distance_m": distance,
                  "watchdog_stop_speed_m_s": speed, "reset_epoch": reset["epoch"],
                  "rgb_size": [rgb.width, rgb.height], "depth_encoding": depth.encoding}
        path = Path(__file__).resolve().parents[1]/".runtime/ros-acceptance.json"
        path.write_text(json.dumps(result, indent=2)+"\n")
        print(json.dumps(result, indent=2))
    finally:
        try:
            request("/command", {"stop": True})
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
