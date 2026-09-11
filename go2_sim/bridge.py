# SPDX-License-Identifier: Apache-2.0
"""ROS 2 bridge for the localhost simulator, isolated from real Go2 DDS."""
import base64
import json
import math
import os
import urllib.request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

URL = "http://127.0.0.1:" + os.environ.get("GO2_SIM_PORT", "18765")


def request(path, payload=None):
    req = urllib.request.Request(URL+path, data=None if payload is None else json.dumps(payload).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=.25) as response:
        return json.load(response)


def stamp(value):
    seconds = int(value)
    return Time(sec=seconds, nanosec=int((value-seconds)*1e9))


def transform(parent, child, position, quat, when):
    result = TransformStamped()
    result.header.stamp, result.header.frame_id, result.child_frame_id = when, parent, child
    result.transform.translation.x, result.transform.translation.y, result.transform.translation.z = map(float, position)
    q = result.transform.rotation
    q.w, q.x, q.y, q.z = map(float, quat)
    return result


class Bridge(Node):
    def __init__(self):
        super().__init__("go2_mujoco_bridge")
        self.clock = self.create_publisher(Clock, "/clock", 10)
        self.odom = self.create_publisher(Odometry, "/go2_sim/odom", 10)
        self.joints = self.create_publisher(JointState, "/go2_sim/joint_states", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/go2_sim/scan", qos_profile_sensor_data)
        self.imu = self.create_publisher(Imu, "/go2_sim/imu", qos_profile_sensor_data)
        self.rgb = self.create_publisher(Image, "/go2_sim/camera/color/image_raw", qos_profile_sensor_data)
        self.depth = self.create_publisher(Image, "/go2_sim/camera/depth/image_raw", qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.info = self.create_publisher(CameraInfo, "/go2_sim/camera/camera_info", latched)
        self.extrinsics = self.create_publisher(TransformStamped, "/go2_sim/camera/extrinsics", latched)
        self.extrinsics.publish(transform("base_link", "sim_camera_optical", [.28, 0, .09],
                                          [.5, -.5, .5, -.5], Time()))
        self.status = self.create_publisher(String, "/go2_sim/status", 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform([
            transform("base_link", "sim_lidar", [0, 0, .15], [1, 0, 0, 0], Time()),
            transform("base_link", "sim_camera_optical", [.28, 0, .09], [.5, -.5, .5, -.5], Time()),
            transform("base_link", "imu", [-.02557, 0, .04232], [1, 0, 0, 0], Time())])
        self.create_subscription(Twist, "/go2_sim/cmd_vel", self.command, 10)
        self.create_service(Trigger, "/go2_sim/stop", self.stop)
        self.create_service(Trigger, "/go2_sim/reset", self.reset)
        self.last_state = None
        self.last_camera = None
        self.failures = 0
        self.create_timer(.05, self.poll)
        self.create_timer(.2, self.poll_camera)

    def command(self, msg):
        try:
            request("/command", {"owner": "ros", "velocity": [msg.linear.x, msg.linear.y, msg.angular.z]})
        except Exception as error:
            self.get_logger().warning(f"Simulation command rejected: {error}")

    def stop(self, _, response):
        try:
            request("/command", {"stop": True})
            response.success, response.message = True, "Simulation target cleared"
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def reset(self, _, response):
        try:
            request("/command", {"reset": True})
            response.success, response.message = True, "Simulation reset requested"
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def poll(self):
        """Publish only advancing real simulation samples, never stale constants."""
        try:
            state = request("/state")
            if not state["ready"]:
                raise RuntimeError("Simulation state expired")
            key = (state["epoch"], state["time"])
            if key == self.last_state:
                return
            self.last_state = key
            when = stamp(state["time"])
            self.clock.publish(Clock(clock=when))
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = when, "odom", "base_link"
            t = transform("odom", "base_link", state["position"], state["quaternion_wxyz"], when)
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = state["position"]
            odom.pose.pose.orientation = t.transform.rotation
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = state["linear_velocity"]
            odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = state["angular_velocity"]
            self.odom.publish(odom)
            self.tf.sendTransform([t] + [transform(v["parent"], v["child"], v["position"],
                                                  v["quaternion_wxyz"], when) for v in state["transforms"]])
            joint = JointState()
            joint.header.stamp = when
            joint.name, joint.position = state["joint_names"], state["joint_positions"]
            joint.velocity, joint.effort = state["joint_velocities"], state["torques"]
            self.joints.publish(joint)
            laser = LaserScan()
            laser.header.stamp, laser.header.frame_id = when, "sim_lidar"
            for field in ("angle_min", "angle_increment", "range_min", "range_max", "ranges"):
                setattr(laser, field, state["scan"][field])
            laser.angle_max = laser.angle_min + (len(laser.ranges)-1)*laser.angle_increment
            laser.scan_time = .05
            self.scan_pub.publish(laser)
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = when, "imu"
            imu.orientation = t.transform.rotation
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = state["imu_gyro"]
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = state["imu_acc"]
            self.imu.publish(imu)
            self.status.publish(String(data=json.dumps({"ready": True, "epoch": key[0], "time": key[1]})))
            self.failures = 0
        except Exception as error:
            self.failures += 1
            if self.failures == 1:
                self.get_logger().warning(f"Simulation feedback unavailable: {error}")
            self.status.publish(String(data=json.dumps({"ready": False, "error": str(error)})))

    def poll_camera(self):
        try:
            value = request("/camera")
            if value["time"] == self.last_camera:
                return
            self.last_camera = value["time"]
            when = stamp(value["time"])
            for publisher, field, encoding, stride in ((self.rgb, "rgb", "rgb8", 3),
                                                       (self.depth, "depth", "32FC1", 4)):
                msg = Image()
                msg.header.stamp, msg.header.frame_id = when, value["frame"]
                msg.width, msg.height = value["width"], value["height"]
                msg.encoding, msg.step = encoding, msg.width*stride
                msg.data = base64.b64decode(value[field])
                publisher.publish(msg)
            info = CameraInfo()
            info.header.stamp, info.header.frame_id = when, value["frame"]
            info.width, info.height = value["width"], value["height"]
            info.distortion_model, info.d = "plumb_bob", [0.]*5
            fx, fy, cx, cy = (value[k] for k in ("fx", "fy", "cx", "cy"))
            info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
            info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
            self.info.publish(info)
            self.extrinsics.publish(transform("base_link", value["frame"], [.28, 0, .09],
                                              [.5, -.5, .5, -.5], when))
        except Exception:
            # No fabricated frames; /status and native health expose availability.
            pass


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "141" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("Run sim/start.sh: this package uses localhost-only ROS domain 141")
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            request("/command", {"stop": True})
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
