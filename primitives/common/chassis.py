"""Go2 chassis primitive backed by simulated ROS topics."""
from __future__ import annotations

import json
import math
import os
import threading

from robonix_api import Primitive, Ok, Err

from .runtime import package_root, provider_id, topic_value

provider = Primitive(
    id=provider_id("go2_chassis"), namespace="robonix/primitive/chassis",
    pkg_root=package_root())
cmd_vel_pub = None
motion_lock = threading.Lock()
stopped = threading.Event()

import chassis_pb2  # noqa: E402
import std_msgs_pb2  # noqa: E402


@provider.grpc("robonix/primitive/chassis/move")
def move(request):
    """Execute a bounded velocity or distance burst and always publish a stop."""
    if cmd_vel_pub is None or stopped.is_set():
        raise RuntimeError("chassis is not active")
    from geometry_msgs.msg import Twist
    command = request.command
    linear_speed = float(os.environ.get("SIM_LINEAR_SPEED", "0.18"))
    angular_speed = float(os.environ.get("SIM_ANGULAR_SPEED", "0.55"))
    if not (math.isfinite(linear_speed) and 0 < linear_speed <= 0.25):
        raise RuntimeError("SIM_LINEAR_SPEED must be in (0, 0.25] m/s")
    if not (math.isfinite(angular_speed) and 0 < angular_speed <= 0.55):
        raise RuntimeError("SIM_ANGULAR_SPEED must be in (0, 0.55] rad/s")
    forward_m = float(getattr(command, "forward_m", 0.0))
    rotate_deg = float(getattr(command, "rotate_deg", 0.0))
    if not all(math.isfinite(float(value)) for value in (
            forward_m, rotate_deg, command.linear_x, command.linear_y,
            command.angular_z, command.duration_sec)):
        raise RuntimeError("motion values must be finite")
    velocity = Twist()
    if forward_m:
        velocity.linear.x = math.copysign(linear_speed, forward_m)
        duration = abs(forward_m) / linear_speed
    elif rotate_deg:
        velocity.angular.z = math.copysign(angular_speed, rotate_deg)
        duration = abs(math.radians(rotate_deg)) / angular_speed
    else:
        velocity.linear.x = float(command.linear_x)
        velocity.linear.y = float(command.linear_y)
        velocity.angular.z = float(command.angular_z)
        duration = max(0.05, float(command.duration_sec or 1.0))
    magnitude = math.hypot(velocity.linear.x, velocity.linear.y)
    if magnitude > 0.25:
        velocity.linear.x *= 0.25 / magnitude
        velocity.linear.y *= 0.25 / magnitude
    velocity.angular.z = max(-0.55, min(0.55, velocity.angular.z))
    if not math.isfinite(duration) or duration <= 0:
        raise RuntimeError("motion duration must be finite and positive")
    if not motion_lock.acquire(blocking=False):
        raise RuntimeError("another direct move is running")
    try:
        try:
            for _ in range(max(1, math.ceil(duration * 10))):
                if stopped.is_set():
                    raise RuntimeError("chassis stopped during move")
                cmd_vel_pub.publish(velocity)
                if stopped.wait(0.1):
                    raise RuntimeError("chassis stopped during move")
        finally:
            cmd_vel_pub.publish(Twist())
    finally:
        motion_lock.release()
    return chassis_pb2.ExecuteMoveCommand_Response(
        status=std_msgs_pb2.String(data=json.dumps({"status": "done", "duration_sec": duration})))


@provider.on_init
def initialize(config):
    """Bind capability metadata to the bridge-owned chassis topics."""
    global cmd_vel_pub
    from geometry_msgs.msg import Twist
    try:
        odom_topic = topic_value(config, "odom_topic", "/odom")
        command_topic = topic_value(config, "command_topic", "/cmd_vel")
    except ValueError as error:
        return Err(str(error))
    cmd_vel_pub = provider.create_publisher(
        "robonix/primitive/chassis/twist_in", topic=command_topic,
        msg_type=Twist, qos="reliable", declare=False)
    provider.declare_ros2_topic("robonix/primitive/chassis/twist_in", command_topic, qos="reliable")
    provider.declare_ros2_topic("robonix/primitive/chassis/odom", odom_topic, qos="reliable")
    return Ok()


@provider.on_activate
def activate():
    """Enable direct moves after initialization or reactivation."""
    stopped.clear()
    return Ok()


@provider.on_deactivate
def deactivate():
    """Interrupt direct motion and serialize the final stop after its last command."""
    stopped.set()
    with motion_lock:
        if cmd_vel_pub is not None:
            from geometry_msgs.msg import Twist
            cmd_vel_pub.publish(Twist())
    return Ok()


@provider.on_shutdown
def shutdown():
    """Stop the mobile base before unregistering its provider."""
    return deactivate()


if __name__ == "__main__":
    provider.run()
