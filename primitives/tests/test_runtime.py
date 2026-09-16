"""Offline chassis and configuration smoke tests; no ROS or motion publisher is started."""
import importlib
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from primitives.common.runtime import timeout_value


class PrimitiveStub:
    """Keep production handlers callable without registering with Atlas."""

    def __init__(self, **kwargs):
        self.id = kwargs["id"]

    def grpc(self, _contract):
        return lambda function: function

    def __getattr__(self, name):
        if name.startswith("on_"):
            return lambda function: function
        raise AttributeError(name)


class Twist:
    """Minimal mutable message used to observe chassis commands offline."""

    def __init__(self):
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


def load_chassis():
    """Import the real chassis implementation with only external transports replaced."""
    modules = {}
    definitions = {
        "robonix_api": dict(Primitive=PrimitiveStub, Ok=lambda: None, Err=str),
        "chassis_pb2": dict(ExecuteMoveCommand_Response=SimpleNamespace),
        "std_msgs_pb2": dict(String=SimpleNamespace),
        "geometry_msgs.msg": dict(Twist=Twist),
    }
    for name, attributes in definitions.items():
        modules[name] = ModuleType(name)
        vars(modules[name]).update(attributes)
    with patch.dict(sys.modules, modules):
        module = importlib.import_module("primitives.common.chassis")
    return module, modules


class RuntimeTests(unittest.TestCase):
    """Verify limits, failure stops and lifecycle rejection using local message capture."""

    def setUp(self):
        """Reset handler state and isolate environment speed settings for each test."""
        self.chassis, modules = load_chassis()
        self.modules = patch.dict(sys.modules, modules)
        self.modules.start()
        self.addCleanup(self.modules.stop)
        self.environment = patch.dict("os.environ", SIM_LINEAR_SPEED="0.18", SIM_ANGULAR_SPEED="0.55")
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.messages = []
        self.chassis.cmd_vel_pub = SimpleNamespace(publish=self.messages.append)
        self.chassis.activate()

    def request(self, **values):
        """Build a complete existing Move command with explicit velocity defaults."""
        command = dict(forward_m=0.0, rotate_deg=0.0, linear_x=0.0,
                       linear_y=0.0, angular_z=0.0, duration_sec=0.01)
        command.update(values)
        return SimpleNamespace(command=SimpleNamespace(**command))

    def test_velocity_limits_and_final_stop(self):
        """Diagonal commands obey the vector speed limit and finish with zero velocity."""
        self.chassis.move(self.request(linear_x=1.0, linear_y=1.0, angular_z=2.0))
        motion, stop = self.messages
        self.assertAlmostEqual(math.hypot(motion.linear.x, motion.linear.y), 0.25)
        self.assertEqual(motion.angular.z, 0.55)
        self.assertEqual((stop.linear.x, stop.linear.y, stop.angular.z), (0.0, 0.0, 0.0))

    def test_failure_still_stops(self):
        """An interrupted wait propagates its error only after publishing the final stop."""
        with patch.object(self.chassis.stopped, "wait", side_effect=RuntimeError("interrupted")):
            with self.assertRaisesRegex(RuntimeError, "interrupted"):
                self.chassis.move(self.request(linear_x=0.18))
        self.assertEqual(self.messages[-1].linear.x, 0.0)
        self.assertFalse(self.chassis.motion_lock.locked())

    def test_deactivated_rejects_motion(self):
        self.chassis.deactivate()
        with self.assertRaisesRegex(RuntimeError, "not active"):
            self.chassis.move(self.request(linear_x=0.18))
        self.assertEqual(len(self.messages), 1)

    def test_nonfinite_motion_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "finite"):
            self.chassis.move(self.request(linear_x=float("nan")))
        self.assertEqual(self.messages, [])

    def test_timeout_validation(self):
        """Sensor readiness cannot use NaN, infinity, or a nonpositive deadline."""
        for value in (float("nan"), float("inf"), 0, -1):
            with self.subTest(value=value), self.assertRaises(ValueError):
                timeout_value({"sentinel_timeout_s": value})
        self.assertEqual(timeout_value({"sentinel_timeout_s": 90}), 90)


if __name__ == "__main__":
    unittest.main()
