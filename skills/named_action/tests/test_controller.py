from __future__ import annotations

import unittest
from unittest.mock import Mock, patch

from skills.named_action.named_action_skill.controller import NamedActionController


class FakeController(NamedActionController):
    def __init__(self, failures: list[str]) -> None:
        super().__init__("http://127.0.0.1:8766", .001, 1.0, .1)
        self.failures = list(failures)
        self.start_attempts = 0

    def list_actions(self) -> dict:
        return {"names": ["jump"], "environment": "yard", "controller_mode": "BASE_GAIT"}

    def _request(self, method: str, path: str, body: dict | None = None) -> dict:
        if method == "POST" and path == "/command" and body and body.get("operation") == "start":
            self.start_attempts += 1
            if self.failures:
                raise RuntimeError(self.failures.pop(0))
            return {"ok": True, "accepted": True}
        raise AssertionError((method, path, body))


class NamedActionAdmissionTests(unittest.TestCase):
    @patch("skills.named_action.named_action_skill.controller.threading.Thread")
    def test_transient_post_navigation_motion_is_retried(self, thread: Mock) -> None:
        controller = FakeController([
            "leg joints are still moving",
            "robot has not remained stable for the configured settle time",
        ])
        task = controller.start("jump", 30)
        self.assertEqual(task.action, "jump")
        self.assertEqual(controller.start_attempts, 3)
        thread.assert_called_once()
        thread.return_value.start.assert_called_once()

    @patch("skills.named_action.named_action_skill.controller.threading.Thread")
    def test_nontransient_admission_error_is_not_retried(self, thread: Mock) -> None:
        controller = FakeController(["robot is outside safe zone jump_start"])
        with self.assertRaisesRegex(RuntimeError, "outside safe zone"):
            controller.start("jump", 30)
        self.assertEqual(controller.start_attempts, 1)
        thread.assert_not_called()


if __name__ == "__main__":
    unittest.main()
