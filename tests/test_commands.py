# SPDX-License-Identifier: Apache-2.0
import math
import unittest
from go2_sim.runtime import CommandState


class Commands(unittest.TestCase):
    def setUp(self):
        self.state = CommandState()

    def test_default_zero(self):
        self.assertEqual(self.state.sample(now=1.), [0., 0., 0.])

    def test_expiry(self):
        self.state.command({"owner": "a", "velocity": [.4, 0, 0]}, now=1.)
        self.assertEqual(self.state.sample(now=1.2), [.4, 0, 0])
        self.assertEqual(self.state.sample(now=1.4), [0, 0, 0])

    def test_limits(self):
        self.state.command({"owner": "a", "velocity": [8, -9, 4]}, now=1.)
        self.assertEqual(self.state.sample(now=1.1), [.5, -.3, .6])

    def test_mutual_exclusion(self):
        self.state.command({"owner": "a", "velocity": [.2, 0, 0]}, now=1.)
        with self.assertRaises(PermissionError):
            self.state.command({"owner": "b", "velocity": [.3, 0, 0]}, now=1.1)
        self.assertEqual(self.state.sample(now=1.2), [.2, 0, 0])

    def test_expired_takeover(self):
        self.state.command({"owner": "a", "velocity": [.2, 0, 0]}, now=1.)
        self.state.command({"owner": "b", "velocity": [.3, 0, 0]}, now=1.5)
        self.assertEqual(self.state.owner, "b")

    def test_stop_and_reset(self):
        for field in ("stop", "reset"):
            self.state.command({"owner": "a", "velocity": [.2, 0, 0]}, now=1.)
            self.state.command({field: True}, now=1.1)
            self.assertEqual(self.state.sample(now=1.2), [0, 0, 0])
            self.assertIsNone(self.state.owner)
        self.assertTrue(self.state.reset_requested)

    def test_invalid(self):
        for velocity in ([math.nan, 0, 0], [math.inf, 0, 0], [], [1, 2], None):
            with self.subTest(velocity=velocity), self.assertRaises((ValueError, TypeError)):
                self.state.command({"owner": "a", "velocity": velocity})
        for owner in (None, "", "a"*65, 42):
            with self.subTest(owner=owner), self.assertRaises(ValueError):
                self.state.command({"owner": owner, "velocity": [0, 0, 0]})

    def test_sample_copy(self):
        self.state.command({"owner": "a", "velocity": [.2, 0, 0]}, now=1.)
        sample = self.state.sample(now=1.1)
        sample[0] = 4
        self.assertEqual(self.state.sample(now=1.1), [.2, 0, 0])


if __name__ == "__main__":
    unittest.main()
