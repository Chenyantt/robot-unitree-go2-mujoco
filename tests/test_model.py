# SPDX-License-Identifier: Apache-2.0
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from go2_sim.scene import load_scene
from go2_sim.controller import Go2Controller
from go2_sim.sensors import Sensors

ROOT = Path(__file__).resolve().parents[1]


class Model(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = load_scene(ROOT/".runtime/assets")

    def setUp(self):
        self.data = mujoco.MjData(self.model)
        self.control = Go2Controller(self.model, self.data, ROOT/".runtime/assets")

    def test_named_motor_mapping(self):
        self.assertEqual(len(set(self.control.names)), 12)
        np.testing.assert_array_equal(self.model.actuator_trnid[self.control.aids, 0], self.control.jids)

    def test_observation(self):
        value = self.control.observation([0, 0, 0])
        self.assertEqual(value.shape, (45,))
        np.testing.assert_allclose(value[3:6], [0, 0, -1])

    def test_scan_geometry(self):
        sensor = Sensors(self.model, self.data, camera=False)
        scan = sensor.scan()
        self.assertEqual(len(scan["ranges"]), 180)
        # Forward ray intersects the x=5 wall (half-thickness 0.1 m).
        self.assertAlmostEqual(scan["ranges"][90], 4.9, places=2)
        sensor.close()

    def test_reset_policy_and_pose(self):
        for _ in range(80):
            self.control.step([.2, 0, .2])
        self.control.reset()
        self.assertEqual(self.control.steps, 0)
        self.assertEqual(self.data.time, 0.)
        np.testing.assert_allclose(self.data.qpos[:3], [0, 0, .34])
        np.testing.assert_array_equal(self.control.actions, np.zeros(12))

    def test_tf_tree_unique(self):
        edges = self.control.state()["transforms"]
        self.assertEqual(len({t["child"] for t in edges}), len(edges))
        for t in edges:
            self.assertAlmostEqual(np.linalg.norm(t["quaternion_wxyz"]), 1.)

    def test_all_assets_pinned(self):
        entries = json.loads((ROOT/"assets.lock.json").read_text())["files"]
        self.assertEqual(len(entries), 22)
        for e in entries:
            self.assertEqual(len(e["commit"]), 40)
            self.assertNotIn("..", Path(e["destination"]).parts)

    def test_description_tree(self):
        root = ET.parse(ROOT/".runtime/go2.urdf").getroot()
        links = {e.attrib["name"] for e in root.findall("link")}
        joints = root.findall("joint")
        self.assertEqual(sum(j.attrib["type"] == "revolute" for j in joints), 12)
        self.assertEqual(len(joints), len(links)-1)
        for j in joints:
            self.assertIn(j.find("parent").attrib["link"], links)
            self.assertIn(j.find("child").attrib["link"], links)


if __name__ == "__main__":
    unittest.main()
