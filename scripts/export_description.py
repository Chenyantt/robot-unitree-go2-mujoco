#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Export MJCF-derived kinematic/inertial URDF for Soma, not another physics model."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from go2_sim.scene import load_scene

ROOT = Path(__file__).resolve().parents[1]


def numbers(values):
    return " ".join(f"{float(v):.10g}" for v in values)


def rpy(q):
    w, x, y, z = map(float, q)
    return [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]


def main():
    model = load_scene(ROOT/".runtime/assets", room=False)
    urdf = ET.Element("robot", name="go2_mujoco")
    for bid in range(1, model.nbody):
        name = model.body(bid).name
        link = ET.SubElement(urdf, "link", name=name)
        if model.body_mass[bid] > 0:
            inertia = ET.SubElement(link, "inertial")
            ET.SubElement(inertia, "origin", xyz=numbers(model.body_ipos[bid]), rpy=numbers(rpy(model.body_iquat[bid])))
            ET.SubElement(inertia, "mass", value=str(model.body_mass[bid]))
            ix, iy, iz = map(str, model.body_inertia[bid])
            ET.SubElement(inertia, "inertia", ixx=ix, iyy=iy, izz=iz, ixy="0", ixz="0", iyz="0")
        parent = int(model.body_parentid[bid])
        if parent == 0:
            continue
        count = model.body_jntnum[bid]
        assert count in (0, 1), "Multi-joint body needs a serial URDF expansion"
        jid = int(model.body_jntadr[bid])
        if count:
            assert np.linalg.norm(model.jnt_pos[jid]) < 1e-8, "Nonzero joint anchor needs an extra URDF link"
            assert model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_HINGE
        joint = ET.SubElement(urdf, "joint", name=model.joint(jid).name if count else name+"_fixed",
                              type="revolute" if count else "fixed")
        ET.SubElement(joint, "parent", link=model.body(parent).name)
        ET.SubElement(joint, "child", link=name)
        ET.SubElement(joint, "origin", xyz=numbers(model.body_pos[bid]), rpy=numbers(rpy(model.body_quat[bid])))
        if count:
            ET.SubElement(joint, "axis", xyz=numbers(model.jnt_axis[jid]))
            ET.SubElement(joint, "limit", lower=str(model.jnt_range[jid, 0]), upper=str(model.jnt_range[jid, 1]),
                          effort="23.5", velocity="30")
    for name, xyz, angles in [("sim_lidar", [0, 0, .15], [0, 0, 0]),
                              ("sim_camera_optical", [.28, 0, .09], [-math.pi/2, 0, -math.pi/2]),
                              ("imu", [-.02557, 0, .04232], [0, 0, 0])]:
        ET.SubElement(urdf, "link", name=name)
        j = ET.SubElement(urdf, "joint", name=name+"_mount", type="fixed")
        ET.SubElement(j, "parent", link="base_link")
        ET.SubElement(j, "child", link=name)
        ET.SubElement(j, "origin", xyz=numbers(xyz), rpy=numbers(angles))
    ET.indent(urdf)
    output = ROOT/".runtime/go2.urdf"
    ET.ElementTree(urdf).write(output, encoding="unicode", xml_declaration=True)
    print(f"Exported {output}: {model.nbody-1} bodies, 12 motors")


if __name__ == "__main__":
    main()
