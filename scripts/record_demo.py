#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Record actual torque-driven MuJoCo dynamics, no pose animation."""
import json
from pathlib import Path
import subprocess
import mujoco
from go2_sim.scene import load_scene
from go2_sim.controller import Go2Controller

ROOT = Path(__file__).resolve().parents[1]


def main():
    out = ROOT/".runtime"
    model = load_scene(out/"assets")
    data = mujoco.MjData(model)
    control = Go2Controller(model, data, out/"assets")
    camera = mujoco.MjvCamera()
    camera.distance, camera.azimuth, camera.elevation = 2.5, 135., -25.
    stages = [("stand", 2, [0, 0, 0]), ("forward", 5, [.4, 0, 0]),
              ("stop", 2, [0, 0, 0]), ("turn", 3, [0, 0, .5]),
              ("forward_after_turn", 3, [.3, 0, 0]), ("stop", 3, [0, 0, 0])]
    renderer = mujoco.Renderer(model, 480, 640)
    with subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                           "-s", "640x480", "-r", "25", "-i", "-", "-an", "-c:v", "libx264",
                           "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out/"walking-demo.mp4")],
                          stdin=subprocess.PIPE) as encoder:
        report = []
        try:
            for label, seconds, command in stages:
                before = control.state()
                for i in range(int(seconds/model.opt.timestep)):
                    control.step(command)
                    state = control.state()
                    assert state["upright"] > .85 and state["position"][2] > .2
                    if i % 8 == 0:
                        camera.lookat[:] = state["position"]
                        renderer.update_scene(data, camera=camera)
                        encoder.stdin.write(renderer.render().tobytes())
                report.append({"stage": label, "seconds": seconds, "command": command,
                               "start_position": before["position"], "end_position": control.state()["position"]})
        finally:
            encoder.stdin.close()
            renderer.close()
        assert encoder.wait() == 0
    (out/"walking-demo.json").write_text(json.dumps(report, indent=2)+"\n")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", "4", "-i", str(out/"walking-demo.mp4"),
                    "-frames:v", "1", str(out/"walking-demo.png")], check=True)
    print(out/"walking-demo.mp4")


if __name__ == "__main__":
    main()
