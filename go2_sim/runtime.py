# SPDX-License-Identifier: Apache-2.0
"""Native simulation, local HTTP controls and web preview; no hardware transport."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import threading
import time
import numpy as np
import mujoco

from .controller import Go2Controller
from .scene import load_scene
from .sensors import Sensors

ROOT = Path(__file__).resolve().parents[1]


class CommandState:
    """Single active velocity writer with an expiring lease; reset clears it."""
    def __init__(self):
        self.lock = threading.RLock()
        self.owner = None
        self.deadline = 0.
        self.velocity = [0., 0., 0.]
        self.reset_requested = False
        self.shutdown_requested = False
        self.epoch = 0
        self.latest = None
        self.latest_time = 0.
        self.camera = None

    def command(self, message, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if any(message.get(k) is True for k in ("stop", "reset", "shutdown")):
                self.velocity, self.owner, self.deadline = [0., 0., 0.], None, 0.
                self.reset_requested |= message.get("reset") is True
                self.shutdown_requested |= message.get("shutdown") is True
                return {"accepted": True}
            owner = message.get("owner")
            if not isinstance(owner, str) or not owner or len(owner) > 64:
                raise ValueError("A short owner string is required")
            v = np.asarray(message.get("velocity"), dtype=float)
            if v.shape != (3,) or not np.isfinite(v).all():
                raise ValueError("velocity must be three finite values [x,y,yaw]")
            if self.owner not in (None, owner) and now < self.deadline:
                raise PermissionError("Another active writer owns the velocity lease")
            self.owner = owner
            self.velocity = np.clip(v, [-.5, -.3, -.6], [.5, .3, .6]).tolist()
            self.deadline = now + .4
            return {"accepted": True, "velocity": self.velocity}

    def sample(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            return self.velocity.copy() if now < self.deadline else [0., 0., 0.]


def handler(state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, status, data, kind="application/json"):
            payload = json.dumps(data, allow_nan=False).encode() if kind == "application/json" else data
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass  # Reader closed its preview; physics is unaffected.

        def do_GET(self):
            with state.lock:
                if self.path in ("/state", "/health"):
                    age = time.monotonic() - state.latest_time
                    value = dict(state.latest or {})
                    value.update({"ready": state.latest is not None and age < 1., "age_s": age,
                                  "backend": "native", "robot": "go2", "pid": os.getpid(),
                                  "epoch": state.epoch, "command": state.sample()})
                    status, value, kind = 200 if value["ready"] else 503, value, "application/json"
                elif self.path == "/camera":
                    status, value, kind = 200 if state.camera else 503, state.camera or {}, "application/json"
                elif self.path in ("/", "/web"):
                    status, value, kind = 200, (ROOT / "web/index.html").read_bytes(), "text/html; charset=utf-8"
                else:
                    status, value, kind = 404, {"error": "not found"}, "application/json"
            # Never block physics while a slow HTTP reader drains its socket.
            self.reply(status, value, kind)

        def do_POST(self):
            if self.path != "/command":
                return self.reply(404, {"error": "not found"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096 or self.headers.get_content_type() != "application/json":
                    raise ValueError("Expected bounded application/json body")
                message = json.loads(self.rfile.read(length))
                if not isinstance(message, dict):
                    raise ValueError("Expected JSON object")
                self.reply(200, state.command(message))
            except PermissionError as error:
                self.reply(409, {"error": str(error)})
            except (ValueError, TypeError) as error:
                self.reply(400, {"error": str(error)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=18765)
    parser.add_argument("--no-camera", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--duration", type=float, default=0.)
    args = parser.parse_args()
    state = CommandState()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler(state))
    model = load_scene(ROOT / ".runtime/assets")
    data = mujoco.MjData(model)
    control = Go2Controller(model, data, ROOT / ".runtime/assets")
    sensors = Sensors(model, data, camera=not args.no_camera)
    stopping = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.set())
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    viewer = None
    if args.viewer:
        from mujoco import viewer as mjviewer
        viewer = mjviewer.launch_passive(model, data)
    start = time.monotonic()
    next_step = start
    tick = 0
    try:
        while not stopping.is_set() and not state.shutdown_requested and (not args.duration or time.monotonic()-start < args.duration):
            if viewer is not None and not viewer.is_running():
                break
            with state.lock:
                if state.reset_requested:
                    control.reset()
                    state.reset_requested = False
                    state.latest, state.camera = None, None
                    state.epoch += 1
            control.step(state.sample())
            if tick % 10 == 0:
                latest = control.state()
                latest["scan"] = sensors.scan()
                latest["imu_gyro"] = data.sensor("imu_gyro").data.tolist()
                latest["imu_acc"] = data.sensor("imu_acc").data.tolist()
                with state.lock:
                    state.latest, state.latest_time = latest, time.monotonic()
            if tick % 40 == 0 and sensors.renderer:
                camera = sensors.camera()
                with state.lock:
                    state.camera = camera
            if viewer and tick % 4 == 0:
                viewer.sync()
            if tick == 0:
                print(f"READY native Go2 simulation http://127.0.0.1:{args.port}/web", flush=True)
            tick += 1
            next_step += model.opt.timestep
            delay = next_step-time.monotonic()
            if delay > 0:
                stopping.wait(delay)
            elif delay < -.2:
                next_step = time.monotonic()
    finally:
        state.command({"stop": True})
        for _ in range(400):
            control.step([0, 0, 0])
        if viewer:
            viewer.close()
        sensors.close()
        server.shutdown()
        server.server_close()
        print("STOPPED simulation only", flush=True)


if __name__ == "__main__":
    main()
