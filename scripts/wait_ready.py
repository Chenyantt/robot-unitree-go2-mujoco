# SPDX-License-Identifier: Apache-2.0
"""Bounded readiness wait; avoids connecting before native model loading completes."""
import json
import os
import time
import urllib.request

deadline = time.monotonic()+30.
while time.monotonic() < deadline:
    try:
        with urllib.request.urlopen("http://127.0.0.1:"+os.environ.get("GO2_SIM_PORT", "18765")+"/health", timeout=.5) as response:
            state = json.load(response)
            expected = os.environ.get("GO2_SIM_EXPECTED_PID")
            if expected and state.get("pid") != int(expected):
                raise SystemExit("Simulation port belongs to another process; existing instance left unchanged")
            if state["ready"]:
                break
    except OSError:
        pass
    time.sleep(.1)
else:
    raise SystemExit("Native Go2 simulation did not become ready within 30 seconds")
