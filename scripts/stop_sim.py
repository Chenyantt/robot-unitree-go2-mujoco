# SPDX-License-Identifier: Apache-2.0
"""Stop this loopback Native runtime through its own API, not process-name kills."""
import json
import os
import urllib.error
import urllib.request

url = "http://127.0.0.1:"+os.environ.get("GO2_SIM_PORT", "18765")
try:
    with urllib.request.urlopen(url+"/health", timeout=2.) as response:
        value = json.load(response)
except urllib.error.HTTPError as error:
    if error.code != 503:
        raise SystemExit(f"Unexpected endpoint response: {error}")
    value = json.load(error)
except urllib.error.URLError as error:
    if isinstance(error.reason, ConnectionRefusedError):
        print("Native simulation is already stopped")
        raise SystemExit(0)
    raise SystemExit(f"Cannot contact simulation: {error}")
if value.get("backend") != "native" or value.get("robot") != "go2":
    raise SystemExit("Endpoint is not this Go2 Native runtime")
req = urllib.request.Request(url+"/command", data=b'{"shutdown":true}', headers={"Content-Type":"application/json"})
with urllib.request.urlopen(req, timeout=2.) as response:
    print(response.read().decode())
