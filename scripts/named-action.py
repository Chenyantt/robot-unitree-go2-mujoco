#!/usr/bin/env python3
"""Direct bridge acceptance client for Native named actions."""
from __future__ import annotations

import argparse
import json
import time
import uuid
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def request(base: str, path: str, body: dict | None = None) -> dict:
    message = None if body is None else json.dumps(body).encode()
    call = Request(base+path, data=message, headers={"Content-Type": "application/json"})
    try:
        with urlopen(call, timeout=5) as response:
            return json.load(response)
    except HTTPError as error:
        detail = json.loads(error.read())
        raise RuntimeError(str(detail.get("error") or detail)) from error


def command(base: str, operation: str, token: str, name: str | None = None) -> dict:
    body = {"type": "named_action", "operation": operation, "actionId": token}
    if name is not None:
        body["name"] = name
    return request(base, "/command", body)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run one Native yard named action")
    parser.add_argument("action", nargs="?", help="action name; omit with --list")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--bridge", default="http://127.0.0.1:8766")
    args = parser.parse_args()
    state = request(args.bridge, "/state")
    robot = state.get("robot") or {}
    if args.list:
        print("\n".join(robot.get("availableActions") or []))
        return
    if not args.action:
        parser.error("action is required unless --list is used")
    if state.get("backend") != "native" or state.get("environment") != "yard":
        raise SystemExit("start the native yard environment before running named actions")
    token = str(uuid.uuid4())
    started = False
    try:
        command(args.bridge, "start", token, args.action)
        started = True
        deadline = time.time()+args.timeout
        while time.time() < deadline:
            state = request(args.bridge, "/state")
            action = (state.get("robot") or {}).get("namedAction") or {}
            if action.get("id") == token:
                print(json.dumps(action, ensure_ascii=False), flush=True)
                if action.get("state") in ("SUCCEEDED", "FAILED", "CANCELED"):
                    command(args.bridge, "release", token)
                    started = False
                    raise SystemExit(0 if action["state"] == "SUCCEEDED" else 1)
            time.sleep(.1)
        raise TimeoutError(f"action did not finish within {args.timeout:g} seconds")
    finally:
        if started:
            try:
                command(args.bridge, "cancel", token)
            except RuntimeError:
                pass
            cleanup_deadline = time.time()+10
            while time.time() < cleanup_deadline:
                try:
                    state = request(args.bridge, "/state")
                    action = (state.get("robot") or {}).get("namedAction") or {}
                    if action.get("id") != token:
                        break
                    if action.get("state") in ("SUCCEEDED", "FAILED", "CANCELED"):
                        command(args.bridge, "release", token)
                        break
                except RuntimeError:
                    pass
                time.sleep(.1)


if __name__ == "__main__":
    main()
