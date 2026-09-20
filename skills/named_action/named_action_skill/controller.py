"""Bridge-backed asynchronous named action lifecycle."""
from __future__ import annotations

import json
import math
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


TRANSIENT_ADMISSION_ERRORS = (
    "robot is still translating",
    "robot is still rotating",
    "leg joints are still moving",
    "required foot contacts are not established",
    "robot has not remained stable",
)


@dataclass
class Task:
    run_id: str
    action: str
    timeout_s: float
    started_at: float
    state: str = "RUNNING"
    phase: str = "EXECUTE"
    detail: str = "accepted"
    result: dict[str, Any] = field(default_factory=dict)
    cancel_requested: bool = False
    thread: threading.Thread | None = None


class NamedActionController:
    """Serialize action tasks and preserve the runtime's exact action token."""

    def __init__(self, bridge_url: str, poll_interval_s: float, release_timeout_s: float,
                 admission_timeout_s: float) -> None:
        self.bridge_url = bridge_url.rstrip("/")
        self.poll_interval_s = poll_interval_s
        self.release_timeout_s = release_timeout_s
        self.admission_timeout_s = admission_timeout_s
        self._lock = threading.Lock()
        self._task: Task | None = None

    def _request(self, method: str, path: str, body: dict | None = None) -> dict:
        payload = None if body is None else json.dumps(body).encode()
        request = Request(self.bridge_url + path, data=payload, method=method,
                          headers={"Content-Type": "application/json"})
        try:
            with urlopen(request, timeout=4) as response:
                result = json.loads(response.read())
        except HTTPError as error:
            try:
                result = json.loads(error.read())
            except Exception:
                result = {"error": str(error)}
            raise RuntimeError(str(result.get("error") or result)) from error
        except (URLError, TimeoutError, json.JSONDecodeError) as error:
            raise RuntimeError(f"simulator bridge unavailable: {error}") from error
        if not result.get("ok"):
            raise RuntimeError(str(result.get("error") or "bridge request failed"))
        return result

    def runtime(self) -> dict:
        return self._request("GET", "/state")

    def list_actions(self) -> dict:
        state = self.runtime()
        robot = state.get("robot") or {}
        return {"names": list(robot.get("availableActions") or []),
                "environment": str(state.get("environment") or ""),
                "controller_mode": str(robot.get("controllerMode") or "")}

    def start(self, action: str, timeout_s: float) -> Task:
        action = action.strip()
        if not action:
            raise RuntimeError("action name is required")
        timeout = float(timeout_s or 45.0)
        if not math.isfinite(timeout) or not 1 <= timeout <= 120:
            raise RuntimeError("timeout_s must be finite and between 1 and 120")
        with self._lock:
            if self._task is not None and self._task.state == "RUNNING":
                raise RuntimeError("another named action task is running")
            runtime = self.list_actions()
            if runtime["environment"] != "yard":
                raise RuntimeError("named actions are currently scoped to the native yard environment")
            if action not in runtime["names"]:
                raise RuntimeError(f"action {action!r} is not available in the active environment")
            run_id = str(uuid.uuid4())
            command = {
                "type": "named_action", "operation": "start",
                "name": action, "actionId": run_id,
            }
            deadline = time.monotonic() + self.admission_timeout_s
            while True:
                try:
                    result = self._request("POST", "/command", command)
                    break
                except RuntimeError as error:
                    transient = any(message in str(error) for message in TRANSIENT_ADMISSION_ERRORS)
                    if not transient or time.monotonic() >= deadline:
                        raise
                    time.sleep(self.poll_interval_s)
            if result.get("accepted") is not True:
                raise RuntimeError(str(result.get("detail") or "runtime rejected named action"))
            task = Task(run_id, action, timeout, time.time())
            self._task = task
            task.thread = threading.Thread(target=self._run, args=(task,), daemon=True)
            task.thread.start()
            return task

    def _run(self, task: Task) -> None:
        deadline = task.started_at + task.timeout_s
        terminal = "FAILED"
        ownership_seen = False
        try:
            while time.time() < deadline:
                state = self.runtime()
                action = (state.get("robot") or {}).get("namedAction") or {}
                if action.get("id") != task.run_id:
                    if not ownership_seen and time.time()-task.started_at < 1.5:
                        time.sleep(self.poll_interval_s)
                        continue
                    raise RuntimeError("runtime lost named action ownership")
                ownership_seen = True
                task.result = action
                task.phase = str(action.get("phase") or "EXECUTE")
                task.detail = str(action.get("detail") or "running")
                action_state = str(action.get("state") or "RUNNING")
                if action_state in ("SUCCEEDED", "FAILED", "CANCELED"):
                    terminal = action_state
                    break
                if task.cancel_requested and action_state == "RUNNING":
                    self._request("POST", "/command", {
                        "type": "named_action", "operation": "cancel", "actionId": task.run_id})
                time.sleep(self.poll_interval_s)
            else:
                task.cancel_requested = True
                self._request("POST", "/command", {
                    "type": "named_action", "operation": "cancel", "actionId": task.run_id})
                terminal = "TIMEOUT"

            task.phase = "RELEASE"
            release_deadline = time.time() + self.release_timeout_s
            last_error = "release not attempted"
            while time.time() < release_deadline:
                try:
                    self._request("POST", "/command", {
                        "type": "named_action", "operation": "release", "actionId": task.run_id})
                    task.state = terminal
                    task.phase = "COMPLETE"
                    task.detail = f"{terminal.lower()}; normal control restored"
                    return
                except RuntimeError as error:
                    last_error = str(error)
                    time.sleep(self.poll_interval_s)
            raise RuntimeError(f"action finished but exclusive hold could not be released: {last_error}")
        except Exception as error:  # noqa: BLE001
            self._recover_control(task)
            task.state = "FAILED" if terminal != "TIMEOUT" else "TIMEOUT"
            task.detail = str(error)

    def _recover_control(self, task: Task) -> None:
        """Best-effort exact-token cancellation so worker faults do not strand HOLD."""
        try:
            self._request("POST", "/command", {
                "type": "named_action", "operation": "cancel", "actionId": task.run_id})
        except RuntimeError:
            pass
        deadline = time.time()+self.release_timeout_s
        while time.time() < deadline:
            try:
                state = self.runtime()
                action = (state.get("robot") or {}).get("namedAction") or {}
                if action.get("id") != task.run_id:
                    return
                if str(action.get("state")) in ("SUCCEEDED", "FAILED", "CANCELED"):
                    self._request("POST", "/command", {
                        "type": "named_action", "operation": "release", "actionId": task.run_id})
                    return
            except RuntimeError:
                pass
            time.sleep(self.poll_interval_s)

    def status(self, run_id: str | None) -> dict | None:
        with self._lock:
            task = self._task
            if task is None or (run_id and task.run_id != run_id):
                return None
            return {"state": task.state, "action": task.action, "phase": task.phase,
                    "elapsed_s": time.time()-task.started_at, "detail": task.detail,
                    "result_json": json.dumps(task.result, separators=(",", ":"), allow_nan=False)}

    def cancel(self, run_id: str | None) -> tuple[bool, str]:
        with self._lock:
            task = self._task
            if task is None or (run_id and task.run_id != run_id):
                return True, "no matching task"
            if task.state != "RUNNING":
                return True, f"task already {task.state}"
            task.cancel_requested = True
        return True, "cancellation requested"

    def stop(self) -> None:
        with self._lock:
            task = self._task
        if task is not None and task.state == "RUNNING":
            task.cancel_requested = True
            if task.thread is not None:
                task.thread.join(timeout=15)
