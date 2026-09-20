"""Typed MCP entry point for named actions."""
from __future__ import annotations

import logging
import math
from urllib.parse import urlsplit

from robonix_api import Err, Ok, Skill

from .controller import NamedActionController

from named_action_mcp import (  # noqa: E402
    CancelNamedAction_Request, CancelNamedAction_Response,
    ExecuteNamedAction_Request, ExecuteNamedAction_Response,
    GetNamedActionStatus_Request, GetNamedActionStatus_Response,
    ListNamedActions_Request, ListNamedActions_Response,
)


logging.basicConfig(level=logging.INFO, format="[named-action] %(levelname)s %(message)s")
skill = Skill(id="named_action", namespace="robonix/skill/named_action")
controller: NamedActionController | None = None
selection = {"bridge_url": "http://127.0.0.1:8766", "poll_interval_s": .05,
             "release_timeout_s": 10.0, "admission_timeout_s": 5.0}


@skill.mcp("robonix/skill/named_action/execute")
def execute(req: ExecuteNamedAction_Request) -> ExecuteNamedAction_Response:
    if controller is None:
        raise RuntimeError("named action controller is not active")
    try:
        task = controller.start(req.name, float(req.timeout_s))
        return ExecuteNamedAction_Response(accepted=True, run_id=task.run_id, message=task.detail)
    except RuntimeError as error:
        return ExecuteNamedAction_Response(accepted=False, run_id="", message=str(error))


@skill.mcp("robonix/skill/named_action/status")
def status(req: GetNamedActionStatus_Request) -> GetNamedActionStatus_Response:
    if controller is None:
        raise RuntimeError("named action controller is not active")
    value = controller.status(req.run_id or None)
    if value is None:
        return GetNamedActionStatus_Response(known=False, state="PENDING", action="", phase="IDLE",
                                             elapsed_s=0.0, detail="unknown run id", result_json="{}")
    return GetNamedActionStatus_Response(known=True, **value)


@skill.mcp("robonix/skill/named_action/cancel")
def cancel(req: CancelNamedAction_Request) -> CancelNamedAction_Response:
    if controller is None:
        raise RuntimeError("named action controller is not active")
    ok, message = controller.cancel(req.run_id or None)
    return CancelNamedAction_Response(ok=ok, message=message)


@skill.mcp("robonix/skill/named_action/list")
def list_actions(_req: ListNamedActions_Request) -> ListNamedActions_Response:
    if controller is None:
        raise RuntimeError("named action controller is not active")
    return ListNamedActions_Response(**controller.list_actions())


@skill.on_init
def init(config):
    global selection
    try:
        raw = dict(config or {})
        unknown = set(raw) - set(selection)
        if unknown:
            raise ValueError(f"unknown named_action config fields: {sorted(unknown)}")
        values = {**selection, **raw}
        parsed = urlsplit(str(values["bridge_url"]))
        if parsed.scheme != "http" or parsed.hostname not in {"127.0.0.1", "localhost"}:
            raise ValueError("bridge_url must be a local HTTP endpoint")
        for name in ("poll_interval_s", "release_timeout_s", "admission_timeout_s"):
            values[name] = float(values[name])
            if not math.isfinite(values[name]) or values[name] <= 0:
                raise ValueError(f"{name} must be finite and positive")
        selection = values
        return Ok()
    except (TypeError, ValueError) as error:
        return Err(str(error))


@skill.on_activate
def activate():
    global controller
    try:
        controller = NamedActionController(**selection)
        controller.list_actions()
        return Ok()
    except Exception as error:  # noqa: BLE001
        controller = None
        return Err(str(error))


@skill.on_deactivate
def deactivate():
    global controller
    if controller is not None:
        controller.stop()
        controller = None
    return Ok()


@skill.on_shutdown
def shutdown():
    return deactivate()


if __name__ == "__main__":
    skill.run()
