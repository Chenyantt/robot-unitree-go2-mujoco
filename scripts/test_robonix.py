#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Discover actual Atlas capabilities and measure a gRPC-driven simulation run."""
import concurrent.futures
import json
import math
from pathlib import Path
import time
import grpc
from robonix_api import ATLAS
import chassis_pb2
import lifecycle_pb2
from go2_sim.bridge import request

ROOT = Path(__file__).resolve().parents[1]


def connect(contract, request_type, response_type):
    cap = ATLAS.find_unique_capability(contract_id=contract, transport="grpc", provider_id="go2_sim_chassis")
    edge = ATLAS.connect_capability(consumer_id="go2-sim-acceptance", provider_id=cap.provider_id,
                                    contract_id=contract, transport="grpc")
    channel = grpc.insecure_channel(edge.endpoint)
    service = edge.params.service_name
    if "." not in service:
        service = "robonix.contracts."+service
    method = channel.unary_unary("/"+service+"/"+edge.params.method,
                                request_serializer=request_type.SerializeToString,
                                response_deserializer=response_type.FromString)
    return edge, channel, method


def main():
    providers = ATLAS.query_primitives()
    expected = {"go2_sim_"+k for k in ("chassis", "lidar", "camera", "imu")}
    assert {p.id for p in providers} == expected
    assert all(p.state.name == "ACTIVE" for p in providers)
    move_edge, move_channel, move = connect("robonix/primitive/chassis/move",
        chassis_pb2.ExecuteMoveCommand_Request, chassis_pb2.ExecuteMoveCommand_Response)
    driver_edge, driver_channel, driver = connect("robonix/primitive/chassis/driver",
        lifecycle_pb2.Driver_Request, lifecycle_pb2.Driver_Response)
    def invoke(**kwargs):
        return json.loads(move(chassis_pb2.ExecuteMoveCommand_Request(
            command=chassis_pb2.MoveCommand(**kwargs)), timeout=35.).status.data)
    results = {}
    try:
        request("/command", {"reset": True})
        time.sleep(2.)
        before = request("/state")
        results["velocity"] = invoke(linear_x=.3, duration_sec=3.)
        time.sleep(1.)
        after = request("/state")
        results["velocity_distance_m"] = math.dist(before["position"][:2], after["position"][:2])
        assert results["velocity"]["status"] == "done" and results["velocity_distance_m"] > .35
        before = request("/state")
        results["distance"] = invoke(forward_m=.5)
        after = request("/state")
        results["relative_distance_m"] = math.dist(before["position"][:2], after["position"][:2])
        assert results["distance"]["status"] == "done", results
        assert abs(results["relative_distance_m"]-.5) < .10
        time.sleep(.5)
        before = request("/state")
        results["angle"] = invoke(rotate_deg=45.)
        after = request("/state")
        results["angle_rad"] = math.atan2(math.sin(after["yaw"]-before["yaw"]),
                                         math.cos(after["yaw"]-before["yaw"]))
        assert results["angle"]["status"] == "done" and abs(results["angle_rad"]-math.pi/4) < .12
        time.sleep(.5)
        with concurrent.futures.ThreadPoolExecutor() as pool:
            future = pool.submit(invoke, linear_x=.2, duration_sec=10.)
            time.sleep(.8)
            results["concurrent"] = invoke(linear_x=.1, duration_sec=1.)
            assert results["concurrent"]["status"] == "busy"
            reply = driver(lifecycle_pb2.Driver_Request(command=2), timeout=5.)
            assert reply.ok, reply
            results["deactivation"] = future.result(timeout=4.)
            assert results["deactivation"]["status"] == "cancelled"
        time.sleep(1.5)
        results["stop_speed_m_s"] = math.hypot(*request("/state")["linear_velocity"][:2])
        assert results["stop_speed_m_s"] < .08
        assert driver(lifecycle_pb2.Driver_Request(command=1), timeout=5.).ok
        results["invalid"] = invoke(linear_x=float("nan"))
        assert results["invalid"]["status"] == "error"
        results["passed"] = True
        results["active_providers"] = sorted(expected)
        results["capability_count"] = sum(len(p.capabilities) for p in providers)
        (ROOT/".runtime/robonix-acceptance.json").write_text(json.dumps(results, indent=2)+"\n")
        print(json.dumps(results, indent=2))
    finally:
        request("/command", {"stop": True})
        for value in (move_edge, driver_edge, move_channel, driver_channel):
            value.close()


if __name__ == "__main__":
    main()
