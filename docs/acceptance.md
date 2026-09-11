# Native Go2 simulation acceptance — 2026-09-11 / 2026-09-12

Measured locally on Ubuntu 22.04 / ROS 2 Humble. MuJoCo 3.3.6, CPU torch 2.8.0.
Robonix CLI build: 178fd2a+; API/codegen source checkout: 2548fe8ba549d39d1b95f98f552372c6e2134769.
This is simulator acceptance, not physical Go2, Jetson or Nav2 performance.

## Physics-only cases

Each case: settle 2 s, command 5 s, stop 2 s, all in simulation time.

| Case | X displacement (m) | Y displacement (m) | Yaw change (rad) | Speed after stop (m/s) | Pass |
| --- | ---: | ---: | ---: | ---: | --- |
| stand | -0.0022 | 0.0008 | 0.0056 | 0.00018 | True |
| forward | 1.9155 | 0.1068 | -0.0789 | 0.00093 | True |
| backward | -0.8833 | -0.0259 | -0.0741 | 0.00063 | True |
| left | 0.1344 | 1.2691 | -0.0645 | 0.00062 | True |
| turn_left | -0.0930 | 0.1562 | 2.4642 | 0.00030 | True |
| turn_right | -0.0959 | -0.0813 | -2.6491 | 0.00356 | True |

## ROS closed loop

- Forward displacement: 0.7982 m from a 3 s, 0.3 m/s ROS command.
- Speed after command expiry: 0.013253 m/s.
- Advancing clock, TF, IMU, 12 joints, 180-ray scan, RGB and float metric depth received.
- Explicit Trigger stop and reset succeeded; reset incremented the epoch and restored origin.

## Robonix closed loop

- Four ACTIVE primitives; 13 capabilities.
- Timed velocity displacement including settling: 0.8597 m.
- Relative 0.5 m command measured at return: 0.4706 m.
- Relative 45-degree command measured at return: 0.7471 rad.
- Speed after lifecycle cancellation: 0.008876 m/s.
- Concurrent RPC returned busy; lifecycle deactivation cancelled the active move.
- Non-finite command rejected; lifecycle activation restored availability.

## Evidence and limits

The adjacent JSON files are test-produced measurements. The MP4 is rendered from actual
MuJoCo stepping and motor torques, in a separate 18-second deterministic dynamics run.
It is not a screen recording of the gRPC test. No base teleporting is used to create walking.
15 unit tests and six physics cases form the CI template; all reported tests ran locally.
GitHub Actions activation is pending workflow-write permission; no cloud CI pass is claimed.
The Sept 12 repeat includes a minimum 0.12 m/s relative-distance command outside tolerance
to avoid the policy's near-zero-velocity standing deadband observed during the Sept 11 repeat.
Complex terrain, calibrated physical sensor noise, autonomous mapping/navigation,
language planning and Web WASM are not included in this acceptance.
