# Unitree Go2 MuJoCo for Robonix

[中文说明](README.zh-CN.md)

![Go2 in SceneSmith House 185](docs/media/native-scenesmith_house_185.png)

A standalone Go2 simulation deployment with Web MuJoCo WASM and native Python
MuJoCo backends. Both expose the same Robonix chassis, LiDAR, IMU and RGB-D
capabilities, and both support loading/unloading the included go2_rl_gym MoE
ONNX policy while running. The policy is **OFF at startup**.

Mapping, Navigation and Scene use existing Robonix services. The local Explore
adaptation uses the same contracts with connected, footprint-safe frontier goals,
failed-goal suppression, verified cancellation and request speed limits.
There is no physical Unitree transport, arm, speech, or training
dependency. Motion uses simulated joint torques; odometry comes from MuJoCo.

## Requirements

The local deployment uses Ubuntu/WSL2, Docker Compose, Node.js 20+, Python 3.10+,
uv, an installed rbnx CLI and a Robonix source checkout.
The native profile supplied here uses WSLg/X11 and NVIDIA GPU rendering.
Native physics and this small ONNX network run on CPU; RGB rendering uses GPU.
Headless mode closes the native viewer but retains RGB rendering.

Web mode needs Chromium/WebGL2. The bootstrap installs its Playwright browser.
Normal Linux machines should adapt the graphics mounts in
sim/compose.native.yaml; the checked-in native profile uses /dev/dxg and WSLg.

## Install

Run from this package directory:

```bash
cp .env.example .env
# Set ROBONIX_SOURCE_PATH, VLM_BASE_URL, VLM_API_KEY and VLM_MODEL in .env.
bash scripts/bootstrap.sh
```

Keep credentials in the ignored local .env or exported environment variables.
The simulator does not need a VLM key; Pilot and Scene use it for language/vision.
Bootstrap installs local Node dependencies, builds the simulation image, generates
primitive bindings, and builds the Robonix deployment. Runtime assets and the
pretrained policy are already included; no dataset download or policy training is
required for ordinary startup.

## Start Manually

Use three terminals. Only one simulation backend and one Robonix stack may own
the configured ROS graph and ports at a time.

Terminal 1, Web physics (default):

```bash
cd ~/robot-unitree-go2_mujoco
bash sim/start.sh --backend web --environment scenesmith_house_185
```

Native physics with its MuJoCo viewer:

```bash
bash sim/start.sh --backend native --viewer --environment scenesmith_house_185
```

Native without the viewer:

```bash
bash sim/start.sh --backend native --headless --environment scenesmith_house_185
```

Wait for the launcher to report ready. The Web viewer is
[http://127.0.0.1:5181/](http://127.0.0.1:5181/).
For native, the browser control panel is
[http://127.0.0.1:5181/?backend=native](http://127.0.0.1:5181/?backend=native);
physics and 3D rendering stay in the native process.
Bridge health is [http://127.0.0.1:8766/health](http://127.0.0.1:8766/health).
The launcher refuses occupied ports instead of stopping unrelated services.

Terminal 2, Robonix:

```bash
cd ~/robot-unitree-go2_mujoco
source scripts/env.sh
rbnx boot --no-update-check
```

Terminal 3, interaction:

```bash
cd ~/robot-unitree-go2_mujoco
source scripts/env.sh
rbnx caps -v
rbnx chat
```

Suggested requests:

- Capture the front camera and describe the room.
- Explore the rooms for 120 seconds at 0.18 m/s, and return the run ID.
- Report the exploration status, then cancel it.
- List the objects observed by Scene.
- Navigate near the closest observed table using Scene's safe approach goal.

Exploration is asynchronous. Finish or cancel it before starting another motion
task. Semantic navigation requires objects actually observed through RGB-D and
an occupancy map; the package does not seed Scene with simulator object positions.
Scene goal_near produces an approach pose, which Navigation executes.
A table outside the observed area cannot yet be resolved.

## Policy and Controls

The Web policy selector and `Load / Disable Policy` action live in the top-right
`Simulation > Policy` folder. Both backends acknowledge the operation and expose
errors; a reload does not reset the robot's map pose. Production mode does not
register Web motion keys or a native viewer key callback, so Robonix owns motion.

Enable local driving explicitly for development:

```bash
bash sim/start.sh --backend web --dev --environment go2_rl_stairs
bash sim/start.sh --backend native --viewer --dev --environment go2_rl_stairs
```

Development keys are W/S forward/backward, A/D yaw, Q/E lateral, Space stop,
and X reset. The native viewer also supports L to load/disable the policy.

The included policy is the existing
go2_moe_cts_high_slope_thre_164k_0.6715 from wty-yy/go2_rl_gym, not the suggested
v4.2 upgrade. It has 225 inputs (five 45-value observation frames), runs at
50 Hz over 0.002 s physics, and uses upstream joint ordering, action scale 0.25,
and PD gains 20/0.5. See
[policy provenance](assets/robots/go2/policy/moe_rough/UPSTREAM.md).

When unloaded, a deterministic gait controls the legs without neural inference.
This controller is intended for indoor flat-floor navigation; load the RL policy
to test the upstream obstacle courses. Neither controller guarantees passage
across every obstacle.

For a policy-only test, Terminal 2/3 are optional:

```bash
bash sim/start.sh --backend native --viewer --dev --environment go2_rl_stairs
# Or:
bash sim/start.sh --backend web --dev --environment go2_rl_track
```

Load the policy explicitly in the UI, then issue motion commands. Unload returns
to the basic controller. Do not reset the robot while Mapping/Navigation are
active: a pose reset invalidates the current navigation run.

## Environments

| ID | Content |
| --- | --- |
| scenesmith_house_185 | New SceneSmith House: living room and bathroom (default) |
| scenesmith_house_186 | New SceneSmith House: bedroom and bathroom |
| go2_rl_stairs | Original go2_rl_gym stairs geometry |
| go2_rl_track | Original go2_rl_gym race-track geometry |

The houses are distinct single-floor multiroom dataset exports, not the prior
House 187 or the rearranged multilevel house. All four environments use textured
meshes or native MJCF geometry and work with either backend. See
[scene provenance and repeatable import](docs/scenes.md).

Stop Robonix and the simulator before changing environments, then restart both
with the new environment ID. This avoids reusing stale maps and semantic poses.

## Stop

```bash
source scripts/env.sh
rbnx shutdown
bash sim/stop.sh
```

Ctrl-C in each owning terminal also stops its respective lifecycle.
Simulation logs are in .runtime; Robonix logs are in rbnx-boot/logs.
These directories and local credentials are excluded from version control.

## Validation

```bash
npm test
python3 -m unittest discover -s primitives/tests
# Live ROS checks, with simulator and Robonix running:
bash scripts/acceptance.sh
# Observe autonomous exploration, then verify cancellation:
bash scripts/acceptance.sh --explore --explore-duration 150 --explore-timeout 240 --explore-speed 0.18
# Native model and sensor smoke:
docker exec mujoco_go2_sim python3 /workspace/sim/tests/native_smoke.py
```

Acceptance moves the simulated robot. Run it without competing manual or
navigation commands. Detailed results and remaining limits are recorded in
[the implementation report](docs/IMPLEMENTATION_REPORT.md).

## Layout and Attribution

- assets/robots/go2: model, meshes, controller and policy.
- assets/environments: four independent environment packages.
- primitives: chassis, LiDAR, IMU and front camera.
- sim/native and sim/bridge: native physics and ROS2 transport.
- src: Web physics, sensors and shared operator controls.
- config, soma.yaml, urdf: navigation parameters and robot geometry.
- robonix_manifest.yaml: deployment entry point.

The integration reuses [mujoco_robonix](../mujoco_robonix) runtime patterns,
[MuJoCo-GS-Web](https://github.com/Vector-Wangel/MuJoCo-GS-Web), and the
[real Go2 Robonix deployment](https://github.com/syswonder/robot-unitree-go2).
MuJoCo model assets retain their Unitree BSD notice; the frontend foundation
retains its MIT license. SceneSmith assets and go2_rl_gym weights/courses keep
their respective upstream notices. See [NOTICE](NOTICE).
