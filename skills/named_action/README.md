# Named Action Skill

`named_action` exposes one asynchronous Robonix task for Native MuJoCo Go2
actions. It does not contain a second motor loop. The skill sends acknowledged
commands to the simulator bridge; `sim/native/named_actions.py` is the sole
action torque owner inside the physics loop.

## Scope

- Current scene scope: `yard` only.
- Actions: `crouch`, `bow`, `dance`, `sway`, `stretch`, `jump`, `backflip`, and
  `handstand_walk` when their pinned ONNX assets are installed.
- Backend: Native MuJoCo only. The Web backend reports no available actions.
- Plant: simulation only. This is not a real-Go2 safety controller or hardware
  certification.
- Locomotion and actions are mutually exclusive. `/cmd_vel`, MoE policy changes,
  another action, and action release are rejected or fenced while action control
  is active.

## Preconditions

`assets/environments/yard/named_actions.json` is the authoritative scene
annotation. Each action names a safe zone and duration. The common gate checks
zone membership, optional heading tolerance, body height, upright attitude,
linear, angular and leg-joint speed, all four foot contacts, and a stable dwell
time. The skill waits for transient post-navigation motion to settle for a
bounded interval before submitting the action; unsafe pose or zone failures are
still rejected immediately.
An action can also name a separate execution envelope. Native monitors base
position against that envelope throughout execution and enters failed recovery
immediately if the robot leaves it.

The green circle in yard is the performance zone for stationary actions. The
blue rectangle is the jump start zone and requires heading toward positive X;
the low red bar is reserved for a future obstacle-specific action and lies
outside the current jump envelope. Visual geoms only help operators see the
marks. Editing visual geometry does not change admission rules.

## Lifecycle

1. `execute` creates an action ID and asks Native to acquire motor control.
2. Native unloads `moe_rough`, clears Twist, and runs one torque controller.
3. The action enters bounded standing recovery.
4. Native holds the standing pose with ordinary motion still blocked.
5. The skill releases the exact action ID after competing nonzero Twist input
   has remained quiet. Base gait control then resumes.

`emergency_stop` or `cancel` enters the same recovery path. Reset is the only
command that can rewrite simulator state. Named actions never write `qpos`,
apply external forces, change gravity, or call `mj_step` themselves.

## Assets

`scripts/install-stunt-assets.py` downloads the two public learned controllers
at commits and SHA-256 digests pinned in `stunt-assets.lock.json`. Files live in
ignored `.runtime/stunt-assets`; they are not silently replaced by moving
upstream branches. The scripted actions remain available without ONNX files.

Starting `yard` on the Native backend installs and verifies these assets:

```bash
bash sim/start.sh --backend native --environment yard --viewer
```

Then boot Robonix normally and use `robonix/skill/named_action/list`, `execute`,
`status`, and `cancel`, or issue equivalent natural-language instructions to
Pilot. Do not run Navigation, Explore, Floor Transition, direct chassis motion,
or a continuously publishing teleop node at the same time.

For simulator-only manual acceptance without booting Robonix:

```bash
bash scripts/named-action.sh --list
bash scripts/named-action.sh bow
bash scripts/named-action.sh backflip
bash scripts/named-action.sh handstand_walk --timeout 45
```

This helper calls the same acknowledged bridge protocol but is not the Robonix
skill endpoint. Use it to isolate Native physics and safety-zone failures.

## Configuration

- `bridge_url`: local acknowledged simulator bridge, default `127.0.0.1:8766`.
- `poll_interval_s`: action state polling interval.
- `release_timeout_s`: bounded wait for external Twist publishers to go quiet.
- `admission_timeout_s`: bounded wait for post-navigation motion to settle.

Adding another scene requires a `namedActionProfile` manifest entry and a
reviewed scene-local JSON annotation. Adding an action also requires a Native
runner, result checks, licensing/provenance, and physics acceptance evidence.
