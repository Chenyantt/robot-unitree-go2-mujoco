---
description: Control the simulated Go2 base and expose its local odometry.
---

# Go2 chassis

`move` executes one bounded motion burst and publishes a zero velocity when it
finishes. `twist_in` is the continuous velocity input used by Nav2. Commands are
expressed in `base_link`; odometry is reported in `odom`.

The simulator and bridge must be ready before this provider is activated. On
shutdown it sends a zero Twist. `move` is intended for short direct motions;
use the navigation service for collision-aware travel through a room.

Direct motion defaults to 0.18 m/s and is capped at 0.25 m/s; angular commands
are capped at 0.55 rad/s. Relative distance and angle requests use timed bursts,
not measured arrival control. SIM_LINEAR_SPEED and SIM_ANGULAR_SPEED may lower
the burst speeds. Do not issue direct moves while navigation or Explore runs.
