# Whole-body MPC handover in the sim (Nav2 MPPI → base + PiPER MPC)

Nav2 (Smac + RotationShim + MPPI, unchanged) drives the Ranger Mini 3.0 to a
handover pose. A gate then hands the base to a kinematic whole-body MPC that
moves the base (translation only) and the six PiPER joints **together** to put
the gripper on a target the arm alone cannot reach, and finally hands the base
back to Nav2. Gazebo Fortress only (`ranger_sim`); nothing here has a
real-robot path.

Videos (house scene, recorded 2026-11-21 with
`record_wbmpc_reach_video.launch.py`):

- `docs/sim_wbmpc_handover_small_house.mp4` — Nav2 away and back, hand-over,
  reach, hand back, Nav2 away again (48 s, 6.4 MB).
- `docs/sim_wbmpc_reach_small_house.mp4` — the parked reach from the spawn
  pose, no Nav2 (13 s, 1.4 MB).

The red sphere is the target (visual only, no collision). The wrist inset shows
the gripper closing on it.

The same MPC also does the pick of the pick-and-place demo
(`pick_place_demo.launch.py approach:=mpc`), compared there with the
scripted pick from wrong base poses: [sim_wbmpc_pick.md](sim_wbmpc_pick.md).

## What runs where

```
sim_bringup.launch.py handover_gate:=true [launch_nav:=true]
  Nav2 / scripted ──/cmd_vel──────┐
  whole_body_mpc ──/cmd_vel_manip─┤ base_handover_gate
                                  └─/cmd_vel_gated─► ranger_motion_adapter ─► Gazebo
  gate modes: NAVIGATE → SETTLING (zeros until the base is
              still) → MANIPULATE → SETTLING → NAVIGATE
  arm: piper_arm_controller (JTC, MoveIt)   ◄─ switch ─►
       piper_arm_stream_controller (Float64MultiArray, MPC)
wbmpc_reach_demo.launch.py execute:=true [use_nav:=true]
  carry → [Nav2 away, Nav2 to handover] → hand over → MPC
  reach (base + arm together) → hold → hand back → [Nav2 away]
```

- **`base_handover_gate`** (`scripts/base_handover_gate.py`, started by
  `spawn_robot.launch.py` when `handover_gate:=true`) is the only publisher of
  the motion adapter's input. A change of owner (`~/set_manipulate`,
  `std_srvs/SetBool`) always goes through SETTLING, which sends zeros until
  `/odom` reports the base still (< 0.005 m/s, < 0.01 rad/s) for 0.3 s. In
  MANIPULATE it drops `angular.z` and caps the speed at 0.15 m/s, so the
  translation-only rule holds even if the MPC node misbehaves. A twist
  received before the current owner took over does not count. With
  `handover_gate` unset (the default), the adapter reads `/cmd_vel` as before
  and no gate runs.

  **Governed mode** (used by the handover baseline,
  [sim_handover_baseline.md](sim_handover_baseline.md)): `~/set_governed`
  (`std_srvs/SetBool`, true) makes `~/speed_scale` (`std_msgs/Float64`,
  [0, 1]) a hard limit. Without a scale fresher than `speed_scale_timeout`
  (0.5 s), or with a scale ≤ 0, the gate sends zeros in every mode.
  Otherwise NAVIGATE clamps the linear speed to `nav_max_speed` (0.26 m/s,
  Nav2's `vx_max`) × scale and scales the whole twist, rotation included, by
  the same ratio; MANIPULATE caps at `manip_max_speed` × scale. It is a
  clamp, not a multiplier: a twist already planned under the scaled bound
  passes unchanged. Ungoverned (the default) the scale is ignored. The gate
  lives as long as `sim_bringup`; if the governor died without releasing it,
  the base stays stopped until
  `ros2 service call /base_handover_gate/set_governed std_srvs/srv/SetBool "{data: false}"`
  (this reaches the gate only, no hardware).
- **`piper_arm_stream_controller`**
  (`forward_command_controller/ForwardCommandController`,
  `config/ros2_controllers_sim.yaml`) claims the same six position interfaces
  as the trajectory controller. It is spawned inactive, and the demo switches
  between the two (`controller_manager/switch_controller`, STRICT). MoveIt only
  knows the trajectory controller.
- **`whole_body_mpc`** (`scripts/whole_body_mpc.py` around
  `scripts/whole_body_mpc_core.py`) builds the kinematics from
  `/robot_description` and solves at 20 Hz while the gate is in MANIPULATE and
  it has a target (`~/target`, `PoseStamped` in `odom`). It publishes the body
  twist on `/cmd_vel_manip` and six joint positions on
  `/piper_arm_stream_controller/commands`. Its state goes on `~/state`: IDLE,
  TRACKING, REACHED (within 5 mm and 2°, latched until 3× that), or
  `FAULT: <reason>` (odom or joint states older than 0.3 s, three failed solves
  in a row, or commanded minus measured joints > 0.15 rad). A fault stops the
  base and holds the arm until the gate leaves MANIPULATE; a lag fault holds
  it where it measurably is, not at the command it could not follow.
  Non-finite odom or joint samples are dropped, so they go stale, which is a
  fault. A target with a zero quaternion is ignored with a warning, and a new
  target always has to earn REACHED again.

  Two runtime inputs serve a governor; a node that never receives them
  behaves exactly as before. `~/speed_scale` (`std_msgs/Float64`) multiplies
  the base and joint speed bounds of the OCP (a solver parameter: the problem
  is not rebuilt), clamped to [0, 1], NaN = 0. Before the first message the
  scale is 1.0; from the first message on, a scale older than
  `speed_scale_timeout` (0.5 s) counts as 0. Under 0.05 the node does not
  solve: state **PAUSED**, zero twist, `q_cmd` held, warm start dropped for
  the resume; REACHED and FAULT keep their meaning and come first.
  `~/base_box` (`std_msgs/Float64MultiArray`, `[x_min, y_min, x_max, y_max]`
  in `odom`) replaces `base_box` while the node is not TRACKING, REACHED or
  PAUSED; a malformed box is ignored with a warning. Under a scale of about
  0.15 the base bound (0.10 × scale) reaches the 0.01 m/s deadband and the
  base parks while the arm still moves.
- **`wbmpc_reach_demo`** (`scripts/wbmpc_reach_demo.py`, a `PickPlaceDemo`
  subclass) runs the sequence. It hands over in order (gate, then arm, then
  target) and back in reverse. Whatever ends the run (a failed step, an error,
  Ctrl-C or SIGTERM), `restore()` does four things, each even if another fails:
  - cancels a Nav2 goal that is still running, waiting for the answer;
  - hands the base back to NAVIGATE;
  - switches the arm back to the trajectory controller (best effort, so it
    also works from an unknown controller state);
  - stops the base.

  The process exits 1 when the sequence did not complete. `execute` defaults
  to false: it then only plans the arm moves with MoveIt.

## The model, and why the base only translates

State `x = [bx, by, q1..q6]` (base position in `odom`, arm joints). Input
`u = [vx, vy, qd1..qd6]` (base velocity in the body frame, joint velocities).
The heading `psi` is measured every cycle and held constant over the horizon:

```
x[k+1] = x[k] + dt * [R2(psi) @ u[:2], u[2:]]
```

The Ranger driver (`ranger_ros2/ranger_base/src/ranger_messenger.cpp`,
`TwistCmdCallback`) picks one motion mode per Twist. Any `linear.y` selects
PARALLEL, which drops `angular.z`; a turn radius under 0.476 m selects
SPINNING, which drops the linear part. `ranger_motion_adapter.py` reproduces
this in the sim. A model with `(vx, vy, wz)` would plan motions the driver
cannot execute, so the MPC never commands `wz`.

OCP: N = 20 stages of 0.1 s, CasADi `Opti` + IPOPT (mumps, tol 1e-4,
max 60 iterations), warm-started from the previous solution shifted by one
stage. Per stage the cost is:

- 200·|TCP position error|²
- 20·(|approach-axis error|² + |finger-axis error|²)
- 0.05·|q − q_carry|²
- 4·|v_base|² + 0.2·|qd|²
- 2·|Δu|²

The terminal stage uses 2000 / 200 for the position and axis terms.

Bounds: |vx|, |vy| ≤ 0.10 m/s; |qd| ≤ 0.6 rad/s (the URDF allows 3–5); the URDF
joint limits; and the base box `[-0.40, -0.20, 0.52, 0.30]` in `odom`. The box
is grown to contain the base, so a Nav2 stop outside it can only move in.

The arm state is the **commanded** position: the measured joints at
activation, then integrated from `qd`. Closing the loop on the measured joints
would feed the position interface's lag back into the plan. The measured
joints give the tracking error and the lag fault instead. A base speed under
0.01 m/s is sent as zero, so the adapter does not re-steer the wheels for
noise.

## Running it

Run every sim command in an isolated domain:
`export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1`. CasADi is not in rosdep:
`pip3 install --user casadi==3.8.2`.

```bash
# Parked reach as a launch test (boots its own sim; GPU)
RANGER_SIM_GPU=1 launch_test src/ranger_sim/test/test_wbmpc_sim.py

# Nav2 → MPC → Nav2
ros2 launch ranger_sim sim_bringup.launch.py headless:=true handover_gate:=true launch_nav:=true
#   wait for "Managed nodes are active" (lifecycle_manager_navigation), then:
ros2 launch ranger_sim wbmpc_reach_demo.launch.py execute:=true use_nav:=true

# Without execute:=true the demo only plans the arm moves (no motion, no goals)
ros2 launch ranger_sim wbmpc_reach_demo.launch.py

# Video (on a freshly started sim: the rig and the marker are spawned once)
ros2 launch ranger_sim record_wbmpc_reach_video.launch.py use_nav:=true \
  output_path:=$(pwd)/docs/sim_wbmpc_handover_small_house.mp4
```

`target_x/y/z/yaw` (default `0.95, 0.15, 1.00, 0.0` in `odom`, gripper level,
approach +x) are launch arguments of both reach launches. They are the one
source of the target for the demo and the marker.

## Parameters (`config/wbmpc.yaml`)

| Node | Parameter | Value | Note |
|---|---|---|---|
| `base_handover_gate` | `settle_sec`, `still_linear`, `still_angular` | 0.3 s, 0.005 m/s, 0.01 rad/s | still this long before the new owner gets the base |
| | `manip_max_speed` | 0.15 m/s | ≥ hypot(0.10, 0.10): a legal MPC command is never scaled |
| | `nav_max_speed`, `speed_scale_timeout` | 0.26 m/s, 0.5 s | governed mode only |
| `whole_body_mpc` | `base_box` | `[-0.40, -0.20, 0.52, 0.30]` | x ≤ 0.52 keeps the base short of the table (the pick's `base_approach_x`); y ≥ −0.20 keeps it clear of `ChairA_01_001` |
| | `rate`, `horizon`, `dt` | 20 Hz, 20, 0.1 s | |
| | `base_speed`, `joint_speed` | 0.10 m/s, 0.6 rad/s | |
| | `position_tolerance`, `angle_tolerance`, `reengage_factor` | 5 mm, 0.035 rad, 3 | REACHED latch |
| | `base_deadband` | 0.01 m/s | |
| | `odom_timeout`, `joint_timeout`, `max_command_lag` | 0.3 s, 0.3 s, 0.15 rad | faults |
| | `speed_scale_timeout` | 0.5 s | after the first `~/speed_scale`, an older one is 0 (PAUSED) |
| `wbmpc_reach_demo` | `nav_away_pose`, `nav_handover_pose` | `(-2, 0, π)`, `(0, 0, 0)` in `map` | goal types reported as reached in [Nav2 with the steering base](sim_pick_place.md#nav2-with-the-steering-base-observed-2026-11-21) |
| | `nav_timeout`, `handover_timeout`, `reach_timeout`, `hold_sec` | 120, 10, 40, 2 s | |

## Measured results (2026-11-21, house scene, RTX 6000 Ada workstation)

Times are wall clock from the node logs.

**The target needs the base.** A level gripper at the target's y and z
reaches TCP x = 0.593 m from the spawn pose, which is 0.357 m short of 0.95
(`test_wbmpc_launch.py` solves this). With the base box collapsed onto the
start, the MPC ends 0.199 m away (planning spike; `test_whole_body_mpc_core.py`
asserts > 0.10 m). From the box edge (x = 0.52) the arm reaches 0.640 m,
more than the 0.43 m needed.

**Reaches in Gazebo:**

| Run | MPC starts at (Nav2 stop) | Time to REACHED | Final error | Base travel under the MPC | Solves (median / max) |
|---|---|---|---|---|---|
| Parked, `test_wbmpc_sim.py` | (0, 0, 0°) | 4.07 s | 4.8 mm, 0.2° | 0.379 m | 81 (12 / 56 ms) |
| Nav2 → MPC (Task 14 run) | (−0.205, −0.026, −1°) | 5.93 s | 4.7 mm, 0.2° | 0.557 m | 118 (13 / 57 ms) |
| Nav2 → MPC, while recording | (−0.253, −0.040, −1°) | 8.33 s | 4.8 mm, 0.2° | 0.607 m | 127 (13 / 59 ms) |
| Parked, while recording | (0, 0, 0°) | 5.52 s | 4.7 mm, 0.2° | 0.382 m | 82 (12 / 54 ms) |

- The base slid diagonally to about (0.34, 0.15) and kept its heading: the
  launch test asserts < 0.02 rad of yaw change and every `/odom` pose inside
  the box. The adapter switched once into PARALLEL and once back, with no
  mode flapping.
- Only the first solve after activation is slower than the 50 ms period
  (53–59 ms, the node warns once). The rest take 12–13 ms median.
- The handover settle took 0.38 s and the hand-back settle 0.32 s. After the
  run, `ros2 control list_controllers` showed `piper_arm_controller active`
  and `piper_arm_stream_controller inactive`, and the gate was in NAVIGATE.
- Nav2 legs (Task 14): drive away 10.7 s, stopped at (−1.741, 0.032, 180°);
  to the handover pose 10.0 s, stopped 0.207 m short at (−0.205, −0.026, −1°),
  which the MPC absorbs; drive away again, succeeded in 12.4 s.
- While recording, the camera rig shares the GPU and CPU, so the reaches
  take longer: 8.3 s instead of 5.9 s after Nav2, and 5.5 s instead of 4.1 s
  parked.

**Streamed setpoints alone** (scratch check before any MPC code: joint 1
following 0.2·sin(2π·0.2·t) at 20 Hz): worst |command − measured| was
0.021 rad at the 0.25 rad/s peak. Neither controller switch caused a
measurable jump, and the trajectory controller was active again afterwards.

**Gate only**: the house pick-and-place (`pick_place_demo.launch.py
execute:=true`) through `handover_gate:=true` completed in 125.6 s wall time,
against the 124 s of sim time documented for the direct path.
`/cmd_vel_gated` had exactly one publisher.

## Known limitations

- No obstacle or self-collision constraints in the OCP. The base is confined
  by the configured box, and the target and the posture term keep the arm away
  from the body. `test_wbmpc_launch.py` checks the default target and box
  against the house world: the target is ≥ 0.15 m above the table, no table
  object stands within 0.10 m of it, the finger pads pass above every object
  within 0.20 m, and the box stays short of the pick's approach line. The
  chair clearance (y ≥ −0.20 vs `ChairA_01_001` at y −0.86) is only reasoned
  in `config/wbmpc.yaml`. Another target needs the same checks before it is
  used.
- The box comes from the config, not from a costmap.
- The sim body is moved kinematically (VelocityControl). The wheels steer and
  roll like the driver's, but there is no slip, and the steering timing is not
  measured on the robot.
- `odom` is the world frame only because the robot spawns at the origin and
  the sim odometry has no noise.
- Solve times are from a workstation, not a Jetson AGX Orin. PARALLEL mode has
  never been run on the hardware, and the real PiPER position streaming and
  CAN latency are not modelled.
- Nav2's lifecycle bringup occasionally stalls on a lost service response
  ("failed to send response to /controller_server/change_state (timeout)").
  It happened once in three starts; restarting the sim fixed it.
- One command per solve at 20 Hz, no interpolation in between.
