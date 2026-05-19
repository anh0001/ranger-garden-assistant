# Gazebo Fortress Pick-and-Place — Status & Notes

Scope: simulation only (`ranger_sim`). Hardware is unaffected.

## What works (verified in live Gazebo Fortress on a GUI display)

- **Sim bringup.** Two real gaps were found and fixed:
  - `spawn_robot.launch.py` never spawned the ros2_control controllers
    (`joint_state_broadcaster`, `piper_arm_controller`, the two gripper
    controllers). Chained spawners were added after entity creation.
  - `/joint_states` only carried the 8 PiPER joints, so MoveIt's planning
    scene monitor reported *"complete state not yet known"* and refused to
    plan. `base_joint_state_publisher.py` publishes the passive Ranger
    wheel/steering joints; it is wired into `spawn_robot.launch.py`.
- **Base mobility** — open-loop `/cmd_vel` drive to a world-x approach pose
  (odom feedback).
- **PiPER arm via MoveIt** — joint-space waypoints (`home → pre-grasp →
  lift`) derived from MoveIt FK so every TCP pose is verified reachable.
  Cartesian/IK pose goals were tried first but fail at this arm's
  workspace edge (OMPL "Unable to sample valid states for goal tree").
- **Gripper** open/close via the gripper `FollowJointTrajectory` action.
- **`pick_place_demo`** orchestrates the full sequence and defaults to
  **plan-only** (`execute:=false`) — safe, no motion, validates planning.
  `execute:=true` runs it in Gazebo (base + arm + gripper all execute
  successfully, every MoveIt step returns SUCCESS).
- Cup placed at the table front edge (`garden_world.sdf`) so it is within
  the small arm's reach after the base approach.

Run:

```bash
ros2 launch ranger_sim sim_bringup.launch.py            # full sim
ros2 launch ranger_sim pick_place_demo.launch.py        # plan-only (safe)
ros2 launch ranger_sim pick_place_demo.launch.py execute:=true
```

## What does NOT work yet: physically holding the object

A simple position-controlled gripper cannot hold an object by friction in
Fortress, so a grasp-fix mechanism is required. Four approaches were tried:

1. **Friction grasp** — fails (known Fortress limitation).
2. **`DetachableJoint` system plugin** (the documented Fortress plugin;
   implemented per Codex guidance). Blocked: in gz-sim6 it *starts
   attached*, so the joint forms at spawn and the arm's initial controller
   settle drags the cup off the table *before any ROS node can detach it*.
   Its Empty trigger / Boolean state also did not bridge reliably. Not
   fixable from the ROS side.
3. **Pose-follow "virtual attach" via TF** (`grasp_magnet.py`) — a node
   that teleports the cup to follow the gripper using the
   `/world/<world>/set_pose` service. The gripper pose was read from ROS
   TF in the `odom` frame, which is **not** the Gazebo world frame → cup
   flung to garbage coordinates.
4. **Pose-follow via the bridged gz pose stream** (current code). Fixes,
   in order, each uncovered the next layer:
   - Source the gripper pose from `/gz_world_poses` (bridged
     `/world/garden_world/dynamic_pose/info`) — same stream as the cup.
   - Movement threshold so the cup does not jitter at rest.
   - Real **place + release** step so it is a pick *and* place and the
     magnet stops following (no "cup flies forever").
   - `set_pose` moved to a **worker thread** (it shells out and was
     blocking the single-threaded executor, so the follow loop never ran).
   - Compose `world←ranger ∘ ranger←piper_link6` because
     `dynamic_pose/info` gives child-link poses **relative to the model**,
     not world.

   Result: the cup is now **stable and smoothly tracks the gripper** (no
   flying, no jitter) — but at a **constant ~0.6 m offset**.

## Precise remaining root cause

`dynamic_pose/info` reports the **`ranger` model root frame fixed at the
spawn origin**. The base is driven by the DiffDrive plugin, which moves
`base_footprint`, **not** the model root. Composing the static model root
with the link-relative pose therefore misses the base's driven
displacement. The cup ends up rigidly offset from the true gripper world
pose by roughly the base travel.

## Recommended next step (for whoever resumes)

Get the gripper's **absolute world pose** from a source that reflects the
driven base, not the static model root. Options, simplest first:

1. Compose **`base_footprint`'s world pose** (from `/odom` or the gz pose
   stream entry for `base_footprint`) with the static
   `base_footprint → piper_link6` transform (from `/tf` / robot_state_
   publisher). `grasp_magnet` already has `_compose`.
2. Or subscribe to `/world/garden_world/pose/info` (non-`dynamic`) and use
   the absolute `piper_link6` pose if present there.
3. Or query the link world pose via a gz Transport service each cycle.

Once the gripper world pose is correct, the existing `grasp_magnet`
machinery (threaded `set_pose`, movement threshold, HOLD/RELEASE,
place-and-release, retract-clear) should yield a clean pick-and-place,
since the same pipeline already tracks smoothly — only the source frame
is wrong.

## Key files

- `src/ranger_sim/launch/sim_bringup.launch.py` — full sim
- `src/ranger_sim/launch/spawn_robot.launch.py` — controller spawners +
  base joint publisher (the two bringup fixes)
- `src/ranger_sim/scripts/base_joint_state_publisher.py`
- `src/ranger_sim/scripts/pick_place_demo.py` — MoveIt motion + sequence
- `src/ranger_sim/scripts/grasp_magnet.py` — pose-follow grasp helper
- `src/ranger_sim/launch/pick_place_demo.launch.py`
- `src/ranger_sim/config/gz_bridge.yaml` — incl. `/gz_world_poses`,
  `/gripper/grasp` bridges
- `src/ranger_sim/worlds/garden_world.sdf` — cup at the reachable table
  front edge
