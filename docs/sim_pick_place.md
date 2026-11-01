# Gazebo Fortress Pick-and-Place — Status & Notes

Scope: simulation only (`ranger_sim`). Hardware is unaffected.

## Status (verified headless, Gazebo Sim 6.16 / Humble)

`pick_place_demo.launch.py execute:=true` on the garden world runs the full
sequence and the cup is physically lifted (~0.19 m) and put back where it
was (final offset a few millimetres, no fling). Every MoveIt step returns
SUCCESS and the joints actually reach their targets.

```bash
ros2 launch ranger_sim sim_bringup.launch.py                      # full sim (starts paused, see below)
ros2 launch ranger_sim pick_place_demo.launch.py                  # plan-only (safe)
ros2 launch ranger_sim pick_place_demo.launch.py execute:=true    # run it in Gazebo
```

## How the grasp works

- **Simulator-native fixed joint.** `ranger_gazebo.urdf.xacro` emits one
  `DetachableJoint` plugin per object listed in
  `config/graspable_objects.yaml` (parent link `piper_link6`). The joint is
  created/removed on demand and keeps the object's current pose relative to
  the gripper. There is no friction grasping.
- **`/grasp/set` service** (`ranger_sim/srv/SetGrasp`: `object`, `attach`)
  is the only interface task code uses; `scripts/grasp_manager.py` serves
  it by publishing on the bridged `/grasp/<obj>/{attach,detach}` topics and
  confirming the plugin's `/grasp/<obj>/state` (`attached`/`detached`).
- **Paused start.** Fortress 6.16 has no `initial_attach`, so every grasp
  joint is created *attached* when the robot spawns. Gazebo now starts
  paused (`paused:=true`, default in `gazebo.launch.py` / `sim_bringup`);
  `grasp_manager` waits for the robot, re-sends `detach` to every graspable
  object present in the world until each confirms, then unpauses the world
  (`/world/<world_name>/control`). Detach is processed while paused. The
  first iterations after spawn are blocked by sensor/rendering start-up, so
  the confirmation can take several seconds headless; the wait defaults to
  20 s (`grasp_detach_timeout:=<s>` on `sim_bringup` / `spawn_robot`, raise
  it on slow GPUs or the Jetson). The world is unpaused even if a step times
  out (logged as an error). Controller spawner timeouts are derived from that
  value (+25 s) because the controller_manager only updates once simulation
  time runs.
- **Recovery.** The plugin only reports transitions and ignores redundant
  requests. A detach that gets no answer while simulation time advances is
  therefore taken as "already detached"; an attach is never inferred (an
  object in an unknown state is detached first). Restarting `grasp_manager`
  against a running sim is ready in ~2 s.
- **World name.** `world_name` must match `<world name>` in the SDF (used for
  `/world/<name>/control`). On a mismatch `grasp_manager` logs an error and
  unpauses through the CLI with the name reported by `ign model --list`.
- **Gripper interface = MoveIt = real robot.** `piper_gripper_controller` is
  a `position_controllers/GripperActionController`
  (`/piper_gripper_controller/gripper_cmd`, `control_msgs/GripperCommand`),
  matching `moveit_controllers.yaml` and the real
  `piper_follow_joint_trajectory_bridge`. The second finger `piper_joint8`
  is a gz_ros2_control mimic of `piper_joint7`; gz_ros2_control 0.7.x
  exports it as `piper_joint8_mimic`, which `piper_mimic_state_broadcaster`
  publishes on its local topic and `mimic_joint_state_relay.py` republishes
  as `piper_joint8` on `/joint_states`.
- The object is attached **before** the fingers close, from a hover pose
  with ~5 cm clearance, so the joint is never created during contact.

## Root causes found while making it work

1. **Shoulder and elbow never moved in sim (pre-existing).** `piper_joint2`
   (lower limit 0) and `piper_joint3` (upper limit 0) started exactly at
   their limits. DART 6.12 skips the servo motor of a DOF that is beyond its
   limit by any amount (zero error allowance); gravity held both joints a
   few nanometres past the limit, so position commands were silently
   ignored while `JointTrajectoryController` still reported success at the
   end of each trajectory (it has no goal tolerances). The old demo's
   "every MoveIt step returns SUCCESS" was therefore true while only the
   wrist moved. Fix: `initial_value` 0.02 / −0.02 rad for these joints in
   the sim `ros2_control` block. The smoke test asserts it.
2. **Waypoints collided with the table once the arm really moved.** With
   the base at the table, `home` puts the gripper under the table top and
   the old pre-grasp pose put the open fingers exactly on the cup surface.
   The demo now raises the arm to the `lift` carry pose before approaching,
   attaches from a hover pose (TCP ≈ 5 cm above the cup top, solved with
   MoveIt IK), and backs the base away before retracting to `home`.
3. **Humble action/discovery race.** The first gripper goal's acceptance
   could be lost right after `wait_for_server()`; the demo waits for
   discovery to settle and re-sends gripper goals (idempotent) once. It
   also waits for the arm to stop moving before the next MoveIt goal
   (MoveIt rejects a start state off by more than 0.01 rad).

## History: the teleport "magnet" (resolved, removed)

Earlier attempts used friction (does not hold in Fortress), a
`DetachableJoint` without the paused start (the joint formed at spawn and
dragged the cup), and `grasp_magnet.py`, which teleported the cup after the
gripper with `/world/<world>/set_pose` and ended up rigidly offset by
roughly the base travel. All of it is superseded by the DetachableJoint +
paused start + `/grasp/set` design above; `grasp_magnet.py` and the
`/gripper/grasp` / `/gz_world_poses` bridges were deleted.

## Known limitations

- `colcon test` with `RANGER_SIM_GPU=1` can leave an orphaned headless
  `ign gazebo` server behind: launch_testing stops the `ruby ign` wrapper
  with SIGTERM when the server takes longer than 5 s to exit, and the
  server process survives. Check with `pgrep -af "ign gazebo"` after the
  test and stop it (`kill -INT <pid>`). The tests themselves run isolated
  (`ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=99`, gz partition
  `ranger_sim_test`, set in `CMakeLists.txt`); without `RANGER_SIM_GPU=1`
  the smoke test is skipped.
- The base approach is still open-loop on `/odom` (Nav2 goals come with the
  task executor in a later phase); the base can be nudged a few
  centimetres by arm contacts.

## Key files

- `src/ranger_sim/launch/sim_bringup.launch.py` — full sim (`paused`,
  `world_name` args)
- `src/ranger_sim/launch/gazebo.launch.py` — paused start
- `src/ranger_sim/launch/spawn_robot.launch.py` — bridges, grasp manager,
  mimic relay, controller spawners
- `src/ranger_sim/config/graspable_objects.yaml` — graspable objects (single
  source of truth)
- `src/ranger_sim/config/gz_bridge.yaml` — incl. the generated
  `/grasp/<obj>/*` section (kept in sync by `test_grasp_manager.py`)
- `src/ranger_sim/config/ros2_controllers_sim.yaml` — GripperActionController,
  mimic broadcaster
- `src/ranger_sim/urdf/ranger_gazebo.urdf.xacro` — DetachableJoint macro,
  joint8 mimic, joint2/3 initial values
- `src/ranger_sim/scripts/grasp_manager.py`,
  `src/ranger_sim/scripts/mimic_joint_state_relay.py`,
  `src/ranger_sim/scripts/pick_place_demo.py`
- `src/ranger_sim/worlds/garden_world.sdf` — cup at the table front edge
