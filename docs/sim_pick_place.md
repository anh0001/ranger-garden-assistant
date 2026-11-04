# Gazebo Fortress Pick-and-Place — Status & Notes

Scope: simulation only (`ranger_sim`). Hardware is unaffected.

## Status (verified headless, Gazebo Sim 6.16 / Humble)

`pick_place_demo.launch.py execute:=true` on the garden world runs the full
sequence: the base rolls straight to the table on aligned wheels, the
gripper descends level so its finger pads straddle the cup body, and the
cup is lifted and put back where it was (final offset ~3 mm). Every MoveIt
step returns SUCCESS and the joints actually reach their targets; the base
holds its position to ±0.2 mm while the arm works.

```bash
ros2 launch ranger_sim sim_bringup.launch.py                      # full sim (starts paused, see below)
ros2 launch ranger_sim pick_place_demo.launch.py                  # plan-only (safe)
ros2 launch ranger_sim pick_place_demo.launch.py execute:=true    # run it in Gazebo
```

The same demo runs in a home dining room with `scene:=small_house` (see
[Small-house scene](#small-house-scene)): it picks a scanned vitamin bottle
off the kitchen table and sets it back within ~2.4 mm. `garden` stays the
default scene.

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
- **Grasp geometry.** The level gripper (approach +x, fingers closing along
  y) goes to a hover pose (pads ≥ 1.7 cm above the cup rim), descends
  straight down to the grasp pose (pads inside the 5 cm cup body,
  ≥ 2.4 cm above the table, palm ≥ 1.2 cm from the cup, 1 cm per side
  between the open fingers and the cup), attaches, then closes to the cup
  radius (`gripper_closed:=0.025`) so the pads rest on its surface. The
  object is attached **before** the fingers close, so the joint is never
  created during contact. `test/test_pick_place_geometry.py` re-derives
  every clearance from the URDF and the world (forward kinematics, no
  simulator).

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
   The demo now raises the arm to the `lift` carry pose before approaching
   and backs the base away before retracting to `home`. (It then attached
   from a hover pose ~5 cm above the cup top; item 4 replaced that.)
3. **Humble action/discovery race.** The first gripper goal's acceptance
   could be lost right after `wait_for_server()`; the demo waits for
   discovery to settle and re-sends gripper goals (idempotent) once. It
   also waits for the arm to stop moving before the next MoveIt goal
   (MoveIt rejects a start state off by more than 0.01 rad).
4. **The gripper never held the cup body.** The demo attached the cup from
   the hover pose, with the fingertips ~5 cm above the rim, so the cup hung
   welded under the gripper; and the PiPER's full 70 mm opening equalled
   the 7 cm cup diameter, so the fingers could not have straddled it
   anyway. The cup is now 5 cm wide and the demo descends from `hover` to
   a level `grasp` pose before attaching. Two more things decided the
   final numbers: the base used to stop anywhere within 3 cm of the
   approach pose (it now stops within 5 mm, with a speed floor so the
   last centimetres do not crawl), and Gazebo rests `base_footprint`
   ~1 cm below the ground, which the waypoints allow for.
5. **Wheels swivelled to random angles.** The `*_steering_joint`s are
   free revolute joints (no actuator, damping or caster trail) under a
   kinematically moved base, so ground contact turned them — to their
   ±2.1 rad limits already while the robot settled after spawning, which
   also slid the base ~6 cm sideways before the demo started.
6. **Aligned wheels exposed a cam.** Once the steering was locked the
   wheels rolled freely, and their collision cylinders sat 5 mm off the
   axle (`ranger_complete.urdf.xacro`), so ride height changed by 1 cm
   per revolution and the idle base crept up to a quarter metre into the
   low spots (2–3 mm per arm move during the demo). The sim now uses
   concentric wheel collisions (see *Base wheels*).

## Base wheels

- **Kinematic base.** `VelocityControl` moves the base from `/cmd_vel`
  (vx, vy, wz) and `OdometryPublisher` reports it; the wheels are passive.
- **Steering lock (sim-only).** `ranger_gazebo.urdf.xacro` holds every
  `*_steering_joint` at 0 rad with one `JointPositionController` each
  (velocity-command mode, the same DART joint servo the arm uses). The
  wheel joints stay free and roll from ground contact. Lateral `vy` makes
  the aligned wheels skid; `nav2_sim.yaml` plans DiffDrive (vy = 0), so
  nothing exercises that.
- **Concentric wheel collisions (sim-only).** `ranger_complete.urdf.xacro`
  takes `wheel_collision_z` (default −0.005, the historical value: the
  hardware URDF is unchanged); the sim wrapper passes 0.0.
- **Live wheel state.** A `JointStatePublisher` plugin publishes the 8
  base joints on gz `/ranger/base_joint_states`, bridged to ROS
  `/gz/base_joint_states`; `base_joint_state_publisher.py` relays them to
  `/joint_states` (zeros until the first message), so RViz shows the
  wheels rolling. The smoke test drives the base 0.3 m and asserts the
  steering stays at 0 and the base does not drift in y.

## History: the teleport "magnet" (resolved, removed)

Earlier attempts used friction (does not hold in Fortress), a
`DetachableJoint` without the paused start (the joint formed at spawn and
dragged the cup), and `grasp_magnet.py`, which teleported the cup after the
gripper with `/world/<world>/set_pose` and ended up rigidly offset by
roughly the base travel. All of it is superseded by the DetachableJoint +
paused start + `/grasp/set` design above; `grasp_magnet.py` and the
`/gripper/grasp` / `/gz_world_poses` bridges were deleted.

## Small-house scene

The dining room of the AWS RoboMaker Small House (MIT-0, via TeamSOBITS'
Fortress port) with Google Scanned Objects on the kitchen table (CC BY 4.0;
see `src/ranger_sim/models/THIRD_PARTY_ASSETS.md`). The assets are fetched,
not committed (~140 MB, git-ignored), once:

```bash
python3 src/ranger_sim/tools/fetch_sim_assets.py     # pinned commit / versions + sha256
colcon build --packages-select ranger_sim --symlink-install
ros2 launch ranger_sim sim_bringup.launch.py scene:=small_house
ros2 launch ranger_sim pick_place_demo.launch.py scene:=small_house                 # plan-only
ros2 launch ranger_sim pick_place_demo.launch.py scene:=small_house execute:=true   # run it
```

Without the assets, `gazebo.launch.py` stops with the number of missing
models and the fetch command instead of starting an empty world.

- **Scenes.** `config/pick_place_scenes.yaml` holds, per scene, the world,
  the object, the base approach, the gripper closing, the hover/grasp
  waypoints and the video camera rig. The launch files take `scene:=`
  (default `garden`); explicit `world:=`, `world_name:=`, `object_name:=`,
  `base_approach_x:=` and `gripper_closed:=` still override it. An unknown
  scene fails at launch with the list of known ones.
- **The house is moved, not the robot.** `tools/generate_small_house_world.py`
  transforms the committed source world
  (`tools/aws_small_house/small_house.world`) rigidly so its kitchen table
  stands where the garden table did: front edge at world x = 0.90, the
  robot spawning at the origin facing +x. The demo's base drive (odom x)
  and sim_bringup's identity `map -> odom` stay as they are. Edit
  `tools/small_house_world.template.sdf`, then regenerate;
  `test_generate_small_house_world.py` fails if the committed world is stale.
- **Primitive table collisions.** The AWS KitchenTable's collision mesh tops
  out at 0.8185 m but its visible top is at 0.801 m, so the world replaces
  it with an inline `table` (the AWS visual, box collisions at the visible
  top); the tableware that stood on it is re-seated on 0.801 m too. Without
  this, everything on the table floats 1.7 cm in the video.
- **Narrow objects only.** The PiPER opens 70 mm and the attach-before-close
  grasp needs >= 5 mm per side, so objects must be <= 60 mm wide; every GSO
  mug is 102-126 mm. The vitamin bottle is 54 mm, wrapped by
  `models/vitamin_bottle` (scan as visual, cylinder collision and inertia).
- **Waypoints.** The table is 3.1 cm higher than the garden's, so hover and
  grasp were re-solved by FK. The bottle is 2 mm wider than the cup, so the
  base stops 5 mm earlier (x = 0.535) to keep the palm >= 1 cm clear across
  the ±5 mm base stop. In the house, `base_footprint` rests at -0.020 m (the
  garden's 1 cm patio slab makes it -0.010 there); both are in the band
  `test_pick_place_geometry.py` checks.
- **Real-time factor:** 0.995 (min 0.990) headless on an RTX 6000 Ada with
  the LiDAR, the front camera and the wrist RGBD rendering, measured on the
  second start (the first one builds the ogre2 caches).

## Recording a video

With the sim up (`sim_bringup.launch.py`, headless is fine, same `scene`):

```bash
ros2 launch ranger_sim record_pick_place_video.launch.py
ros2 launch ranger_sim record_pick_place_video.launch.py output_path:=/tmp/demo.mp4
ros2 launch ranger_sim record_pick_place_video.launch.py scene:=small_house
```

- Spawns the scene's camera rig (`models/video_rig.sdf` for the garden,
  `models/video_rig_small_house.sdf` for the house; two static cameras, no
  collisions) at the world origin, bridges `/video/wide` and `/video/close`,
  and runs
  `scripts/record_demo_video.py`. The result is a 1280×720 H.264 MP4
  (default `~/.ros/videos/pick_place_<time>.mp4`) with a close-up inset, the
  current demo step (from the `pick_place_demo` log), the grasp state and
  the sim time.
- Frames are placed on simulation time, one per 1/30 s, so playback runs at
  sim speed even when the extra cameras slow the sim down.
- The demo (`execute:=true`) starts only after the recorder has its first
  camera frame. `ros_gz_sim create` exits 0 even when no Gazebo answers, so
  the rig's frames are the gate that keeps this launch from moving a real
  arm. No frame within `first_frame_timeout` (default 60 s) ends the launch
  without running the demo.
- The launch ends when the demo exits; the recorder then closes the file.
- Each scene names its rig (`rig_file` in `config/pick_place_scenes.yaml`);
  the poses are tuned to that scene's table and keep the object and the
  gripper path clear of the close-up inset (top left). The cameras need GPU
  rendering, like the robot's own sensors.
- The status bar right-aligns the grasp state and the sim time and cuts the
  step caption to the space left, so long object names fit.
- In the house, the 29 s of sim time of a recorded demo took ~105 s of wall
  time (measured once; the sim runs at ~0.93 with the rig cameras streaming
  and the arm idle).

## Known limitations

- `colcon test` with `RANGER_SIM_GPU=1` can leave an orphaned headless
  `ign gazebo` server behind: launch_testing stops the `ruby ign` wrapper
  with SIGTERM when the server takes longer than 5 s to exit, and the
  server process survives. Stopping `ros2 launch ranger_sim
  sim_bringup.launch.py` with Ctrl-C leaves it behind the same way. Check
  with `pgrep -af "ign gazebo"` afterwards and stop it (`kill -INT <pid>`).
  An orphan keeps serving `/controller_manager` on its `ROS_DOMAIN_ID`: a new
  sim on the same domain fails with "Controller already loaded … Failed to
  configure controller". `tr '\0' '\n' < /proc/<pid>/environ | grep
  ROS_DOMAIN_ID` shows which domain to avoid. The tests themselves run isolated
  (`ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=99`, gz partition
  `ranger_sim_test`, set in `CMakeLists.txt`); without `RANGER_SIM_GPU=1`
  the smoke test is skipped.
- The base approach is proportional control on `/odom` along x only (Nav2
  goals come with the task executor in a later phase); it relies on the
  base starting at y = 0, yaw = 0, which the steering lock now keeps.
- The sim base has no drivetrain: lateral `vy` skids the wheels, and the
  wheel collisions differ from the hardware description (concentric).
- The waypoints are joint-space and per scene in
  `config/pick_place_scenes.yaml`; re-solve them with FK when moving an
  object (see `test_pick_place_geometry.py`, which checks every scene).
- The wrist RGBD camera renders in Gazebo on `/piper/wrist_camera/image` and
  `/piper/wrist_camera/depth_image`, but `config/gz_bridge.yaml` bridges
  `/piper/wrist_camera/color/image_raw` and `/depth/image_rect_raw` without
  a `gz_topic_name`, so no wrist images reach ROS (true in both scenes).

## Key files

- `src/ranger_sim/launch/sim_bringup.launch.py` — full sim (`scene`,
  `paused`, `world_name` args)
- `src/ranger_sim/launch/gazebo.launch.py` — paused start, model resource
  paths, missing-asset check
- `src/ranger_sim/config/pick_place_scenes.yaml`,
  `src/ranger_sim/launch/pick_place_scenes.py` — scenes and the launch
  helper that reads them (`SceneValue`, `from_scene`)
- `src/ranger_sim/tools/fetch_sim_assets.py`, `tools/sim_assets.yaml` —
  pinned third-party asset download (`models/third_party/`, git-ignored)
- `src/ranger_sim/tools/generate_small_house_world.py`,
  `tools/small_house_world.template.sdf`, `tools/aws_small_house/` —
  generator, template and source of `worlds/small_house_world.sdf`
- `src/ranger_sim/models/{vitamin_bottle,supplement_bottle,sprinkles_jar}`,
  `src/ranger_sim/models/THIRD_PARTY_ASSETS.md` — GSO wrappers, attribution
- `src/ranger_sim/launch/spawn_robot.launch.py` — bridges, grasp manager,
  mimic relay, controller spawners
- `src/ranger_sim/config/graspable_objects.yaml` — graspable objects (single
  source of truth)
- `src/ranger_sim/config/gz_bridge.yaml` — incl. the generated
  `/grasp/<obj>/*` section (kept in sync by `test_grasp_manager.py`)
- `src/ranger_sim/config/ros2_controllers_sim.yaml` — GripperActionController,
  mimic broadcaster
- `src/ranger_sim/urdf/ranger_gazebo.urdf.xacro` — DetachableJoint macro,
  joint8 mimic, joint2/3 initial values, steering lock, base
  JointStatePublisher, concentric wheel collisions
- `src/ranger_description/urdf/ranger_complete.urdf.xacro` —
  `wheel_collision_z` arg
- `src/ranger_sim/scripts/grasp_manager.py`,
  `src/ranger_sim/scripts/mimic_joint_state_relay.py`,
  `src/ranger_sim/scripts/base_joint_state_publisher.py`,
  `src/ranger_sim/scripts/pick_place_demo.py`
- `src/ranger_sim/worlds/garden_world.sdf` — 5 cm cup at the table front
  edge
- `src/ranger_sim/test/test_pick_place_geometry.py` — FK clearances of the
  grasp waypoints for every scene, steering-lock / bridge / relay
  consistency
- `src/ranger_sim/test/test_pick_place_scenes.py`,
  `test_generate_small_house_world.py`, `test_fetch_sim_assets.py` —
  scenes and launch helper, world generator, fetch tool (offline)
- `src/ranger_sim/test/test_sim_smoke.py` — headless bringup incl. the
  drive-straight check (`RANGER_SIM_GPU=1`, `RANGER_SIM_SCENE=small_house`)
- `src/ranger_sim/launch/record_pick_place_video.launch.py`,
  `src/ranger_sim/scripts/record_demo_video.py`,
  `src/ranger_sim/models/video_rig.sdf`,
  `src/ranger_sim/models/video_rig_small_house.sdf` — demo video recording
