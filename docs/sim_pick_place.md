# Gazebo Fortress Pick-and-Place — Status & Notes

Scope: simulation only (`ranger_sim`). Hardware is unaffected.

## Status (verified headless, Gazebo Sim 6.16 / Humble)

The default scene is the AWS Small House (`scene:=small_house`, see
[Small-house scene](#small-house-scene); its assets are fetched once):
`pick_place_demo.launch.py execute:=true` picks a scanned vitamin bottle off
the dining table, carries it 4 m west through the dining room and 2 m south
into the kitchen, and sets it down on the kitchen counter, 10 cm higher than
the table (final offset 4.5 mm from the planned spot, upright, resting at
the counter top), then drives back to within 1.1 mm of its start pose.
Every MoveIt step returns SUCCESS and the Gazebo log has no `[Err]` line.

```bash
ros2 launch ranger_sim sim_bringup.launch.py                      # full sim, the house (starts paused, see below)
ros2 launch ranger_sim pick_place_demo.launch.py                  # plan-only (safe)
ros2 launch ranger_sim pick_place_demo.launch.py execute:=true    # run it in Gazebo
```

`scene:=garden` (needs no fetched assets) runs the same demo on the
primitive garden world: `pick_place_demo.launch.py execute:=true` moves the cup
from table A to a second table, `table_b`: the base stops 15 cm short of
the cup with the gripper level at cup height, so the wrist camera has the
cup centred (see [Approach along the wrist camera](#approach-along-the-wrist-camera)),
then slides in until the open finger pads straddle the cup body. The cup is
attached, lifted and retracted upright into the level carry pose; the base
backs to the origin, turns left and drives to `table_b`, where the cup is
set down 1.5 cm in from the front edge (final offset 2 mm from the planned
spot), and the base drives back to where it started
(see [Transfer](#transfer)). Every MoveIt step returns SUCCESS and the
joints actually reach their targets; the base holds its position to
±0.2 mm while the arm works and stops within 5 mm of every pose.

```bash
ros2 launch ranger_sim sim_bringup.launch.py scene:=garden
ros2 launch ranger_sim pick_place_demo.launch.py scene:=garden execute:=true
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
- **Grasp geometry.** At the grasp pose the level gripper (approach +x,
  fingers closing along y) has its pads inside the 5 cm cup body
  (≥ 2.4 cm above the table, palm ≥ 1.2 cm from the cup, 1 cm per side
  between the open fingers and the cup). The pick reaches it horizontally
  (the base slides in at grasp height); the place comes straight down from
  the hover pose (pads ≥ 1.7 cm above the rim). The demo attaches, then
  closes to the cup radius (`gripper_closed:=0.025`) so the pads rest on
  its surface. The object is attached **before** the fingers close, so the
  joint is never created during contact. `test/test_pick_place_geometry.py`
  re-derives every clearance from the URDF and the world (forward
  kinematics, no simulator).

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
   The demo now raises the arm to a carry pose before approaching (`lift`
   then, the level `CARRY` since item 7) and backs the base away before
   retracting to `home`. (It then attached
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
7. **The gripper pointed up and the wrist camera never saw the cup.** The
   old carry pose `lift` had its approach axis at (0.63, 0, 0.78): the
   gripper pointed 51° upward and the held cup leaned back 51°. And the
   grasp came straight down from above, which left the cup 49–54° off the
   D405's optical axis, outside its 28° half vertical field of view. Not a
   URDF bug but waypoint choices; see
   [Approach along the wrist camera](#approach-along-the-wrist-camera).
8. **The first in-place turn stalled at 85°.** Turning in place drags the
   straight-locked wheels sideways, and below ~0.13 rad/s the sim base does
   not turn at all; the proportional turn slowed into that dead zone. See
   [Transfer](#transfer) for the measured dead zones and the floors.

## Approach along the wrist camera

The RealSense D405 rides on top of the gripper (`piper_camera_*`, the
calibrated mount in `ranger_complete.urdf.xacro`). With the gripper level
it looks 30° down along the approach: optical axis (0.86, 0.06, −0.51) in
`base_footprint`, aimed ~10 cm beyond the TCP.

- **Look, then approach.** The base stops `standoff_m` = 0.15 m short of the
  pick pose and the arm goes to the grasp pose there ("look at the
  object"). The object is then 4° off the optical axis at 0.20 m (D405
  depth range 7–50 cm): centred in the wrist image. Halfway through the
  slide-in it is 15–19° off, still inside the 28° half vertical field of
  view.
  The base then slides in at 0.08 m/s until the open fingers straddle it.
- **Carry level.** `CARRY` (TCP (0.340, 0, 0.950), level, retracted over
  the base; `tools/solve_level_waypoint.py 0.20 0.95`) replaces `lift`:
  the held object stays upright, 5 cm or more above both tables and
  behind the table edge whenever the base moves. HOVER, GRASP and CARRY
  share the sum of joints 2, 3 and 5, so the joint-space paths between
  them stay level too.
- **Wrist camera in ROS.** `config/gz_bridge.yaml` maps the sensor's gz
  topics to `/piper/wrist_camera/color/image_raw` and
  `/piper/wrist_camera/depth/image_rect_raw` (848×480 at 15 Hz, a native
  D405 mode). The recorded video uses it as its inset.
- `test_pick_place_geometry.py` checks all of it per scene: the carry pose,
  the camera view at the standoff (±5 mm base stop), the paths from CARRY
  at the standoff, and the held object on the way into the carry pose.
  `tools/solve_level_waypoint.py X Z` solves new level waypoints and is
  checked against the scene waypoints.

## Transfer

- **Scene keys.** Per scene in `config/pick_place_scenes.yaml`:
  `standoff_m`; `place_base_pose` `[x, y, yaw]` (the base at the place
  approach; the object goes down as far ahead of the base as it was picked);
  `place_route`, a flat `[x0, y0, yaw0, x1, …]` list of base poses between
  the pick standoff and the place standoff, driven in reverse on the way
  home (never empty: a ROS parameter cannot be an empty list); and
  `place_surface`, the world model the place spot must rest on.
  `load_scenes` rejects a scene without them. Optional:
  `place_hover_joints` / `place_grasp_joints`, the level hover and grasp
  at the place; they default to the pick's `hover_joints` / `grasp_joints`
  (`launch/pick_place_scenes.py` `OPTIONAL_KEYS`), which set the object down
  on a surface as high as the pick table, as far ahead of the base as it
  was picked. `load_scenes` rejects one that is not 6 floats.
- **Routes.**
  - garden: back to the origin, turn to 90°, drive to (0, 0.92), slide in to
    (0, 1.07). `table_b` is a copy of `table` at (0, 1.75), so the cup is
    set down at (0, 1.465), 1.5 cm in from the front edge as at table A;
    `planter_2` moved from (0, 1.5) to (1.2, 1.5) to make room.
  - small_house: dining table -> kitchen counter. Back to the origin, turn
    to 180°, drive 4 m west along y = 0, turn to −90°, drive to
    (−4.0, −1.824), slide in to (−4.0, −1.974); the bottle goes down at
    (−4.0, −2.454) on the counter, 3.8 cm in from its front face, between
    the sink and the cooktop. The counter (0.904 m) is 10.3 cm above the
    table, so the place has its own waypoints, re-solved level at link6
    x 0.370 (TCP (0.510, 0, 0.974) / (0.510, 0, 1.034)): 10.3 cm higher than
    the pick's, and 8 cm further out, because the counter is a cabinet down
    to the floor. The base cannot reach under it as it does under the
    tables, and its lidar sticks out 0.394 m ahead of `base_footprint`, so
    it stops 0.442 m short of the face (4.8 cm lidar clearance) and the arm
    reaches the extra 8 cm. The place hover is only 6 cm above the place
    grasp, not the pick's 9.1: joint 5 reaches its 1.22 rad limit near
    link6 z 1.05. The bottle hangs 6 cm above the counter during the
    slide-in. After the release the open pads still reach 4–14 mm down
    around its top, so the slow back-off slides them off it, the pick's
    slide-in reversed (8 mm per side).
- **Pose controller.** `drive_to_pose()` turns the base in place until its
  yaw is within 0.005 rad of the target, then translates it holonomically
  (`VelocityControl` takes vx, vy, wz), until it is within 5 mm and
  0.005 rad. While translating it turns back only beyond 0.015 rad, and
  turns a last time once the position is reached: a `/cmd_vel` that arrives
  an update late carries a turn past the 0.005 rad band, and turning back
  every update made the base crawl (18 s instead of 6 s for 0.92 m with one
  update of latency, never at three; `test_pick_place_demo.py` runs every
  drive test with 0–2 updates of latency). The base is stopped when a leg
  ends for any reason, Ctrl-C included, and whenever `/odom` is older than
  0.5 s: it never drives on a stale pose. Odom is the world frame here: the robot spawns at the origin
  facing +x and the sim odometry has no noise. Nav2 is not used: its
  `xy_goal_tolerance` is 0.25 m and the place needs millimetres.
- **The sim base skids.** With the steering locked straight, turning in
  place and moving sideways drag the wheels over the ground. Measured with
  constant `/cmd_vel` commands in `garden_world`: nothing moves below
  ~0.13 rad/s (turn) or ~0.035 m/s (sideways), and above that the base
  moves at about (command − dead zone); forward and backward are exact.
  The controller's floors sit above the dead zones (0.35 rad/s turning,
  0.08 m/s sideways). A turn also drags the base ~0.1 mm per degree off its
  spot; the translation removes it. `test_pick_place_demo.py` runs the
  controller against a fake base with these dead zones.
- **Order of steps.** The arm is in CARRY on every base leg away from a
  table, at GRASP for the pick slide-in and at the place hover for the place
  slide-in and the back-off (`test_pick_place_demo.py` checks the sequence
  for every scene). At the place standoff the arm rises from CARRY to the
  place hover; in CARRY the held bottle is below the counter top, so it
  rises behind the counter's edge (`test_pick_place_geometry.py`).

## Base wheels

- **Kinematic base.** `VelocityControl` moves the base from `/cmd_vel`
  (vx, vy, wz) and `OdometryPublisher` reports it; the wheels are passive.
- **Steering lock (sim-only).** `ranger_gazebo.urdf.xacro` holds every
  `*_steering_joint` at 0 rad with one `JointPositionController` each
  (velocity-command mode, the same DART joint servo the arm uses). The
  wheel joints stay free and roll from ground contact. Turning in place
  and lateral `vy` make the aligned wheels skid, with a dead zone the
  transfer's pose controller works around (see [Transfer](#transfer));
  `nav2_sim.yaml` plans DiffDrive (vy = 0).
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

The default scene: the dining room and kitchen of the AWS RoboMaker Small
House (MIT-0, via TeamSOBITS' Fortress port) with Google Scanned Objects on
the dining table (CC BY 4.0; see `src/ranger_sim/models/THIRD_PARTY_ASSETS.md`).
The assets are fetched, not committed (~140 MB, git-ignored), once:

```bash
python3 src/ranger_sim/tools/fetch_sim_assets.py     # pinned commit / versions + sha256
colcon build --packages-select ranger_sim --symlink-install
ros2 launch ranger_sim sim_bringup.launch.py
ros2 launch ranger_sim pick_place_demo.launch.py                  # plan-only
ros2 launch ranger_sim pick_place_demo.launch.py execute:=true    # run it
```

Without the assets, `gazebo.launch.py` stops with the number of missing
models and the fetch command instead of starting an empty world; use
`scene:=garden` there.

- **Scenes.** `config/pick_place_scenes.yaml` holds, per scene, the world,
  the object, the base approach, the gripper closing, the hover/grasp
  waypoints (and the place's, optional), the transfer (standoff, place
  pose, route, place surface) and the video camera rig. The launch files
  take `scene:=` (default `small_house`; `garden` needs no fetched assets);
  explicit `world:=`, `world_name:=`, `object_name:=`, `base_approach_x:=`
  and `gripper_closed:=` still override it. An unknown scene fails at
  launch with the list of known ones.
- **The house is moved, not the robot.** `tools/generate_small_house_world.py`
  transforms the committed source world
  (`tools/aws_small_house/small_house.world`) rigidly so its kitchen table
  stands where the garden table did: front edge at world x = 0.90, the
  robot spawning at the origin facing +x. The demo's odom base poses and
  sim_bringup's identity `map -> odom` stay as they are. Edit
  `tools/small_house_world.template.sdf`, then regenerate;
  `test_generate_small_house_world.py` fails if the committed world is stale.
- **Primitive table collisions.** The AWS KitchenTable's collision mesh tops
  out at 0.8185 m but its visible top is at 0.801 m, so the world replaces
  it with an inline `table` (the AWS visual, box collisions at the visible
  top); the tableware that stood on it is re-seated on 0.801 m too. Without
  this, everything on the table floats 1.7 cm in the video.
- **Primitive counter collisions.** The same for the L-shaped kitchen
  cabinet, the place surface: its 32-vertex collision hull tops out at
  0.909–0.913 m (and wraps the faucet), its visible top is 0.9044 m. The
  inline `kitchen_counter` keeps the AWS visual at the cabinet's generated
  pose and puts world-axis-aligned boxes under it: the top slab
  (`top_collision`, 0.854–0.904 m), the body below it down to the floor, and
  the sink leg. The mesh is 0.29° off square, so each box face sits at the
  proud end of the visible face (front face y −2.416 in front of the
  robot's lane, sink leg x −4.540). The generator drops
  `KitchenCabinet_01_001`; `COUNTER_TOP_M` in the generator is checked
  against the template and the scene's place surface.
- **Kitchen lane.** Measured from the meshes: along y = 0 nothing stands
  within 0.45 m of the robot's line between the table and x = −4.0; on the
  lane x = −4.0 the sink leg's face is at x −4.54, 0.29 m from the base's
  side, the cooktop at x −3.57..−2.57 (0.43 m from the place spot, on the
  counter), the rangehood above it from z 1.49 m (the arm stays below
  1.15 m), the fridge at x ≥ −1.27. `test_generate_small_house_world.py`
  keeps every house model's origin out of both lanes; the base's own
  collisions (wheels, lidar, camera) are checked against the counter's
  boxes at the place approach (`test_pick_place_geometry.py`).
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

Recorded transfers, wrist-camera inset top left: the house, dining table to
kitchen counter (124 s of sim time), in
[sim_pick_place_small_house.mp4](sim_pick_place_small_house.mp4), and the
garden (65 s) in [sim_pick_place_garden.mp4](sim_pick_place_garden.mp4).

With the sim up (`sim_bringup.launch.py`, headless is fine, same `scene`):

```bash
ros2 launch ranger_sim record_pick_place_video.launch.py                 # the house
ros2 launch ranger_sim record_pick_place_video.launch.py output_path:=/tmp/demo.mp4
ros2 launch ranger_sim record_pick_place_video.launch.py scene:=garden
```

- Spawns the scene's camera rig (`models/video_rig.sdf` for the garden,
  `models/video_rig_small_house.sdf` for the house; one static wide camera,
  no collisions) at the world origin, bridges `/video/wide`, and runs
  `scripts/record_demo_video.py`. The result is a 1280×720 H.264 MP4
  (default `~/.ros/videos/pick_place_<time>.mp4`) with the robot's wrist
  camera (`/piper/wrist_camera/color/image_raw`) as the inset, the current
  demo step (from the `pick_place_demo` log), the grasp state and the sim
  time. The inset shows the object centred at "look at the object" and the
  held object at the bottom of the image while it is carried.
- Frames are placed on simulation time, one per 1/30 s, so playback runs at
  sim speed even when the extra cameras slow the sim down.
- The demo (`execute:=true`) starts only after the recorder has its first
  camera frame. `ros_gz_sim create` exits 0 even when no Gazebo answers, so
  the rig's frames are the gate that keeps this launch from moving a real
  arm. No frame within `first_frame_timeout` (default 60 s) ends the launch
  without running the demo.
- The launch ends when the demo exits; the recorder then closes the file.
- Each scene names its rig (`rig_file` in `config/pick_place_scenes.yaml`);
  the poses are tuned to that scene's tables and keep the pick, the place
  and the gripper path clear of the inset (top left). The garden camera
  looks down 43° from (1.20, 2.77, 2.80), beyond the garden border, so both
  tables and the route are in view and no table top hides the robot. The
  house camera looks south-west and 31° down from (1.9, 1.75, 2.75) in the
  dining room's north-east corner (horizontal FOV 1.5 rad): the dining
  table and the start pose 2 m away, the route west and the kitchen counter
  7 m away, seen past the robot's right side so the gripper stays in view
  at the counter. It is west of the folding door at x 2.3–3.4 (which
  would fill the foreground) and under the header over the room's open
  north side (y 2.04, from z 2.5). The cameras need GPU rendering, like the
  robot's own sensors.
- The status bar right-aligns the grasp state and the sim time and cuts the
  step caption to the space left, so long object names fit.
- The recorded transfer is 63 s of sim time in the garden and 124 s in the
  house (dining table to kitchen counter and back).

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
- The base moves by a pose controller on `/odom` (Nav2 goals come with the
  task executor in a later phase); base poses are world poses only because
  the robot spawns at the origin facing +x and the sim odometry is
  noise-free.
- The sim base has no drivetrain: turning in place and lateral `vy` skid the
  wheels (wheels visibly dragged, dead zones measured above), and the wheel
  collisions differ from the hardware description (concentric).
- The approach is scripted. The wrist camera's view is checked by FK and
  shown in the video; nothing reads the image yet (no visual servoing).
- At the house counter the place hover is only 6 cm above the place grasp
  (joint 5 limit): after the release the open fingers still straddle the
  top of the bottle, and the base backs off slowly to slide them off it.
- The counter spot is 7 m from the house camera: the place is small in the
  wide view; the wrist inset shows it up close.
- The waypoints are joint-space and per scene in
  `config/pick_place_scenes.yaml`; re-solve them with FK when moving an
  object (see `test_pick_place_geometry.py`, which checks every scene).

## Key files

- `src/ranger_sim/launch/sim_bringup.launch.py` — full sim (`scene`,
  `paused`, `world_name` args)
- `src/ranger_sim/launch/gazebo.launch.py` — paused start, model resource
  paths, missing-asset check
- `src/ranger_sim/config/pick_place_scenes.yaml`,
  `src/ranger_sim/launch/pick_place_scenes.py` — scenes and the launch
  helper that reads them (`SceneValue`, `from_scene`, `OPTIONAL_KEYS`)
- `src/ranger_sim/tools/fetch_sim_assets.py`, `tools/sim_assets.yaml` —
  pinned third-party asset download (`models/third_party/`, git-ignored)
- `src/ranger_sim/tools/generate_small_house_world.py`,
  `tools/small_house_world.template.sdf`, `tools/aws_small_house/` —
  generator, template (inline `table` and `kitchen_counter`) and source of
  `worlds/small_house_world.sdf`
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
  edge, `table_b` (the place target)
- `src/ranger_sim/tools/solve_level_waypoint.py` — planar IK for level arm
  waypoints (how `CARRY` was solved)
- `src/ranger_sim/test/test_pick_place_geometry.py` — FK clearances of the
  grasp, carry and place waypoints, the wrist camera's view, the place spot
  and the base's clearance to the surfaces it approaches for every scene;
  steering-lock / bridge / relay consistency
- `src/ranger_sim/test/test_pick_place_demo.py` — pose controller (ideal and
  skidding fake base, 0–2 updates of `/cmd_vel` latency, stale `/odom`,
  Ctrl-C), step sequence, plan-only safety
- `src/ranger_sim/test/test_pick_place_scenes.py`,
  `test_generate_small_house_world.py`, `test_fetch_sim_assets.py` —
  scenes and launch helper, world generator, fetch tool (offline)
- `src/ranger_sim/test/test_sim_smoke.py` — headless bringup incl. the
  drive-straight check and the wrist image (`RANGER_SIM_GPU=1`; the house
  once its assets are fetched, `RANGER_SIM_SCENE=garden` for the garden)
- `src/ranger_sim/launch/record_pick_place_video.launch.py`,
  `src/ranger_sim/scripts/record_demo_video.py`,
  `src/ranger_sim/models/video_rig.sdf`,
  `src/ranger_sim/models/video_rig_small_house.sdf` — demo video recording
