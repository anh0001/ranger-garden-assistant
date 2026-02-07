# MoveIt Setup Assistant Guide - Ranger + PiPER

Step-by-step guide to create a MoveIt 2 configuration for the Ranger Garden Assistant using the containerized MoveIt workspace.

## Recommended Approach: ros2-moveit-workspace

**Repository**: https://github.com/anh0001/ros2-moveit-workspace

This containerized development environment solves MoveIt Setup Assistant crashes and provides a reproducible setup with patched RViz components.

---

## Setup Steps

### Step 1: Copy Robot Description Packages

Copy the required description packages from this workspace to the MoveIt workspace:

```bash
cd ~/codes/ros2-moveit-workspace/src
cp -r ~/codes/ranger-garden-assistant/src/ranger_description .
cp -r ~/codes/ranger-garden-assistant/src/piper_ros/src/piper_description .
```

`ranger_complete.urdf.xacro` references meshes from `piper_description`, so that package must be present for the model to load.

### Step 2: Open in VSCode Dev Container

```bash
cd ~/codes/ros2-moveit-workspace
code .
```

Press **F1** and select **"Dev Containers: Rebuild and Reopen in Container"**

### Step 3: Build Workspace and Launch Setup Assistant

Inside the container terminal:

```bash
cd /workspace
colcon build --symlink-install --packages-select ranger_description piper_description
source install/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

### Step 4: Load Robot URDF

In the Setup Assistant GUI:
1. Click **"Create New MoveIt Configuration Package"**
2. Click **"Browse"**
3. Navigate to: `/workspace/src/ranger_description/urdf/ranger_complete.urdf.xacro`
4. Click **"Load Files"**
5. Wait for robot model to appear in 3D viewer

### Step 5: Configure in Setup Assistant

See **"Detailed Configuration Steps"** section below for complete GUI configuration instructions.

### Step 6: Copy Configuration to Main Workspace

```bash
# On host machine (outside container)
cp -r ~/codes/ros2-moveit-workspace/src/ranger_piper_moveit ~/codes/ranger-garden-assistant/src/

# Build in main workspace
cd ~/codes/ranger-garden-assistant
colcon build --packages-select ranger_piper_moveit --symlink-install
source install/setup.bash
```

### Step 7: Test the Configuration

```bash
# Demo with fake hardware
ros2 launch ranger_piper_moveit demo.launch.py

# With real hardware
ros2 launch robofi_bringup ranger_complete_bringup.launch.py
ros2 launch ranger_piper_moveit move_group.launch.py
```

---

## Troubleshooting

### Setup Assistant Issues

| Problem | Solution |
|---------|----------|
| URDF load fails | Check file path and rebuild workspace |
| Joints missing | Verify `piper_` prefix in joint names |
| Crash on start | Use ros2-moveit-workspace container |
| Package not found | Run `colcon build` and `source install/setup.bash` |

### Build/Runtime Issues

| Problem | Solution |
|---------|----------|
| Package build fails | `rosdep install --from-paths src --ignore-src -r -y` |
| Move group error | Check `ros2_controllers.yaml` configuration |
| Planning fails | Regenerate collision matrix |

---

## Quick Reference

### Planning Groups
- **piper_arm**: Joints 1-6, KDL solver
- **piper_gripper**: `piper_joint7` (with `piper_joint8` passive/mirrored), no solver
- **mobile_base** (optional): `virtual_joint` only, no solver
- **mobile_manipulator** (optional): combines `mobile_base` + `piper_arm`

### Workspace Aliases (in container)
- `cb` — builds all packages
- `cbs package_name` — builds specific package
- `setup` — sources workspace

---

## Detailed Configuration Steps

For detailed step-by-step instructions for each configuration tab, see the sections below.

---

### Step 3: Self-Collisions Tab

Generate the collision matrix to disable collision checking between adjacent/never-colliding links.

1. Click **"Self-Collisions"** in the left sidebar
2. Set **Sampling Density** to `10000` (default)
   - Higher values = more accurate but slower
   - 10000 is good for most robots
3. Click **"Generate Collision Matrix"**
4. Wait for completion (1-2 minutes)
5. Review the matrix - green cells indicate disabled collision pairs
6. Click **"Next"**

**What this does:** MoveIt samples random robot configurations and determines which link pairs can never collide, improving planning performance.

---

### Step 4: Virtual Joints Tab

Define how the robot connects to the world frame.

1. Click **"Virtual Joints"** in the left sidebar
2. Click **"Add Virtual Joint"**
3. Fill in the fields:
   - **Virtual Joint Name**: `virtual_joint`
   - **Child Link**: `base_footprint`
   - **Parent Frame Name**: `map` *(or `odom` if you are not running SLAM/localization yet)*
   - **Joint Type**:
     - `fixed` for arm-only planning
     - `planar` if you want MoveIt to plan base + arm together
4. Click **"Save"**

**What this does:** `fixed` keeps the base stationary for arm-only planning. `planar` adds x/y/yaw base motion to the planning scene for mobile manipulation.

---

### Step 5: Planning Groups - ARM

**This is critical!** Your arm joints have the `piper_` prefix.

1. Click **"Planning Groups"** in the left sidebar
2. Click **"Add Group"**
3. Fill in the group configuration:
   - **Group Name**: `piper_arm`
   - **Kinematic Solver**: `kdl_kinematics_plugin/KDLKinematicsPlugin`
   - **Group Default Planner**: `RRTConnect` (or leave default)
   - **RVIZ Marker Scale**: `0.3`

4. Click **"Add Kin. Chain"** button (in the middle section)

5. In the kinematic chain dialog:
   - **Base Link**: Select `piper_base_link`
     *(First actuated joint `piper_joint1` is attached here; `piper_world` is a fixed mount link)*
   - **Tip Link**: Select `piper_link6`
     *(This is the wrist link, before the gripper)*

6. Click **"Save"** (in the kinematic chain dialog)
7. Click **"Save"** (in the main group dialog)

**Expected joints in chain:** `piper_joint1`, `piper_joint2`, `piper_joint3`, `piper_joint4`, `piper_joint5`, `piper_joint6`

---

### Step 6: Planning Groups - GRIPPER

1. Still in **"Planning Groups"**, click **"Add Group"**
2. Fill in the group configuration:
   - **Group Name**: `piper_gripper`
   - **Kinematic Solver**: `None`
     *(Grippers don't need inverse kinematics)*
   - Leave other fields default

3. Click **"Add Kin. Chain"** button (in the middle section)

4. In the kinematic chain dialog:
   - **Base Link**: Select `piper_gripper_base`
   - **Tip Link**: Select `piper_link7`

5. Click **"Save"** (in the kinematic chain dialog)
6. Click **"Save"** (in the main group dialog)

**Note:** The URDF includes `piper_joint8` for the mirrored finger. Keep it passive (Step 9) and only include `piper_joint7` in the gripper group.

---

### Step 6B: Planning Groups - MOBILE BASE (Optional)

If you set the virtual joint to `planar`, add a base-only group:

1. Click **"Add Group"**
2. Fill in:
   - **Group Name**: `mobile_base`
   - **Kinematic Solver**: `None`
3. Click **"Add Joints"**
4. Select `virtual_joint`
5. Click **"Save"**

---

### Step 6C: Planning Groups - MOBILE MANIPULATOR (Optional)

Create a combined group so MoveIt can plan base + arm together:

1. Click **"Add Group"**
2. Fill in:
   - **Group Name**: `mobile_manipulator`
   - **Kinematic Solver**: `kdl_kinematics_plugin/KDLKinematicsPlugin`
3. Click **"Add Subgroup"**
4. Select `mobile_base`, then click **"Add"**
5. Click **"Add Subgroup"** again
6. Select `piper_arm`, then click **"Add"**
7. Click **"Save"**

**Hybrid Nav2 + MoveIt workflow:** Use Nav2 to get close to the object, then plan with the `mobile_manipulator` group for the final base + arm alignment. The base portion of the plan still needs to be executed by Nav2 or a custom bridge/controller (see Step 10 note).

---

### Step 7: Robot Poses - Define Useful Presets

Define preset joint configurations for common poses.

#### Pose 1: Home Position (Arm)

1. Click **"Robot Poses"** in the left sidebar
2. Click **"Add Pose"**
3. Configure:
   - **Pose Name**: `home`
   - **Planning Group**: `piper_arm`
4. Set all joint values to `0.0` (or adjust to your preferred home position):
   - `piper_joint1`: `0.0`
   - `piper_joint2`: `0.0`
   - `piper_joint3`: `0.0`
   - `piper_joint4`: `0.0`
   - `piper_joint5`: `0.0`
   - `piper_joint6`: `0.0`
5. Click **"Save"**

#### Pose 2: Stowed Position (Arm) - Optional but Recommended

This is useful for navigation - keeps the arm folded and out of the way.

1. Click **"Add Pose"**
2. Configure:
   - **Pose Name**: `stowed`
   - **Planning Group**: `piper_arm`
3. Set joint values to fold the arm vertically:
   - `piper_joint1`: `0.0`
   - `piper_joint2`: `1.57` (fold shoulder down, within limits)
   - `piper_joint3`: `-1.57` (fold elbow up, within limits)
   - `piper_joint4`: `0.0`
   - `piper_joint5`: `0.0`
   - `piper_joint6`: `0.0`
4. Click **"Save"**

**Note:** Keep `piper_joint2` in `[0, 3.14]` and `piper_joint3` in `[-2.967, 0]` per the URDF limits. You can test poses later in RViz.

#### Pose 3: Gripper Open

1. Click **"Add Pose"**
2. Configure:
   - **Pose Name**: `open`
   - **Planning Group**: `piper_gripper`
3. Set gripper joint to max open value:
   - `piper_joint7`: `0.035` (adjust based on your gripper's range)
4. Click **"Save"**

#### Pose 4: Gripper Closed

1. Click **"Add Pose"**
2. Configure:
   - **Pose Name**: `closed`
   - **Planning Group**: `piper_gripper`
3. Set gripper joint to closed value:
   - `piper_joint7`: `0.0`
4. Click **"Save"**

---

### Step 8: End Effectors

Define the gripper as an end effector attached to the arm.

1. Click **"End Effectors"** in the left sidebar
2. Click **"Add End Effector"**
3. Fill in:
   - **End Effector Name**: `piper_gripper`
   - **End Effector Group**: `piper_gripper` (select from dropdown)
   - **Parent Link**: `piper_link6` (the wrist link where gripper attaches)
   - **Parent Group**: `piper_arm` (select from dropdown)
4. Click **"Save"**

**What this does:** Links the gripper group to the arm, enabling coordinated manipulation planning.

---

### Step 9: Passive Joints

Mark joints that MoveIt should ignore during motion planning (like wheels and casters).

1. Click **"Passive Joints"** in the left sidebar
2. Click **"Add Passive Joint"** for each of these joints:

   **Wheel joints** (continuous rotation):
   - `fl_wheel`
   - `fr_wheel`
   - `rl_wheel`
   - `rr_wheel`

   **Steering joints** (omnidirectional wheels):
   - `fl_steering_joint`
   - `fr_steering_joint`
   - `rl_steering_joint`
   - `rr_steering_joint`

   **Gripper mirror joint**:
   - `piper_joint8`

3. Select each joint from the dropdown and click **"Save"**

**What this does:** Tells MoveIt these joints are not part of the manipulator and should be ignored during arm motion planning. `piper_joint8` mirrors `piper_joint7` in hardware/sim and should not be independently planned.

---

### Step 10: ROS 2 Controllers

Configure the ros2_control controllers for your planning groups.

1. Click **"ROS 2 Controllers"** in the left sidebar
2. Click **"Auto Add FollowJointsTrajectory Controllers For Each Planning Group"**

This will automatically create:
- `piper_arm_controller` for the arm
- `piper_gripper_controller` for the gripper

If you created `mobile_base` or `mobile_manipulator`, **do not** keep their auto-generated controllers unless you have a dedicated base controller plugin that can execute planar base joints.

3. Review the auto-generated controllers:
   - **Controller Name**: `piper_arm_controller`
     - **Controller Type**: `FollowJointTrajectory`
     - **Joints**: `piper_joint1` through `piper_joint6`

   - **Controller Name**: `piper_gripper_controller`
     - **Controller Type**: `FollowJointTrajectory`
     - **Joints**: `piper_joint7`

4. If they look correct, no changes needed

**Note:** Ensure the gripper controller only includes `piper_joint7`; `piper_joint8` should remain passive/mirrored by the driver or sim. For hybrid Nav2 + MoveIt, execution of base motion is handled outside MoveIt (Nav2 or a custom bridge), so leave only arm/gripper controllers enabled.

---

### Step 11: Author Information

1. Click **"Author Information"** in the left sidebar
2. Fill in:
   - **Author Name**: Your name (e.g., `RoboFi Team`)
   - **Author Email**: Your email (e.g., `robofi@example.com`)
3. **Maintainer Name** and **Maintainer Email** will auto-populate

This information goes into the package.xml file.

---

### Step 12: Configuration Files - GENERATE PACKAGE

**This is the final step!**

1. Click **"Configuration Files"** in the left sidebar
2. Set the package save location:
   - Click **"Browse"** next to "Configuration Package Save Path"
   - Navigate to: `/workspace/src/` *(inside the dev container)*
   - Select the `src` folder

This ensures the generated package is in the MoveIt workspace so you can copy it back to `ranger-garden-assistant` in Step 6.

3. Set package name:
   - **New Package Name**: `ranger_piper_moveit`

4. Click **"Generate Package"**

5. Wait for generation to complete (you'll see a progress bar and then a success message)

6. Review the generated files in the output window

7. Click **"Exit Setup Assistant"**

**Generated package structure:**
```
ranger_piper_moveit/
├── config/
│   ├── ranger_mini_complete.srdf              # Semantic robot description
│   ├── joint_limits.yaml                      # Joint velocity/acceleration limits
│   ├── kinematics.yaml                        # IK solver configurations
│   ├── moveit_controllers.yaml                # MoveIt controller mappings
│   ├── moveit.rviz                            # RViz config for MoveIt
│   ├── pilz_cartesian_limits.yaml             # Cartesian motion limits
│   ├── sensors_3d.yaml                        # 3D sensor configuration (optional)
│   └── initial_positions.yaml                 # Default joint positions
├── launch/
│   ├── demo.launch.py                         # Demo with fake controllers
│   ├── move_group.launch.py                   # MoveIt move_group node
│   ├── setup_assistant.launch.py              # Re-open Setup Assistant
│   └── ...
├── CMakeLists.txt
└── package.xml
```

---

## Step 13: Build the New Package

After exiting the Setup Assistant:

```bash
cd /home/robofi/codes/ranger-garden-assistant
source /opt/ros/humble/setup.bash
colcon build --packages-select ranger_piper_moveit --symlink-install
source install/setup.bash
```

---

## Step 14: Test the Configuration

### Test 1: Demo Mode (Fake Hardware)

Launch the demo to test motion planning without real hardware:

```bash
ros2 launch ranger_piper_moveit demo.launch.py
```

**In RViz:**
1. In the "MotionPlanning" panel, select planning group `piper_arm`
   - If you created it, use `mobile_manipulator` to plan base + arm together
2. Drag the interactive marker to set a goal pose
3. Click **"Plan"** to compute a trajectory
4. Click **"Execute"** to visualize the motion
5. Try planning to the preset poses (`home`, `stowed`) using the dropdown

### Test 2: With Real Hardware

To use with your actual PiPER arm:

1. Launch your robot base and arm driver:
   ```bash
   ros2 launch robofi_bringup ranger_complete_bringup.launch.py
   ros2 launch piper_ros start_single_piper.launch.py can_port:=can_piper
   ```

2. Launch MoveIt move_group (without demo mode):
   ```bash
   ros2 launch ranger_piper_moveit move_group.launch.py
   ```

3. Launch RViz with MoveIt plugin:
   ```bash
   ros2 launch ranger_piper_moveit moveit_rviz.launch.py
   ```

---

## Common Issues and Fixes

### Issue 1: Controllers Not Found

If MoveIt can't find the controllers, edit:
- `ranger_piper_moveit/config/moveit_controllers.yaml`

Ensure controller names match your actual ros2_control configuration.

### Issue 2: Wrong Joint Limits

Edit:
- `ranger_piper_moveit/config/joint_limits.yaml`

Adjust velocity and acceleration limits to match your hardware specifications.

### Issue 3: IK Solver Fails

Try different solvers in:
- `ranger_piper_moveit/config/kinematics.yaml`

Options:
- `kdl_kinematics_plugin/KDLKinematicsPlugin` (default, fast)
- `srv_kinematics_plugin/SRVKinematicsPlugin` (more accurate)
- `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` (best, requires installation)

### Issue 4: Setup Assistant Crashes

See [TROUBLESHOOTING_MOVEIT_SETUP_ASSISTANT.md](TROUBLESHOOTING_MOVEIT_SETUP_ASSISTANT.md)

---

## Next Steps

After successfully creating the MoveIt config:

1. **Tune parameters**: Adjust planning parameters in `config/ompl_planning.yaml` for your specific use case
2. **Add perception**: Configure 3D sensors in `config/sensors_3d.yaml` for collision avoidance
3. **Create custom launch files**: Integrate MoveIt with your navigation stack
4. **Write motion planning code**: Use MoveIt's Python or C++ API to program manipulation tasks

---

## Reference Links

- [MoveIt 2 Documentation](https://moveit.picknik.ai/humble/index.html)
- [MoveIt Setup Assistant Tutorial](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html)
- [MoveIt 2 Tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html)
- [MoveIt2 Docker Documentation](https://moveit.picknik.ai/humble/doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu.html)
- [gui-docker script source](https://github.com/moveit/moveit2_tutorials/tree/main/.docker)
- [ros2_control Documentation](https://control.ros.org/humble/index.html)

---

## Troubleshooting

For issues specific to the Setup Assistant crashing, see:
- [TROUBLESHOOTING_MOVEIT_SETUP_ASSISTANT.md](TROUBLESHOOTING_MOVEIT_SETUP_ASSISTANT.md)

For general MoveIt issues, check:
- MoveIt logs: `~/.ros/log/`
- Controller status: `ros2 control list_controllers`
- Joint states: `ros2 topic echo /joint_states`
