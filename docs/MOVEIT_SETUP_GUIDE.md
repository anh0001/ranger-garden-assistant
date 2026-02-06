# MoveIt Setup Assistant Guide for Ranger + PiPER

This guide walks you through creating a custom MoveIt 2 configuration for the Ranger Garden Assistant mobile manipulator using the MoveIt Setup Assistant GUI.

## 🐳 Quick Start with Docker (Recommended)

The MoveIt Setup Assistant crashes on Jetson/ARM platforms due to a RViz/Qt bug. The recommended solution is to run it in a Docker container with GUI support.

### Prerequisites

1. **Install Docker:**
   ```bash
   # Follow official Docker installation
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh
   sudo usermod -aG docker $USER
   # Log out and back in for group changes to take effect
   ```

2. **(Optional) Install NVIDIA Docker Runtime for GPU acceleration:**
   ```bash
   # For NVIDIA GPU support
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
   curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
   curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
       sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
       sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
   sudo apt-get update
   sudo apt-get install -y nvidia-container-toolkit
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker
   ```

### Docker Workflow

#### Step 1: Launch Docker Container with Setup Assistant

```bash
cd /home/robofi/codes/ranger-garden-assistant
./scripts/moveit_setup_assistant_docker.sh
```

This will:
- Pull the MoveIt2 Docker image (first time only)
- Mount your workspace at `/root/ws_moveit` in the container
- Start an interactive bash session with GUI support

#### Step 2: Inside Container - Launch Setup Assistant

```bash
# Inside the container
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

The MoveIt Setup Assistant GUI will open.

#### Step 3: Load Your Robot URDF

In the Setup Assistant GUI:

1. Click **"Create New MoveIt Configuration Package"**
2. Click **"Browse"** next to URDF field
3. Navigate to: `/root/ws_moveit/src/ranger_description/urdf/ranger_complete.urdf.xacro`
4. Click **"Load Files"**
5. Wait for the robot to load in the 3D viewer

#### Step 4: Configure MoveIt (Follow Steps 3-12 Below)

Continue with the detailed configuration steps from **Step 3: Self-Collisions** through **Step 12: Configuration Files** in this guide.

**Important:** When saving the package in Step 12, use the path:
- **Configuration Package Save Path**: `/root/ws_moveit/src/`
- **Package Name**: `ranger_piper_moveit`

#### Step 5: Exit Container

```bash
# Inside container
exit
```

The generated `ranger_piper_moveit` package will be in your host workspace at:
```
/home/robofi/codes/ranger-garden-assistant/src/ranger_piper_moveit/
```

#### Step 6: Build the Generated Package

Back on your host machine:

```bash
cd /home/robofi/codes/ranger-garden-assistant
colcon build --symlink-install --packages-select ranger_piper_moveit
source install/setup.bash
```

### Docker Troubleshooting

**Docker Permission Denied:**
```bash
sudo usermod -aG docker $USER
# Log out and back in
```

**X11 Connection Issues:**
```bash
xhost +local:docker
```

**Container Won't Start:**
```bash
# Check Docker status
sudo systemctl status docker

# Pull image manually
docker pull moveit/moveit2:humble-release
```

**URDF Not Found in Container:**
```bash
# Inside container, verify workspace is mounted
ls -la /root/ws_moveit/src/ranger_description/urdf/
```

**Re-entering Container:**
```bash
# Start the same container again
./scripts/moveit_setup_assistant_docker.sh

# Or use gui-docker directly
./scripts/gui-docker -c ranger_moveit_setup
```

### Advanced Docker Usage

**Custom Docker Image:**
```bash
./scripts/gui-docker \
    -c ranger_moveit_setup \
    -v "$PWD:/root/ws_moveit:rw" \
    --rm -it \
    moveit/moveit2:rolling-release \
    bash
```

**Mount Additional Directories:**
```bash
./scripts/gui-docker \
    -c ranger_moveit_setup \
    -v "$PWD:/root/ws_moveit:rw" \
    -v "$HOME/my_configs:/root/configs:rw" \
    --rm -it \
    moveit/moveit2:humble-release \
    bash
```

---

## Overview

We're creating a MoveIt config package (`ranger_piper_moveit`) that uses the complete robot description from `ranger_description` package, including the PiPER arm with `piper_` prefix.

## Prerequisites

- ROS 2 Humble installed
- Workspace built successfully
- MoveIt 2 packages installed:
  ```bash
  sudo apt install ros-humble-moveit ros-humble-moveit-setup-assistant
  ```

## ⚠️ KNOWN ISSUE: MoveIt Setup Assistant Crashes

**The MoveIt Setup Assistant currently crashes due to a RViz/Qt bug when loading URDF files on this platform.**

### Root Cause
The crash occurs in `rviz_common::properties::PropertyTreeModel::propertyHiddenChanged()` - a known RViz Humble bug that affects Jetson/ARM platforms even with recent package versions.

### ✅ RECOMMENDED SOLUTION: Docker with GUI Support

**See the complete Docker workflow above in the "🐳 Quick Start with Docker" section** for detailed step-by-step instructions including:
- Docker installation and prerequisites
- Launching the Setup Assistant in a container
- Loading your URDF and configuring MoveIt
- Saving the generated package back to your host
- Building and testing the configuration

The Docker approach completely avoids the platform-specific RViz/Qt crash by running in an x86_64 container environment.

### Alternative Workaround Options

**Option 1: Manual Configuration**
Use the provided script to create a MoveIt config package template manually:
```bash
./scripts/create_moveit_config_manual.sh
```
Then edit the generated SRDF and config files based on the existing `piper_with_gripper_moveit` package.

**Option 2: Copy & Adapt Existing Config**
Copy the PiPER-only config and extend it for the full Ranger system:
```bash
cp -r src/piper_ros/src/piper_moveit/piper_with_gripper_moveit src/ranger_piper_moveit
# Then edit URDFs and frame references
```

**Option 3: Use Another Machine**
Run the Setup Assistant on a x86_64 Ubuntu 22.04 machine with ROS 2 Humble, then copy the generated package back.

📘 **See [MANUAL_MOVEIT_CONFIG.md](MANUAL_MOVEIT_CONFIG.md) for detailed manual configuration instructions.**

---

## Continue Reading (for GUI method on x86_64)

## Step-by-Step Guide

### Step 1: Launch MoveIt Setup Assistant

Open a terminal and run:

```bash
cd /home/robofi/codes/ranger-garden-assistant
source install/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

The MoveIt Setup Assistant GUI will open.

---

### Step 2: Start Screen - Load Robot URDF

1. Click **"Create New MoveIt Configuration Package"**
2. Click the **"Browse"** button next to "URDF/COLLADA/URDF Package" field
3. Navigate to and select:
   ```
   /home/robofi/codes/ranger-garden-assistant/src/ranger_description/urdf/ranger_complete.urdf.xacro
   ```
4. Click **"Load Files"**
5. Wait for the robot to load (you should see the Ranger base with PiPER arm in the 3D viewer)
6. Click **"Next"** or select the next tab

**Troubleshooting:** If it crashes here, see the troubleshooting doc mentioned above.

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
   - **Parent Frame Name**: `world`
   - **Joint Type**: `fixed`
4. Click **"Save"**

**What this does:** Since the Ranger is a mobile robot, this creates a fixed attachment to the world. For mobile manipulation, you can later change this to a planar or floating joint.

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
   - **Base Link**: Select `piper_world`
     *(This is the arm's mounting point on the base)*
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

3. Click **"Add Joints"** button (in the middle section)

4. In the joint selection dialog:
   - Find and select `piper_joint7` (the gripper joint)
   - Move it to the "Selected Joints" column

5. Click **"Save"** (in the joint selection dialog)
6. Click **"Save"** (in the main group dialog)

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
   - `piper_joint2`: `-1.57` (fold shoulder down)
   - `piper_joint3`: `1.57` (fold elbow up)
   - `piper_joint4`: `0.0`
   - `piper_joint5`: `0.0`
   - `piper_joint6`: `0.0`
4. Click **"Save"**

**Note:** Adjust these values as needed for your specific robot configuration. You can test poses later in RViz.

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

3. Select each joint from the dropdown and click **"Save"**

**What this does:** Tells MoveIt these joints are not part of the manipulator and should be ignored during arm motion planning.

---

### Step 10: ROS 2 Controllers

Configure the ros2_control controllers for your planning groups.

1. Click **"ROS 2 Controllers"** in the left sidebar
2. Click **"Auto Add FollowJointsTrajectory Controllers For Each Planning Group"**

This will automatically create:
- `piper_arm_controller` for the arm
- `piper_gripper_controller` for the gripper

3. Review the auto-generated controllers:
   - **Controller Name**: `piper_arm_controller`
     - **Controller Type**: `FollowJointTrajectory`
     - **Joints**: `piper_joint1` through `piper_joint6`

   - **Controller Name**: `piper_gripper_controller`
     - **Controller Type**: `FollowJointTrajectory`
     - **Joints**: `piper_joint7`

4. If they look correct, no changes needed

**Note:** You'll need to ensure these controller names match your actual ros2_control configuration in your robot driver.

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
   - Navigate to: `/home/robofi/codes/ranger-garden-assistant/src/`
   - Select the `src` folder

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
- [MoveIt2 Docker Documentation](https://moveit.picknik.ai/main/doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu.html)
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
