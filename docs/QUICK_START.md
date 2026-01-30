# Quick Start Guide

This guide will help you get the Ranger Garden Assistant up and running quickly.

## Prerequisites Checklist

- [ ] Ubuntu 22.04 LTS installed
- [ ] ROS 2 Humble installed
- [ ] Hardware connected (base, LiDAR, camera)
- [ ] CAN adapters plugged in
- [ ] Internet connection for package installation

## Step-by-Step Setup

### 1. Install System Dependencies (5 minutes)

```bash
# Install ROS 2 packages
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-v4l2-camera \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    can-utils \
    v4l-utils \
    python3-pip

# Install Python dependencies
pip3 install python-can piper_sdk scipy
```

### 2. Clone and Build Workspace (10 minutes)

```bash
# Clone repository
cd ~/
git clone https://github.com/yourusername/ranger-garden-assistant.git
cd ranger-garden-assistant

# Initialize submodules
git submodule update --init --recursive

# Build workspace
./scripts/build_workspace.sh

# Source workspace
source install/setup.bash
echo "source ~/ranger-garden-assistant/install/setup.bash" >> ~/.bashrc
```

### 3. Setup Hardware Connections

**CAN Interfaces:**
```bash
# One-time setup (automatic configuration on boot)
sudo ./scripts/install_can_udev.sh

# After installation, unplug and replug CAN adapters
# Interfaces will be automatically configured

# Verify CAN interfaces are up
ip link show can_base
ip link show can_piper
```

**Tier IV Camera udev rules:**
```bash
sudo ./scripts/install_camera_udev.sh
```

**Verify Sensors:**
```bash
# Check Tier IV camera device
ls -l /dev/tieriv_c2_video0
v4l2-ctl --device=/dev/tieriv_c2_video0 --all

# Check LiDAR (should see device on network)
ping 192.168.1.1XX  # Replace XX with your LiDAR's IP

# Check CAN communication
candump can_base  # Should see messages when base is powered on
```

### 4. First Launch - Test Individual Components

**Test 1: Robot Description**
```bash
source install/setup.bash
ros2 launch ranger_description display.launch.py
```
- You should see the robot model in RViz
- All links should be connected in the TF tree

**Test 2: Base Controller**
```bash
# Terminal 1: Launch base
ros2 launch robofi_bringup ranger_base.launch.py

# Terminal 2: Check topics
ros2 topic list | grep -E "(odom|cmd_vel)"
ros2 topic echo /odom

# Terminal 3: Test control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- Robot should move when you press keys
- Odometry should update in Terminal 2

**Test 3: LiDAR**
```bash
# Terminal 1: Launch LiDAR
ros2 launch robofi_bringup livox_lidar.launch.py xfer_format:=4

# Terminal 2: Check point cloud
ros2 topic hz /livox/lidar
rviz2
```
- In RViz, add PointCloud2 display
- Subscribe to `/livox/lidar`
- You should see point cloud data

**Test 4: Camera**
```bash
# Terminal 1: Launch camera
ros2 run v4l2_camera v4l2_camera_node \
  --ros-args -r __ns:=/camera \
  -p video_device:=/dev/tieriv_c2_video0 \
  -p image_width:=2880 \
  -p image_height:=1860 \
  -p pixel_format:=UYVY \
  -p output_encoding:=bgr8 \
  -p camera_frame_id:=camera_optical_frame \
  -p camera_name:=tier4_c2_176 \
  -p camera_info_url:=file://$(ros2 pkg prefix robofi_bringup)/share/robofi_bringup/config/tier4_c2_176_2880x1860_intrinsic.yaml

# Terminal 2: View image
ros2 run rqt_image_view rqt_image_view
```
- Select `/camera/image_raw` from dropdown
- You should see camera feed

### 5. Full System Launch

Once individual components work:

```bash
# Launch everything
ros2 launch robofi_bringup ranger_complete_bringup.launch.py

# In another terminal, open RViz
rviz2 -d src/ranger_description/rviz/view_robot.rviz
```

Expected result:
- Robot model visible in RViz
- Point cloud data visible
- Camera feed available
- Odometry updating
- All TF frames connected

### 6. Create Your First Map

```bash
# Terminal 1: Launch robot
ros2 launch robofi_bringup ranger_complete_bringup.launch.py

# Terminal 2: Start SLAM
ros2 launch robofi_bringup slam.launch.py

# Terminal 3: Teleoperate
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: Visualize
rviz2
# In RViz: Add -> By topic -> /map
```

Drive around slowly to build a map. When done:

```bash
# Save the map
cd ~
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/my_first_map
```

### 7. First Autonomous Navigation

```bash
# Terminal 1: Launch robot
ros2 launch robofi_bringup ranger_complete_bringup.launch.py

# Terminal 2: Launch navigation
ros2 launch robofi_bringup navigation.launch.py map:=~/maps/my_first_map.yaml

# Terminal 3: Open Nav2 RViz
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

In RViz:
1. Click "2D Pose Estimate" button
2. Click on map where robot is, drag to set orientation
3. Click "Nav2 Goal" button
4. Click where you want robot to go
5. Robot should plan path and navigate!

## Common First-Time Issues

### Issue: CAN interface not found

```bash
# Check USB devices
lsusb | grep -i can

# Check dmesg for CAN adapter
dmesg | tail -20 | grep -i can

# Verify udev rules are installed
ls -l /etc/udev/rules.d/99-ranger-can.rules

# If not installed, run:
sudo ./scripts/install_can_udev.sh

# Check interface status
ip link show can_base
```

### Issue: LiDAR not publishing

```bash
# Check if driver node is running
ros2 node list | grep livox

# Check network settings
ifconfig
# Ensure your PC's IP is on same subnet as LiDAR (192.168.1.x)

# Try pinging LiDAR
ping 192.168.1.1XX  # Use your LiDAR's IP
```

### Issue: Camera not detected

```bash
# Check if device nodes exist
ls -l /dev/tieriv_c2_video*

# Verify udev rules were installed
ls -l /etc/udev/rules.d/99-tieriv-c2-camera.rules

# Check camera capabilities
v4l2-ctl --list-devices

# Check USB connection (must be USB 3.0)
lsusb -t
```

### Issue: Build fails

```bash
# Clean and rebuild
cd ~/ranger-garden-assistant
rm -rf build install log
colcon build --symlink-install

# If Livox driver fails
cd src/livox_ros_driver2
./build.sh humble
cd ../..
colcon build
```

## Next Steps

After completing this quick start:

1. **Tune Navigation Parameters**: Edit `src/robofi_bringup/config/nav2_params.yaml`
2. **Calibrate Sensors**: Measure and update sensor positions in URDF
3. **Test Arm Control**: Uncomment PiPER launch in bringup file
4. **Create Waypoint Missions**: Use Nav2 waypoint follower
5. **Add Object Detection**: Integrate perception pipeline with camera

## Getting Help

- Check the main [README.md](../README.md) for detailed documentation
- Review [ARCHITECTURE.md](ARCHITECTURE.md) for system overview
- Open an issue on GitHub for bugs or questions
- Join ROS Discourse for community support

## Success Checklist

- [ ] All components launch without errors
- [ ] Robot responds to teleoperation commands
- [ ] LiDAR point cloud visible in RViz
- [ ] Camera feed displays correctly
- [ ] Successfully created a map
- [ ] Robot navigates autonomously to goal
- [ ] All transforms connected in TF tree

If you've checked all boxes, congratulations! Your Ranger Garden Assistant is ready for advanced tasks.
