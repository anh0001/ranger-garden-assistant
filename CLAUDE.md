# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 2 Humble workspace for the **Ranger Garden Assistant** - a mobile manipulation platform combining:
- **AgileX Ranger Mini 3.0** omnidirectional base (CAN bus @ 500 kbps)
- **Livox Mid-360** LiDAR for 3D perception
- **Intel RealSense D435** RGB-D camera
- **AgileX PiPER** 6-DOF robotic arm (CAN bus @ 1000 kbps, optional)
- **FAST_LIO** for LiDAR-Inertial Odometry and real-time mapping
- **Navigation2** for autonomous navigation
- **MoveIt 2** for motion planning

## Build and Setup Commands

### Initial Workspace Setup
```bash
# Clone with submodules
git submodule update --init --recursive

# Build entire workspace
./scripts/build_workspace.sh

# Or manually:
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Rebuild After Changes
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Build Specific Package
```bash
colcon build --packages-select <package_name> --symlink-install
```

### Clean Build (if needed)
```bash
rm -rf build install log
colcon build --symlink-install
```

### Special: Livox Driver Build Issues
If `livox_ros_driver2` fails to build:
```bash
cd src/livox_ros_driver2
./build.sh humble
cd ../..
colcon build
```

## Hardware Setup

### CAN Bus Configuration
**MUST be run after each reboot** before launching robot:
```bash
sudo ./scripts/setup_can.sh
```

Or manually:
```bash
# can0 for Ranger base (500 kbps)
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# can1 for PiPER arm (1000 kbps)
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

### Verify Hardware
```bash
# Check CAN interfaces
ip link show can0
candump can0  # Should see messages if base is powered on

# Check RealSense
rs-enumerate-devices

# Check LiDAR network (Mid-360 typically at 192.168.1.1XX)
ping 192.168.1.1XX
```

## Launch Commands

### Complete System
```bash
ros2 launch robofi_bringup ranger_complete_bringup.launch.py
```
Starts: base controller, LiDAR, RealSense, robot state publisher, TF tree

### Individual Components
```bash
# Base controller only
ros2 launch robofi_bringup ranger_base.launch.py

# LiDAR only
ros2 launch robofi_bringup livox_lidar.launch.py

# Visualize robot model
ros2 launch ranger_description display.launch.py

# View in RViz with custom config
rviz2 -d src/ranger_description/rviz/view_robot.rviz
```

### SLAM Mapping

#### Option 1: FAST_LIO (LiDAR-Inertial Odometry)
High-accuracy real-time localization using LiDAR + IMU fusion:
```bash
# Terminal 1: Launch robot (base + sensors)
ros2 launch robofi_bringup ranger_complete_bringup.launch.py

# Terminal 2: Start FAST_LIO mapping
ros2 launch fast_lio mapping.launch.py

# Terminal 3: Teleoperate to build map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Map is automatically saved as PCD file in current directory
# Convert PCD to 2D occupancy grid if needed for Nav2
```

#### Option 2: SLAM Toolbox (2D Grid-based)
Traditional 2D SLAM for navigation:
```bash
# Terminal 1: Launch robot
ros2 launch robofi_bringup ranger_complete_bringup.launch.py

# Terminal 2: Start SLAM
ros2 launch robofi_bringup slam.launch.py

# Terminal 3: Teleoperate to build map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save map when done
ros2 run nav2_map_server map_saver_cli -f my_map
```

**Choosing between FAST_LIO and SLAM Toolbox:**
- **FAST_LIO**: Better accuracy, 3D mapping, works in feature-rich environments, requires IMU
- **SLAM Toolbox**: 2D maps compatible with Nav2, works without IMU, lighter computational load

### Autonomous Navigation
```bash
# Terminal 1: Launch robot
ros2 launch robofi_bringup ranger_complete_bringup.launch.py

# Terminal 2: Launch navigation stack
ros2 launch robofi_bringup navigation.launch.py map:=/path/to/map.yaml

# Terminal 3: Open Nav2 RViz for goal setting
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### Arm Control (PiPER)
```bash
# Launch arm separately
ros2 launch piper_ros start_single_piper.launch.py can_port:=can1

# Launch with MoveIt for motion planning
ros2 launch piper_moveit piper_moveit.launch.py
```

## Testing and Debugging

### Check Active Topics
```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu  # For FAST_LIO

# When running FAST_LIO
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
```

### Verify TF Tree
```bash
ros2 run tf2_tools view_frames
# Generates frames.pdf with complete TF tree
```

### Check Node Status
```bash
ros2 node list
ros2 node info /ranger_base_node
```

### View Images
```bash
ros2 run rqt_image_view rqt_image_view
# Select topic from dropdown: /camera/color/image_raw
```

### Monitor System Resources
```bash
ros2 run rqt_top rqt_top
```

## Architecture Highlights

### Package Structure
- `robofi_bringup/` - Main integration package with launch files and configs
- `ranger_description/` - URDF/xacro robot descriptions and meshes
- `ranger_ros2/` - Base controller driver (submodule)
- `livox_ros_driver2/` - LiDAR driver (submodule)
- `FAST_LIO/` - LiDAR-Inertial Odometry for localization and mapping (submodule)
- `ugv_sdk/` - AgileX UGV SDK (submodule)
- `piper_ros/` - Arm driver and MoveIt config (submodule)

### Key Launch Files
Located in [src/robofi_bringup/launch/](src/robofi_bringup/launch/):
- `ranger_complete_bringup.launch.py` - Complete system launch
- `ranger_base.launch.py` - Base controller only
- `livox_lidar.launch.py` - LiDAR driver
- `navigation.launch.py` - Nav2 navigation stack
- `slam.launch.py` - SLAM mapping with slam_toolbox

Located in [src/FAST_LIO/launch/](src/FAST_LIO/launch/):
- `mapping.launch.py` - FAST_LIO LiDAR-inertial odometry and mapping

### Critical Configuration Files
- [src/robofi_bringup/config/nav2_params.yaml](src/robofi_bringup/config/nav2_params.yaml) - Navigation parameters (costmaps, planners, controllers)
- [src/ranger_description/urdf/ranger_complete.urdf.xacro](src/ranger_description/urdf/ranger_complete.urdf.xacro) - Robot URDF with sensor positions
- [src/livox_ros_driver2/config/MID360_config.json](src/livox_ros_driver2/config/MID360_config.json) - LiDAR configuration
- [src/FAST_LIO/config/mid360.yaml](src/FAST_LIO/config/mid360.yaml) - FAST_LIO parameters for Mid-360 LiDAR

### TF Tree Structure
```
map (from SLAM/AMCL)
└─ odom (from wheel odometry or FAST_LIO)
   └─ base_footprint
      └─ base_link
         ├─ lidar_link (Livox Mid-360)
         │  └─ livox_frame (IMU frame for FAST_LIO)
         ├─ camera_link (RealSense D435)
         │  ├─ camera_depth_optical_frame
         │  └─ camera_color_optical_frame
         └─ piper_base_link (arm mount)
            └─ piper_link_1 → ... → piper_tool0
```

**Note:** When using FAST_LIO, it publishes the `camera_init` → `body` transform, which should be remapped or integrated into your TF tree as needed.

### Important ROS Topics
**Base Control:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands to base
- `/odom` (nav_msgs/Odometry) - Wheel odometry

**Sensors:**
- `/livox/lidar` (sensor_msgs/PointCloud2) - 3D point cloud
- `/livox/imu` (sensor_msgs/Imu) - IMU data from Livox Mid-360
- `/camera/color/image_raw` - RGB camera
- `/camera/depth/image_rect_raw` - Depth image
- `/camera/depth/color/points` - Colored point cloud

**FAST_LIO (when enabled):**
- `/Odometry` (nav_msgs/Odometry) - High-accuracy LiDAR-inertial odometry
- `/cloud_registered` (sensor_msgs/PointCloud2) - Registered point cloud in world frame
- `/cloud_registered_body` (sensor_msgs/PointCloud2) - Point cloud in body frame
- `/path` (nav_msgs/Path) - Robot trajectory path
- `/tf` - Publishes `camera_init` (world) → `body` (robot) transform

**Navigation:**
- `/map` - Occupancy grid map
- `/local_costmap/costmap` - Local costmap for navigation
- `/global_costmap/costmap` - Global costmap

**Arm (if enabled):**
- `/joint_states` - Current joint positions
- `/piper_arm_controller/follow_joint_trajectory` - Trajectory commands

## Common Workflows

### Adding New Sensors
1. Add sensor URDF description to [ranger_description/urdf/](src/ranger_description/urdf/)
2. Create launch file in [robofi_bringup/launch/](src/robofi_bringup/launch/)
3. Add static transforms or update URDF with proper mounting position
4. Include in `ranger_complete_bringup.launch.py` if needed
5. Update costmap configs if sensor provides obstacle data

### Modifying Sensor Positions
Edit URDF properties in `ranger_description/urdf/ranger_complete.urdf.xacro`:
```xml
<!-- Example: LiDAR position -->
<xacro:property name="lidar_x" value="0.0"/>
<xacro:property name="lidar_y" value="0.0"/>
<xacro:property name="lidar_z" value="0.70"/>
```

### Tuning Navigation
Edit parameters in [src/robofi_bringup/config/nav2_params.yaml](src/robofi_bringup/config/nav2_params.yaml):
- **Robot footprint**: Match physical robot dimensions
- **Costmap resolution**: Balance accuracy vs performance (default 0.05m)
- **Max velocities**: Adjust `max_vel_x`, `max_vel_theta` for base capabilities
- **DWB controller weights**: Tune path following behavior
- **Inflation radius**: Set safety buffer around obstacles

### Creating Custom Behaviors
Add behavior tree XML files to `src/robofi_bringup/config/behavior_trees/` and reference in nav2_params.yaml

### Tuning FAST_LIO Parameters
Edit parameters in [src/FAST_LIO/config/mid360.yaml](src/FAST_LIO/config/mid360.yaml):
- **Extrinsic calibration**: `extrinsic_T` and `extrinsic_R` define LiDAR-to-IMU transformation
  - Enable `extrinsic_est_en: true` for online estimation
  - Or manually calibrate and set `extrinsic_est_en: false`
- **Noise parameters**: Adjust `acc_cov`, `gyr_cov`, `b_acc_cov`, `b_gyr_cov` based on IMU specs
- **Filter sizes**: `filter_size_surf` and `filter_size_map` control voxel grid downsampling
  - Smaller values = more detail but slower processing
  - Larger values = faster but less accurate
- **Detection range**: `det_range` sets maximum LiDAR detection distance (default 100m)
- **Map saving**: Set `pcd_save_en: true` and `interval: -1` to save complete map

### Integrating FAST_LIO with Navigation
To use FAST_LIO odometry for Nav2 navigation:
1. Launch FAST_LIO separately: `ros2 launch fast_lio mapping.launch.py`
2. Remap FAST_LIO's `/Odometry` topic to `/odom` for Nav2:
   ```bash
   ros2 run topic_tools relay /Odometry /odom
   ```
3. Or modify nav2_params.yaml to subscribe to `/Odometry` instead of `/odom`
4. Ensure TF frames are properly connected (`camera_init` → `odom`, `body` → `base_link`)

## Important Notes

### CAN Bus Dependency
The CAN interfaces **must be configured after every system reboot**. Without running `sudo ./scripts/setup_can.sh`, the base and arm controllers will fail to start.

### Submodule Updates
When pulling changes that affect submodules:
```bash
git submodule update --recursive
```

### Omnidirectional Kinematics
The Ranger Mini 3.0 is a **holonomic** (omnidirectional) robot, meaning it can move in any direction without rotating. This affects:
- Controller configurations (supports lateral motion)
- Path planning (can use shorter, more direct paths)
- Velocity commands (x, y, and theta are independent)

### Multi-Robot Namespacing
To run multiple robots, use namespace parameter:
```bash
ros2 launch robofi_bringup ranger_complete_bringup.launch.py namespace:=robot1
```

### Performance on Embedded Platforms
For Jetson or lower-power computers:
- Reduce sensor rates (LiDAR to 5Hz, camera to 15 FPS)
- Increase costmap resolution to 0.1m
- Lower costmap update frequencies in nav2_params.yaml
- Disable RViz visualization on robot itself

## Troubleshooting Reference

### Build Issues
- **Livox driver fails**: Run `cd src/livox_ros_driver2 && ./build.sh humble`
- **Missing dependencies**: Run `rosdep install --from-paths src --ignore-src -r -y`
- **Stale build**: Remove build/install/log directories and rebuild

### Runtime Issues
- **CAN not found**: Run `sudo ./scripts/setup_can.sh` and verify with `ip link show`
- **LiDAR no data**: Check network config (PC must be on 192.168.1.x subnet)
- **Camera not detected**: Verify USB 3.0 connection, run `rs-enumerate-devices`
- **TF errors**: Check all required nodes are running with `ros2 node list`
- **Navigation failures**: Verify sensor data visible in RViz, check costmap topics
- **FAST_LIO not receiving IMU data**:
  - Verify `/livox/imu` topic is publishing: `ros2 topic hz /livox/imu`
  - Check Livox Mid-360 is configured to publish IMU data in MID360_config.json
- **FAST_LIO drift or poor accuracy**:
  - Check IMU noise parameters match your sensor specs
  - Enable online extrinsic calibration: `extrinsic_est_en: true`
  - Ensure sufficient feature-rich environment (not recommended in featureless areas)
- **FAST_LIO high CPU usage**: Increase filter sizes in config, reduce point cloud density

### Diagnostic Commands
```bash
# View TF problems
ros2 run tf2_ros tf2_echo base_link camera_link

# Check for missing transforms
ros2 run tf2_tools view_frames

# Monitor topic rates
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry  # FAST_LIO odometry

# View FAST_LIO point cloud output
ros2 topic echo /cloud_registered --no-arr

# Check FAST_LIO odometry quality
ros2 topic echo /Odometry

# View node logs
ros2 run rqt_console rqt_console

# Visualize FAST_LIO in RViz
rviz2 -d $(ros2 pkg prefix fast_lio)/share/fast_lio/rviz/fastlio.rviz
```

## Additional Resources

- Full documentation: [README.md](README.md)
- Architecture details: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- Quick start guide: [docs/QUICK_START.md](docs/QUICK_START.md)
- FAST_LIO GitHub: https://github.com/hku-mars/FAST_LIO
- FAST_LIO paper: [src/FAST_LIO/doc/Fast_LIO_2.pdf](src/FAST_LIO/doc/Fast_LIO_2.pdf)
- Navigation2 docs: https://navigation.ros.org/
- MoveIt 2 docs: https://moveit.ros.org/
