# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 2 Humble workspace for the **Ranger Garden Assistant** - a mobile manipulation platform combining:
- **AgileX Ranger Mini 3.0** omnidirectional base (CAN bus @ 500 kbps)
- **Livox Mid-360** LiDAR for 3D perception
- **Intel RealSense D435** RGB-D camera
- **AgileX PiPER** 6-DOF robotic arm (CAN bus @ 1000 kbps, optional)
- **FASTLIO2_ROS2** for LiDAR-Inertial Odometry, loop closure, and localization
- **Octomap Server 2** for 3D volumetric mapping and 2D occupancy grid generation
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

### SLAM and Mapping

#### FASTLIO2_ROS2 SLAM Backend (Recommended)
FASTLIO2_ROS2 provides a complete SLAM pipeline with three specialized nodes:

**Architecture Overview:**
- **`fastlio2::lio_node`**: LiDAR-Inertial Odometry (publishes `/Odometry`)
- **`pgo::pgo_node`**: Pose Graph Optimization with loop closure (publishes `map → odom` transform)
- **`localizer::localizer_node`**: Localization against saved maps for re-use

**Option 1: LIO Mode (Odometry Only)**
Real-time high-accuracy odometry without loop closure:
```bash
# Terminal 1: Launch robot (base + sensors)
ros2 launch robofi_bringup ranger_complete_bringup.launch.py

# Terminal 2: Start FASTLIO2 LIO node
ros2 launch robofi_bringup fastlio2_navigation.launch.py mode:=lio

# Terminal 3: Teleoperate robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# LIO provides odometry on /Odometry topic
# Best for short-term navigation or when combined with external localization
```

**Option 2: PGO Mode (Mapping with Loop Closure)**
Full SLAM with loop closure detection and map optimization:
```bash
# Terminal 1: Launch robot (base + sensors)
ros2 launch robofi_bringup ranger_complete_bringup.launch.py

# Terminal 2: Start FASTLIO2 with PGO (mapping mode)
ros2 launch robofi_bringup fastlio2_navigation.launch.py mode:=pgo

# Terminal 3: Teleoperate to build map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# PGO node detects loop closures and optimizes the map
# Publishes map → odom transform for global consistency
# Save map when complete (see Map Saving section)
```

**Option 3: Localizer Mode (Localization on Saved Map)**
Localize robot against a previously saved map:
```bash
# Terminal 1: Launch robot (base + sensors)
ros2 launch robofi_bringup ranger_complete_bringup.launch.py

# Terminal 2: Start FASTLIO2 localizer with saved map
ros2 launch robofi_bringup fastlio2_navigation.launch.py mode:=localizer map_path:=/path/to/saved_map.pcd

# Localizer node loads the map and provides localization
# Publishes map → odom transform based on map matching
```

#### Octomap Server 2 (3D Volumetric Mapping)
Maintains a 3D occupancy map (OctoMap) and generates 2D projections for Nav2:
```bash
# Launch alongside FASTLIO2 (typically included in fastlio2_navigation.launch.py)
ros2 run octomap_server2 octomap_server_node \
  --ros-args \
  --params-file src/robofi_bringup/config/octomap_server.yaml

# Octomap subscribes to /cloud_registered from FASTLIO2
# Publishes:
#   - /octomap_binary or /octomap_full (3D map)
#   - /projected_map (2D occupancy grid for Nav2)
```

**Benefits of Octomap + FASTLIO2:**
- **3D awareness**: Handles multi-level environments, overhangs, and dynamic obstacles
- **Memory efficient**: Octree compression reduces memory footprint
- **Nav2 integration**: Projected 2D map works directly with Nav2 costmaps
- **Real-time updates**: Incrementally updates as robot explores

#### SLAM Toolbox (Alternative 2D SLAM)
Traditional 2D SLAM for simpler environments:
```bash
# Terminal 1: Launch robot
ros2 launch robofi_bringup ranger_complete_bringup.launch.py

# Terminal 2: Start SLAM Toolbox
ros2 launch robofi_bringup slam.launch.py

# Terminal 3: Teleoperate to build map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save map when done
ros2 run nav2_map_server map_saver_cli -f my_map
```

**Choosing Your SLAM Approach:**
- **FASTLIO2 + Octomap** (Recommended): Best accuracy, 3D mapping, loop closure, works in complex environments
- **SLAM Toolbox**: Simpler 2D approach, lighter computational load, no IMU required

### Autonomous Navigation

#### Navigation with FASTLIO2 + Octomap (Recommended)
Complete navigation stack using FASTLIO2 odometry and Octomap costmaps:
```bash
# Terminal 1: Launch robot (base + sensors)
ros2 launch robofi_bringup ranger_complete_bringup.launch.py

# Terminal 2: Launch FASTLIO2 in localizer mode with saved map
ros2 launch robofi_bringup fastlio2_navigation.launch.py mode:=localizer map_path:=/path/to/map.pcd

# Terminal 3: Launch Nav2 stack
ros2 launch robofi_bringup navigation.launch.py

# Terminal 4: Open Nav2 RViz for goal setting
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**Navigation Architecture:**
- **Odometry**: FASTLIO2 LIO node provides high-accuracy `/Odometry` topic
- **Localization**: PGO or Localizer node provides `map → odom` transform
- **Global costmap**: Uses static map (from Octomap projection) + obstacle layer
- **Local costmap**: Uses voxel layer from Mid-360 point cloud + optional D435 depth
- **Controllers**: Use FASTLIO2 odometry directly for smooth, accurate control

#### Navigation with Static Map (Alternative)
Traditional navigation with pre-built map:
```bash
# Terminal 1: Launch robot
ros2 launch robofi_bringup ranger_complete_bringup.launch.py

# Terminal 2: Launch navigation stack with static map
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
ros2 topic hz /livox/imu  # For FASTLIO2

# When running FASTLIO2 LIO node
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic hz /path  # Robot trajectory

# When running FASTLIO2 PGO node
ros2 topic hz /optimized_path  # Optimized trajectory after loop closure
ros2 topic echo /loop_closure_info  # Loop closure detections

# When running Octomap Server
ros2 topic hz /octomap_binary  # 3D volumetric map
ros2 topic hz /projected_map  # 2D occupancy grid for Nav2
ros2 topic hz /octomap_point_cloud_centers  # Point cloud visualization of occupied voxels
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
- `FASTLIO2_ROS2/` - Complete SLAM backend with LIO, PGO, and localization (submodule)
- `octomap_server2/` - 3D volumetric mapping and 2D projection (submodule)
- `ugv_sdk/` - AgileX UGV SDK (submodule)
- `piper_ros/` - Arm driver and MoveIt config (submodule)

### Key Launch Files
Located in [src/robofi_bringup/launch/](src/robofi_bringup/launch/):
- `ranger_complete_bringup.launch.py` - Complete system launch (base + sensors)
- `ranger_base.launch.py` - Base controller only
- `livox_lidar.launch.py` - LiDAR driver
- `fastlio2_navigation.launch.py` - FASTLIO2 SLAM with Octomap (supports mode:=lio/pgo/localizer)
- `navigation.launch.py` - Nav2 navigation stack
- `slam.launch.py` - SLAM Toolbox mapping (alternative 2D SLAM)

Located in [src/FASTLIO2_ROS2/launch/](src/FASTLIO2_ROS2/launch/):
- `fastlio2_lio.launch.py` - LIO node only (odometry)
- `fastlio2_pgo.launch.py` - LIO + PGO nodes (mapping with loop closure)
- `fastlio2_localizer.launch.py` - Localizer node (localization on saved maps)

### Critical Configuration Files

**Robot and Sensors:**
- [src/ranger_description/urdf/ranger_complete.urdf.xacro](src/ranger_description/urdf/ranger_complete.urdf.xacro) - Robot URDF with sensor positions
- [src/livox_ros_driver2/config/MID360_config.json](src/livox_ros_driver2/config/MID360_config.json) - LiDAR configuration

**FASTLIO2 SLAM:**
- [src/robofi_bringup/config/fastlio2_lio.yaml](src/robofi_bringup/config/fastlio2_lio.yaml) - LIO node parameters (odometry)
- [src/robofi_bringup/config/fastlio2_pgo.yaml](src/robofi_bringup/config/fastlio2_pgo.yaml) - PGO node parameters (loop closure)
- [src/robofi_bringup/config/fastlio2_localizer.yaml](src/robofi_bringup/config/fastlio2_localizer.yaml) - Localizer node parameters

**3D Mapping:**
- [src/robofi_bringup/config/octomap_server.yaml](src/robofi_bringup/config/octomap_server.yaml) - Octomap Server 2 parameters

**Navigation:**
- [src/robofi_bringup/config/nav2_params.yaml](src/robofi_bringup/config/nav2_params.yaml) - Nav2 parameters (costmaps, planners, controllers)

### TF Tree Structure

**With FASTLIO2 + PGO/Localizer:**
```
map (from PGO or Localizer node)
└─ odom (from PGO or Localizer node)
   └─ base_footprint
      └─ base_link
         ├─ lidar_link (Livox Mid-360)
         │  └─ livox_frame (IMU frame for FASTLIO2)
         ├─ camera_link (RealSense D435)
         │  ├─ camera_depth_optical_frame
         │  └─ camera_color_optical_frame
         └─ piper_base_link (arm mount)
            └─ piper_link_1 → ... → piper_tool0
```

**With FASTLIO2 LIO only (no global localization):**
```
odom (provided by LIO node)
└─ base_footprint
   └─ base_link
      ├─ lidar_link (Livox Mid-360)
      │  └─ livox_frame (IMU frame)
      ├─ camera_link (RealSense D435)
      │  ├─ camera_depth_optical_frame
      │  └─ camera_color_optical_frame
      └─ piper_base_link (arm mount)
         └─ piper_link_1 → ... → piper_tool0
```

**TF Publishing Responsibilities:**
- **FASTLIO2 LIO node**: Publishes `odom → base_link` based on LiDAR-inertial fusion
- **FASTLIO2 PGO node**: Publishes `map → odom` after loop closure and optimization
- **FASTLIO2 Localizer node**: Publishes `map → odom` based on localization against saved map
- **robot_state_publisher**: Publishes all sensor frames relative to `base_link` from URDF

### Important ROS Topics

**Base Control:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands to base
- `/odom` (nav_msgs/Odometry) - Wheel odometry (if using wheel encoders)

**Sensors:**
- `/livox/lidar` (sensor_msgs/PointCloud2) - Raw 3D point cloud from LiDAR
- `/livox/imu` (sensor_msgs/Imu) - IMU data from Livox Mid-360
- `/camera/color/image_raw` (sensor_msgs/Image) - RGB camera stream
- `/camera/depth/image_rect_raw` (sensor_msgs/Image) - Depth image
- `/camera/depth/color/points` (sensor_msgs/PointCloud2) - RGB-D point cloud

**FASTLIO2 LIO Node:**
- `/Odometry` (nav_msgs/Odometry) - High-accuracy LiDAR-inertial odometry
- `/cloud_registered` (sensor_msgs/PointCloud2) - Registered point cloud in world frame
- `/cloud_registered_body` (sensor_msgs/PointCloud2) - Point cloud in body frame
- `/path` (nav_msgs/Path) - Robot trajectory path
- `/tf` - Publishes `odom → base_link` transform

**FASTLIO2 PGO Node (when enabled):**
- `/optimized_path` (nav_msgs/Path) - Optimized trajectory after loop closure
- `/loop_closure_info` (custom msg) - Loop closure detection information
- `/pgo_map_cloud` (sensor_msgs/PointCloud2) - Globally optimized map
- `/tf` - Publishes `map → odom` transform

**FASTLIO2 Localizer Node (when enabled):**
- `/localization_pose` (geometry_msgs/PoseStamped) - Current localized pose
- `/tf` - Publishes `map → odom` transform based on map matching

**Octomap Server 2:**
- `/octomap_binary` (octomap_msgs/Octomap) - Compressed 3D volumetric map
- `/octomap_full` (octomap_msgs/Octomap) - Full 3D volumetric map
- `/projected_map` (nav_msgs/OccupancyGrid) - 2D occupancy grid for Nav2
- `/octomap_point_cloud_centers` (sensor_msgs/PointCloud2) - Visualization of occupied voxels
- `/occupied_cells_vis_array` (visualization_msgs/MarkerArray) - Visualization markers

**Navigation:**
- `/map` (nav_msgs/OccupancyGrid) - Static occupancy grid map
- `/local_costmap/costmap` (nav_msgs/OccupancyGrid) - Local costmap (voxel layer from LiDAR)
- `/global_costmap/costmap` (nav_msgs/OccupancyGrid) - Global costmap (static + obstacle layer)
- `/plan` (nav_msgs/Path) - Global path plan
- `/local_plan` (nav_msgs/Path) - Local trajectory

**Arm (if enabled):**
- `/joint_states` (sensor_msgs/JointState) - Current joint positions
- `/piper_arm_controller/follow_joint_trajectory` (control_msgs/FollowJointTrajectory) - Trajectory commands

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

**Robot Configuration:**
- **Robot footprint**: Match physical robot dimensions (important for omnidirectional base)
- **Max velocities**: Adjust `max_vel_x`, `max_vel_y`, `max_vel_theta` for holonomic motion

**Global Costmap (for path planning):**
- **Layers**: `static_layer` (from Octomap projection) + `obstacle_layer` (recent obstacles)
- **Static map**: Subscribe to `/projected_map` from Octomap Server
- **Resolution**: Balance accuracy vs performance (default 0.05m)

**Local Costmap (for obstacle avoidance):**
- **Layers**: `voxel_layer` (primary, from Mid-360) + `rgbd_obstacle_layer` (optional, from D435)
- **Voxel layer**: Subscribe to `/cloud_registered` from FASTLIO2 for 3D obstacle detection
- **Update frequency**: Higher frequency for dynamic environments (10-20 Hz)
- **Inflation radius**: Set safety buffer around obstacles

**Controllers:**
- **Odometry source**: Use `/Odometry` from FASTLIO2 for smooth, accurate control
- **DWB controller weights**: Tune path following behavior for omnidirectional motion
- **Goal tolerance**: Adjust based on application requirements

### Creating Custom Behaviors
Add behavior tree XML files to `src/robofi_bringup/config/behavior_trees/` and reference in nav2_params.yaml

### Map Saving and Management

**Saving FASTLIO2 PCD Maps:**
```bash
# Maps are automatically saved by PGO node if configured
# Check config: save_map_enable: true in fastlio2_pgo.yaml

# Manual save via service (if supported)
ros2 service call /fastlio2_pgo_node/save_map std_srvs/srv/Trigger

# Maps saved to: map_save_path directory (default: ~/FASTLIO2_maps/)
# Format: PCD files with timestamp
```

**Saving Octomap:**
```bash
# Save full 3D Octomap
ros2 service call /octomap_server/save_map octomap_msgs/srv/SaveOctomap \
  "{filename: '~/maps/my_environment.ot'}"

# Save 2D projected map for Nav2
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_2d_map \
  --ros-args -p map_subscribe_transient_local:=true
```

**Converting Between Map Formats:**
```bash
# Convert PCD to other formats
pcl_viewer my_map.pcd  # Visualize
pcl_convert my_map.pcd my_map.ply  # Convert to PLY

# Load Octomap in Python for processing
# Use octomap_msgs or octomap Python bindings

# Generate 2D occupancy grid from PCD (custom script needed)
# Or use Octomap Server's built-in projection
```

**Map Reuse Workflow:**
1. **Create map with PGO**: Use PGO mode to build and save optimized PCD map
2. **Use with Localizer**: Launch localizer mode with saved PCD map for localization
3. **Load in Octomap**: Optionally load PCD map into Octomap for real-time 2D projection
4. **Navigate with Nav2**: Use projected 2D map for global costmap in Nav2

**Example Complete Workflow:**
```bash
# Step 1: Mapping phase (do this once)
ros2 launch robofi_bringup fastlio2_navigation.launch.py mode:=pgo
# Drive around, let PGO detect loops and optimize
# Map saved automatically to ~/FASTLIO2_maps/map_YYYYMMDD_HHMMSS.pcd

# Step 2: Save Octomap projection (optional, for static map)
ros2 service call /octomap_server/save_map octomap_msgs/srv/SaveOctomap \
  "{filename: '~/maps/garden_3d.ot'}"
ros2 run nav2_map_server map_saver_cli -f ~/maps/garden_2d

# Step 3: Navigation phase (use this for daily operations)
ros2 launch robofi_bringup fastlio2_navigation.launch.py \
  mode:=localizer \
  map_path:=~/FASTLIO2_maps/map_20250114_143022.pcd
ros2 launch robofi_bringup navigation.launch.py map:=~/maps/garden_2d.yaml
```

### Tuning FASTLIO2 Parameters

**LIO Node Parameters** ([src/robofi_bringup/config/fastlio2_lio.yaml](src/robofi_bringup/config/fastlio2_lio.yaml)):
- **Extrinsic calibration**: `extrinsic_T` and `extrinsic_R` define LiDAR-to-IMU transformation
  - Enable `extrinsic_est_en: true` for automatic online calibration
  - Or manually calibrate and set `extrinsic_est_en: false` for better performance
- **IMU noise parameters**: Adjust based on your IMU specifications
  - `acc_cov`: Accelerometer measurement covariance
  - `gyr_cov`: Gyroscope measurement covariance
  - `b_acc_cov`: Accelerometer bias covariance
  - `b_gyr_cov`: Gyroscope bias covariance
- **Point cloud processing**:
  - `filter_size_surf`: Voxel grid size for downsampling (smaller = more detail, slower)
  - `filter_size_map`: Map voxel grid size (affects memory usage)
  - `point_filter_num`: Point skip ratio (higher = faster but less accurate)
- **Detection range**: `det_range` sets maximum LiDAR detection distance (default 100m)
- **Frame IDs**: Ensure `lidar_frame`, `imu_frame`, and `base_frame` match your URDF

**PGO Node Parameters** ([src/robofi_bringup/config/fastlio2_pgo.yaml](src/robofi_bringup/config/fastlio2_pgo.yaml)):
- **Loop closure detection**:
  - `loop_closure_search_radius`: Distance threshold for loop candidate search
  - `loop_closure_fitness_threshold`: ICP fitness score threshold for valid loop
  - `loop_closure_frequency`: How often to check for loops (Hz)
- **Map saving**:
  - `save_map_enable: true` to enable automatic map saving
  - `save_map_interval`: Time interval between saves (seconds, -1 for end only)
  - `map_save_path`: Directory to save PCD maps

**Localizer Node Parameters** ([src/robofi_bringup/config/fastlio2_localizer.yaml](src/robofi_bringup/config/fastlio2_localizer.yaml)):
- **Initial pose**: Set approximate starting pose if known
- **Localization fitness**: ICP matching threshold for localization confidence
- **Map file path**: Path to saved PCD map file

### Tuning Octomap Server
Edit parameters in [src/robofi_bringup/config/octomap_server.yaml](src/robofi_bringup/config/octomap_server.yaml):
- **Resolution**: Octree voxel size (default 0.05m, smaller = more detail but more memory)
- **Sensor max range**: Maximum range for inserting point cloud data
- **Height filtering**: Min/max height for 2D projection (important for navigation)
- **Point cloud topics**: Subscribe to `/cloud_registered` from FASTLIO2
- **Frame ID**: Should match the map frame (usually `map` or `camera_init`)
- **Publish frequency**: Rate for publishing 2D projected map

### Integrating FASTLIO2 with Navigation
The integrated system uses FASTLIO2 odometry directly with Nav2:

**Recommended Setup:**
1. **Odometry**: Nav2 subscribes to `/Odometry` from FASTLIO2 LIO node
2. **Localization**: PGO or Localizer node publishes `map → odom` transform
3. **Global costmap**: Uses `/projected_map` from Octomap as static layer
4. **Local costmap**: Uses `/cloud_registered` from FASTLIO2 for voxel layer
5. **Controllers**: Configured to use `/Odometry` for velocity commands

**Configuration in nav2_params.yaml:**
```yaml
# Use FASTLIO2 odometry
odom_topic: /Odometry

# Global costmap layers
global_costmap:
  plugins:
    - static_layer  # Subscribe to /projected_map from Octomap
    - obstacle_layer

# Local costmap layers
local_costmap:
  plugins:
    - voxel_layer  # Subscribe to /cloud_registered from FASTLIO2
    - rgbd_obstacle_layer  # Optional: from D435 depth camera
    - inflation_layer
```

**Benefits of this architecture:**
- High-accuracy odometry from LiDAR-inertial fusion
- Global consistency from loop closure (PGO mode)
- 3D obstacle detection from voxel layer
- Efficient 2D planning from Octomap projection

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

**FASTLIO2 Issues:**
- **LIO not receiving IMU data**:
  - Verify `/livox/imu` topic is publishing: `ros2 topic hz /livox/imu`
  - Check Livox Mid-360 is configured to publish IMU data in MID360_config.json
  - Verify frame IDs match in LIO config and URDF
- **LIO drift or poor accuracy**:
  - Check IMU noise parameters match your sensor specs in fastlio2_lio.yaml
  - Enable online extrinsic calibration: `extrinsic_est_en: true`
  - Ensure sufficient feature-rich environment (not recommended in featureless areas)
  - Verify LiDAR and IMU extrinsic calibration is correct
- **PGO not detecting loop closures**:
  - Increase `loop_closure_search_radius` in fastlio2_pgo.yaml
  - Lower `loop_closure_fitness_threshold` (but may get false positives)
  - Ensure robot revisits same locations with similar viewpoints
- **Localizer poor localization**:
  - Verify map file path is correct and map loads successfully
  - Check initial pose estimate is reasonably close to true position
  - Ensure environment hasn't changed significantly from mapping
- **High CPU usage**:
  - Increase `filter_size_surf` and `filter_size_map` in LIO config
  - Increase `point_filter_num` to skip more points
  - Reduce PGO `loop_closure_frequency`

**Octomap Issues:**
- **Octomap not updating**:
  - Verify `/cloud_registered` topic is publishing: `ros2 topic hz /cloud_registered`
  - Check octomap_server node is running: `ros2 node list`
  - Verify frame_id in octomap config matches FASTLIO2 output
- **Projected map empty or incorrect**:
  - Adjust height filtering parameters (min_z, max_z) in octomap config
  - Check resolution is appropriate for your environment
  - Verify sensor_max_range is not too restrictive
- **High memory usage**:
  - Increase octomap resolution (larger voxels)
  - Enable periodic pruning of free space
  - Limit sensor max range

### Diagnostic Commands

**TF Debugging:**
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
```

**Topic Monitoring:**
```bash
# Monitor sensor topics
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu

# Monitor FASTLIO2 LIO node
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic hz /path

# Monitor FASTLIO2 PGO node (if running)
ros2 topic hz /optimized_path
ros2 topic echo /loop_closure_info

# Monitor Octomap Server
ros2 topic hz /octomap_binary
ros2 topic hz /projected_map
ros2 topic hz /octomap_point_cloud_centers

# Monitor Nav2
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
ros2 topic hz /plan
```

**Data Inspection:**
```bash
# Check FASTLIO2 odometry quality
ros2 topic echo /Odometry

# View point cloud output
ros2 topic echo /cloud_registered --no-arr

# Check loop closure detections
ros2 topic echo /loop_closure_info

# Inspect projected map
ros2 topic echo /projected_map --no-arr
```

**Node Information:**
```bash
# List all active nodes
ros2 node list

# Check FASTLIO2 nodes
ros2 node info /fastlio2_lio_node
ros2 node info /fastlio2_pgo_node
ros2 node info /fastlio2_localizer_node

# Check Octomap node
ros2 node info /octomap_server_node

# Check Nav2 nodes
ros2 node info /controller_server
ros2 node info /planner_server
```

**Visualization:**
```bash
# View node logs
ros2 run rqt_console rqt_console

# Visualize FASTLIO2 + Octomap in RViz
rviz2  # Then add displays for /Odometry, /cloud_registered, /projected_map, etc.

# Monitor system resources
ros2 run rqt_top rqt_top

# View computation graph
rqt_graph
```

**Map Saving and Inspection:**
```bash
# Save current Octomap
ros2 service call /octomap_server/save_map octomap_msgs/srv/SaveOctomap "{filename: '/path/to/map.ot'}"

# Save FASTLIO2 PCD map (check PGO config for auto-save location)
# Usually saved automatically to map_save_path directory

# Convert PCD to other formats (requires PCL tools)
pcl_viewer /path/to/map.pcd
```

## Additional Resources

**Project Documentation:**
- Full documentation: [README.md](README.md)
- Architecture details: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- Quick start guide: [docs/QUICK_START.md](docs/QUICK_START.md)

**SLAM and Mapping:**
- FASTLIO2_ROS2 GitHub: https://github.com/Ericsii/FASTLIO2_ROS2
- Original FAST-LIO paper: https://github.com/hku-mars/FAST_LIO
- Octomap Server 2: https://github.com/iKrishneel/octomap_server2
- OctoMap library: https://octomap.github.io/

**Navigation and Control:**
- Navigation2 docs: https://navigation.ros.org/
- Nav2 costmap plugins: https://navigation.ros.org/plugins/index.html
- MoveIt 2 docs: https://moveit.ros.org/

**Hardware:**
- AgileX Ranger documentation: https://www.agilex.ai/
- Livox Mid-360: https://www.livoxtech.com/mid-360
- RealSense D435: https://www.intelrealsense.com/depth-camera-d435/
