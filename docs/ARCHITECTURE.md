# System Architecture

This document describes the architecture of the Ranger Garden Assistant ROS 2 system.

## Overview

The Ranger Garden Assistant is a mobile manipulation platform built on ROS 2 Humble, integrating:
- **Mobility**: 4-wheel omnidirectional base (Ranger Mini 3.0)
- **Perception**: 3D LiDAR (Livox Mid-360) and RGB-D camera (RealSense D435)
- **Manipulation**: 6-DOF robotic arm (PiPER)
- **Autonomy**: Navigation2 stack for path planning and obstacle avoidance
- **Motion Planning**: MoveIt 2 for arm trajectory planning

## System Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                         ROS 2 Humble                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │  Perception  │  │  Navigation  │  │ Manipulation │         │
│  ├──────────────┤  ├──────────────┤  ├──────────────┤         │
│  │ Livox Driver │  │    Nav2      │  │   MoveIt 2   │         │
│  │ RealSense    │  │  SLAM        │  │  PiPER Ctrl  │         │
│  │              │  │  AMCL        │  │              │         │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘         │
│         │                 │                  │                 │
│         └─────────────────┴──────────────────┘                 │
│                           │                                    │
│                  ┌────────▼────────┐                          │
│                  │   TF / URDF     │                          │
│                  │ Robot State Pub │                          │
│                  └────────┬────────┘                          │
│                           │                                    │
│         ┌─────────────────┴─────────────────┐                 │
│         │                                   │                 │
│    ┌────▼────┐                         ┌────▼────┐           │
│    │  Base   │                         │   Arm   │           │
│    │ Control │                         │ Control │           │
│    └────┬────┘                         └────┬────┘           │
├─────────┼──────────────────────────────────┼─────────────────┤
│         │         Hardware Layer           │                 │
└─────────┼──────────────────────────────────┼─────────────────┘
          │                                  │
     ┌────▼────┐                        ┌────▼────┐
     │  CAN0   │                        │  CAN1   │
     │ Ranger  │                        │ PiPER   │
     │  Base   │                        │  Arm    │
     └─────────┘                        └─────────┘
```

## Component Architecture

### 1. Hardware Layer

#### Ranger Mini 3.0 Base
- **Interface**: CAN bus (can0, 500 kbps)
- **Driver**: `ranger_ros2` package
- **Topics**:
  - `/odom` (nav_msgs/Odometry) - Wheel odometry
  - `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
  - `/motion_state` - Motor states
  - `/battery_state` - Battery information
- **TF**: Publishes `odom` → `base_link`

#### Livox Mid-360 LiDAR
- **Interface**: Ethernet or USB
- **Driver**: `livox_ros_driver2` package
- **Topics**:
  - `/livox/lidar` (sensor_msgs/PointCloud2) - 3D point cloud
  - `/livox/imu` (sensor_msgs/Imu) - IMU data (if available)
- **TF**: `livox_frame` (configured via launch parameters)
- **Update Rate**: 10 Hz
- **FOV**: 360° horizontal, ~±52° vertical

#### Intel RealSense D435
- **Interface**: USB 3.0
- **Driver**: `realsense2_camera` package
- **Topics**:
  - `/camera/color/image_raw` - RGB image
  - `/camera/depth/image_rect_raw` - Depth image
  - `/camera/aligned_depth_to_color/image_raw` - Aligned depth
  - `/camera/depth/color/points` - Colored point cloud
  - `/camera/infra1/image_rect_raw` - IR camera 1
  - `/camera/infra2/image_rect_raw` - IR camera 2
- **TF**: `camera_link` → various optical frames
- **Resolution**: 640x480 to 1920x1080 (configurable)
- **Framerate**: 6-90 FPS (configurable)

#### PiPER 6-DOF Arm
- **Interface**: CAN bus (can1, 1000 kbps)
- **Driver**: `piper_ros` package
- **Topics**:
  - `/joint_states` - Current joint positions
  - `/piper_arm_controller/follow_joint_trajectory` - Trajectory commands
  - `/piper_gripper/command` - Gripper control
- **TF**: `piper_base_link` → `piper_link_1` → ... → `piper_tool0`
- **DOF**: 6 (arm) + 1 (gripper)
- **Payload**: ~1 kg

### 2. Perception Layer

#### Point Cloud Processing
- **Input**: `/livox/lidar` (PointCloud2)
- **Processing**:
  - Frame transformation to `base_link`
  - Ground plane filtering (optional)
  - Downsampling for performance
- **Output**: Used by Navigation costmaps

#### Vision Processing
- **Input**: Camera RGB and depth streams
- **Potential Uses**:
  - Object detection (YOLO, etc.)
  - Grasp pose estimation
  - Visual servoing
  - AprilTag detection
- **Output**: Object poses in `base_link` frame

#### Sensor Fusion
- LiDAR for long-range obstacle detection
- Camera for close-range and textured obstacles
- Both feed into Nav2 costmaps

### 3. Navigation Layer (Nav2)

#### Components

**SLAM (Mapping)**
- **Package**: `slam_toolbox`
- **Mode**: Online async SLAM
- **Input**: LiDAR point cloud (converted to scan if needed)
- **Output**:
  - `/map` topic (occupancy grid)
  - `map` → `odom` transform

**Localization (AMCL)**
- **Package**: `nav2_amcl`
- **Input**: Map, LiDAR scan, odometry
- **Output**: `map` → `odom` transform
- **Algorithm**: Adaptive Monte Carlo Localization

**Costmaps**
- **Global Costmap**:
  - Frame: `map`
  - Size: Full known area
  - Update rate: 1 Hz
  - Layers: Static map, obstacles, inflation

- **Local Costmap**:
  - Frame: `odom`
  - Size: 6x6 m (configurable)
  - Update rate: 5 Hz
  - Layers: Voxel layer (3D obstacles), inflation
  - Plugins: Point cloud from LiDAR

**Path Planning**
- **Global Planner**: NavFn (Dijkstra-based)
- **Local Planner**: DWB (Dynamic Window Approach)
- **Input**: Goal pose, costmaps
- **Output**: `/cmd_vel` commands

**Behavior Trees**
- Navigation logic (navigate_to_pose, navigate_through_poses)
- Recovery behaviors (spin, backup, wait)
- Customizable for task-specific behaviors

#### Nav2 Data Flow

```
Sensors (LiDAR/Camera) → Costmaps → Planners → Controllers → /cmd_vel → Base
                             ↑                                      ↓
                             └─────────── Odometry ←────────────────┘
```

### 4. Manipulation Layer (MoveIt 2)

#### Components

**Motion Planning**
- **Package**: `moveit_ros_planning`
- **Algorithms**: OMPL (RRT, PRM, etc.)
- **Input**: Target pose for end effector
- **Output**: Joint trajectory

**Collision Checking**
- Uses robot URDF + environment representation
- Planning Scene: Updated with sensor data
- Self-collision and environment collision checking

**Kinematics**
- Forward kinematics: Joint angles → end effector pose
- Inverse kinematics: End effector pose → joint angles
- Solver: KDL or TracIK

**Controllers**
- **Joint Trajectory Controller**: Executes planned trajectories
- **Gripper Controller**: Opens/closes gripper
- Interface: ros2_control framework

#### MoveIt Data Flow

```
Goal Pose → IK Solver → Motion Planner → Trajectory → Controller → Arm
                ↑                            ↑
                └── URDF/SRDF               └── Collision Checking
                                                    ↓
                                           Planning Scene (sensors)
```

### 5. Control Layer

#### Base Control
- **Package**: `ranger_ros2`
- **Type**: Velocity control (Twist messages)
- **Kinematics**: Omnidirectional (holonomic)
- **Interface**: CAN bus direct motor control

#### Arm Control
- **Package**: `piper_ros`
- **Type**: Position/Trajectory control
- **Interface**: CAN bus servo control
- **Framework**: ros2_control
- **Controllers**:
  - JointTrajectoryController (arm)
  - GripperActionController (gripper)

### 6. State Estimation

#### Transform Tree (TF)

```
map (from AMCL or SLAM)
 └─ odom (from base odometry)
     └─ base_footprint
         └─ base_link
             ├─ lidar_link (Livox sensor)
             ├─ camera_link (RealSense)
             │   ├─ camera_depth_optical_frame
             │   └─ camera_color_optical_frame
             └─ piper_base_link (Arm mount)
                 └─ piper_link_1
                     └─ piper_link_2
                         └─ piper_link_3
                             └─ piper_link_4
                                 └─ piper_link_5
                                     └─ piper_link_6
                                         └─ piper_tool0 (end effector)
                                             └─ piper_gripper
```

#### Robot State Publisher
- Publishes complete TF tree from URDF
- Updates with joint states from arm and base

## Communication Patterns

### Topics
- **Command Flow**: High-level commands → Nav2/MoveIt → Low-level controllers
- **State Flow**: Sensors → Processing → Navigation/Planning
- **QoS**: Sensor data uses best-effort, commands use reliable

### Actions
- **Navigation**: `NavigateToPose`, `NavigateThroughPoses`
- **Manipulation**: `FollowJointTrajectory`, `GripperCommand`
- **Feedback**: Progress updates during execution

### Services
- **Map Server**: Load/save maps
- **Lifecycle Management**: Start/stop nodes
- **Costmap Updates**: Clear costmaps, set parameters

## Concurrency Model

### Real-time Priorities
1. **High**: CAN communication, safety monitoring
2. **Medium**: Sensor drivers, controllers
3. **Low**: Planning, mapping, visualization

### Threading
- Each ROS 2 node runs in separate process
- Multi-threaded executors for performance
- Separate threads for different callbacks

### Synchronization
- TF for spatial synchronization
- Message timestamps for temporal synchronization
- Approximate time synchronization for sensor fusion

## Data Flow Example: Pick and Place

1. **Perception**: Camera detects object, publishes pose
2. **Navigation**: Nav2 drives robot near object
3. **Grasp Planning**: MoveIt plans pre-grasp approach
4. **Execution**: Arm moves to pre-grasp, then grasp
5. **Gripper**: Close gripper to pick object
6. **Transport**: Arm moves to carry position
7. **Navigation**: Nav2 drives to drop-off location
8. **Place**: Arm places object, gripper opens
9. **Retract**: Arm returns to home position

## Performance Considerations

### CPU Usage
- **Navigation**: ~20-30% (single core)
- **Perception**: ~15-25% (single core)
- **Manipulation**: ~10-15% (single core)
- **Total**: Expect ~60-80% on quad-core system

### Memory Usage
- **Base system**: ~500 MB
- **Navigation (with maps)**: ~300-500 MB
- **Perception**: ~200-300 MB
- **Total**: ~1-1.5 GB typical

### Network Bandwidth
- **LiDAR**: ~10-20 Mbps
- **Camera (uncompressed)**: ~30-50 Mbps
- **Can reduce with compression or lower rates**

## Safety Features

### Hardware Safeties
- E-stop button (if available)
- CAN bus timeout protection
- Battery voltage monitoring

### Software Safeties
- Velocity limits in controllers
- Costmap inflation for safe distance
- Collision checking in motion planning
- Watchdog timers for critical nodes

### Recovery Behaviors
- Costmap clearing
- Rotation recovery (if stuck)
- Backup behavior
- Graceful failure handling

## Extensibility

### Adding Sensors
1. Add sensor driver package
2. Update URDF with sensor frame
3. Configure sensor-specific launch files
4. Add sensor data to costmaps (if needed)

### Adding Behaviors
1. Create behavior tree XML
2. Register custom actions/conditions
3. Configure Nav2 to use custom tree

### Adding Perception
1. Subscribe to camera/LiDAR topics
2. Process and publish detected objects
3. Integrate with Planning Scene (MoveIt)
4. Use for semantic navigation/manipulation

## Configuration Files

### Key Configuration Locations
- **Nav2**: `ranger_bringup/config/nav2_params.yaml`
- **URDF**: `ranger_description/urdf/ranger_complete.urdf.xacro`
- **MoveIt**: `piper_ros/piper_moveit/config/`
- **Sensors**: Launch file parameters

### Tuning Parameters
- **Costmap resolution**: Balance accuracy vs performance
- **DWB weights**: Tune for smooth navigation
- **AMCL particles**: More = better localization, more CPU
- **MoveIt planning time**: Longer = better paths, slower

## Deployment Scenarios

### Lab Testing
- All processing on single workstation
- RViz for visualization
- Manual goal setting

### Field Operation
- Processing on onboard computer (NUC/Jetson)
- Remote monitoring via WiFi
- Waypoint-based missions

### Multi-Robot
- Separate namespaces per robot
- Centralized map server (optional)
- Coordinated task allocation

## Future Enhancements

### Planned Features
- [ ] Semantic mapping (object-level SLAM)
- [ ] Visual servoing for precise manipulation
- [ ] Learning-based grasping
- [ ] Multi-robot coordination
- [ ] Cloud integration for map sharing

### Experimental
- [ ] Adaptive behaviors based on task success
- [ ] Human-robot collaboration
- [ ] Outdoor operation (GPS integration)

## References

- [Nav2 Architecture](https://navigation.ros.org/concepts/index.html)
- [MoveIt 2 Concepts](https://moveit.ros.org/documentation/concepts/)
- [ROS 2 Design](https://design.ros2.org/)
- [TF2 Design](http://wiki.ros.org/tf2/Design)

---

This architecture is designed to be modular, scalable, and maintainable. Each component can be developed, tested, and deployed independently while maintaining system-wide coherence through well-defined interfaces.
