# Copilot Instructions for Ranger Garden Assistant

## Project Overview
ROS 2 Humble workspace for an omnidirectional mobile manipulation platform combining AgileX Ranger Mini 3.0 base, Livox Mid-360 LiDAR, RealSense D435 camera, and PiPER 6-DOF arm. Built for autonomous navigation (Nav2) and manipulation (MoveIt 2).

## Critical Architecture

### Hardware & Communication
- **Dual CAN buses**: `can0` for base (500 kbps), `can1` for arm (1000 kbps). Must run `sudo ./scripts/setup_can.sh` after every reboot before launching.
- **Vendor submodules** (read-only): `livox_ros_driver2`, `ranger_ros2`, `ugv_sdk`, `piper_ros` - never modify directly; overlay via launch/config.
- **Frame hierarchy**: `map` → `odom` → `base_footprint` → `base_link` → sensor frames (defined in `ranger_description/urdf/ranger_complete.urdf.xacro`).

### Package Structure
- **`robofi_bringup`**: Integration layer - all launch files and Nav2 config. Add new launches/params here.
- **`ranger_description`**: URDF/xacro models. Sensor positions defined as `xacro:property` (e.g., `lidar_z`, `camera_x`).
- **Vendor packages**: Drivers for hardware - treat as black boxes accessed via launch includes.

### Launch Composition Pattern
All launches use `FindPackageShare` + `PathJoinSubstitution` for paths and `DeclareLaunchArgument` for configurables. Example from `ranger_complete_bringup.launch.py`:
```python
livox_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare("robofi_bringup"), "launch", "livox_lidar.launch.py"])
    ),
    launch_arguments={"frame_id": "livox_frame"}.items()
)
```
Never hardcode paths or device names - use launch arguments with defaults.

## Build & Development Workflow

### Standard Build
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Quick Rebuild (after changes to Python launch or config)
```bash
colcon build --symlink-install --packages-select robofi_bringup
```

### Livox Driver Special Case
If `livox_ros_driver2` build fails, use its custom build script:
```bash
cd src/livox_ros_driver2 && ./build.sh humble && cd ../..
colcon build
```

### Validation After Changes
1. Build succeeds: `colcon build` (watch for new warnings)
2. Launches start: `ros2 launch robofi_bringup ranger_complete_bringup.launch.py`
3. TF tree valid: `ros2 run tf2_tools view_frames` (check `map→odom→base_*→sensors`)
4. Topics align: `ros2 topic list | grep -E "(lidar|camera|odom)"`

## Configuration Conventions

### Nav2 Parameters
Located in `src/robofi_bringup/config/nav2_params.yaml`. Key patterns:
- **Omnidirectional**: `robot_model_type: "nav2_amcl::OmniMotionModel"`
- **Costmap sensors**: Update `observation_sources` when adding/changing LiDAR topic or frame
- **Frames**: `global_frame_id: "map"`, `base_frame_id: "base_footprint"`, `robot_base_frame: "base_link"`

### URDF Sensor Mounting
Define positions as properties at top of `ranger_complete.urdf.xacro`:
```xml
<xacro:property name="lidar_x" value="0.0"/>
<xacro:property name="lidar_z" value="0.70"/>
```
Then reference in joint origins. Never hardcode transforms in launch files.

### Launch Arguments Pattern
Expose all tunables as arguments with sensible defaults:
```python
DeclareLaunchArgument("can_device", default_value="can0", description="CAN device for base")
```

## Common Tasks

### Adding a New Sensor
1. Mount in URDF: Add link/joint with `xacro:property` position in `ranger_description/urdf/ranger_complete.urdf.xacro`
2. Create launch wrapper: New file in `robofi_bringup/launch/` that includes vendor driver
3. Add config if needed: Parameters in `robofi_bringup/config/`
4. Integrate: Include in `ranger_complete_bringup.launch.py` with conditional argument
5. Update costmap: Add to `observation_sources` in `nav2_params.yaml` if for navigation

### Modifying Navigation Behavior
Edit `robofi_bringup/config/nav2_params.yaml`:
- Controller params: `controller_server` → `FollowPath` plugin
- Local costmap: `local_costmap` → layers/plugins
- Goal tolerances: `controller_server` → `goal_checker`
Wire through launch: `params_file` argument in `navigation.launch.py`

### Debugging CAN Issues
```bash
# Check interfaces up
ip link show can0 can1

# Monitor traffic
candump can0

# Re-setup if needed
sudo ./scripts/setup_can.sh
```

## Critical Pitfalls

1. **Missing CAN setup**: Launches fail silently if `setup_can.sh` not run. Always check `ip link show can0` first.
2. **Frame mismatches**: URDF defines `livox_frame`, driver must match via `frame_id` argument. Check with `ros2 run tf2_tools view_frames`.
3. **Submodule modifications**: Never edit code in `src/{livox_ros_driver2,ranger_ros2,ugv_sdk,piper_ros}`. Overlay via config/launch in `robofi_bringup`.
4. **Forgotten `source install/setup.bash`**: After build, must source before launch. Add to `~/.bashrc` for convenience.
5. **User permissions**: Add user to `dialout` group for CAN access: `sudo usermod -a -G dialout $USER` (requires logout/login).

## Testing Shortcuts

### Quick Hardware Checks
```bash
# CAN traffic (base powered)
candump can0

# RealSense connected
rs-enumerate-devices

# LiDAR reachable
ping 192.168.1.1XX  # Check LiDAR IP

# TF sanity
ros2 run tf2_tools view_frames
```

### Minimal Launch Tests
```bash
# URDF valid
ros2 launch ranger_description display.launch.py

# Base comms
ros2 launch robofi_bringup ranger_base.launch.py
ros2 topic echo /odom

# LiDAR data
ros2 launch robofi_bringup livox_lidar.launch.py
ros2 topic hz /livox/lidar
```

## Key Files Reference

- **Main integration**: `src/robofi_bringup/launch/ranger_complete_bringup.launch.py`
- **Robot model**: `src/ranger_description/urdf/ranger_complete.urdf.xacro`
- **Nav2 tuning**: `src/robofi_bringup/config/nav2_params.yaml`
- **CAN setup**: `scripts/setup_can.sh`
- **Build helper**: `scripts/build_workspace.sh`
- **Architecture docs**: `docs/ARCHITECTURE.md` (component diagrams, TF tree)
- **Agent guidance**: `AGENTS.md` (full conventions, this file's companion)

## ROS 2 Idioms Used

- Python launch API (no XML): All launches use `LaunchDescription`, `Node`, `IncludeLaunchDescription`
- Xacro for modularity: URDF uses `xacro:property` and `xacro:include` patterns
- `colcon --symlink-install`: Enables Python/launch edit without rebuild
- `FindPackageShare`: Portable path resolution instead of hardcoded paths
- Composition via includes: Launch files include vendor launches rather than duplicating

## When to Consult Humans

- Changing vendor submodule code (requires fork + rationale)
- New hardware integration (verify electrical safety, CAN IDs, network config)
- Major Nav2 parameter changes affecting safety (speed limits, costmap inflation)
- Build system modifications outside packages (root CMakeLists.txt, unusual deps)
