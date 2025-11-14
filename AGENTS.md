# AGENTS.md — Agent Guide for Ranger Garden Assistant

This file provides instructions and conventions for agents working in this repository. Its scope is the entire repository.

## Goals

- Keep changes minimal, surgical, and aligned with ROS 2 Humble practices.
- Prefer configuration and launch changes over modifying upstream/vendor submodules.
- Avoid introducing new dependencies unless necessary and justified.
- Ensure the workspace still builds with `colcon` and launches via existing entrypoints.

## Do / Don’t

- Do: Update `robofi_bringup` and `ranger_description` for integration, parameters, and robot model.
- Do: Add new configuration under `src/robofi_bringup/config/` and new launch files under `src/robofi_bringup/launch/`.
- Do: Extend `fastlio2_navigation.launch.py` when wiring the FASTLIO2_ROS2 + OctoMap + Nav2 stack; keep arguments for toggling PGO, localizer, OctoMap, and Nav2.
- Do: Use launch arguments instead of hard‑coding device names, topics, or frame IDs.
- Do: Validate by building with `colcon` and sanity‑launching minimal stacks when feasible.
- Do: Integrate FAST_LIO via wrapper launches and overlay config under `robofi_bringup`.
- Do: Keep FASTLIO2 (`fastlio2`, `pgo`, `localizer`) topics aligned with `base_footprint`, `odom`, and `map` frames, and feed `/fastlio2/world_cloud` into OctoMap_server2 for Nav2.
- Don’t: Edit vendor submodules in `src/livox_ros_driver2`, `src/ranger_ros2`, `src/ugv_sdk`, or `src/piper_ros` unless explicitly requested.
- Don’t: Modify `src/FAST_LIO` code directly; prefer overlay config + launch wrappers.
- Don’t: Patch `src/FASTLIO2_ROS2` or `src/octomap_server2` directly; use launch/config overlays unless an upstream change is explicitly requested.
- Don’t: Commit generated artifacts (build/, install/, log/). Keep patches focused on source/config.

## Repository Layout (where to change things)

- `src/robofi_bringup/launch/`: Compose/extend system launch files. Add arguments; include other launches via `FindPackageShare` and `PathJoinSubstitution`.
- `src/robofi_bringup/config/`: Place Nav2 and node parameter YAMLs. Keep names descriptive (e.g., `nav2_params.yaml`).
- `src/ranger_description/urdf/`: Xacro/URDF for robot and sensors. Use `xacro:property` for tunables.
- `scripts/`: Shell helpers for building and CAN setup. Keep privileged actions (sudo) inside scripts, not launch files.
- Vendor packages (submodules): Treat as read‑only; prefer overlay via launch/config/URDF.
- FAST_LIO package lives in `src/FAST_LIO` (read‑only). Add your own FAST_LIO params under `src/robofi_bringup/config/fast_lio.yaml` and a wrapper launch under `src/robofi_bringup/launch/fast_lio.launch.py` that includes `fast_lio/mapping.launch.py` with arguments.

## Build & Run (local validation)

Environment: Ubuntu 22.04, ROS 2 Humble, Python 3.10.

- Install/prepare deps (see README for details):
  - `rosdep update && rosdep install --from-paths src --ignore-src -r -y`
  - Optional: `./scripts/build_workspace.sh` for full build flow
- Build:
  - `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
- Source overlay:
  - `source install/setup.bash`
- Quick smoke checks:
  - `ros2 launch robofi_bringup ranger_complete_bringup.launch.py`
  - FAST_LIO mapping: `ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml` (or via our wrapper once added)
  - `ros2 node list` / `ros2 topic list` / `ros2 run tf2_tools view_frames`

## Launch File Conventions (ROS 2 Python launch)

- Use `DeclareLaunchArgument` for tunables (e.g., `can_device`, `use_sim_time`, `params_file`). Provide sensible defaults.
- Use `FindPackageShare` + `PathJoinSubstitution` for file paths.
- Include upstream launches rather than duplicating logic.
- Keep frames, topics, and namespaces configurable. Accept a `namespace` argument where useful for multi‑robot.

Example patterns used in this repo:

- URDF from xacro via `Command([FindExecutable(name="xacro"), " ", <path>])` into `robot_state_publisher`.
- Sensor drivers parameterized via launch arguments and config files (e.g., Livox `MID360_config.json`).
- FAST_LIO via include of `fast_lio/mapping.launch.py` with arguments like `config_path` and `config_file`, and pass topics via YAML (e.g., `lid_topic`, `imu_topic`). Prefer overlay YAML in `robofi_bringup/config/`.
- `fastlio2_navigation.launch.py` composes the entire perception/SLAM stack; keep arguments like `launch_nav2`, `launch_localizer`, `octomap_point_topic`, and `nav2_use_amcl` consistent so the user can toggle components without editing files.

## URDF/Xacro Conventions

- Define sensor/base positions via `xacro:property`; avoid hard‑coding numeric transforms in launch files.
- Standard frames: `map` → `odom` → `base_footprint` → `base_link`. Keep sensor frames parented to `base_link` unless justified.
- Keep optical frames following REP 103/105 conventions.
- For FAST_LIO, keep LiDAR and IMU frames consistent with URDF. Prefer URDF‑defined static transforms (`livox_frame`, `imu_link`) and disable/confine FAST_LIO online extrinsic estimation as appropriate. Avoid duplicating extrinsics both in URDF and FAST_LIO params.

## Navigation (Nav2) Parameters

- Default parameters live in `src/robofi_bringup/config/nav2_params.yaml`.
- The global costmap blends the `/projected_map` generated by OctoMap (static layer) with a Livox obstacle layer; update `map_topic` or obstacle parameters together.
- The local costmap uses a voxel layer fed by `/livox/lidar` and an optional `/camera/depth/color/points` feed from the D435. Remove/adjust `d435_depth` if the camera is not present.
- Neatly set `controller_server.odom_topic`, `bt_navigator.odom_topic`, and `velocity_smoother.odom_topic` to `/fastlio2/lio_odom` whenever you retarget odometry topics.
- If changing LiDAR topic or frame, update both local/global costmap observation sources accordingly.
- Tune controller/goal tolerances conservatively; keep omnidirectional params consistent with the base’s capabilities.
- When using FAST_LIO for pose, avoid conflicting localization sources (e.g., do not run AMCL/slam_toolbox simultaneously publishing `map`→`odom`). Configure Nav2 to consume the TF tree provided by FAST_LIO (`map`→`odom` or `map`→`base_link`, depending on config).

## Submodules Policy

- Do not modify code inside:
- `src/livox_ros_driver2`
- `src/ranger_ros2`
- `src/ugv_sdk`
- `src/piper_ros`
- `src/FAST_LIO`
- `src/FASTLIO2_ROS2`
- `src/octomap_server2`
- If a change is unavoidable, prefer:
  1) Configuration/launch overrides,
  2) Fork upstream and update submodule ref (document in PR),
  3) As a last resort, apply a clearly‑scoped patch with rationale.

## Coding Style

- Python: PEP 8, 4‑space indentation, descriptive names, keep imports standard library → third‑party → local.
- CMake/package.xml: Keep dependencies minimal; add only what is used. Prefer `ament_cmake`/`ament_python` idioms.
- YAML: ROS 2 parameter schema; avoid comments that duplicate obvious defaults.
- Launch files: keep top‑level docstring summarizing purpose; prefer composition over duplication.

## Device & Networking Conventions

- CAN bus:
  - Base on `can0` at 500 kbps, PiPER arm on `can1` at 1000 kbps by default.
  - Use `scripts/setup_can.sh` to configure; don’t embed `sudo` in launches.
- RealSense: use `realsense2_camera` `rs_launch.py` with minimal parameters; align depth and enable pointcloud as needed.
- Livox Mid‑360: use `livox_ros_driver2` with `MID360_config.json` and `frame_id=livox_frame` unless changed in URDF.
- FAST_LIO:
  - Defaults expect `/livox/lidar` and `/livox/imu` from Livox driver. Match these in your overlay YAML (e.g., `lid_topic`, `imu_topic`).
  - Ensure clock sync: prefer hardware timestamps from Livox; otherwise keep NTP/PTP in good shape to minimize IMU/LiDAR time skew.

## Adding Features Safely

- New sensor:
  - Add URDF mount + frame, create a launch wrapper in `robofi_bringup/launch/`, add parameters under `robofi_bringup/config/`, and connect topics to Nav2/perception as needed.
- New behavior/navigation tweaks:
  - Add or extend parameter YAMLs under `robofi_bringup/config/`; wire through via launch `params_file`.
- Arm integration:
  - The PiPER arm launch is present but commented in the complete bringup. Expose control via arguments and keep defaults conservative.
- FAST_LIO integration:
  - Copy a base config (e.g., `src/FAST_LIO/config/mid360.yaml`) into `src/robofi_bringup/config/fast_lio.yaml` and edit there.
  - Add `src/robofi_bringup/launch/fast_lio.launch.py` to include `fast_lio/mapping.launch.py` and point it at the overlay YAML via `config_path`/`config_file`.
  - Keep frame IDs aligned with URDF (`map`, `odom`, `base_link`, `livox_frame`, `imu_link`) and avoid duplicate TF publishers.
- FASTLIO2 + Octomap integration:
  - Overlay configs live under `src/robofi_bringup/config/fastlio2_*.yaml` (LIO, PGO, localizer) and `octomap_server.yaml`.
  - `fastlio2_navigation.launch.py` is the top-level launch composing sensors, FASTLIO2, pose graph (map→odom), optional localizer, OctoMap_server2, and Nav2.
  - Use `/fastlio2/lio_odom` as the odom topic for controllers, `/fastlio2/world_cloud` as the OctoMap input, and `/projected_map` as the Nav2 static layer.
  - The localizer loads `.pcd` files produced by `/pgo/save_maps` via the `/localizer/relocalize` service; provide a coarse pose guess when calling it.

## Validation Checklist (before concluding changes)

- Build succeeds with `colcon build` (no warnings introduced where avoidable).
- Core launches start without fatal errors:
  - `ranger_complete_bringup.launch.py` for integrated bringup, or
  - Individual launches for base, LiDAR, and Nav2 as appropriate.
- TF tree is coherent (`map`→`odom`→`base_*`→sensors) and topics align with config.
- FAST_LIO starts without errors; odometry/TF outputs are present and stable. Visualize trajectories/pointclouds in RViz (`rviz` arg available in `fast_lio/mapping.launch.py`).
- README or docs updated if user‑facing behavior or commands change.

## Common Pitfalls

- Missing permissions on CAN devices: ensure user in `dialout`; use the provided setup script.
- Inconsistent frames between URDF and drivers: prefer URDF‑defined frames, adjust driver `frame_id` or static TFs only if required.
- Overwriting upstream configs in submodules: copy/overlay into `robofi_bringup/config/` instead.
- Running slam_toolbox/AMCL alongside FAST_LIO: avoid conflicting `map` frame publishers; choose one localization/SLAM source.
- FAST_LIO extrinsics: don’t set both URDF and FAST_LIO YAML extrinsics in conflicting ways; choose a single source of truth.
- Topic mismatches: ensure FAST_LIO `lid_topic`/`imu_topic` match the Livox driver outputs.
- FASTLIO2 topic alignment: keep `/fastlio2/lio_odom`, `/fastlio2/world_cloud`, and `/fastlio2/body_cloud` consistent across the LIO, PGO, localizer, and OctoMap to avoid TF breaks.
- OctoMap static layer: Nav2 subscribes to `/projected_map`, so update `octomap_point_topic` or `map_topic` together if you feed another sensor.

## References

- See `README.md` for full installation, quick start, and troubleshooting.
- See `docs/ARCHITECTURE.md` and `docs/QUICK_START.md` for deeper context.
- FAST_LIO usage/config: `src/FAST_LIO/README.md` and `src/FAST_LIO/config/`
