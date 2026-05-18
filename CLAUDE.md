# Ranger Garden Assistant

ROS 2 Humble mobile-manipulation stack: AgileX Ranger Mini 3.0 omnidirectional base
+ PiPER 6-DOF arm, Livox Mid-360 LiDAR, Tier IV C2-176 fisheye camera, RealSense
D405 wrist camera. Targets NVIDIA Jetson AGX Orin (Ubuntu 22.04 / JetPack 6).

## Environment

Every shell that runs ROS commands must source, in order:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash          # after a build, from the workspace root
```

Python is 3.10+. Hardware deps not managed by rosdep: `python-can`, `piper_sdk`, `scipy`.

## Build

Use the wrapper — it initializes submodules, builds Sophus + Livox-SDK2 from
source (FASTLIO2 / livox_ros_driver2 prerequisites), then colcon-builds:

```bash
./scripts/build_workspace.sh
```

Direct colcon (after prerequisites exist):

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Build a single package: `colcon build --packages-select <pkg> --symlink-install`.

## Submodules

`src/` mixes local packages and 7 submodules (see `.gitmodules`). Submodules are
vendored drivers — do not edit them as part of feature work; changes belong
upstream. `piper_ros` tracks the `ranger` branch. Always
`git submodule update --init --recursive` after a fresh clone or branch switch.

Local packages: `robofi_bringup` (integration + launch files),
`ranger_bringup`, `ranger_description` (URDF/XACRO), `ranger_piper_moveit`
(MoveIt 2 config), `ranger_sim` (Gazebo Fortress), `camera_lidar_fuse` (sensor
fusion, pure Python — the most unit-testable package).

## Launch entry points

`src/robofi_bringup/launch/` — `ranger_complete_bringup.launch.py` (full stack),
`fastlio2_navigation.launch.py`, `navigation.launch.py`, `ranger_base.launch.py`,
`livox_lidar.launch.py`, `slam.launch.py`.

## Architectural notes

- **FASTLIO2_ROS2** (not AMCL/slam_toolbox) is the primary localization source:
  tightly-coupled LiDAR-inertial odometry, loop-closure pose-graph optimization
  publishing `map -> odom`, optional relocalization on saved maps. It feeds both
  Navigation2 and octomap_server2. Prefer this path when touching localization;
  `slam.launch.py` exists for comparison/fallback only.
- CAN topology uses named interfaces `can_base` and `can_piper` (not
  `can0`/`can1`); MTTCAN is intentionally not configured. See
  `docs/gs_usb_installation.md`.

## Hardware safety

This repo commands real actuators. Treat anything that moves the base or arm or
writes to a CAN bus as requiring explicit human confirmation — never run it
speculatively to "test". This includes `scripts/piper_*.py`, `ros2 topic pub`
to cmd_vel / joint / servo topics, `ros2 action send_goal` for motion, and
`cansend` / `ip link` on `can_base`/`can_piper`.

## Conventions

- Commits follow Conventional Commits (`feat:`, `fix:`, `refactor:`, `docs:`),
  matching existing history.
- Documentation lives in `docs/`; `README.md` is the canonical setup guide —
  keep it in sync when changing build or launch behavior.

<!-- ARIS:BEGIN -->
## ARIS Skill Scope
ARIS skills installed in this project: 75 entries.
Manifest: `.aris/installed-skills.txt` (lists every skill ARIS installed and its upstream target).
For ARIS workflows, prefer the project-local skills under `.claude/skills/` over global skills.
Do not modify or delete files inside any skill that is a symlink (symlinks point into `/home/anhar/codes/Auto-claude-code-research-in-sleep`).
Update with: `bash /home/anhar/codes/Auto-claude-code-research-in-sleep/tools/install_aris.sh`  (re-runnable; reconciles new/removed skills).
<!-- ARIS:END -->
