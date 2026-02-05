# Setup Scripts

This directory contains installation and configuration scripts for the Ranger Garden Assistant robot platform.

## Quick Start Scripts

### System Setup

#### `build_workspace.sh`
Builds the entire ROS 2 workspace with all packages.

```bash
./scripts/build_workspace.sh
```

### Hardware Configuration

#### `install_can_udev.sh` ⭐ **Run First**
Sets up CAN bus interfaces for Ranger base and PiPER arm with automatic configuration via udev rules.

```bash
sudo ./scripts/install_can_udev.sh
```

**What it does:**
- Creates udev rules for USB CAN adapters
- Configures `can_base` (500 kbps) for Ranger base
- Configures `can_piper` (1000 kbps) for PiPER arm
- Sets up systemd services for automatic interface activation
- Adds user to `dialout` group

**After installation:** Unplug and replug CAN adapters for rules to take effect.

#### `setup_piper.sh` 🦾 **PiPER Arm Setup**
Interactive setup script for PiPER 6-DOF robotic arm dependencies and testing.

```bash
# Interactive mode (recommended)
./scripts/setup_piper.sh

# Install dependencies only
./scripts/setup_piper.sh --install-deps

# Check CAN interface
./scripts/setup_piper.sh --check-can

# Test arm connection
./scripts/setup_piper.sh --test

# Full setup and check
./scripts/setup_piper.sh --install-deps --check-can
```

**What it does:**
- Installs Python dependencies: `python-can`, `scipy`, `piper_sdk`
- Installs ROS 2 control packages
- Verifies CAN interface configuration
- Tests arm connectivity

### Sensor Configuration

#### `setup_livox_network.sh`
Configures network interface for Livox Mid-360 LiDAR communication.

```bash
sudo ./scripts/setup_livox_network.sh
```

**What it does:**
- Sets up static IP (192.168.1.50) for LiDAR communication
- Configures netplan or NetworkManager
- Creates connection for Livox LiDAR subnet

#### `connect_livox.sh`
Quick script to connect to Livox LiDAR network (alternative to full network setup).

```bash
./scripts/connect_livox.sh
```

#### `install_camera_udev.sh`
Sets up udev rules for Tier IV C2-176 fisheye camera.

```bash
sudo ./scripts/install_camera_udev.sh
```

**What it does:**
- Creates `/dev/tieriv_c2_video0` symlink
- Ensures consistent device naming

#### `install_realsense.sh`
Installs Intel RealSense SDK and drivers for D405 depth camera (optional).

```bash
sudo ./scripts/install_realsense.sh
```

**What it does:**
- Installs librealsense2
- Installs ROS 2 RealSense wrapper
- Configures udev rules for RealSense cameras

### Mapping Tools

#### `save_maps.sh`
Saves current SLAM maps (2D and 3D) to specified directory.

```bash
./scripts/save_maps.sh [output_directory]
```

**What it does:**
- Saves Nav2 2D occupancy grid map
- Saves Octomap 3D volumetric map
- Saves FASTLIO2 PCD point cloud map (if available)

## Installation Order

For a fresh system setup, run scripts in this order:

1. **CAN Bus** (required for base and arm):
   ```bash
   sudo ./scripts/install_can_udev.sh
   # Unplug and replug CAN adapters
   ```

2. **LiDAR Network** (required for SLAM/navigation):
   ```bash
   sudo ./scripts/setup_livox_network.sh
   ```

3. **Camera** (optional, for vision):
   ```bash
   sudo ./scripts/install_camera_udev.sh
   ```

4. **PiPER Arm** (optional, if using arm):
   ```bash
   ./scripts/setup_piper.sh --install-deps --check-can
   ```

5. **RealSense** (optional, if using depth camera):
   ```bash
   sudo ./scripts/install_realsense.sh
   ```

6. **Build Workspace**:
   ```bash
   ./scripts/build_workspace.sh
   ```

## Verification Commands

After setup, verify hardware connections:

```bash
# Check CAN interfaces
ip link show can_base can_piper

# Check CAN messages (arm/base powered on)
candump can_base    # Ranger base
candump can_piper   # PiPER arm

# Check camera
ls -l /dev/tieriv_c2_video0

# Check LiDAR network
ping 192.168.1.1XX  # Replace XX with your LiDAR IP

# Check RealSense
rs-enumerate-devices
```

## Configuration Files

Scripts install these configuration files:

- **udev rules**: `/etc/udev/rules.d/`
  - `99-ranger-can.rules` - CAN bus interfaces
  - `99-tieriv-c2-camera.rules` - Tier IV camera
  - `99-realsense-d405.rules` - RealSense cameras

- **systemd services**: `/etc/systemd/system/`
  - `can-base-setup.service` - Ranger base CAN
  - `can-piper-setup.service` - PiPER arm CAN

## Troubleshooting

### CAN Interface Issues

**Interface not found:**
```bash
# Verify udev rules installed
ls -l /etc/udev/rules.d/99-ranger-can.rules

# Reload udev and replug adapters
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**No CAN messages:**
- Ensure device is powered on
- Check CAN adapter LED (should be blinking)
- Verify bitrate: `ip -details link show can_base`

### LiDAR Connection Issues

**Cannot ping LiDAR:**
- Check Ethernet cable connection
- Verify network interface configured: `ip addr show`
- Run setup script again: `sudo ./scripts/setup_livox_network.sh`

### PiPER Arm Issues

**Python dependencies:**
```bash
# Check installed versions
python3 -c "import can; print(can.__version__)"
python3 -c "import piper_sdk; print('OK')"

# Reinstall
./scripts/setup_piper.sh --install-deps
```

**CAN interface:**
```bash
# Detailed check
./scripts/setup_piper.sh --check-can
```

## Additional Resources

- **Main Documentation**: [CLAUDE.md](../CLAUDE.md)
- **Quick Start Guide**: [docs/QUICK_START.md](../docs/QUICK_START.md)
- **Architecture**: [docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md)
- **PiPER Documentation**: [src/piper_ros/README.MD](../src/piper_ros/README.MD)

## Script Development

All scripts follow these conventions:
- Colored output (RED/GREEN/YELLOW/BLUE) for readability
- Error checking with `set -e`
- Help messages with `--help`
- Idempotent (safe to run multiple times)
- Verbose output explaining each step
