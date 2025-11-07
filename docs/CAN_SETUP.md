# CAN Bus Setup and Configuration

This document describes the CAN bus setup for the Ranger Garden Assistant, including automatic configuration using udev rules.

## Hardware Configuration

The Ranger Garden Assistant uses USB CAN adapters for communication with:
- **Ranger Mini 3.0 Base**: CAN bus @ 500 kbps
- **PiPER Robotic Arm**: CAN bus @ 1000 kbps (optional)

### Detected Hardware
- **Adapter Type**: candleLight/gs_usb (Geschwister Schneider CAN adapter)
- **Vendor ID**: 1d50 (OpenMoko, Inc.)
- **Product ID**: 606f
- **Manufacturer**: bytewerk
- **Serial Number**: 002300405246571220393733

## Port Naming Convention

### Recommended Names
- **`can_base`** - CAN adapter for Ranger base controller (500 kbps)
- **`can_arm`** - CAN adapter for PiPER arm controller (1000 kbps)

### Why These Names?
✅ **Descriptive**: Clearly indicates which hardware the interface connects to
✅ **Persistent**: Won't change between reboots or USB port changes
✅ **Intuitive**: Easy to remember and use in launch files
✅ **Compatible**: Works with ROS 2 naming conventions

### Alternative Naming Schemes Considered
| Name | Pros | Cons | Verdict |
|------|------|------|---------|
| `can_base`, `can_arm` | Descriptive, intuitive | None | **SELECTED** ✓ |
| `can0`, `can1` | Standard Linux naming | Not persistent, unclear purpose | ✗ |
| `ranger_base_can`, `piper_arm_can` | Very descriptive | Too verbose | ✗ |
| `canbus0`, `canbus1` | Clear it's CAN | Generic, doesn't indicate purpose | ✗ |

## Installation

### Automatic Setup (Recommended)

Run the installation script to set up udev rules and systemd services:

```bash
cd ~/codes/ranger-garden-assistant
sudo ./scripts/install_can_udev.sh
```

This will:
1. Install udev rules to `/etc/udev/rules.d/99-ranger-can.rules`
2. Install systemd services for automatic interface configuration
3. Add your user to the `dialout` group for CAN access
4. Reload udev rules and trigger device detection

After installation:
```bash
# Log out and back in (for group changes to take effect)
# Or run: newgrp dialout

# Unplug and replug CAN adapter(s)

# Verify new interface name
ls -l /dev/can_base

# Check interface status
ip link show can_base
```

### Manual Setup

If you prefer manual configuration:

```bash
# Copy udev rules
sudo cp scripts/99-ranger-can.rules /etc/udev/rules.d/

# Copy systemd services
sudo cp scripts/can-base-setup.service /etc/systemd/system/
sudo cp scripts/can-arm-setup.service /etc/systemd/system/

# Reload and apply
sudo systemctl daemon-reload
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and back in
```

## Usage

### Checking Interface Status

```bash
# Check if interface exists
ip link show can_base

# Check detailed status
ip -details link show can_base

# View systemd service status
sudo systemctl status can-base-setup.service
```

### Manual Interface Configuration

If systemd services are not enabled or you need manual control:

```bash
# Bring up Ranger base interface
sudo ip link set can_base down
sudo ip link set can_base type can bitrate 500000
sudo ip link set can_base up

# Bring up PiPER arm interface (if available)
sudo ip link set can_arm down
sudo ip link set can_arm type can bitrate 1000000
sudo ip link set can_arm up
```

### Testing CAN Communication

```bash
# Monitor CAN traffic (Ranger base)
candump can_base

# Send a test frame
cansend can_base 123#DEADBEEF

# Monitor with timestamps
candump can_base -t a

# Monitor both interfaces
candump any
```

### Verifying Hardware Connection

```bash
# Check USB device
lsusb | grep -i "can\|1d50:606f"

# Check interface statistics
ip -s link show can_base

# View error counters
cat /sys/class/net/can_base/statistics/tx_errors
cat /sys/class/net/can_base/statistics/rx_errors
```

## Integration with ROS 2

### Updating Launch Files

After installing udev rules, update your launch files to use the new interface names:

**Before:**
```python
can_interface = LaunchConfiguration('can_interface', default='can0')
```

**After:**
```python
can_interface = LaunchConfiguration('can_interface', default='can_base')
```

### Example Launch Configuration

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface',
            default_value='can_base',
            description='CAN interface name for Ranger base'
        ),
        Node(
            package='ranger_base',
            executable='ranger_base_node',
            name='ranger_base_node',
            parameters=[{
                'can_device': LaunchConfiguration('can_interface'),
                'can_bitrate': 500000
            }]
        )
    ])
```

## Troubleshooting

### Interface Not Found

```bash
# Check if USB device is detected
lsusb | grep 1d50:606f

# Check kernel messages
sudo dmesg | grep -i can

# Check udev rules
ls -l /etc/udev/rules.d/99-ranger-can.rules

# Manually trigger udev
sudo udevadm trigger --subsystem-match=net --verbose
```

### Interface Exists But Down

```bash
# Check systemd service
sudo systemctl status can-base-setup.service

# View service logs
sudo journalctl -u can-base-setup.service -n 50

# Manually bring up interface
sudo ip link set can_base up
```

### Permission Denied

```bash
# Check group membership
groups $USER

# Should include 'dialout' group
# If not, add and re-login:
sudo usermod -a -G dialout $USER
```

### Multiple Adapters Not Detected

If you have multiple CAN adapters:

1. Plug in the second adapter
2. Find its serial number:
   ```bash
   lsusb -v -d 1d50:606f | grep iSerial
   ```
3. Edit `scripts/99-ranger-can.rules` and uncomment the second rule
4. Replace `YOUR_SECOND_ADAPTER_SERIAL` with the actual serial number
5. Reinstall: `sudo ./scripts/install_can_udev.sh`

### Reverting to Default Names

To remove custom naming and go back to `can0`, `can1`:

```bash
# Disable services
sudo systemctl disable can-base-setup.service
sudo systemctl disable can-arm-setup.service

# Remove udev rules
sudo rm /etc/udev/rules.d/99-ranger-can.rules

# Reload
sudo udevadm control --reload-rules
sudo udevadm trigger

# Unplug and replug adapters
```

## systemd Service Details

### Automatic Startup

The udev rules automatically trigger systemd services when CAN adapters are plugged in. The services:
- Wait for the device to stabilize
- Configure the correct bitrate
- Bring the interface up
- Remain active as long as the device is connected

### Manual Service Control

```bash
# Start service
sudo systemctl start can-base-setup.service

# Stop service
sudo systemctl stop can-base-setup.service

# Enable at boot (optional, udev triggers it anyway)
sudo systemctl enable can-base-setup.service

# View logs
sudo journalctl -u can-base-setup.service -f
```

## Configuration Files

### Created Files
- [`scripts/99-ranger-can.rules`](../scripts/99-ranger-can.rules) - udev rules for device naming
- [`scripts/can-base-setup.service`](../scripts/can-base-setup.service) - systemd service for base interface
- [`scripts/can-arm-setup.service`](../scripts/can-arm-setup.service) - systemd service for arm interface
- [`scripts/install_can_udev.sh`](../scripts/install_can_udev.sh) - Installation script

### Installed Locations
- `/etc/udev/rules.d/99-ranger-can.rules`
- `/etc/systemd/system/can-base-setup.service`
- `/etc/systemd/system/can-arm-setup.service`

## See Also

- [CLAUDE.md](../CLAUDE.md#hardware-setup) - Hardware setup overview
- [README.md](../README.md) - Project overview
- [setup_can.sh](../scripts/setup_can.sh) - Original manual setup script
