# gs_usb Installation Guide for Jetson Orin AGX

## Summary
Successfully installed the gs_usb kernel module on Jetson Orin AGX running Ubuntu 22.04 with JetPack 6.2.1 (L4T R36.4.7).

## What was Done

### 1. Downloaded Kernel Sources

**Option A: Using jetson-orin-kernel-builder (Recommended)**
Clone the Jetson Orin kernel builder repository:
```bash
git clone https://github.com/jetsonhacks/jetson-orin-kernel-builder.git
cd jetson-orin-kernel-builder
```

This repository provides scripts to automate the kernel building process for Jetson Orin.

**Option B: Manual Download**
- Downloaded L4T R36.4.0 kernel sources (compatible with R36.4.7)
- Extracted to `/usr/src/kernel/kernel-jammy-src`

### 2. Configured Kernel
- Copied current kernel configuration from `/proc/config.gz`
- Enabled `CONFIG_CAN_GS_USB=m` in kernel config
- CAN bus support was already enabled (`CONFIG_CAN=m`, `CONFIG_CAN_DEV=m`)

### 3. Built and Installed Module
- Compiled CAN network drivers including gs_usb
- Installed modules to `/lib/modules/5.15.148-tegra/updates/usb/`
- Updated module dependencies with `depmod -a`

### 4. Configured Auto-load
- Created `/etc/modules-load.d/gs_usb.conf` to load gs_usb at boot

## Verification

The gs_usb module is now loaded and working:

```bash
$ lsmod | grep gs_usb
gs_usb                 24576  0
can_dev                36864  2 mttcan,gs_usb
```

Note: Jetson MTTCAN (`can0`/`can1`) may still be loaded by the kernel, but this project uses gs_usb adapters only.

Your CAN devices appear as `can_base` and (if a second adapter is connected) `can_piper` after installing the udev rules:
```bash
$ ip link show | grep can
can_base: <NOARP,ECHO> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
can_piper: <NOARP,ECHO> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
```

## Using the CAN Interface

### Basic Setup
Set up your CAN interface (replace 500000 with your desired bitrate):
```bash
sudo ip link set can_base type can bitrate 500000
sudo ip link set can_base up
```

For the PiPER arm interface:
```bash
sudo ip link set can_piper type can bitrate 1000000
sudo ip link set can_piper up
```

### Monitor CAN Bus
```bash
candump can_base
```

### Send CAN Message
```bash
cansend can_base 123#DEADBEEF
```

### Check Interface Status
```bash
ip -details link show can_base
```

### Bring Interface Down
```bash
sudo ip link set can_base down
```

## Available CAN Interfaces

Your system may have the following CAN interfaces:
- `can_base` - USB CAN adapter for the Ranger base (gs_usb driver)
- `can_piper` - USB CAN adapter for the PiPER arm (gs_usb driver)
- `can0`, `can1` - Built-in MTTCAN interfaces (leave down; not used by this repo)

## CAN Utils Commands

Useful commands with can-utils package:

| Command | Description |
|---------|-------------|
| `candump` | Display received CAN frames |
| `cansend` | Send a single CAN frame |
| `cangen` | Generate random CAN traffic |
| `cansequence` | Send and check sequence of CAN frames |
| `cansniffer` | Display CAN data changes |
| `canplayer` | Replay CAN log files |
| `canlogserver` | Log CAN frames to file |

## Troubleshooting

### Module not loading
If the module doesn't load automatically after reboot:
```bash
sudo modprobe gs_usb
```

### Check kernel messages
```bash
sudo dmesg | grep -i can
sudo dmesg | grep gs_usb
```

### Verify module info
```bash
modinfo gs_usb
```

### Rebuild if needed
If you need to rebuild the module after a kernel update:
```bash
cd /usr/src/kernel/kernel-jammy-src
sudo make M=drivers/net/can modules
sudo make M=drivers/net/can modules_install
sudo cp /lib/modules/5.15.148/updates/usb/gs_usb.ko /lib/modules/$(uname -r)/updates/usb/
sudo cp /lib/modules/5.15.148/updates/dev/can-dev.ko /lib/modules/$(uname -r)/updates/usb/
sudo depmod -a
```

## References

- [Linux SocketCAN Documentation](https://www.kernel.org/doc/html/latest/networking/can.html)
- [can-utils GitHub](https://github.com/linux-can/can-utils)
- gs_usb supports:
  - Geschwister Schneider USB/CAN adapters
  - candleLight USB CAN adapters
  - Many other compatible USB-to-CAN devices

## Installation Date
January 30, 2026

## System Info
- Device: Jetson Orin AGX
- OS: Ubuntu 22.04 (Jammy)
- JetPack: 6.2.1
- L4T: R36.4.7
- Kernel: 5.15.148-tegra
