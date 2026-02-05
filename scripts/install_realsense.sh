#!/bin/bash
#
# Installation script for Intel RealSense SDK 2.0 and ROS 2 wrapper
# For Intel RealSense D405 depth camera (PiPER gripper)
#
# Usage: sudo ./install_realsense.sh
#

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "=========================================="
echo "Intel RealSense Camera Installation"
echo "  - RealSense SDK 2.0 (librealsense)"
echo "  - RealSense ROS 2 Humble wrapper"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Error: This script must be run as root${NC}"
    echo "Please run: sudo ./install_realsense.sh"
    exit 1
fi

# Detect OS version
OS_CODENAME=$(lsb_release -cs)
echo "Detected OS: Ubuntu ${OS_CODENAME}"
echo ""

# Check ROS 2 installation
if [ ! -d "/opt/ros/humble" ]; then
    echo -e "${RED}Error: ROS 2 Humble not found${NC}"
    echo "Please install ROS 2 Humble first"
    exit 1
fi
echo -e "${GREEN}✓${NC} ROS 2 Humble detected"
echo ""

# Step 1: Install dependencies
echo "======================================"
echo "Step 1: Installing dependencies"
echo "======================================"
apt-get update
apt-get install -y \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    gnupg \
    wget \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev

echo -e "${GREEN}✓${NC} Dependencies installed"
echo ""

# Step 2: Add Intel RealSense public key
echo "======================================"
echo "Step 2: Adding Intel RealSense repository"
echo "======================================"

# Remove old key if exists
apt-key del F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE 2>/dev/null || true

# Add new GPG key
mkdir -p /etc/apt/keyrings
wget -qO- https://librealsense.intel.com/Debian/librealsense.pgp | \
    gpg --dearmor -o /etc/apt/keyrings/librealsense.gpg

# Add repository
echo "deb [signed-by=/etc/apt/keyrings/librealsense.gpg] https://librealsense.intel.com/Debian/apt-repo ${OS_CODENAME} main" | \
    tee /etc/apt/sources.list.d/librealsense.list > /dev/null

echo -e "${GREEN}✓${NC} Intel RealSense repository added"
echo ""

# Step 3: Install RealSense SDK
echo "======================================"
echo "Step 3: Installing RealSense SDK 2.0"
echo "======================================"
apt-get update
apt-get install -y \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg

echo -e "${GREEN}✓${NC} RealSense SDK installed"
echo ""

# Step 4: Verify SDK installation
echo "Verifying SDK installation..."
rs-enumerate-devices --compact 2>/dev/null && echo -e "${GREEN}✓${NC} SDK verification successful" || echo -e "${YELLOW}⚠${NC} No RealSense devices detected (plug in camera to test)"
echo ""

# Step 5: Install RealSense ROS 2 wrapper
echo "======================================"
echo "Step 4: Installing RealSense ROS 2 wrapper"
echo "======================================"
apt-get install -y ros-humble-realsense2-camera ros-humble-realsense2-description

echo -e "${GREEN}✓${NC} RealSense ROS 2 wrapper installed"
echo ""

# Step 6: Install udev rules (included in librealsense2-dkms, but verify)
echo "======================================"
echo "Step 5: Configuring udev rules"
echo "======================================"

# The librealsense2-dkms package should install rules at:
# /etc/udev/rules.d/99-realsense-libusb.rules
if [ -f "/etc/udev/rules.d/99-realsense-libusb.rules" ]; then
    echo -e "${GREEN}✓${NC} RealSense udev rules already installed"
else
    echo -e "${YELLOW}⚠${NC} RealSense udev rules not found, installing manually..."

    # Download and install udev rules manually
    wget -q https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
        -O /etc/udev/rules.d/99-realsense-libusb.rules

    echo -e "${GREEN}✓${NC} RealSense udev rules installed"
fi

# Reload udev rules
udevadm control --reload-rules
udevadm trigger
echo -e "${GREEN}✓${NC} udev rules reloaded"
echo ""

# Step 7: Add user to video group
if [ -n "$SUDO_USER" ]; then
    echo "Adding user '$SUDO_USER' to 'video' group..."
    usermod -a -G video "$SUDO_USER"
    echo -e "${GREEN}✓${NC} User added to video group"
    echo -e "${YELLOW}⚠${NC} User must log out and back in for group changes to take effect"
fi
echo ""

# Installation complete
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "RealSense SDK version:"
dpkg -l | grep librealsense2 | head -n 1
echo ""
echo "RealSense ROS 2 wrapper:"
dpkg -l | grep ros-humble-realsense2-camera
echo ""
echo "=========================================="
echo "Testing and Verification"
echo "=========================================="
echo ""
echo "1. Plug in your RealSense D405 camera"
echo ""
echo "2. List connected RealSense devices:"
echo "   ${BLUE}rs-enumerate-devices${NC}"
echo ""
echo "3. Test camera with RealSense Viewer (GUI):"
echo "   ${BLUE}realsense-viewer${NC}"
echo ""
echo "4. Test with ROS 2 (RGB only):"
echo "   ${BLUE}source /opt/ros/humble/setup.bash${NC}"
echo "   ${BLUE}ros2 launch realsense2_camera rs_launch.py \\${NC}"
echo "   ${BLUE}    enable_depth:=false \\${NC}"
echo "   ${BLUE}    enable_infra:=false \\${NC}"
echo "   ${BLUE}    enable_color:=true \\${NC}"
echo "   ${BLUE}    rgb_camera.profile:=640x480x30${NC}"
echo ""
echo "5. View RGB image in RViz:"
echo "   ${BLUE}rviz2${NC}"
echo "   Then add Image display and subscribe to: ${BLUE}/camera/color/image_raw${NC}"
echo ""
echo "6. Check RGB topic is publishing:"
echo "   ${BLUE}ros2 topic hz /camera/color/image_raw${NC}"
echo ""
echo "=========================================="
echo "Troubleshooting"
echo "=========================================="
echo ""
echo "- Check if camera is detected:"
echo "  ${BLUE}lsusb | grep 8086:0b5b${NC}  (Intel RealSense D405)"
echo ""
echo "- View camera details:"
echo "  ${BLUE}rs-enumerate-devices -c${NC}"
echo ""
echo "- Check kernel modules:"
echo "  ${BLUE}lsmod | grep uvcvideo${NC}"
echo ""
echo "- Test firmware version:"
echo "  ${BLUE}rs-fw-update -l${NC}"
echo ""
echo "- If camera not detected after installation:"
echo "  1. Unplug and replug the camera"
echo "  2. Try a different USB 3.0 port"
echo "  3. Reboot the system"
echo "  4. Check USB 3.0 cable quality"
echo ""
echo "For more help, visit: https://github.com/IntelRealSense/librealsense"
echo ""
