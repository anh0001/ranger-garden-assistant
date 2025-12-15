#!/bin/bash
#
# Installation script for Tier IV C2-176 Camera udev rules
# This script sets up automatic camera device configuration and consistent naming
#
# Usage: sudo ./install_camera_udev.sh

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "======================================"
echo "Tier IV C2-176 Camera udev Rules Installer"
echo "======================================"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Error: This script must be run as root${NC}"
    echo "Please run: sudo ./install_camera_udev.sh"
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Script directory: $SCRIPT_DIR"
echo ""

# Install udev rules
echo "Installing udev rules..."
if [ -f "$SCRIPT_DIR/99-tieriv-c2-camera.rules" ]; then
    cp "$SCRIPT_DIR/99-tieriv-c2-camera.rules" /etc/udev/rules.d/
    echo -e "${GREEN}✓${NC} Copied 99-tieriv-c2-camera.rules to /etc/udev/rules.d/"
else
    echo -e "${RED}✗${NC} File not found: $SCRIPT_DIR/99-tieriv-c2-camera.rules"
    exit 1
fi

# Reload udev rules
echo ""
echo "Reloading udev rules..."
udevadm control --reload-rules
echo -e "${GREEN}✓${NC} udev rules reloaded"

# Trigger udev to apply rules to existing devices
echo ""
echo "Applying rules to existing devices..."
udevadm trigger --subsystem-match=video4linux
udevadm trigger --subsystem-match=media
echo -e "${GREEN}✓${NC} udev rules triggered"

# Add current user to video group (if not root)
if [ -n "$SUDO_USER" ]; then
    echo ""
    echo "Adding user '$SUDO_USER' to 'video' group for camera access..."
    usermod -a -G video "$SUDO_USER"
    echo -e "${GREEN}✓${NC} User added to video group"
    echo -e "${YELLOW}⚠${NC} User must log out and back in for group changes to take effect"
fi

echo ""
echo "======================================"
echo "Installation Complete!"
echo "======================================"
echo ""
echo "Camera Information:"
echo "  - Vendor: ABILITY ENTERPRISE CO., LTD."
echo "  - Model: Tier IV C2-176 (GMSL2-USB3.0 Conversion Kit)"
echo "  - Serial: C2-Master-10fps"
echo ""
echo "Device Symlinks Created:"
echo "  - ${BLUE}/dev/tieriv_c2_video0${NC} (main video device)"
echo "  - ${BLUE}/dev/tieriv_c2_video1${NC} (metadata/secondary)"
echo "  - ${BLUE}/dev/tieriv_c2_media${NC} (media controller)"
echo ""
echo "Next steps:"
echo "1. Unplug and replug your Tier IV C2-176 camera"
echo "2. Verify symlinks were created:"
echo "   ${BLUE}ls -l /dev/tieriv_c2_*${NC}"
echo ""
echo "3. Check camera detection:"
echo "   ${BLUE}v4l2-ctl --list-devices${NC}"
echo ""
echo "4. View camera info:"
echo "   ${BLUE}v4l2-ctl --device=/dev/tieriv_c2_video0 --all${NC}"
echo ""
echo "5. Test camera with (if gstreamer installed):"
echo "   ${BLUE}gst-launch-1.0 v4l2src device=/dev/tieriv_c2_video0 ! videoconvert ! autovideosink${NC}"
echo ""
echo "Troubleshooting:"
echo "  - Check if camera is detected: ${BLUE}lsusb | grep 1419${NC}"
echo "  - View udev events: ${BLUE}udevadm monitor --subsystem-match=video4linux${NC}"
echo "  - Check device permissions: ${BLUE}ls -l /dev/video*${NC}"
echo ""
