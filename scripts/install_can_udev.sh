#!/bin/bash
#
# Installation script for Ranger CAN bus configuration
# This script sets up USB CAN adapters via udev rules on all platforms.
# Jetson systems should use gs_usb adapters (can_base/can_piper); MTTCAN is not configured.
#
# Usage: sudo ./install_can_udev.sh

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "======================================"
echo "Ranger CAN Bus Configuration Installer"
echo "======================================"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Error: This script must be run as root${NC}"
    echo "Please run: sudo ./install_can_udev.sh"
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Script directory: $SCRIPT_DIR"
echo ""

# Detect platform
echo "Detecting platform..."
IS_JETSON=false
if [ -f /etc/nv_tegra_release ]; then
    IS_JETSON=true
    JETSON_INFO=$(cat /etc/nv_tegra_release | head -n1)
    echo -e "${BLUE}Platform: NVIDIA Jetson (gs_usb USB CAN, MTTCAN not used)${NC}"
    echo "Info: $JETSON_INFO"
else
    echo -e "${BLUE}Platform: Standard Linux (using USB CAN adapters)${NC}"
fi
echo ""
echo "Setting up USB CAN adapters with udev rules..."
echo ""

# Disable and remove old CAN services if they exist
for old_service in \
    ranger-can-setup.service \
    can-base-setup.service \
    can-arm-setup.service \
    can-piper-setup.service \
    can0-setup.service \
    can1-setup.service; do
    if systemctl is-enabled "$old_service" &>/dev/null; then
        echo "Disabling old $old_service..."
        systemctl disable "$old_service" &>/dev/null || true
        systemctl stop "$old_service" &>/dev/null || true
    fi
    if [ -f "/etc/systemd/system/$old_service" ]; then
        rm -f "/etc/systemd/system/$old_service"
        echo -e "${YELLOW}⚠${NC} Removed old $old_service"
    fi
done

# Install udev rules
echo "Installing udev rules..."
if [ -f "$SCRIPT_DIR/99-ranger-can.rules" ]; then
    cp "$SCRIPT_DIR/99-ranger-can.rules" /etc/udev/rules.d/
    echo -e "${GREEN}✓${NC} Copied 99-ranger-can.rules to /etc/udev/rules.d/"
else
    echo -e "${RED}✗${NC} File not found: $SCRIPT_DIR/99-ranger-can.rules"
    exit 1
fi

# Install systemd services
echo "Installing systemd services..."

if [ -f "$SCRIPT_DIR/can-base-setup.service" ]; then
    cp "$SCRIPT_DIR/can-base-setup.service" /etc/systemd/system/
    echo -e "${GREEN}✓${NC} Copied can-base-setup.service to /etc/systemd/system/"
else
    echo -e "${YELLOW}⚠${NC} File not found: $SCRIPT_DIR/can-base-setup.service (skipping)"
fi

if [ -f "$SCRIPT_DIR/can-piper-setup.service" ]; then
    cp "$SCRIPT_DIR/can-piper-setup.service" /etc/systemd/system/
    echo -e "${GREEN}✓${NC} Copied can-piper-setup.service to /etc/systemd/system/"
else
    echo -e "${YELLOW}⚠${NC} File not found: $SCRIPT_DIR/can-piper-setup.service (skipping)"
fi

# Reload systemd daemon
echo ""
echo "Reloading systemd daemon..."
systemctl daemon-reload
echo -e "${GREEN}✓${NC} Systemd daemon reloaded"

# Reload udev rules
echo ""
echo "Reloading udev rules..."
udevadm control --reload-rules
echo -e "${GREEN}✓${NC} udev rules reloaded"

# Trigger udev to apply rules to existing devices
echo ""
echo "Applying rules to existing devices..."
udevadm trigger --subsystem-match=net
echo -e "${GREEN}✓${NC} udev rules triggered"

# Add current user to dialout group
echo ""
if [ -n "$SUDO_USER" ]; then
    TARGET_USER="$SUDO_USER"
    echo "Adding user '$TARGET_USER' to 'dialout' group for CAN access..."
elif [ -n "$USER" ] && [ "$USER" != "root" ]; then
    TARGET_USER="$USER"
    echo "Adding user '$TARGET_USER' to 'dialout' group for CAN access..."
else
    echo -e "${YELLOW}⚠${NC} Could not detect user automatically"
    echo "Please manually add your user to dialout group:"
    echo "  sudo usermod -a -G dialout \$USER"
    TARGET_USER=""
fi

if [ -n "$TARGET_USER" ]; then
    if id "$TARGET_USER" &>/dev/null; then
        usermod -a -G dialout "$TARGET_USER"
        echo -e "${GREEN}✓${NC} User '$TARGET_USER' added to dialout group"
        echo -e "${YELLOW}⚠${NC} User must log out and back in for group changes to take effect"
    else
        echo -e "${RED}✗${NC} User '$TARGET_USER' not found"
    fi
fi

echo ""
echo "======================================"
echo "Installation Complete!"
echo "======================================"
echo ""

echo "Configuration: USB CAN Adapters (gs_usb)"
echo "  can_base  -> Ranger base @ 500 kbps"
echo "  can_piper -> PiPER arm @ 1000 kbps"
echo ""
if [ "$IS_JETSON" = true ]; then
    echo "Note: Jetson MTTCAN (can0/can1) is not configured by this script."
    echo ""
fi
echo "Next steps:"
echo "1. Log out and back in (or reboot) for group changes to take effect"
echo "2. Unplug and replug your CAN adapter(s)"
echo "3. Check interface status: ip link show can_base can_piper"
echo "4. Verify you're in dialout group: groups | grep dialout"
echo "5. Monitor Ranger base: candump can_base"
echo "6. Monitor PiPER arm: candump can_piper"
echo ""
echo "Service management:"
echo "  Enable: sudo systemctl enable can-base-setup.service"
echo "  Enable: sudo systemctl enable can-piper-setup.service"
echo "  Check status: sudo systemctl status can-base-setup.service"
echo "  Check status: sudo systemctl status can-piper-setup.service"
echo ""
