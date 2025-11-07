#!/bin/bash
#
# Installation script for Ranger CAN bus udev rules and systemd services
# This script sets up automatic CAN interface configuration on device plug-in
#
# Usage: sudo ./install_can_udev.sh

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "======================================"
echo "Ranger CAN Bus udev Rules Installer"
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

if [ -f "$SCRIPT_DIR/can-arm-setup.service" ]; then
    cp "$SCRIPT_DIR/can-arm-setup.service" /etc/systemd/system/
    echo -e "${GREEN}✓${NC} Copied can-arm-setup.service to /etc/systemd/system/"
else
    echo -e "${YELLOW}⚠${NC} File not found: $SCRIPT_DIR/can-arm-setup.service (skipping)"
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

# Add current user to dialout group (if not root)
if [ -n "$SUDO_USER" ]; then
    echo ""
    echo "Adding user '$SUDO_USER' to 'dialout' group for CAN access..."
    usermod -a -G dialout "$SUDO_USER"
    echo -e "${GREEN}✓${NC} User added to dialout group"
    echo -e "${YELLOW}⚠${NC} User must log out and back in for group changes to take effect"
fi

echo ""
echo "======================================"
echo "Installation Complete!"
echo "======================================"
echo ""
echo "Next steps:"
echo "1. Unplug and replug your CAN adapter(s)"
echo "2. Check interface status with: ip link show"
echo "3. Your CAN interface should now be named 'can_base' (instead of can0)"
echo "4. Verify with: ls -l /dev/can_base"
echo "5. Monitor CAN traffic with: candump can_base"
echo ""
echo "To manually enable services:"
echo "  sudo systemctl enable can-base-setup.service"
echo "  sudo systemctl enable can-arm-setup.service"
echo ""
echo "To check service status:"
echo "  sudo systemctl status can-base-setup.service"
echo ""
