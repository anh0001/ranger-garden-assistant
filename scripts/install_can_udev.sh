#!/bin/bash
#
# Installation script for Ranger CAN bus configuration
# This script automatically detects the platform and sets up CAN interfaces:
#   - On Jetson: Uses built-in hardware CAN (can0/can1)
#   - On other platforms: Uses USB CAN adapters with udev rules
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
    echo -e "${BLUE}Platform: NVIDIA Jetson${NC}"
    echo "Info: $JETSON_INFO"
elif [ -d /sys/class/net/can0 ] && [ -d /sys/class/net/can1 ]; then
    # Check if these are hardware CAN interfaces (not USB)
    CAN0_KERNEL=$(readlink -f /sys/class/net/can0/device 2>/dev/null | grep -o "mttcan\|c_can\|flexcan" || echo "")
    if [ -n "$CAN0_KERNEL" ]; then
        IS_JETSON=true
        echo -e "${BLUE}Platform: Embedded system with hardware CAN${NC}"
    fi
fi

if [ "$IS_JETSON" = false ]; then
    echo -e "${BLUE}Platform: Standard Linux (using USB CAN adapters)${NC}"
fi
echo ""

if [ "$IS_JETSON" = true ]; then
    # Jetson platform - use hardware CAN interfaces
    echo "Setting up Jetson hardware CAN interfaces..."
    echo "Mapping: can0 -> Ranger base (500 kbps), can1 -> PiPER arm (1000 kbps)"
    echo ""

    # Disable and remove old USB adapter services if they exist (old naming)
    for old_service in can-base-setup.service can-arm-setup.service can0-setup.service can1-setup.service; do
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

    # Install systemd service for hardware CAN
    if [ -f "$SCRIPT_DIR/ranger-can-setup.service" ]; then
        cp "$SCRIPT_DIR/ranger-can-setup.service" /etc/systemd/system/
        echo -e "${GREEN}✓${NC} Installed ranger-can-setup.service"
    else
        echo -e "${RED}✗${NC} File not found: $SCRIPT_DIR/ranger-can-setup.service"
        echo "Creating service file inline as fallback..."
        cat > /etc/systemd/system/ranger-can-setup.service << 'EOF'
[Unit]
Description=Configure Jetson Hardware CAN Interfaces for Ranger
After=network.target
Before=ros.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c '\
  ip link set can0 down 2>/dev/null || true && \
  ip link set can0 type can bitrate 500000 && \
  ip link set can0 up && \
  ip link set can1 down 2>/dev/null || true && \
  ip link set can1 type can bitrate 1000000 && \
  ip link set can1 up'
ExecStop=/bin/bash -c '\
  ip link set can0 down 2>/dev/null || true && \
  ip link set can1 down 2>/dev/null || true'

[Install]
WantedBy=multi-user.target
EOF
        echo -e "${GREEN}✓${NC} Created ranger-can-setup.service"
    fi

    # Reload systemd and enable service
    systemctl daemon-reload
    systemctl enable ranger-can-setup.service
    echo -e "${GREEN}✓${NC} Enabled ranger-can-setup.service"

    # Start the service now
    systemctl start ranger-can-setup.service
    echo -e "${GREEN}✓${NC} Started CAN interfaces"

else
    # Standard platform - use USB CAN adapters with udev rules
    echo "Setting up USB CAN adapters with udev rules..."
    echo ""

    # Disable and remove Jetson hardware CAN service if it exists
    if systemctl is-enabled ranger-can-setup.service &>/dev/null; then
        echo "Disabling old ranger-can-setup.service..."
        systemctl disable ranger-can-setup.service &>/dev/null || true
        systemctl stop ranger-can-setup.service &>/dev/null || true
    fi
    if [ -f /etc/systemd/system/ranger-can-setup.service ]; then
        rm -f /etc/systemd/system/ranger-can-setup.service
        echo -e "${YELLOW}⚠${NC} Removed old ranger-can-setup.service"
    fi

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

    if [ -f "$SCRIPT_DIR/can0-setup.service" ]; then
        cp "$SCRIPT_DIR/can0-setup.service" /etc/systemd/system/
        echo -e "${GREEN}✓${NC} Copied can0-setup.service to /etc/systemd/system/"
    else
        echo -e "${YELLOW}⚠${NC} File not found: $SCRIPT_DIR/can0-setup.service (skipping)"
    fi

    if [ -f "$SCRIPT_DIR/can1-setup.service" ]; then
        cp "$SCRIPT_DIR/can1-setup.service" /etc/systemd/system/
        echo -e "${GREEN}✓${NC} Copied can1-setup.service to /etc/systemd/system/"
    else
        echo -e "${YELLOW}⚠${NC} File not found: $SCRIPT_DIR/can1-setup.service (skipping)"
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
fi

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

if [ "$IS_JETSON" = true ]; then
    echo "Configuration: Jetson Hardware CAN"
    echo "  can0 -> Ranger base @ 500 kbps"
    echo "  can1 -> PiPER arm @ 1000 kbps"
    echo ""
    echo "Current CAN interface status:"
    ip link show can0 2>/dev/null || echo "  can0: Not available"
    ip link show can1 2>/dev/null || echo "  can1: Not available"
    echo ""
    echo "Next steps:"
    echo "1. Check interface status: ip link show can0 can1"
    echo "2. Monitor Ranger base: candump can0"
    echo "3. Monitor PiPER arm: candump can1"
    echo ""
    echo "Service management:"
    echo "  Check status: sudo systemctl status ranger-can-setup.service"
    echo "  Restart: sudo systemctl restart ranger-can-setup.service"
else
    echo "Configuration: USB CAN Adapters"
    echo "  can0 -> Ranger base @ 500 kbps"
    echo "  can1 -> PiPER arm @ 1000 kbps"
    echo ""
    echo "Next steps:"
    echo "1. Unplug and replug your CAN adapter(s)"
    echo "2. Check interface status: ip link show can0 can1"
    echo "3. Monitor Ranger base: candump can0"
    echo "4. Monitor PiPER arm: candump can1"
    echo ""
    echo "Service management:"
    echo "  Enable: sudo systemctl enable can0-setup.service"
    echo "  Check status: sudo systemctl status can0-setup.service"
fi
echo ""
