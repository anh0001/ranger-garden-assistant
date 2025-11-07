#!/bin/bash
# Configure static IP for Livox MID-360 LiDAR connectivity
# The MID-360 expects your PC to be on 192.168.1.x/24 subnet

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default configuration
DEFAULT_IP="192.168.1.5"
DEFAULT_NETMASK="255.255.255.0"
DEFAULT_CIDR="24"

echo -e "${GREEN}=== Livox MID-360 Network Configuration ===${NC}"
echo ""
echo "This script will configure your network interface with a static IP"
echo "to communicate with the Livox MID-360 LiDAR sensor."
echo ""

# Function to list network interfaces
list_interfaces() {
    echo -e "${YELLOW}Available network interfaces:${NC}"
    ip -o link show | awk -F': ' '{print NR") "$2}' | grep -v "lo"
}

# Function to get interface name by number
get_interface_by_number() {
    ip -o link show | awk -F': ' '{print $2}' | grep -v "lo" | sed -n "${1}p"
}

# List available interfaces
list_interfaces
echo ""

# Get interface selection
read -p "Select interface number (or press Enter to manually type name): " IFACE_NUM

if [ -z "$IFACE_NUM" ]; then
    read -p "Enter interface name (e.g., enp3s0, eth0): " INTERFACE
else
    INTERFACE=$(get_interface_by_number $IFACE_NUM)
fi

if [ -z "$INTERFACE" ]; then
    echo -e "${RED}Error: No interface specified${NC}"
    exit 1
fi

# Verify interface exists
if ! ip link show "$INTERFACE" &> /dev/null; then
    echo -e "${RED}Error: Interface '$INTERFACE' not found${NC}"
    exit 1
fi

echo -e "${GREEN}Selected interface: $INTERFACE${NC}"
echo ""

# Get IP configuration
read -p "Enter static IP [default: $DEFAULT_IP]: " STATIC_IP
STATIC_IP=${STATIC_IP:-$DEFAULT_IP}

read -p "Enter CIDR prefix [default: $DEFAULT_CIDR]: " CIDR
CIDR=${CIDR:-$DEFAULT_CIDR}

echo ""
echo -e "${YELLOW}Configuration Summary:${NC}"
echo "  Interface: $INTERFACE"
echo "  IP Address: $STATIC_IP/$CIDR"
echo "  Network: 192.168.1.0/24"
echo ""

# Choose configuration method
echo "Select configuration method:"
echo "1) NetworkManager (nmcli) - Recommended for desktop"
echo "2) Netplan - Recommended for server/headless"
echo "3) Both (create netplan config + nmcli)"
read -p "Choice [1-3]: " METHOD

case $METHOD in
    1|"")
        echo -e "${GREEN}Configuring via NetworkManager...${NC}"

        # Check if NetworkManager is running
        if ! systemctl is-active --quiet NetworkManager; then
            echo -e "${RED}NetworkManager is not running. Install it or use netplan (option 2).${NC}"
            exit 1
        fi

        # Create connection name
        CONN_NAME="Livox-MID360-Static"

        # Remove existing connection if it exists
        nmcli con delete "$CONN_NAME" 2>/dev/null || true

        # Create new static IP connection
        nmcli con add \
            type ethernet \
            con-name "$CONN_NAME" \
            ifname "$INTERFACE" \
            ipv4.method manual \
            ipv4.addresses "$STATIC_IP/$CIDR"

        # Bring up the connection
        nmcli con up "$CONN_NAME"

        echo -e "${GREEN}✓ NetworkManager configuration complete${NC}"
        echo "  Connection name: $CONN_NAME"
        echo ""
        echo "To switch back to DHCP later, run:"
        echo "  nmcli con down '$CONN_NAME' && nmcli con up Wired\\ connection\\ 1"
        ;;

    2)
        echo -e "${GREEN}Creating Netplan configuration...${NC}"

        NETPLAN_FILE="/etc/netplan/99-livox-static.yaml"

        # Create netplan config
        sudo tee "$NETPLAN_FILE" > /dev/null <<EOF
# Livox MID-360 static IP configuration
network:
  version: 2
  renderer: networkd
  ethernets:
    $INTERFACE:
      dhcp4: no
      addresses:
        - $STATIC_IP/$CIDR
      # Gateway intentionally omitted - sensor doesn't need internet routing
      # optional-addresses:
      #   - 192.168.1.1
EOF

        echo -e "${YELLOW}Netplan config created at: $NETPLAN_FILE${NC}"
        echo ""
        echo "Apply the configuration? This will restart networking."
        read -p "Continue? [y/N]: " CONFIRM

        if [[ "$CONFIRM" =~ ^[Yy]$ ]]; then
            sudo netplan apply
            echo -e "${GREEN}✓ Netplan configuration applied${NC}"
        else
            echo "Configuration saved but not applied."
            echo "To apply manually, run: sudo netplan apply"
        fi

        echo ""
        echo "To revert, delete $NETPLAN_FILE and run: sudo netplan apply"
        ;;

    3)
        echo -e "${GREEN}Creating both configurations...${NC}"

        # Netplan
        NETPLAN_FILE="/etc/netplan/99-livox-static.yaml"
        sudo tee "$NETPLAN_FILE" > /dev/null <<EOF
# Livox MID-360 static IP configuration
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    $INTERFACE:
      dhcp4: no
      addresses:
        - $STATIC_IP/$CIDR
EOF

        echo -e "${GREEN}✓ Netplan config created: $NETPLAN_FILE${NC}"

        # NetworkManager
        CONN_NAME="Livox-MID360-Static"
        nmcli con delete "$CONN_NAME" 2>/dev/null || true
        nmcli con add \
            type ethernet \
            con-name "$CONN_NAME" \
            ifname "$INTERFACE" \
            ipv4.method manual \
            ipv4.addresses "$STATIC_IP/$CIDR"

        nmcli con up "$CONN_NAME"

        echo -e "${GREEN}✓ NetworkManager connection created and activated${NC}"
        ;;

    *)
        echo -e "${RED}Invalid choice${NC}"
        exit 1
        ;;
esac

# Wait a moment for network to settle
sleep 2

# Verify configuration
echo ""
echo -e "${YELLOW}Verifying network configuration...${NC}"
ip addr show "$INTERFACE" | grep "inet "

echo ""
echo -e "${GREEN}=== Next Steps ===${NC}"
echo ""
echo "1. Find your MID-360 sensor IP:"
echo "   - Default: 192.168.1.1XX (XX = last 2 digits of S/N)"
echo "   - Example: S/N ending in 23 → IP is 192.168.1.123"
echo ""
echo "2. Test connectivity:"
echo "   ping 192.168.1.1XX"
echo ""
echo "3. Update driver config file:"
echo "   Edit: livox_ros_driver2/config/MID360_config.json"
echo "   Set host_net_info.host_ip to: $STATIC_IP"
echo "   Set lidar_configs[0].ip to: 192.168.1.1XX"
echo ""
echo "4. Test with SDK sample:"
echo "   cd ~/Livox-SDK2/samples/livox_lidar_quick_start"
echo "   ./livox_lidar_quick_start ./mid360_config.json"
echo ""
echo "5. Launch ROS2 driver:"
echo "   ros2 launch livox_ros_driver2 rviz_MID360_launch.py"
echo ""
