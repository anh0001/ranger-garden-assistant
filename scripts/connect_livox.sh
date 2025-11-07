#!/bin/bash
# Quick NetworkManager setup for Livox MID-360
# Usage: ./connect_livox.sh [connect|disconnect|status]

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

CONN_NAME="Livox-MID360-Static"
INTERFACE="enp2s0"
STATIC_IP="192.168.1.5"
CIDR="24"
LIDAR_IP="192.168.1.169"

show_status() {
    echo -e "${YELLOW}Network Status:${NC}"
    nmcli device status
    echo ""
    echo -e "${YELLOW}Livox Connection:${NC}"
    nmcli connection show "$CONN_NAME" 2>/dev/null || echo "Not configured"
}

connect_livox() {
    echo -e "${GREEN}Connecting to Livox LiDAR...${NC}"
    
    # Check if connection exists
    if ! nmcli connection show "$CONN_NAME" &>/dev/null; then
        echo "Creating NetworkManager connection profile..."
        sudo nmcli connection add \
            type ethernet \
            con-name "$CONN_NAME" \
            ifname "$INTERFACE" \
            ipv4.method manual \
            ipv4.addresses "$STATIC_IP/$CIDR" \
            ipv4.never-default yes \
            autoconnect no
    fi
    
    # Activate connection
    echo "Activating connection on $INTERFACE..."
    sudo nmcli connection up "$CONN_NAME" ifname "$INTERFACE"
    
    sleep 1
    
    # Verify
    echo ""
    echo -e "${GREEN}✓ Connected${NC}"
    ip addr show "$INTERFACE" | grep "inet " || true
    
    echo ""
    echo "Testing LiDAR connectivity..."
    if ping -c 2 -W 1 "$LIDAR_IP" &>/dev/null; then
        echo -e "${GREEN}✓ LiDAR reachable at $LIDAR_IP${NC}"
    else
        echo -e "${YELLOW}⚠ Cannot reach LiDAR at $LIDAR_IP${NC}"
        echo "  - Check LiDAR is powered on"
        echo "  - Check ethernet cable is connected"
        echo "  - Verify LiDAR IP matches (check last 2 digits of S/N)"
    fi
}

disconnect_livox() {
    echo -e "${YELLOW}Disconnecting from Livox...${NC}"
    sudo nmcli connection down "$CONN_NAME" 2>/dev/null || echo "Already disconnected"
    echo -e "${GREEN}✓ Disconnected${NC}"
}

remove_config() {
    echo -e "${YELLOW}Removing Livox connection profile...${NC}"
    sudo nmcli connection delete "$CONN_NAME" 2>/dev/null || echo "Already removed"
    echo -e "${GREEN}✓ Profile removed${NC}"
}

case "${1:-connect}" in
    connect|c|up)
        connect_livox
        ;;
    disconnect|d|down)
        disconnect_livox
        ;;
    status|s)
        show_status
        ;;
    remove|rm)
        remove_config
        ;;
    *)
        echo "Usage: $0 [connect|disconnect|status|remove]"
        echo ""
        echo "Commands:"
        echo "  connect     - Configure and connect to Livox LiDAR (default)"
        echo "  disconnect  - Disconnect from Livox"
        echo "  status      - Show connection status"
        echo "  remove      - Remove connection profile"
        exit 1
        ;;
esac
