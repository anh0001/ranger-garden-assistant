#!/bin/bash

# Script to setup CAN bus interfaces for Ranger Mini 3.0 and PiPER arm
# Run with sudo: sudo ./setup_can.sh

set -e

echo "======================================"
echo "Setting up CAN bus interfaces"
echo "======================================"

# CAN0 for Ranger base (bitrate 500000)
if ip link show can0 &> /dev/null; then
    echo "Setting up can0 (Ranger base)..."
    sudo ip link set can0 down 2>/dev/null || true
    sudo ip link set can0 type can bitrate 500000
    sudo ip link set can0 up
    echo "✓ can0 is up at 500 kbps"
else
    echo "⚠ can0 not found. Please check CAN adapter connection."
fi

# CAN1 for PiPER arm (bitrate 1000000)
if ip link show can1 &> /dev/null; then
    echo "Setting up can1 (PiPER arm)..."
    sudo ip link set can1 down 2>/dev/null || true
    sudo ip link set can1 type can bitrate 1000000
    sudo ip link set can1 up
    echo "✓ can1 is up at 1000 kbps"
else
    echo "⚠ can1 not found. If you have PiPER arm, please check CAN adapter connection."
fi

echo ""
echo "CAN interfaces status:"
ip -details -statistics link show can0 2>/dev/null || echo "can0: not available"
ip -details -statistics link show can1 2>/dev/null || echo "can1: not available"

echo ""
echo "======================================"
echo "CAN setup complete!"
echo "======================================"
