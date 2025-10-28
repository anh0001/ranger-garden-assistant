#!/bin/bash

# Script to build the complete Ranger workspace
# This script handles dependencies and builds all packages

set -e

echo "======================================"
echo "Building Ranger Garden Assistant Workspace"
echo "======================================"

# Check if we're in the right directory
if [ ! -d "src" ]; then
    echo "Error: Please run this script from the workspace root directory"
    exit 1
fi

# Source ROS 2 Humble
echo "Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

# Initialize submodules
echo "Initializing git submodules..."
git submodule update --init --recursive

# Install dependencies
echo "Installing ROS dependencies..."
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build Livox SDK2 (required by livox_ros_driver2)
echo "Building Livox SDK2..."
if [ -d "src/livox_ros_driver2/Livox-SDK2" ]; then
    cd src/livox_ros_driver2/Livox-SDK2
    mkdir -p build && cd build
    cmake .. && make -j$(nproc)
    sudo make install
    cd ../../../..
fi

# Build the workspace
echo "Building workspace with colcon..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "======================================"
echo "Build complete!"
echo "======================================"
echo ""
echo "To use this workspace, run:"
echo "  source install/setup.bash"
echo ""
