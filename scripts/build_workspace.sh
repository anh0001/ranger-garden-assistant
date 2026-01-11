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
rosdep install --from-paths src --ignore-src -r -y || echo "Warning: Some rosdep dependencies could not be installed (this may be OK)"

# Build Sophus library (required by FASTLIO2_ROS2)
echo "Checking for Sophus library..."
if ! pkg-config --exists sophus 2>/dev/null; then
    echo "Sophus not found, building from source..."
    cd /tmp
    if [ ! -d "Sophus" ]; then
        git clone https://github.com/strasdat/Sophus.git
    fi
    cd Sophus
    git checkout 1.22.10
    rm -rf build && mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
    make -j$(nproc)
    sudo make install
    cd -
    echo "Sophus installed successfully"
else
    echo "Sophus already installed"
fi

# Build Livox SDK2 (required by livox_ros_driver2)
echo "Building Livox SDK2..."
if [ -d "src/Livox-SDK2" ]; then
    cd src/Livox-SDK2
    mkdir -p build && cd build
    cmake .. && make -j$(nproc)
    sudo make install
    cd ../../..
    echo "Livox SDK2 installed successfully"
else
    echo "Warning: Livox-SDK2 submodule not found at src/Livox-SDK2"
fi

# Build the workspace
echo "Building workspace with colcon..."
# Disable desktop_notification to avoid timeout on SSH/remote sessions
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+ desktop_notification-

BUILD_STATUS=$?
echo ""
if [ $BUILD_STATUS -eq 0 ]; then
    echo "======================================"
    echo "Build complete!"
    echo "======================================"
else
    echo "======================================"
    echo "Build FAILED with exit code $BUILD_STATUS"
    echo "======================================"
    exit $BUILD_STATUS
fi
echo ""
echo "To use this workspace, run:"
echo "  source install/setup.bash"
echo ""
