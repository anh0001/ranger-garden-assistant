#!/bin/bash
#
# Build librealsense2 v2.56.4 with RSUSB backend for Jetson
# Replaces the ROS-packaged librealsense2 (V4L2 backend) which crashes on Jetson
# because the Tegra kernel lacks DKMS patches required by the V4L2 backend.
#
# The RSUSB backend uses libusb directly, bypassing kernel V4L2/HID drivers.
#
# Usage: ./scripts/build_librealsense_rsusb.sh
#

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

LIBREALSENSE_VERSION="v2.56.4"
BUILD_DIR="/tmp/librealsense-build"
INSTALL_PREFIX="/opt/ros/humble"

echo "=============================================="
echo "Building librealsense2 ${LIBREALSENSE_VERSION}"
echo "  Backend: RSUSB (libusb, no kernel patches)"
echo "  Install prefix: ${INSTALL_PREFIX}"
echo "=============================================="
echo ""

# Check dependencies
echo "Step 1: Installing build dependencies..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    git cmake build-essential \
    libusb-1.0-0-dev \
    libssl-dev \
    libcurl4-openssl-dev \
    pkg-config

echo -e "${GREEN}✓${NC} Dependencies installed"
echo ""

# Clone source
echo "Step 2: Cloning librealsense ${LIBREALSENSE_VERSION}..."
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

git clone --depth 1 --branch "${LIBREALSENSE_VERSION}" \
    https://github.com/IntelRealSense/librealsense.git
cd librealsense

echo -e "${GREEN}✓${NC} Source cloned"
echo ""

# Backup existing libraries
echo "Step 3: Backing up existing libraries..."
LIBDIR="${INSTALL_PREFIX}/lib/aarch64-linux-gnu"
if [ -f "${LIBDIR}/librealsense2.so.2.56.4" ]; then
    sudo cp "${LIBDIR}/librealsense2.so.2.56.4" "${LIBDIR}/librealsense2.so.2.56.4.v4l2-backup"
    echo -e "${GREEN}✓${NC} Backup created at ${LIBDIR}/librealsense2.so.2.56.4.v4l2-backup"
else
    echo -e "${YELLOW}⚠${NC} No existing library found to backup"
fi
echo ""

# Build with RSUSB backend
echo "Step 4: Building with FORCE_RSUSB_BACKEND=ON..."
mkdir build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_WITH_OPENMP=ON \
    -DHWM_OVER_XU=OFF \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_INSTALL_LIBDIR=lib/aarch64-linux-gnu

echo ""
NPROC=$(nproc)
echo "Building with ${NPROC} threads..."
make -j"${NPROC}"

echo -e "${GREEN}✓${NC} Build complete"
echo ""

# Install (overwrites the ROS-packaged .so files)
echo "Step 5: Installing to ${INSTALL_PREFIX}..."
sudo make install

echo -e "${GREEN}✓${NC} Installed"
echo ""

# Verify
echo "Step 6: Verifying..."
source /opt/ros/humble/setup.bash
if rs-enumerate-devices --compact 2>/dev/null; then
    echo -e "${GREEN}✓${NC} rs-enumerate-devices works!"
else
    echo -e "${YELLOW}⚠${NC} rs-enumerate-devices failed (camera may not be plugged in)"
fi
echo ""

# Cleanup
echo "Step 7: Cleaning up build directory..."
rm -rf "${BUILD_DIR}"

echo ""
echo "=============================================="
echo -e "${GREEN}Done!${NC} librealsense2 ${LIBREALSENSE_VERSION} installed with RSUSB backend."
echo ""
echo "To restore the original V4L2 backend:"
echo "  sudo cp ${LIBDIR}/librealsense2.so.2.56.4.v4l2-backup ${LIBDIR}/librealsense2.so.2.56.4"
echo ""
echo "Test with:"
echo "  source /opt/ros/humble/setup.bash"
echo "  rs-enumerate-devices"
echo "=============================================="
