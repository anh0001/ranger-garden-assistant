#!/bin/bash
#
# PiPER Robotic Arm Setup Script
# Sets up dependencies and environment for AgileX PiPER 6-DOF arm
#
# Usage: ./setup_piper.sh [--install-deps] [--check-can] [--test]
#

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "    PiPER Robotic Arm Setup Script"
echo "=========================================="
echo ""

# Parse command line arguments
INSTALL_DEPS=false
CHECK_CAN=false
TEST_ARM=false

for arg in "$@"; do
    case $arg in
        --install-deps)
            INSTALL_DEPS=true
            shift
            ;;
        --check-can)
            CHECK_CAN=true
            shift
            ;;
        --test)
            TEST_ARM=true
            shift
            ;;
        --help)
            echo "Usage: ./setup_piper.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --install-deps    Install Python dependencies (python-can, scipy, piper_sdk)"
            echo "  --check-can       Check CAN interface status and verify connection"
            echo "  --test           Launch PiPER test node to verify arm functionality"
            echo "  --help           Show this help message"
            echo ""
            echo "Examples:"
            echo "  ./setup_piper.sh --install-deps              # Install dependencies only"
            echo "  ./setup_piper.sh --check-can                 # Check CAN interface"
            echo "  ./setup_piper.sh --install-deps --check-can  # Full setup check"
            echo "  ./setup_piper.sh --test                      # Test arm connection"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $arg${NC}"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# If no arguments provided, show interactive menu
if [ "$INSTALL_DEPS" = false ] && [ "$CHECK_CAN" = false ] && [ "$TEST_ARM" = false ]; then
    echo "No options specified. What would you like to do?"
    echo ""
    echo "1) Install Python dependencies"
    echo "2) Check CAN interface status"
    echo "3) Test PiPER arm connection"
    echo "4) Full setup (all of the above)"
    echo "5) Exit"
    echo ""
    read -p "Select option [1-5]: " choice

    case $choice in
        1)
            INSTALL_DEPS=true
            ;;
        2)
            CHECK_CAN=true
            ;;
        3)
            TEST_ARM=true
            ;;
        4)
            INSTALL_DEPS=true
            CHECK_CAN=true
            TEST_ARM=true
            ;;
        5)
            echo "Exiting."
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid option${NC}"
            exit 1
            ;;
    esac
fi

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check Python package
check_python_package() {
    python3 -c "import $1" 2>/dev/null
    return $?
}

# Install Python dependencies
if [ "$INSTALL_DEPS" = true ]; then
    echo -e "${BLUE}=== Installing Python Dependencies ===${NC}"
    echo ""

    # Check Python version
    echo "Checking Python version..."
    PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
    PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d'.' -f1)
    PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d'.' -f2)

    if [ "$PYTHON_MAJOR" -lt 3 ] || ([ "$PYTHON_MAJOR" -eq 3 ] && [ "$PYTHON_MINOR" -lt 10 ]); then
        echo -e "${RED}Error: Python 3.10 or higher is required${NC}"
        echo "Current version: $PYTHON_VERSION"
        exit 1
    else
        echo -e "${GREEN}Python version OK: $PYTHON_VERSION${NC}"
    fi
    echo ""

    # Install python-can (>=4.3.1)
    echo "Installing python-can (>=4.3.1)..."
    if check_python_package "can"; then
        CAN_VERSION=$(python3 -c "import can; print(can.__version__)" 2>/dev/null || echo "unknown")
        echo -e "${YELLOW}python-can already installed (version: $CAN_VERSION)${NC}"
        echo "Upgrading to latest version..."
        pip3 install --upgrade python-can
    else
        pip3 install python-can
    fi
    echo -e "${GREEN}python-can installed${NC}"
    echo ""

    # Install scipy
    echo "Installing scipy..."
    if check_python_package "scipy"; then
        echo -e "${YELLOW}scipy already installed${NC}"
    else
        pip3 install scipy
    fi
    echo -e "${GREEN}scipy installed${NC}"
    echo ""

    # Install piper_sdk
    echo "Installing piper_sdk..."
    if check_python_package "piper_sdk"; then
        echo -e "${YELLOW}piper_sdk already installed${NC}"
        echo "Upgrading to latest version..."
        pip3 install --upgrade piper_sdk
    else
        pip3 install piper_sdk
    fi
    echo -e "${GREEN}piper_sdk installed${NC}"
    echo ""

    # Install ROS 2 control packages
    echo "Checking ROS 2 control packages..."
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${RED}Warning: ROS_DISTRO not set. Assuming humble.${NC}"
        ROS_DISTRO="humble"
    fi

    echo "Installing ros-${ROS_DISTRO}-ros2-control packages..."
    sudo apt update
    sudo apt install -y \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-controller-manager

    echo -e "${GREEN}ROS 2 control packages installed${NC}"
    echo ""

    echo -e "${GREEN}=== Python Dependencies Installation Complete ===${NC}"
    echo ""
fi

# Check CAN interface
if [ "$CHECK_CAN" = true ]; then
    echo -e "${BLUE}=== Checking CAN Interface ===${NC}"
    echo ""

    # Check if CAN utils are installed
    if ! command_exists ip; then
        echo -e "${RED}Error: 'ip' command not found${NC}"
        echo "Install with: sudo apt-get install iproute2"
        exit 1
    fi

    if ! command_exists candump; then
        echo -e "${RED}Error: 'candump' not found${NC}"
        echo "Install with: sudo apt install can-utils"
        exit 1
    fi

    echo -e "${GREEN}CAN utilities installed${NC}"
    echo ""

    # Check for can_piper interface
    echo "Checking for can_piper interface..."
    if ip link show can_piper &>/dev/null; then
        echo -e "${GREEN}can_piper interface found${NC}"

        # Check interface status
        CAN_STATE=$(ip -details link show can_piper | grep -oP 'state \K\w+')
        echo "Interface state: $CAN_STATE"

        if [ "$CAN_STATE" != "UP" ]; then
            echo -e "${YELLOW}Warning: can_piper is not UP${NC}"
            echo "The CAN interface may need to be configured."
            echo "Run: sudo ./scripts/install_can_udev.sh (if not already done)"
            echo "Then unplug and replug the CAN adapter."
        else
            echo -e "${GREEN}can_piper is UP${NC}"

            # Check bitrate
            BITRATE=$(ip -details link show can_piper | grep -oP 'bitrate \K\d+')
            echo "Bitrate: $BITRATE bps"

            if [ "$BITRATE" != "1000000" ]; then
                echo -e "${YELLOW}Warning: Expected bitrate 1000000, got $BITRATE${NC}"
            else
                echo -e "${GREEN}Bitrate correct (1000 kbps)${NC}"
            fi
        fi
        echo ""

        # Test for CAN messages
        echo "Checking for CAN messages (3 second test)..."
        echo -e "${CYAN}If the arm is powered on, you should see CAN messages below:${NC}"
        timeout 3 candump can_piper 2>/dev/null || true
        echo ""
        echo -e "${CYAN}Tip: If no messages appear, ensure the PiPER arm is powered on${NC}"

    else
        echo -e "${YELLOW}can_piper interface not found${NC}"
        echo ""
        echo "The CAN interface may not be configured yet."
        echo ""
        echo "To set up CAN interfaces:"
        echo "  1. Run: sudo ./scripts/install_can_udev.sh"
        echo "  2. Unplug and replug your CAN adapters"
        echo "  3. Verify with: ip link show can_piper"
        echo ""
        echo "Note: The PiPER arm uses CAN bus @ 1000 kbps on interface 'can_piper'"
    fi
    echo ""

    echo -e "${GREEN}=== CAN Interface Check Complete ===${NC}"
    echo ""
fi

# Test PiPER arm
if [ "$TEST_ARM" = true ]; then
    echo -e "${BLUE}=== Testing PiPER Arm Connection ===${NC}"
    echo ""

    # Check if workspace is built
    if [ ! -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
        echo -e "${YELLOW}Workspace not built. Building now...${NC}"
        cd "$WORKSPACE_ROOT"
        source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
        colcon build --packages-select piper piper_msgs --symlink-install
        echo ""
    fi

    # Source workspace
    echo "Sourcing workspace..."
    source "$WORKSPACE_ROOT/install/setup.bash"
    echo ""

    # Check if can_piper is available
    if ! ip link show can_piper &>/dev/null; then
        echo -e "${RED}Error: can_piper interface not found${NC}"
        echo "Please run with --check-can first to set up CAN interface"
        exit 1
    fi

    # Launch PiPER node
    echo -e "${CYAN}Launching PiPER test node...${NC}"
    echo "This will start the PiPER controller on can_piper interface."
    echo ""
    echo -e "${YELLOW}Press Ctrl+C to stop the test${NC}"
    echo ""

    ros2 launch piper start_single_piper.launch.py \
        can_port:=can_piper \
        auto_enable:=true \
        gripper_exist:=false \
        rviz_ctrl_flag:=false

    echo ""
    echo -e "${GREEN}=== PiPER Test Complete ===${NC}"
    echo ""
fi

# Summary
echo -e "${GREEN}=========================================="
echo "         Setup Complete!"
echo "==========================================${NC}"
echo ""
echo "Next steps:"
echo ""
echo "1. Launch PiPER arm only:"
echo "   ${CYAN}ros2 launch piper start_single_piper.launch.py can_port:=can_piper${NC}"
echo ""
echo "2. Launch PiPER with MoveIt for motion planning:"
echo "   ${CYAN}ros2 launch piper_moveit piper_moveit.launch.py${NC}"
echo ""
echo "3. Launch complete system (base + sensors + arm):"
echo "   ${CYAN}ros2 launch robofi_bringup ranger_complete_bringup.launch.py${NC}"
echo ""
echo "4. Monitor arm joint states:"
echo "   ${CYAN}ros2 topic echo /joint_states${NC}"
echo ""
echo "Documentation:"
echo "  - PiPER README: src/piper_ros/README.MD"
echo "  - MoveIt guide: src/piper_ros/src/piper_moveit/README.md"
echo "  - Project docs: CLAUDE.md"
echo ""
