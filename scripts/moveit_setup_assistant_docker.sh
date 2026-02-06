#!/bin/bash
# Launch MoveIt Setup Assistant in Docker with access to ranger_description package
# This solves the RViz/Qt crash issue on Jetson/ARM platforms

set -e

# Get the workspace directory
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  MoveIt Setup Assistant - Docker Launcher${NC}"
echo -e "${GREEN}  Ranger Garden Assistant${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Error: Docker is not installed${NC}"
    echo "Please install Docker first:"
    echo "  https://docs.docker.com/engine/install/ubuntu/"
    exit 1
fi

# Check if gui-docker script exists
if [ ! -f "$SCRIPT_DIR/gui-docker" ]; then
    echo -e "${RED}Error: gui-docker script not found${NC}"
    echo "Expected at: $SCRIPT_DIR/gui-docker"
    exit 1
fi

# Check if ranger_description exists
if [ ! -d "$WORKSPACE_DIR/src/ranger_description" ]; then
    echo -e "${RED}Error: ranger_description package not found${NC}"
    echo "Expected at: $WORKSPACE_DIR/src/ranger_description"
    exit 1
fi

echo -e "${YELLOW}Workspace directory:${NC} $WORKSPACE_DIR"
echo -e "${YELLOW}Using Docker image:${NC} moveit/moveit2:humble-release"
echo ""

# Information message
echo -e "${GREEN}This will:${NC}"
echo "  1. Mount your workspace into the container at /root/ws_moveit"
echo "  2. Launch MoveIt Setup Assistant with GUI support"
echo "  3. Allow you to load URDF from: /root/ws_moveit/src/ranger_description/urdf/"
echo ""
echo -e "${YELLOW}Inside the container:${NC}"
echo "  - Your workspace: /root/ws_moveit"
echo "  - URDF location: /root/ws_moveit/src/ranger_description/urdf/ranger_complete.urdf.xacro"
echo ""
echo -e "${YELLOW}To launch Setup Assistant inside container:${NC}"
echo "  ros2 launch moveit_setup_assistant setup_assistant.launch.py"
echo ""
echo -e "${GREEN}Starting Docker container...${NC}"
echo ""

# Remove existing container if it exists (to avoid mount conflicts)
CONTAINER_NAME="ranger_moveit_setup"
if [ -n "$(docker ps -aq --filter name=^${CONTAINER_NAME}\$)" ]; then
    echo -e "${YELLOW}Removing existing container: ${CONTAINER_NAME}${NC}"
    docker rm -f ${CONTAINER_NAME} > /dev/null 2>&1
fi

# Create a startup script inside the container
STARTUP_SCRIPT=$(cat <<'EOF'
#!/bin/bash
# Source ROS setup if it exists
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/humble/local_setup.bash ]; then
    source /opt/ros/humble/local_setup.bash
fi

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "  MoveIt Setup Assistant Ready"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "Your workspace is mounted at: /root/ws_moveit"
echo ""
echo "To launch MoveIt Setup Assistant:"
echo "  ros2 launch moveit_setup_assistant setup_assistant.launch.py"
echo ""
echo "To load your robot URDF:"
echo "  Browse to: /root/ws_moveit/src/ranger_description/urdf/ranger_complete.urdf.xacro"
echo ""
echo "To save the generated MoveIt config:"
echo "  Save to: /root/ws_moveit/src/ranger_piper_moveit"
echo "  (Will be accessible from host at: ~/codes/ranger-garden-assistant/src/ranger_piper_moveit)"
echo ""
exec bash
EOF
)

# Write startup script to temp file
TEMP_STARTUP="/tmp/moveit_startup_$$.sh"
echo "$STARTUP_SCRIPT" > "$TEMP_STARTUP"
chmod +x "$TEMP_STARTUP"

# Detect GPU and set appropriate passthrough options
GPU_ARGS=""

# Check for NVIDIA GPU
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    echo -e "${GREEN}NVIDIA GPU detected - enabling GPU passthrough${NC}"
    GPU_ARGS="--gpus all"
# Check for Intel/AMD GPU (DRI devices)
elif [ -d "/dev/dri" ] && [ -n "$(ls -A /dev/dri 2>/dev/null)" ]; then
    echo -e "${GREEN}Intel/AMD GPU detected - enabling DRI device passthrough${NC}"
    GPU_ARGS="--device=/dev/dri"
else
    echo -e "${YELLOW}No GPU detected - running without GPU acceleration${NC}"
fi

# Launch Docker container with workspace mounted
# Container name: ranger_moveit_setup
# The gui-docker script automatically sets:
#   - DISPLAY, QT_X11_NO_MITSHM=1, XAUTHORITY
#   - X11 socket mounting and xauth configuration
#   - NVIDIA GPU support detection (if available)
"$SCRIPT_DIR/gui-docker" \
    -c ranger_moveit_setup \
    -v "$WORKSPACE_DIR:/root/ws_moveit:rw" \
    -v "$TEMP_STARTUP:/tmp/startup.sh:ro" \
    $GPU_ARGS \
    -it \
    moveit/moveit2:humble-release \
    /tmp/startup.sh

# Cleanup
rm -f "$TEMP_STARTUP"
