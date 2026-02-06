#!/bin/bash
# Launch MoveIt Setup Assistant in Docker with SSH X11 forwarding support

set -e

# Get the workspace directory
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "═══════════════════════════════════════════════════════════════"
echo "  MoveIt Setup Assistant - Docker (SSH X11 Forwarding)"
echo "  Ranger Garden Assistant"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "Workspace: $WORKSPACE_DIR"
echo "DISPLAY: $DISPLAY"
echo ""

# Launch Docker with --net=host for SSH X11 forwarding compatibility
sg docker -c "docker run --rm -it \
    -e DISPLAY=\$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v \$HOME/.Xauthority:/root/.Xauthority:ro \
    -v $WORKSPACE_DIR:/root/ws_moveit:rw \
    --runtime=nvidia \
    --gpus all \
    --net=host \
    moveit/moveit2:humble-release \
    bash -c 'source /opt/ros/humble/setup.bash && \
             echo "" && \
             echo "═══════════════════════════════════════════════════════════════" && \
             echo "  MoveIt Setup Assistant Ready" && \
             echo "═══════════════════════════════════════════════════════════════" && \
             echo "" && \
             echo "Workspace: /root/ws_moveit" && \
             echo "URDF: /root/ws_moveit/src/ranger_description/urdf/ranger_complete.urdf.xacro" && \
             echo "" && \
             echo "Launching MoveIt Setup Assistant..." && \
             echo "" && \
             ros2 launch moveit_setup_assistant setup_assistant.launch.py'"
