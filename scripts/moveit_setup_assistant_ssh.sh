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
    bash -c \"source /opt/ros/humble/setup.bash && \
             cd /root/ws_moveit && \
             echo \\\"\\\" && \
             echo \\\"═══════════════════════════════════════════════════════════════\\\" && \
             echo \\\"  Cleaning stale build artifacts...\\\" && \
             echo \\\"═══════════════════════════════════════════════════════════════\\\" && \
             echo \\\"\\\" && \
             rm -rf build/ranger_description install/ranger_description build/piper_description install/piper_description build/piper_msgs install/piper_msgs build/piper_gazebo install/piper_gazebo && \
             echo \\\"\\\" && \
             echo \\\"═══════════════════════════════════════════════════════════════\\\" && \
             echo \\\"  Building ranger_description and piper packages...\\\" && \
             echo \\\"═══════════════════════════════════════════════════════════════\\\" && \
             echo \\\"\\\" && \
             colcon build --packages-select ranger_description piper_description piper_msgs piper_gazebo --symlink-install && \
             source install/setup.bash && \
             echo \\\"\\\" && \
             echo \\\"═══════════════════════════════════════════════════════════════\\\" && \
             echo \\\"  MoveIt Setup Assistant Ready\\\" && \
             echo \\\"═══════════════════════════════════════════════════════════════\\\" && \
             echo \\\"\\\" && \
             echo \\\"Workspace: /root/ws_moveit\\\" && \
             echo \\\"URDF: /root/ws_moveit/src/ranger_description/urdf/ranger_complete.urdf.xacro\\\" && \
             echo \\\"\\\" && \
             echo \\\"Launching MoveIt Setup Assistant...\\\" && \
             echo \\\"\\\" && \
             ros2 launch moveit_setup_assistant setup_assistant.launch.py\""
