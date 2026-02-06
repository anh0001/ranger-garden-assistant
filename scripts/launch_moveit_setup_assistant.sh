#!/bin/bash
# Launch MoveIt Setup Assistant with clean environment to avoid snap library conflicts

# Get the workspace directory
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

# Clear problematic environment variables that may point to snap libraries
unset GTK_PATH
unset LD_LIBRARY_PATH

# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

# Launch MoveIt Setup Assistant
ros2 launch moveit_setup_assistant setup_assistant.launch.py
