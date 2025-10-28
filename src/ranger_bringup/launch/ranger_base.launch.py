#!/usr/bin/env python3

"""
Launch file for Ranger Mini 3.0 base controller
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "can_device",
            default_value="can0",
            description="CAN device name (e.g., can0, can1)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_model",
            default_value="ranger_mini_v2",
            description="Robot model (ranger_mini_v1, ranger_mini_v2). Note: v2 works for v3.0",
        )
    )

    # Initialize arguments
    can_device = LaunchConfiguration("can_device")
    robot_model = LaunchConfiguration("robot_model")

    # Include the ranger_ros2 launch file
    ranger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ranger_bringup"),
                "launch",
                "ranger_mini_v2.launch.py"
            ])
        ),
        launch_arguments={
            "can_device": can_device,
            "robot_model": robot_model,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [ranger_launch])
