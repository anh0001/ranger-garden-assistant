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
            description="CAN device name (standardized to can0 for both Jetson and USB adapters)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_model",
            default_value="ranger_mini_v3",
            description="Robot model (ranger_mini_v1, ranger_mini_v2). Note: v2 works for v3.0",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_footprint",
            description="Base frame id published by the driver (connects odom to base_footprint).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_odom_tf",
            default_value="true",
            description="Publish the base driver's odom -> base_link TF to satisfy Nav2 costmaps.",
        )
    )

    # Initialize arguments
    can_device = LaunchConfiguration("can_device")
    robot_model = LaunchConfiguration("robot_model")
    base_frame = LaunchConfiguration("base_frame")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")

    # Include the ranger_ros2 launch file
    ranger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ranger_bringup"),
                "launch",
                "ranger_mini_v3.launch.py"
            ])
        ),
        launch_arguments={
            "port_name": can_device,
            "robot_model": robot_model,
            "base_frame": base_frame,
            "publish_odom_tf": publish_odom_tf,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [ranger_launch])
