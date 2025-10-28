#!/usr/bin/env python3

"""
Launch file for Livox Mid-360 LiDAR
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "xfer_format",
            default_value="2",
            description="Transfer format (0: Livox custom, 1: Livox custom, 2: PointCloud2)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "multi_topic",
            default_value="0",
            description="Multi-topic mode (0: single topic, 1: multiple topics)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "frame_id",
            default_value="livox_frame",
            description="Frame ID for LiDAR point cloud",
        )
    )

    # Initialize arguments
    xfer_format = LaunchConfiguration("xfer_format")
    multi_topic = LaunchConfiguration("multi_topic")
    frame_id = LaunchConfiguration("frame_id")

    # Get config file path
    config_file = PathJoinSubstitution([
        FindPackageShare("livox_ros_driver2"),
        "config",
        "MID360_config.json"
    ])

    # Livox driver node
    livox_driver_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar",
        output="screen",
        parameters=[{
            "xfer_format": xfer_format,
            "multi_topic": multi_topic,
            "frame_id": frame_id,
            "user_config_path": config_file,
            "publish_freq": 10.0,
            "output_data_type": 2,  # PointCloud2
        }],
    )

    return LaunchDescription(declared_arguments + [livox_driver_node])
