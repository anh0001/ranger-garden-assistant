#!/usr/bin/env python3

"""Launch Livox Mid-360 LiDAR driver with configurable network settings."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "xfer_format",
            default_value="4",
            description="Transfer format: 0=PointCloud2, 1=Livox custom, 4=both (custom+PointCloud2).",
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
            default_value="livox_link",
            description="Frame ID for LiDAR point cloud",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_freq",
            default_value="10.0",
            description="Publish frequency used by livox_ros_driver2 (Hz).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "output_data_type",
            default_value="0",
            description="0 outputs to ROS topics, 1 outputs only to bag/log files.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("robofi_bringup"),
                    "config",
                    "livox_mid360_config.json",
                ]
            ),
            description="Path to the Livox MID-360 JSON configuration file.",
        )
    )

    # Initialize arguments
    xfer_format = LaunchConfiguration("xfer_format")
    multi_topic = LaunchConfiguration("multi_topic")
    frame_id = LaunchConfiguration("frame_id")
    publish_freq = LaunchConfiguration("publish_freq")
    output_data_type = LaunchConfiguration("output_data_type")
    config_file = LaunchConfiguration("config_file")

    livox_driver_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar",
        output="screen",
        parameters=[{
            "xfer_format": ParameterValue(xfer_format, value_type=int),
            "multi_topic": ParameterValue(multi_topic, value_type=int),
            "data_src": 0,  # 0 = LiDAR source
            "publish_freq": ParameterValue(publish_freq, value_type=float),
            "output_data_type": ParameterValue(output_data_type, value_type=int),
            "frame_id": frame_id,
            "user_config_path": config_file,
        }],
    )

    return LaunchDescription(declared_arguments + [livox_driver_node])
