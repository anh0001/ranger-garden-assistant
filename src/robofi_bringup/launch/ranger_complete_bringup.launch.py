#!/usr/bin/env python3

"""
Complete system bringup for Ranger Mini 3.0 with:
- Base controller (ranger_ros2)
- Livox Mid-360 LiDAR
- RealSense D435 camera
- PiPER 6-DOF arm
- Robot description and TF
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "can_device",
            default_value="can_base",
            description="CAN device for base controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_can_port",
            default_value="can1",
            description="CAN port for PiPER arm",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "livox_frame_id",
            default_value="lidar_link",
            description="Frame ID assigned to Livox point clouds.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "livox_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("robofi_bringup"),
                    "config",
                    "livox_mid360_config.json",
                ]
            ),
            description="Path to the Livox Mid-360 JSON configuration file.",
        )
    )

    # Initialize arguments
    can_device = LaunchConfiguration("can_device")
    arm_can_port = LaunchConfiguration("arm_can_port")
    use_sim_time = LaunchConfiguration("use_sim_time")
    livox_frame_id = LaunchConfiguration("livox_frame_id")
    livox_config_file = LaunchConfiguration("livox_config_file")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ranger_description"),
                    "urdf",
                    "ranger_complete.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Ranger base driver launch (disabled by default; uncomment when chassis is present)
    # ranger_base_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare("robofi_bringup"),
    #             "launch",
    #             "ranger_base.launch.py"
    #         ])
    #     ),
    #     launch_arguments={"can_device": can_device}.items(),
    # )

    # Livox LiDAR driver launch
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robofi_bringup"),
                "launch",
                "livox_lidar.launch.py"
            ])
        ),
        launch_arguments={
            "frame_id": livox_frame_id,
            "config_file": livox_config_file,
        }.items(),
    )

    # # RealSense camera launch
    # realsense_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare("realsense2_camera"),
    #             "launch",
    #             "rs_launch.py"
    #         ])
    #     ),
    #     launch_arguments={
    #         "align_depth.enable": "true",
    #         "pointcloud.enable": "true",
    #         "enable_color": "true",
    #         "enable_depth": "true",
    #     }.items(),
    # )

    # PiPER arm driver (commented out by default, uncomment when ready)
    # piper_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare("piper_ros"),
    #             "launch",
    #             "start_single_piper.launch.py"
    #         ])
    #     ),
    #     launch_arguments={"can_port": arm_can_port, "auto_enable": "true"}.items(),
    # )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            livox_launch,
            # static_tf_lidar,
            # ranger_base_launch,
            # realsense_launch,
            # piper_launch,
        ]
    )
