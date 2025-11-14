#!/usr/bin/env python3

"""
Wrapper launch for FAST_LIO mapping using Ranger Garden Assistant defaults.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="fast_lio.yaml",
            description="FAST_LIO configuration file located in robofi_bringup/config",
        ),
        DeclareLaunchArgument(
            "config_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robofi_bringup"), "config"]
            ),
            description="Directory containing FAST_LIO configuration files",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz for visualization",
        ),
        DeclareLaunchArgument(
            "rviz_cfg",
            default_value=PathJoinSubstitution(
                [FindPackageShare("fast_lio"), "rviz", "fastlio.rviz"]
            ),
            description="RViz configuration file",
        ),
        DeclareLaunchArgument(
            "body_frame_id",
            default_value="body",
            description="FAST_LIO body frame (parent frame in the transform)",
        ),
        DeclareLaunchArgument(
            "base_frame_id",
            default_value="base_footprint",
            description="Robot frame to connect to (child frame, typically base_footprint)",
        ),
        DeclareLaunchArgument(
            "body_to_base_x",
            default_value="0.0",
            description="Static transform translation X from body to base frame (m)",
        ),
        DeclareLaunchArgument(
            "body_to_base_y",
            default_value="0.0",
            description="Static transform translation Y from body to base frame (m)",
        ),
        DeclareLaunchArgument(
            "body_to_base_z",
            default_value="0.0",
            description="Static transform translation Z from body to base frame (m)",
        ),
        DeclareLaunchArgument(
            "body_to_base_roll",
            default_value="0.0",
            description="Static transform roll from body to base frame (rad)",
        ),
        DeclareLaunchArgument(
            "body_to_base_pitch",
            default_value="0.0",
            description="Static transform pitch from body to base frame (rad)",
        ),
        DeclareLaunchArgument(
            "body_to_base_yaw",
            default_value="0.0",
            description="Static transform yaw from body to base frame (rad)",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")
    config_path = LaunchConfiguration("config_path")
    rviz = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    body_frame_id = LaunchConfiguration("body_frame_id")
    base_frame_id = LaunchConfiguration("base_frame_id")
    body_to_base_x = LaunchConfiguration("body_to_base_x")
    body_to_base_y = LaunchConfiguration("body_to_base_y")
    body_to_base_z = LaunchConfiguration("body_to_base_z")
    body_to_base_roll = LaunchConfiguration("body_to_base_roll")
    body_to_base_pitch = LaunchConfiguration("body_to_base_pitch")
    body_to_base_yaw = LaunchConfiguration("body_to_base_yaw")

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("fast_lio"), "launch", "mapping.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "config_file": config_file,
            "config_path": config_path,
            "rviz": rviz,
            "rviz_cfg": rviz_cfg,
        }.items(),
    )

    body_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fast_lio_body_to_base_tf",
        arguments=[
            body_to_base_x,
            body_to_base_y,
            body_to_base_z,
            body_to_base_roll,
            body_to_base_pitch,
            body_to_base_yaw,
            body_frame_id,
            base_frame_id,
        ],
    )

    return LaunchDescription(declared_arguments + [fast_lio_launch, body_to_base_tf])
