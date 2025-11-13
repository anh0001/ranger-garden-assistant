#!/usr/bin/env python3

"""
Wrapper launch for FAST_LIO mapping using Ranger Garden Assistant defaults.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")
    config_path = LaunchConfiguration("config_path")
    rviz = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")

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

    return LaunchDescription(declared_arguments + [fast_lio_launch])
