#!/usr/bin/env python3

"""
Launch file for Nav2 navigation stack
"""

import os
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
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ranger_bringup"),
                "config",
                "nav2_params.yaml"
            ]),
            description="Full path to the ROS2 parameters file for navigation",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Full path to map yaml file to load (leave empty for SLAM)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the navigation stack",
        )
    )

    # Initialize arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml_file = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")

    # Navigation launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
        }.items(),
    )

    # Localization launch (AMCL) if map is provided
    # If map is empty, you should run SLAM separately
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "localization_launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "map": map_yaml_file,
            "autostart": autostart,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [nav2_launch, localization_launch])
