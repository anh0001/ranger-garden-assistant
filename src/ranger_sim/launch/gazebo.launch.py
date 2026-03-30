#!/usr/bin/env python3
"""Launch Gazebo Fortress with the specified world."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_gazebo(context, *args, **kwargs):
    """Resolve headless flag at launch time and build gz_args accordingly."""
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context)

    gz_args = f"-r {world}"
    if headless.lower() == "true":
        gz_args += " --headless-rendering -s"

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
                ])
            ),
            launch_arguments={"gz_args": gz_args}.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("ranger_sim"), "worlds", "garden_world.sdf"
            ]),
            description="Path to the SDF world file.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo headless (no GUI) for CI.",
        ),
        OpaqueFunction(function=_launch_gazebo),
    ])
