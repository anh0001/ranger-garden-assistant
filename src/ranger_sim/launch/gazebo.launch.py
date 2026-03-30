#!/usr/bin/env python3
"""Launch Gazebo Fortress with the specified world."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def _launch_gazebo(context, *args, **kwargs):
    """Resolve headless flag at launch time and build gz_args accordingly."""
    try:
        get_package_share_directory("gz_ros2_control")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "Missing gz_ros2_control. Install ros-humble-gz-ros2-control "
            "(or ros-humble-ign-ros2-control on Humble Ignition setups) "
            "before launching backend:=gazebo."
        ) from exc

    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    resource_paths = [
        os.path.dirname(get_package_share_directory("piper_description")),
        os.path.dirname(get_package_share_directory("ranger_description")),
    ]
    existing_gz_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_resource_path = os.pathsep.join(
        [path for path in [existing_gz_resource_path, *resource_paths] if path]
    )

    gz_args = f"-r {world}"
    if headless.lower() == "true":
        gz_args += " --headless-rendering -s"

    return [
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=gz_resource_path),
        SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=gz_resource_path),
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
