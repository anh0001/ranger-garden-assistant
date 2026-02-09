#!/usr/bin/env python3

"""
Launch file for PiPER 6-DOF robotic arm only.
This launches the arm controller without the base or sensors.
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
            "can_port",
            default_value="can_piper",
            description="CAN port for PiPER arm (standardized to can_piper for USB gs_usb adapters)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "auto_enable",
            default_value="true",
            description="Automatically enable the PiPER arm on startup",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_exist",
            default_value="true",
            description="Whether the gripper is attached to the arm",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_val_mutiple",
            default_value="1",
            description="Gripper value multiplier for calibration",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level (debug, info, warn, error, fatal)",
        )
    )

    # Initialize arguments
    can_port = LaunchConfiguration("can_port")
    auto_enable = LaunchConfiguration("auto_enable")
    gripper_exist = LaunchConfiguration("gripper_exist")
    gripper_val_mutiple = LaunchConfiguration("gripper_val_mutiple")
    log_level = LaunchConfiguration("log_level")

    # Include the PiPER launch file
    piper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("piper"),
                "launch",
                "start_single_piper.launch.py"
            ])
        ),
        launch_arguments={
            "can_port": can_port,
            "auto_enable": auto_enable,
            "gripper_exist": gripper_exist,
            "gripper_val_mutiple": gripper_val_mutiple,
            "log_level": log_level,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [piper_launch])
