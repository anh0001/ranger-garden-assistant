#!/usr/bin/env python3
"""
Run the pick-and-place demo against an already-running sim.

Prereq: `ros2 launch ranger_sim sim_bringup.launch.py` is up (Gazebo +
controllers + MoveIt move_group).

Examples:
  # Safe: validate planning only, no motion
  ros2 launch ranger_sim pick_place_demo.launch.py

  # Actually run it in Gazebo (drives base + moves arm + gripper)
  ros2 launch ranger_sim pick_place_demo.launch.py execute:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("execute", default_value="false",
                              description="false = plan only (no motion); true = run in sim"),
        DeclareLaunchArgument("base_approach_x", default_value="0.52",
                              description="World x the base drives to before grasping"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]

    # Pose-follow virtual-attach grasp helper (always on; idle until told
    # to hold). Kept here rather than in sim_bringup so it stays a
    # demo-scoped component.
    magnet = Node(
        package="ranger_sim",
        executable="grasp_magnet.py",
        name="grasp_magnet",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    demo = Node(
        package="ranger_sim",
        executable="pick_place_demo.py",
        name="pick_place_demo",
        output="screen",
        parameters=[{
            "execute": LaunchConfiguration("execute"),
            "base_approach_x": LaunchConfiguration("base_approach_x"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription(args + [magnet, demo])
