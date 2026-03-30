#!/usr/bin/env python3
"""
Full simulation bringup: Gazebo Fortress + robot spawn + MoveIt + optional Nav2.

Usage:
  ros2 launch ranger_sim sim_bringup.launch.py
  ros2 launch ranger_sim sim_bringup.launch.py headless:=true   # CI mode
  ros2 launch ranger_sim sim_bringup.launch.py launch_nav:=true  # with Nav2
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    launch_moveit = LaunchConfiguration("launch_moveit")
    launch_nav = LaunchConfiguration("launch_nav")
    use_rviz = LaunchConfiguration("use_rviz")

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="Run Gazebo without GUI (CI mode)."),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("ranger_sim"), "worlds", "garden_world.sdf"
            ]),
        ),
        DeclareLaunchArgument("launch_moveit", default_value="true"),
        DeclareLaunchArgument("launch_nav", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
    ]

    # ---- 1. Start Gazebo Fortress ----
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ranger_sim"), "launch", "gazebo.launch.py"
            ])
        ),
        launch_arguments={
            "world": world,
            "headless": headless,
        }.items(),
    )

    # ---- 2. Spawn robot + controllers + bridge (slight delay for Gazebo init) ----
    spawn_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ranger_sim"), "launch", "spawn_robot.launch.py"
                    ])
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ],
    )

    # ---- 3. MoveIt move_group (delayed to let controllers stabilize) ----
    moveit_launch = TimerAction(
        period=8.0,
        actions=[
            GroupAction(
                condition=IfCondition(launch_moveit),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([
                                FindPackageShare("ranger_piper_moveit"),
                                "launch", "move_group.launch.py",
                            ])
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                        }.items(),
                    ),
                ],
            ),
        ],
    )

    # ---- 4. Nav2 (optional, delayed) ----
    nav2_launch = TimerAction(
        period=10.0,
        actions=[
            GroupAction(
                condition=IfCondition(launch_nav),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([
                                FindPackageShare("nav2_bringup"),
                                "launch", "navigation_launch.py",
                            ])
                        ),
                        launch_arguments={
                            "use_sim_time": "true",
                        }.items(),
                    ),
                ],
            ),
        ],
    )

    # ---- 5. RViz (optional) ----
    rviz_launch = GroupAction(
        condition=IfCondition(use_rviz),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ranger_piper_moveit"),
                        "launch", "moveit_rviz.launch.py",
                    ])
                ),
            ),
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            # Ensure use_sim_time is set globally
            SetParameter(name="use_sim_time", value=use_sim_time),
            gazebo_launch,
            spawn_launch,
            moveit_launch,
            nav2_launch,
            rviz_launch,
        ]
    )
