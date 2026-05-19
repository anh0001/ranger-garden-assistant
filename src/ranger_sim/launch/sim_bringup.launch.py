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
from launch_ros.actions import Node, SetParameter
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
                    # SIM-ONLY: disable the Nav2 lifecycle bond watchdog.
                    # Stock navigation_launch.py constructs
                    # lifecycle_manager_navigation with an inline 3-key param
                    # dict and does NOT pass it the params_file, so
                    # bond_timeout in nav2_sim.yaml is ignored (stays 4.0s).
                    # A scoped SetParameter injects it into every node in
                    # this GroupAction, including the lifecycle manager
                    # (which sets no bond_timeout of its own, so this lands).
                    # On a loaded box / with intermittent Gazebo TF the
                    # controller_server executor stalls >4s and the watchdog
                    # kills the whole stack on a false positive. This is a
                    # SIM workaround only: real hardware uses a different
                    # params file + FASTLIO2 and MUST keep the watchdog live
                    # (a wedged server there is a real fault). The proper
                    # sim correctness fix is making odom->base_footprint
                    # continuous; tracked separately.
                    SetParameter(name="bond_timeout", value=0.0),
                    SetParameter(name="attempt_respawn_reconnection",
                                 value=True),
                    # Sim localization: Gazebo's DiffDrive plugin already
                    # publishes a drift-free odom->base_footprint, so the
                    # real-stack FASTLIO2 map->odom (which needs Livox 3D
                    # points, not bridged here) is replaced by a static
                    # identity map->odom. This is the only piece Nav2 needs
                    # to have a usable `map` frame in simulation.
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="sim_map_to_odom",
                        arguments=["0", "0", "0", "0", "0", "0",
                                   "map", "odom"],
                        parameters=[{"use_sim_time": use_sim_time}],
                        output="screen",
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([
                                FindPackageShare("nav2_bringup"),
                                "launch", "navigation_launch.py",
                            ])
                        ),
                        launch_arguments={
                            "use_sim_time": "true",
                            # Exercise the repo's tuned Nav2 config (RotationShimController +
                            # MPPI) in sim, not the nav2_bringup defaults.
                            # Sim-specific Nav2 params: same controller/
                            # planner tuning as robofi_bringup/config/
                            # nav2_params.yaml, but costmaps fed by the
                            # bridged /scan and odom from Gazebo /odom (the
                            # real file uses STVL + FASTLIO2 cloud + octomap,
                            # none of which exist in sim).
                            "params_file": PathJoinSubstitution([
                                FindPackageShare("ranger_sim"),
                                "config", "nav2_sim.yaml",
                            ]),
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
