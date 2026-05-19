#!/usr/bin/env python3
"""
Spawn the Ranger + PiPER robot into a running Gazebo Fortress instance,
start ros2_control controllers, and launch the ros_gz_bridge.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.33"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
    ]

    # Build robot description from the Gazebo-specific xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ranger_sim"), "urdf", "ranger_gazebo.urdf.xacro"
        ]),
        " mesh_dir:=file://",
        PathJoinSubstitution([FindPackageShare("ranger_description"), "meshes"]),
        " piper_mesh_dir:=file://",
        PathJoinSubstitution([FindPackageShare("piper_description"), "meshes"]),
        " sim_gazebo_classic:=false",
        " gz_controller_config:=",
        PathJoinSubstitution([
            FindPackageShare("ranger_sim"), "config", "ros2_controllers_sim.yaml"
        ]),
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Spawn entity into Gazebo Fortress
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "ranger",
            "-topic", "robot_description",
            "-x", x, "-y", y, "-z", z,
            "-Y", yaw,
        ],
        output="screen",
    )

    # Static joint states for the Ranger base wheel/steering joints.
    # gz_ros2_control only publishes the PiPER arm joints; without these
    # MoveIt's planning_scene_monitor never gets a complete robot state
    # and refuses to plan for piper_arm.
    base_joint_state_publisher = Node(
        package="ranger_sim",
        executable="base_joint_state_publisher.py",
        name="base_joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ros_gz_bridge for topic bridging
    bridge_config = PathJoinSubstitution([
        FindPackageShare("ranger_sim"), "config", "gz_bridge.yaml"
    ])
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", ["config_file:=", bridge_config]],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ---- ros2_control controller spawners ----
    # gz_ros2_control loads the controller_manager (params from
    # ros2_controllers_sim.yaml via the URDF). Controllers still need
    # spawning; chain them off the entity-create exit so the
    # controller_manager exists first, and bring the broadcaster up
    # before the trajectory controllers.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["piper_arm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["piper_gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    gripper8_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["piper_gripper8_controller", "-c", "/controller_manager"],
        output="screen",
    )

    spawn_jsb_after_create = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    spawn_controllers_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                arm_controller_spawner,
                gripper_controller_spawner,
                gripper8_controller_spawner,
            ],
        )
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            spawn_robot,
            base_joint_state_publisher,
            gz_bridge,
            spawn_jsb_after_create,
            spawn_controllers_after_jsb,
        ]
    )
