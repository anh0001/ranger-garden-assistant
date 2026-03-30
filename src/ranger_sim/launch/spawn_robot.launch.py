#!/usr/bin/env python3
"""
Spawn the Ranger + PiPER robot into a running Gazebo Fortress instance,
start ros2_control controllers, and launch the ros_gz_bridge.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
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

    # Spawn ros2_control controllers sequentially
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"],
        output="screen",
    )

    load_arm_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "piper_arm_controller"],
        output="screen",
    )

    load_gripper_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "piper_gripper_controller"],
        output="screen",
    )

    load_gripper8_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "piper_gripper8_controller"],
        output="screen",
    )

    # Chain controller loading: JSB -> arm -> gripper -> gripper8
    chain_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller],
        )
    )
    chain_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=load_arm_controller,
            on_exit=[load_gripper_controller],
        )
    )
    chain_gripper8 = RegisterEventHandler(
        OnProcessExit(
            target_action=load_gripper_controller,
            on_exit=[load_gripper8_controller],
        )
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            spawn_robot,
            gz_bridge,
            load_joint_state_broadcaster,
            chain_arm,
            chain_gripper,
            chain_gripper8,
        ]
    )
