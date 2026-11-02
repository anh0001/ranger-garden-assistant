#!/usr/bin/env python3
"""
Spawn the Ranger + PiPER robot into a running Gazebo Fortress instance.

Also starts the ros2_control controllers, the ros_gz bridges and the grasp
manager. Gazebo is expected to be PAUSED (gazebo.launch.py default):
grasp_manager detaches every DetachableJoint grasp joint created at spawn,
then unpauses the world; controllers finish activating after that.
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
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

# The world starts paused and the controller_manager only updates while sim
# time advances, so controller activation waits for grasp_manager's unpause
# (bridge wait + robot discovery + startup detach). The spawner default
# (5 s switch / 10 s service call) is far too short; the switch timeout is
# derived from grasp_detach_timeout so raising one cannot break the other.
SPAWNER_MARGIN_SEC = 25.0  # bridge wait (<= 10 s) + discovery + unpause


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_name = LaunchConfiguration("world_name")
    grasp_detach_timeout = LaunchConfiguration("grasp_detach_timeout")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "world_name",
            default_value="garden_world",
            description="<world name> inside the loaded SDF (for /world/<name>/... services).",
        ),
        DeclareLaunchArgument(
            "grasp_detach_timeout",
            default_value="20.0",
            description="Seconds grasp_manager waits for the startup detach "
                        "(raise on slow GPUs / Jetson).",
        ),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.33"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
    ]

    graspable_objects_file = PathJoinSubstitution([
        FindPackageShare("ranger_sim"), "config", "graspable_objects.yaml"
    ])

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
        " graspable_objects_file:=",
        graspable_objects_file,
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

    # Live base wheel/steering joint states relayed from Gazebo's
    # JointStatePublisher (bridged to /gz/base_joint_states; zeros until the
    # sim reports them). gz_ros2_control only publishes the PiPER arm
    # joints; without these MoveIt's planning_scene_monitor never gets a
    # complete robot state and refuses to plan for piper_arm.
    base_joint_state_publisher = Node(
        package="ranger_sim",
        executable="base_joint_state_publisher.py",
        name="base_joint_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "/gz/base_joint_states",
        }],
    )

    # gz_ros2_control exports the joint8 mimic as "piper_joint8_mimic"
    # (broadcast by piper_mimic_state_broadcaster on its local topic);
    # republish it on /joint_states as piper_joint8 for MoveIt /
    # robot_state_publisher.
    mimic_joint_state_relay = Node(
        package="ranger_sim",
        executable="mimic_joint_state_relay.py",
        name="mimic_joint_state_relay",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "/piper_mimic_state_broadcaster/joint_states",
        }],
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

    # World services (CLI form; the YAML config form bridges topics only):
    # ControlWorld for grasp_manager's unpause, SetEntityPose for
    # kinematic scene props.
    world_services_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="world_services_bridge",
        arguments=[
            ["/world/", world_name, "/control@ros_gz_interfaces/srv/ControlWorld"],
            ["/world/", world_name, "/set_pose@ros_gz_interfaces/srv/SetEntityPose"],
        ],
        output="screen",
    )

    # Detaches every graspable object (all start welded to the gripper,
    # see ranger_gazebo.urdf.xacro), unpauses the world, serves /grasp/set.
    grasp_manager = Node(
        package="ranger_sim",
        executable="grasp_manager.py",
        name="grasp_manager",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "world_name": world_name,
            "objects_file": graspable_objects_file,
            "detach_timeout_sec": ParameterValue(grasp_detach_timeout, value_type=float),
        }],
    )

    switch_timeout = PythonExpression(
        [grasp_detach_timeout, f" + {SPAWNER_MARGIN_SEC}"]
    )
    service_call_timeout = PythonExpression(
        [grasp_detach_timeout, f" + {SPAWNER_MARGIN_SEC + 5.0}"]
    )
    spawner_timeouts = [
        "--switch-timeout", switch_timeout,
        "--service-call-timeout", service_call_timeout,
    ]

    # ---- ros2_control controller spawners ----
    # gz_ros2_control loads the controller_manager (params from
    # ros2_controllers_sim.yaml via the URDF). Controllers still need
    # spawning; chain them off the entity-create exit so the
    # controller_manager exists first, and bring the broadcaster up
    # before the arm/gripper controllers. Activation completes once
    # grasp_manager unpauses the world (the controller_manager only
    # updates while simulation time advances).
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "piper_mimic_state_broadcaster",
            "-c", "/controller_manager",
            *spawner_timeouts,
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["piper_arm_controller", "-c", "/controller_manager", *spawner_timeouts],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["piper_gripper_controller", "-c", "/controller_manager", *spawner_timeouts],
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
            ],
        )
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            gz_bridge,
            world_services_bridge,
            grasp_manager,
            spawn_robot,
            base_joint_state_publisher,
            mimic_joint_state_relay,
            spawn_jsb_after_create,
            spawn_controllers_after_jsb,
        ]
    )
