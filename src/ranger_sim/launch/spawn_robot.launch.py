#!/usr/bin/env python3
"""
Spawn the Ranger + PiPER robot into a running Gazebo Fortress instance.

Also starts the ros2_control controllers, the ros_gz bridges, the grasp
manager and ranger_motion_adapter (/cmd_vel -> steering, wheel and body
commands, steer-then-drive like the real swerve driver). With
handover_gate:=true, base_handover_gate sits in front of the adapter
(/cmd_vel or /cmd_vel_manip -> /cmd_vel_gated, see
scripts/base_handover_gate.py) for the whole-body MPC. Gazebo is expected
to be PAUSED (gazebo.launch.py default): grasp_manager detaches every
DetachableJoint grasp joint created at spawn, then unpauses the world;
controllers finish activating after that.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
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
    handover_gate = LaunchConfiguration("handover_gate")

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
        DeclareLaunchArgument(
            "handover_gate",
            default_value="false",
            description="true: base_handover_gate owns the adapter's input "
                        "(/cmd_vel_gated) and hands the base between /cmd_vel and the "
                        "whole-body MPC's /cmd_vel_manip; false: the adapter reads /cmd_vel.",
        ),
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

    # /cmd_vel is not bridged: this node turns it into the Ranger driver's
    # motion mode (dual Ackermann / parallel / spinning), four steering
    # targets, four wheel speeds and the body twist on /cmd_vel_body, which
    # it holds back until the steering angles on /gz/base_joint_states have
    # reached their targets. Its timer runs on sim time, so it publishes
    # nothing until grasp_manager unpauses the world.
    ranger_motion_adapter = Node(
        package="ranger_sim",
        executable="ranger_motion_adapter.py",
        name="ranger_motion_adapter",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "joint_state_topic": "/gz/base_joint_states",
            # The gate's output_topic default (scripts/base_handover_gate.py).
            # Same truth values as the gate's IfCondition (true/1, any case).
            "input_topic": PythonExpression([
                "'/cmd_vel_gated' if '", handover_gate,
                "'.lower() in ('true', '1') else '/cmd_vel'",
            ]),
        }],
    )

    # The single owner of the adapter's input when handover_gate:=true:
    # /cmd_vel (Nav2, scripted demos) or /cmd_vel_manip (whole_body_mpc),
    # switched through ~/set_manipulate once the base is still.
    base_handover_gate = Node(
        package="ranger_sim",
        executable="base_handover_gate.py",
        name="base_handover_gate",
        output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("ranger_sim"), "config", "wbmpc.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(handover_gate),
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

    # Loaded and configured, NOT activated: it shares the arm's command
    # interfaces with piper_arm_controller; wbmpc_reach_demo.py switches
    # between the two (controller_manager/switch_controller, STRICT).
    arm_stream_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "piper_arm_stream_controller", "--inactive", "-c", "/controller_manager",
            *spawner_timeouts,
        ],
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
                arm_stream_controller_spawner,
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
            ranger_motion_adapter,
            base_handover_gate,
            mimic_joint_state_relay,
            spawn_jsb_after_create,
            spawn_controllers_after_jsb,
        ]
    )
