#!/usr/bin/env python3

"""
Complete system bringup for Ranger Mini 3.0 with:
- Base controller (ranger_ros2)
- Livox Mid-360 LiDAR
- Tier IV C2-176 fisheye camera
- RealSense D405 (PiPER wrist camera)
- PiPER 6-DOF arm
- Robot description and TF
- MoveIt 2 move_group (optional)

Supports three backends:
  backend:=real   - Real hardware (CAN drivers, physical sensors)
  backend:=mock   - MoveIt-only with mock_components/GenericSystem (no Gazebo)
  backend:=gazebo - Full Gazebo Fortress simulation (delegates to ranger_sim)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "backend",
            default_value="real",
            description="Execution backend: 'real' (hardware), 'mock' (MoveIt-only), or 'gazebo' (Gazebo Fortress).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="",
            description="Gazebo world file (only used when backend:=gazebo).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "can_device",
            default_value="can_base",
            description="CAN device for base controller (standardized to can_base for USB gs_usb adapters)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_can_port",
            default_value="can_piper",
            description="CAN port for PiPER arm (standardized to can_piper for USB gs_usb adapters)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "livox_frame_id",
            default_value="lidar_link",
            description="Frame ID assigned to Livox point clouds.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_joint_states",
            default_value="true",
            description="Publish default joint states so wheel transforms exist when the base driver is disabled.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz for visualization.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "livox_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("robofi_bringup"),
                    "config",
                    "livox_mid360_config.json",
                ]
            ),
            description="Path to the Livox Mid-360 JSON configuration file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_odom_tf",
            default_value="false",
            description="Publish wheel odometry TF (set to true only if not using FASTLIO2).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_camera",
            default_value="true",
            description="Whether to launch the Tier IV C2-176 camera driver.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_device",
            default_value="/dev/tieriv_c2_video0",
            description="V4L2 device path for the Tier IV camera.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_namespace",
            default_value="/camera",
            description="Namespace applied to the camera topics.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_frame_id",
            default_value="camera_link",
            description="Frame ID reported by the camera driver.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_width",
            default_value="2880",
            description="Image width for the Tier IV camera (native C2-176 resolution; hardware only supports 2880x1860).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_height",
            default_value="1860",
            description="Image height for the Tier IV camera (native C2-176 resolution; hardware only supports 2880x1860).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_pixel_format",
            default_value="UYVY",
            description="Pixel format configured in the camera driver.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_output_encoding",
            default_value="yuv422",
            description="Desired ROS image encoding for the camera (yuv422=native, no conversion overhead).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_name",
            default_value="tier4_c2_176",
            description="Camera name for calibration and CameraInfo.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_calibration_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("camera_lidar_fuse"),
                    "config",
                    "tier4_c2_176_2880x1860.yaml",
                ]
            ),
            description="Calibration YAML for the Tier IV C2-176 camera.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "fix_camera_info_frame_id",
            default_value="true",
            description="Republish CameraInfo with a populated frame_id when the driver leaves it empty.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="camera_info",
            description="CameraInfo topic name exposed by the bringup.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_info_raw_topic",
            default_value="camera_info_raw",
            description="Internal CameraInfo topic name used before frame_id fixup.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_info_only_if_empty",
            default_value="true",
            description="Only override CameraInfo frame_id when it is empty.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_wrist_camera",
            default_value="true",
            description="Whether to launch the RealSense D405 wrist camera.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_name",
            default_value="piper_d405",
            description="RealSense camera name for the PiPER wrist camera.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_namespace",
            default_value="/piper/wrist_camera",
            description="Namespace applied to the PiPER wrist camera topics.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_serial",
            default_value="''",
            description="Serial number for selecting the RealSense D405.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_usb_port_id",
            default_value="''",
            description="USB port ID for selecting the RealSense D405.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_device_type",
            default_value="D405",
            description=(
                "Device type filter for the RealSense camera. Defaults to 'D405' so "
                "the wrist node only ever binds the D405 wrist camera and never "
                "silently grabs another connected RealSense (e.g. the rear D435i) "
                "when 'wrist_camera_serial' is unset."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_config_file",
            default_value="''",
            description="Optional YAML config file for the RealSense D405.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_base_frame_id",
            default_value="piper_camera_link",
            description="Base frame for the RealSense TF tree.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_enable_color",
            default_value="true",
            description="Enable the RealSense color stream.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_enable_depth",
            default_value="true",
            description="Enable the RealSense depth stream.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_enable_rgbd",
            default_value="false",
            description="Enable the RealSense RGBD topic.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_pointcloud",
            default_value="false",
            description="Enable the RealSense pointcloud topic.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_align_depth",
            default_value="false",
            description="Enable align depth filter for the RealSense camera.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_color_profile",
            default_value="0,0,0",
            description="Color stream profile (width,height,fps).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_depth_profile",
            default_value="0,0,0",
            description="Depth stream profile (width,height,fps).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera_publish_tf",
            default_value="true",
            description="Publish TF frames from the RealSense driver.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "piper_log_level",
            default_value="warn",
            description="Log level for PiPER arm controller (debug, info, warn, error, fatal).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "piper_joint_states_topic",
            default_value="/piper/joint_states",
            description="Raw JointState feedback topic from the PiPER driver.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "piper_joint_command_topic",
            default_value="/piper/joint_cmd",
            description="JointState command topic consumed by the PiPER driver.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "piper_joint_name_prefix",
            default_value="piper_",
            description="Prefix applied to PiPER joint names (e.g., piper_joint1).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_piper",
            default_value="true",
            description="Launch the PiPER arm driver (disable if the arm CAN device is unavailable).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_piper_bridge",
            default_value="true",
            description="Launch the FollowJointTrajectory bridge for the PiPER arm.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_piper_joint_states",
            default_value="true",
            description="Fuse PiPER joint states into /joint_states via joint_state_publisher source_list.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "piper_arm_action_name",
            default_value="/piper_arm_controller/follow_joint_trajectory",
            description="FollowJointTrajectory action name for the PiPER arm controller.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "piper_gripper_action_name",
            default_value="/piper_gripper_controller/follow_joint_trajectory",
            description="FollowJointTrajectory action name for the PiPER gripper controller.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "piper_bridge_publish_rate",
            default_value="50.0",
            description="Publish rate (Hz) for PiPER joint command streaming.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "piper_bridge_speed",
            default_value="20",
            description="Default speed (1-100) forwarded via PiPER joint command velocity[6].",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_moveit",
            default_value="true",
            description="Launch MoveIt 2 move_group for arm motion planning.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_nav",
            default_value="false",
            description="Launch Nav2 when backend:=gazebo.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_delay",
            default_value="5.0",
            description="Delay (seconds) before launching MoveIt move_group to allow robot model setup.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_moveit_servo",
            default_value="true",
            description="Launch MoveIt Servo for real-time Cartesian arm control.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_servo_delay",
            default_value="7.0",
            description="Delay (seconds) before launching MoveIt Servo (should be after move_group).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo_command_bridge",
            default_value="true",
            description="Launch bridge from Servo joint arrays to PiPER /piper/joint_cmd.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "servo_command_out_topic",
            default_value="/moveit_servo/piper_arm_joint_targets",
            description="MoveIt Servo command_out topic consumed by the PiPER servo bridge.",
        )
    )

    # Initialize arguments
    backend = LaunchConfiguration("backend")
    world = LaunchConfiguration("world")
    can_device = LaunchConfiguration("can_device")
    arm_can_port = LaunchConfiguration("arm_can_port")
    use_sim_time = LaunchConfiguration("use_sim_time")
    livox_frame_id = LaunchConfiguration("livox_frame_id")
    livox_config_file = LaunchConfiguration("livox_config_file")
    publish_joint_states = LaunchConfiguration("publish_joint_states")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    use_rviz = LaunchConfiguration("use_rviz")
    launch_camera = LaunchConfiguration("launch_camera")
    camera_device = LaunchConfiguration("camera_device")
    camera_namespace = LaunchConfiguration("camera_namespace")
    camera_frame_id = LaunchConfiguration("camera_frame_id")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_pixel_format = LaunchConfiguration("camera_pixel_format")
    camera_output_encoding = LaunchConfiguration("camera_output_encoding")
    camera_name = LaunchConfiguration("camera_name")
    camera_calibration_file = LaunchConfiguration("camera_calibration_file")
    fix_camera_info_frame_id = LaunchConfiguration("fix_camera_info_frame_id")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    camera_info_raw_topic = LaunchConfiguration("camera_info_raw_topic")
    camera_info_only_if_empty = LaunchConfiguration("camera_info_only_if_empty")
    launch_wrist_camera = LaunchConfiguration("launch_wrist_camera")
    wrist_camera_name = LaunchConfiguration("wrist_camera_name")
    wrist_camera_namespace = LaunchConfiguration("wrist_camera_namespace")
    wrist_camera_serial = LaunchConfiguration("wrist_camera_serial")
    wrist_camera_usb_port_id = LaunchConfiguration("wrist_camera_usb_port_id")
    wrist_camera_device_type = LaunchConfiguration("wrist_camera_device_type")
    wrist_camera_config_file = LaunchConfiguration("wrist_camera_config_file")
    wrist_camera_base_frame_id = LaunchConfiguration("wrist_camera_base_frame_id")
    wrist_camera_enable_color = LaunchConfiguration("wrist_camera_enable_color")
    wrist_camera_enable_depth = LaunchConfiguration("wrist_camera_enable_depth")
    wrist_camera_enable_rgbd = LaunchConfiguration("wrist_camera_enable_rgbd")
    wrist_camera_pointcloud = LaunchConfiguration("wrist_camera_pointcloud")
    wrist_camera_align_depth = LaunchConfiguration("wrist_camera_align_depth")
    wrist_camera_color_profile = LaunchConfiguration("wrist_camera_color_profile")
    wrist_camera_depth_profile = LaunchConfiguration("wrist_camera_depth_profile")
    wrist_camera_publish_tf = LaunchConfiguration("wrist_camera_publish_tf")
    piper_log_level = LaunchConfiguration("piper_log_level")
    piper_joint_states_topic = LaunchConfiguration("piper_joint_states_topic")
    piper_joint_command_topic = LaunchConfiguration("piper_joint_command_topic")
    piper_joint_name_prefix = LaunchConfiguration("piper_joint_name_prefix")
    launch_piper = LaunchConfiguration("launch_piper")
    launch_piper_bridge = LaunchConfiguration("launch_piper_bridge")
    use_piper_joint_states = LaunchConfiguration("use_piper_joint_states")
    piper_arm_action_name = LaunchConfiguration("piper_arm_action_name")
    piper_gripper_action_name = LaunchConfiguration("piper_gripper_action_name")
    piper_bridge_publish_rate = LaunchConfiguration("piper_bridge_publish_rate")
    piper_bridge_speed = LaunchConfiguration("piper_bridge_speed")
    launch_moveit = LaunchConfiguration("launch_moveit")
    launch_nav = LaunchConfiguration("launch_nav")
    moveit_delay = LaunchConfiguration("moveit_delay")
    launch_moveit_servo = LaunchConfiguration("launch_moveit_servo")
    moveit_servo_delay = LaunchConfiguration("moveit_servo_delay")
    launch_servo_command_bridge = LaunchConfiguration("launch_servo_command_bridge")
    servo_command_out_topic = LaunchConfiguration("servo_command_out_topic")

    # Get URDF via xacro with mesh_dir argument
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ranger_description"),
                    "urdf",
                    "ranger_complete.urdf.xacro",
                ]
            ),
            " mesh_dir:=file://",
            PathJoinSubstitution(
                [
                    FindPackageShare("ranger_description"),
                    "meshes",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Joint state publisher keeps wheel/steering/passive joints available.
    # When PiPER is enabled, merge its raw state topic into /joint_states.
    piper_joint_state_sanitizer_node = Node(
        package="robofi_bringup",
        executable="piper_joint_state_sanitizer.py",
        name="piper_joint_state_sanitizer",
        output="both",
        parameters=[
            {
                "input_topic": piper_joint_states_topic,
                "output_topic": "/piper/joint_states_sanitized",
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    publish_joint_states,
                    "' == 'true' and '",
                    use_piper_joint_states,
                    "' == 'true' and '",
                    launch_piper,
                    "' == 'true'",
                ]
            )
        ),
    )

    joint_state_publisher_with_piper_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[
            robot_description,
            {
                "use_sim_time": use_sim_time,
                "source_list": ["/piper/joint_states_sanitized"],
            },
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    publish_joint_states,
                    "' == 'true' and '",
                    use_piper_joint_states,
                    "' == 'true' and '",
                    launch_piper,
                    "' == 'true'",
                ]
            )
        ),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    publish_joint_states,
                    "' == 'true' and (not '",
                    use_piper_joint_states,
                    "' == 'true' or not '",
                    launch_piper,
                    "' == 'true')",
                ]
            )
        ),
    )

    # Ranger base driver launch (disabled by default; uncomment when chassis is present)
    ranger_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robofi_bringup"),
                "launch",
                "ranger_base.launch.py"
            ])
        ),
        launch_arguments={
            "can_device": can_device,
            "publish_odom_tf": publish_odom_tf,
        }.items(),
    )

    # Livox LiDAR driver launch
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robofi_bringup"),
                "launch",
                "livox_lidar.launch.py"
            ])
        ),
        launch_arguments={
            "frame_id": livox_frame_id,
            "config_file": livox_config_file,
        }.items(),
    )

    # PiPER arm controller
    piper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("piper"),
                "launch",
                "start_single_controller.launch.py"
            ])
        ),
        launch_arguments={
            "can_port": arm_can_port,
            "auto_enable": "true",
            "log_level": piper_log_level,
            "joint_states_topic": piper_joint_states_topic,
            "joint_cmd_topic": piper_joint_command_topic,
            "joint_name_prefix": piper_joint_name_prefix,
        }.items(),
        condition=IfCondition(launch_piper),
    )

    piper_bridge_node = Node(
        package="piper",
        executable="piper_follow_joint_trajectory_bridge",
        name="piper_follow_joint_trajectory_bridge",
        output="both",
        parameters=[
            {
                "arm_action_name": piper_arm_action_name,
                "gripper_action_name": piper_gripper_action_name,
                "command_topic": piper_joint_command_topic,
                "state_topic": piper_joint_states_topic,
                "joint_name_prefix": piper_joint_name_prefix,
                "publish_rate_hz": ParameterValue(
                    piper_bridge_publish_rate, value_type=float
                ),
                "default_speed": ParameterValue(piper_bridge_speed, value_type=int),
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    launch_piper_bridge,
                    "' == 'true' and '",
                    launch_piper,
                    "' == 'true'",
                ]
            )
        ),
    )

    servo_command_bridge_node = Node(
        package="robofi_bringup",
        executable="servo_array_to_piper_joint_cmd.py",
        name="servo_array_to_piper_joint_cmd",
        output="both",
        parameters=[
            {
                "input_topic": servo_command_out_topic,
                "joint_states_topic": piper_joint_states_topic,
                "output_topic": piper_joint_command_topic,
                "joint_prefix": piper_joint_name_prefix,
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    launch_servo_command_bridge,
                    "' == 'true' and '",
                    launch_moveit_servo,
                    "' == 'true' and '",
                    launch_piper,
                    "' == 'true'",
                ]
            )
        ),
    )

    camera_info_url = ParameterValue(
        [TextSubstitution(text="file://"), camera_calibration_file],
        value_type=str,
    )

    camera_parameters = {
        "video_device": camera_device,
        "image_size": [2880, 1860],  # Native C2-176 resolution (hardware supports only this size)
        "pixel_format": camera_pixel_format,
        "output_encoding": camera_output_encoding,
        "camera_frame_id": camera_frame_id,
        "camera_name": camera_name,
        "camera_info_url": camera_info_url,
        "use_sensor_data_qos": True,
    }

    tier4_camera_node_fixed = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace=camera_namespace,
        name="tier4_c2_176_camera",
        output="both",
        parameters=[camera_parameters],
        remappings=[("camera_info", camera_info_raw_topic)],
        condition=IfCondition(fix_camera_info_frame_id),
    )

    tier4_camera_node_unfixed = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace=camera_namespace,
        name="tier4_c2_176_camera",
        output="both",
        parameters=[camera_parameters],
        remappings=[("camera_info", camera_info_topic)],
        condition=UnlessCondition(fix_camera_info_frame_id),
    )

    camera_info_fixer_node = Node(
        package="robofi_bringup",
        executable="camera_info_frame_id_fixer.py",
        namespace=camera_namespace,
        name="camera_info_frame_id_fixer",
        output="both",
        parameters=[
            {
                "frame_id": camera_frame_id,
                "only_if_empty": ParameterValue(camera_info_only_if_empty, value_type=bool),
            }
        ],
        remappings=[
            ("camera_info_raw", camera_info_raw_topic),
            ("camera_info", camera_info_topic),
        ],
        condition=IfCondition(fix_camera_info_frame_id),
    )

    camera_group = GroupAction(
        condition=IfCondition(launch_camera),
        actions=[
            tier4_camera_node_fixed,
            tier4_camera_node_unfixed,
            camera_info_fixer_node,
        ],
    )

    wrist_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": wrist_camera_name,
            "camera_namespace": wrist_camera_namespace,
            "serial_no": wrist_camera_serial,
            "usb_port_id": wrist_camera_usb_port_id,
            "device_type": wrist_camera_device_type,
            "config_file": wrist_camera_config_file,
            "enable_color": wrist_camera_enable_color,
            "enable_depth": wrist_camera_enable_depth,
            "enable_rgbd": wrist_camera_enable_rgbd,
            "pointcloud.enable": wrist_camera_pointcloud,
            "align_depth.enable": wrist_camera_align_depth,
            "rgb_camera.color_profile": wrist_camera_color_profile,
            "depth_module.depth_profile": wrist_camera_depth_profile,
            "publish_tf": wrist_camera_publish_tf,
            "base_frame_id": wrist_camera_base_frame_id,
        }.items(),
        condition=IfCondition(launch_wrist_camera),
    )

    # RViz visualization (optional)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robofi_bringup"), "rviz", "robot_bringup.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # MoveIt move_group (delayed to allow robot model and arm driver to initialize)
    moveit_move_group_launch = TimerAction(
        period=moveit_delay,
        actions=[
            GroupAction(
                condition=IfCondition(launch_moveit),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([
                                FindPackageShare("ranger_piper_moveit"),
                                "launch",
                                "move_group.launch.py",
                            ])
                        ),
                    ),
                ],
            ),
        ],
    )

    moveit_servo_launch = TimerAction(
        period=moveit_servo_delay,
        actions=[
            GroupAction(
                condition=IfCondition(launch_moveit_servo),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([
                                FindPackageShare("ranger_piper_moveit"),
                                "launch",
                                "moveit_servo.launch.py",
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

    # -----------------------------------------------------------------
    # Backend-conditional helpers
    # -----------------------------------------------------------------
    is_real = PythonExpression(["'", backend, "' == 'real'"])
    is_mock = PythonExpression(["'", backend, "' == 'mock'"])
    is_gazebo = PythonExpression(["'", backend, "' == 'gazebo'"])

    # When backend:=gazebo, delegate entirely to ranger_sim's sim_bringup
    gazebo_bringup = GroupAction(
        condition=IfCondition(is_gazebo),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ranger_sim"),
                        "launch", "sim_bringup.launch.py",
                    ])
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "world": world,
                    "launch_moveit": launch_moveit,
                    "launch_nav": launch_nav,
                    "use_rviz": use_rviz,
                }.items(),
            ),
        ],
    )

    # When backend:=mock, launch MoveIt demo (mock_components) directly
    mock_bringup = GroupAction(
        condition=IfCondition(is_mock),
        actions=[
            SetParameter(name="use_sim_time", value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ranger_piper_moveit"),
                        "launch", "demo.launch.py",
                    ])
                ),
            ),
        ],
    )

    # When backend:=real, launch the full hardware stack
    real_bringup = GroupAction(
        condition=IfCondition(is_real),
        actions=[
            robot_state_publisher_node,
            piper_joint_state_sanitizer_node,
            joint_state_publisher_with_piper_node,
            joint_state_publisher_node,
            livox_launch,
            camera_group,
            wrist_camera_launch,
            rviz_node,
            ranger_base_launch,
            piper_launch,
            piper_bridge_node,
            servo_command_bridge_node,
            moveit_move_group_launch,
            moveit_servo_launch,
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            real_bringup,
            mock_bringup,
            gazebo_bringup,
        ]
    )
