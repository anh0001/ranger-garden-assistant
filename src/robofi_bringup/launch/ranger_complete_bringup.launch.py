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
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

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
            description="Image width for the Tier IV camera.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_height",
            default_value="1860",
            description="Image height for the Tier IV camera.",
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
            default_value="bgr8",
            description="Desired ROS image encoding for the camera.",
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
            default_value="''",
            description="Device type filter for the RealSense camera (e.g., D405).",
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
            "launch_moveit",
            default_value="true",
            description="Launch MoveIt 2 move_group for arm motion planning.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_delay",
            default_value="5.0",
            description="Delay (seconds) before launching MoveIt move_group to allow robot model setup.",
        )
    )

    # Initialize arguments
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
    launch_moveit = LaunchConfiguration("launch_moveit")
    moveit_delay = LaunchConfiguration("moveit_delay")

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

    # Optional joint state publisher keeps wheel/steering transforms available when
    # the hardware base controller is not running.
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        condition=IfCondition(publish_joint_states),
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

    # PiPER arm driver
    piper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("piper"),
                "launch",
                "start_single_piper.launch.py"
            ])
        ),
        launch_arguments={
            "can_port": arm_can_port,
            "auto_enable": "true",
            "log_level": piper_log_level,
        }.items(),
    )

    camera_info_url = ParameterValue(
        [TextSubstitution(text="file://"), camera_calibration_file],
        value_type=str,
    )

    camera_parameters = {
        "video_device": camera_device,
        "image_size": [2880, 1860],  # Fixed: must be integer array, not LaunchConfiguration
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

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            joint_state_publisher_node,
            livox_launch,
            camera_group,
            wrist_camera_launch,
            rviz_node,
            # static_tf_lidar,
            ranger_base_launch,
            piper_launch,
            moveit_move_group_launch,
        ]
    )
