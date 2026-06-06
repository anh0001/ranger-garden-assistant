#!/usr/bin/env python3

"""
Launch file for Ranger Mini 3.0 base controller.

The AgileX driver integrates wheel odometry internally and offers no reset. When
`enable_odom_reset` is true (default) we redirect the driver's output to
`odom_raw` and disable its TF, then run `odom_reset_relay` to republish the
resettable `/odom` (and own the odom -> base TF when `publish_odom_tf` is true).
Zero the odometry at any time with:

    ros2 service call /reset_odom std_srvs/srv/Trigger
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "can_device",
            default_value="can_base",
            description="CAN device name (standardized to can_base for USB gs_usb adapters)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_model",
            default_value="ranger_mini_v3",
            description="Robot model (ranger_mini_v1, ranger_mini_v2). Note: v2 works for v3.0",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_footprint",
            description="Base frame id published by the driver (connects odom to base_footprint).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_odom_tf",
            default_value="true",
            description="Publish the odom -> base_frame TF to satisfy Nav2 costmaps.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_odom_reset",
            default_value="true",
            description=(
                "Insert odom_reset_relay in front of the driver so /odom can be "
                "zeroed at runtime via the /reset_odom service."
            ),
        )
    )

    # Initialize arguments
    can_device = LaunchConfiguration("can_device")
    robot_model = LaunchConfiguration("robot_model")
    base_frame = LaunchConfiguration("base_frame")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    enable_odom_reset = LaunchConfiguration("enable_odom_reset")

    # When the relay is enabled the driver publishes raw odom on `odom_raw` and the
    # relay owns `/odom` + TF; otherwise the driver publishes `/odom` directly.
    driver_odom_topic = PythonExpression(
        ["'odom_raw' if '", enable_odom_reset, "' == 'true' else 'odom'"]
    )
    driver_publish_tf = PythonExpression(
        ["'false' if '", enable_odom_reset, "' == 'true' else '", publish_odom_tf, "'"]
    )

    # Include the ranger_ros2 launch file
    ranger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ranger_bringup"),
                "launch",
                "ranger_mini_v3.launch.py"
            ])
        ),
        launch_arguments={
            "port_name": can_device,
            "robot_model": robot_model,
            "base_frame": base_frame,
            "odom_topic_name": driver_odom_topic,
            "publish_odom_tf": driver_publish_tf,
        }.items(),
    )

    odom_reset_relay = Node(
        package="robofi_bringup",
        executable="odom_reset_relay.py",
        name="odom_reset_relay",
        output="screen",
        condition=IfCondition(enable_odom_reset),
        parameters=[{
            "input_topic": "odom_raw",
            "output_topic": "odom",
            "odom_frame": "odom",
            "base_frame": base_frame,
            "publish_tf": ParameterValue(publish_odom_tf, value_type=bool),
            "zero_on_start": True,
        }],
    )

    return LaunchDescription(declared_arguments + [ranger_launch, odom_reset_relay])
