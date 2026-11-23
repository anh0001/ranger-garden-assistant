#!/usr/bin/env python3
"""
Whole-body MPC reach: launch arguments and nodes shared by the reach launches.

A helper module, not a launch file. wbmpc_reach_demo.launch.py and
record_wbmpc_reach_video.launch.py import it by path (it is installed next
to them; `ros2 launch` only lists *.launch.py):

    _spec = importlib.util.spec_from_file_location(
        "wbmpc_reach_common", Path(__file__).with_name("wbmpc_reach_common.py"))
    common = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(common)

The nodes take config/wbmpc.yaml, which addresses them by name, plus the
reach target from the target_x/y/z/yaw arguments: the one source of the
target for the demo and the video's marker.
"""

from typing import Optional

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

MPC_NODE_NAME = "whole_body_mpc"
DEMO_NODE_NAME = "wbmpc_reach_demo"
# Level gripper 0.20 m beyond the arm-only reach from the spawn pose, above
# the house table (test/test_wbmpc_launch.py checks it against the world).
TARGET_DEFAULTS = (
    ("target_x", "0.95", "TCP target x in odom (m)"),
    ("target_y", "0.15", "TCP target y in odom (m)"),
    ("target_z", "1.00", "TCP target z in odom (m; world z = odom z - 0.010)"),
    ("target_yaw", "0.0", "Approach heading of the level gripper in odom (rad)"),
)


def config_file() -> PathJoinSubstitution:
    """Return config/wbmpc.yaml of the installed package."""
    return PathJoinSubstitution([FindPackageShare("ranger_sim"), "config", "wbmpc.yaml"])


def reach_arguments() -> list[DeclareLaunchArgument]:
    """Return the target and use_nav arguments of the reach launches."""
    return [
        *(DeclareLaunchArgument(name, default_value=value, description=description)
          for name, value, description in TARGET_DEFAULTS),
        DeclareLaunchArgument(
            "use_nav", default_value="false",
            description="true: Nav2 drives away and to the handover pose first (sim_bringup "
                        "launch_nav:=true); false: the reach starts where the base stands",
        ),
    ]


def _float(name: str) -> PythonExpression:
    # float(): target_x:=1 must still give a double, not an int, to the node.
    return PythonExpression(["float(", LaunchConfiguration(name), ")"])


def target_position() -> ParameterValue:
    """Return the [x, y, z] parameter built from the target arguments (YAML float list)."""
    return ParameterValue(
        ["[", _float("target_x"), ", ", _float("target_y"), ", ", _float("target_z"), "]"]
    )


def mpc_node(use_sim_time, overrides: Optional[dict] = None, condition=None) -> Node:
    """Return the whole_body_mpc node; `overrides` take precedence over config/wbmpc.yaml."""
    return Node(
        package="ranger_sim",
        executable="whole_body_mpc.py",
        name=MPC_NODE_NAME,
        output="screen",
        parameters=[config_file(), {"use_sim_time": use_sim_time, **(overrides or {})}],
        condition=condition,
    )


def demo_node(execute, use_sim_time) -> Node:
    """Return the wbmpc_reach_demo node; `execute` false plans only."""
    return Node(
        package="ranger_sim",
        executable="wbmpc_reach_demo.py",
        name=DEMO_NODE_NAME,
        output="screen",
        parameters=[config_file(), {
            "execute": execute,
            "use_nav": ParameterValue(LaunchConfiguration("use_nav"), value_type=bool),
            "target_position": target_position(),
            "target_yaw": ParameterValue(_float("target_yaw"), value_type=float),
            "use_sim_time": use_sim_time,
        }],
    )
