#!/usr/bin/env python3
"""
Run the whole-body MPC reach against an already-running sim.

Prereq: the sim with the base handover gate (and Nav2 for use_nav):

  ros2 launch ranger_sim sim_bringup.launch.py handover_gate:=true [launch_nav:=true]

Safe, plan the arm moves only (no motion, no hand-over, no Nav2 goal):
  ros2 launch ranger_sim wbmpc_reach_demo.launch.py

Run it in Gazebo (moves the base and the arm):
  ros2 launch ranger_sim wbmpc_reach_demo.launch.py execute:=true
  ros2 launch ranger_sim wbmpc_reach_demo.launch.py execute:=true use_nav:=true

Starts whole_body_mpc and wbmpc_reach_demo (scripts/wbmpc_reach_demo.py:
carry pose, [Nav2 away and to the handover pose], hand the base and the arm
to the MPC, reach target_x/y/z, hold, hand back). The launch ends with the
demo, which takes the MPC node down with it. Parameters: config/wbmpc.yaml.
See docs/sim_whole_body_mpc.md.
"""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

_spec = importlib.util.spec_from_file_location(
    "wbmpc_reach_common", Path(__file__).with_name("wbmpc_reach_common.py"))
common = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(common)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    args = [
        DeclareLaunchArgument("execute", default_value="false",
                              description="false = plan only (no motion); true = run in sim"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        *common.reach_arguments(),
    ]
    mpc = common.mpc_node(use_sim_time)
    demo = common.demo_node(
        ParameterValue(LaunchConfiguration("execute"), value_type=bool), use_sim_time
    )
    stop_with_the_demo = RegisterEventHandler(
        OnProcessExit(
            target_action=demo,
            on_exit=[EmitEvent(event=Shutdown(reason="wbmpc_reach_demo finished"))],
        )
    )
    return LaunchDescription(args + [mpc, demo, stop_with_the_demo])
