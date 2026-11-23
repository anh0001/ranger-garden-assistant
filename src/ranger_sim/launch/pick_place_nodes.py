#!/usr/bin/env python3
"""
Pick-and-place demo nodes shared by the pick-and-place launches (a helper, not a launch file).

pick_place_demo.launch.py and record_pick_place_video.launch.py import it by
path (it is installed next to them; `ros2 launch` only lists *.launch.py):

    _spec = importlib.util.spec_from_file_location(
        "pick_place_nodes", Path(__file__).with_name("pick_place_nodes.py"))
    nodes = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(nodes)

The `approach` argument picks the pick (docs/sim_wbmpc_pick.md):
  sequential  scripts/pick_place_demo.py: drive, look, slide the base in
  mpc         scripts/wbmpc_pick_place_demo.py and whole_body_mpc, with the
              scene's grasp_joints as its posture (q_nominal), the scene's
              mpc_base_box as its base box and a 3 mm position tolerance;
              needs sim_bringup handover_gate:=true
Both demo nodes are named pick_place_demo: the video captions follow that
name. Every node here moves the robot: the record launch starts them only
from its first-frame gate.
"""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(f"{name}.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


scenes = _load("pick_place_scenes")
common = _load("wbmpc_reach_common")

APPROACHES = ("sequential", "mpc")
DEMO_EXECUTABLES = ("pick_place_demo.py", "wbmpc_pick_place_demo.py")  # one per approach
DEMO_NODE_NAME = "pick_place_demo"  # record_demo_video.py's caption_node default
# In the planning spike a 2 mm tolerance left one start in fourteen at
# 2.2 mm until the step limit (the MPC's steady-state offset from its
# posture and velocity costs); 3 mm is inside the palm's 1 cm clearance.
MPC_PICK_POSITION_TOLERANCE_M = 0.003
# Must equal scripts/record_demo_video.py DEFAULT_TITLE (test_wbmpc_launch.py).
VIDEO_TITLE = "Ranger Mini 3.0 + PiPER  |  Gazebo Fortress pick & place"
MPC_VIDEO_TITLE = "Ranger Mini 3.0 + PiPER  |  pick & place, whole-body MPC pick"


def approach_arguments() -> list[DeclareLaunchArgument]:
    """Return the approach, approach_error and approach_only arguments."""
    return [
        DeclareLaunchArgument(
            "approach", default_value="sequential", choices=list(APPROACHES),
            description="sequential: scripted base and arm (pick_place_demo.py); mpc: the pick "
                        "by the whole-body MPC (needs sim_bringup handover_gate:=true)",
        ),
        DeclareLaunchArgument(
            "approach_error", default_value="[0.0, 0.0, 0.0]",
            description="[dx, dy, dyaw]: the first leg ends this far off the pick standoff "
                        "(m along / across the pick heading, rad), a coarse stop",
        ),
        DeclareLaunchArgument(
            "approach_only", default_value="false",
            description="true: set the object back after the lift and return (no transfer)",
        ),
    ]


def _float_list(name: str) -> ParameterValue:
    # float(): approach_error:="[0, 0.1, 0]" must still give doubles, not a mixed list.
    return ParameterValue(
        PythonExpression(["[float(v) for v in ", LaunchConfiguration(name), "]"]))


def _demo_parameters(scene, execute, use_sim_time) -> dict:
    s = scenes.from_scene
    return {
        "execute": execute,
        "object_name": s(scene, "object_name", str),
        "base_approach_x": s(scene, "base_approach_x", float),
        "gripper_closed": s(scene, "gripper_closed", float),
        "hover_joints": s(scene, "hover_joints"),
        "grasp_joints": s(scene, "grasp_joints"),
        "place_hover_joints": s(scene, "place_hover_joints"),
        "place_grasp_joints": s(scene, "place_grasp_joints"),
        "standoff_m": s(scene, "standoff_m", float),
        "place_base_pose": s(scene, "place_base_pose"),
        "place_route": s(scene, "place_route"),
        "grasp_tcp": s(scene, "grasp_tcp"),
        "hover_tcp": s(scene, "hover_tcp"),
        "approach_error": _float_list("approach_error"),
        "approach_only": ParameterValue(LaunchConfiguration("approach_only"), value_type=bool),
        "use_sim_time": use_sim_time,
    }


def pick_place_nodes(scene, execute, use_sim_time) -> list[Node]:
    """Return [scripted demo, MPC demo, whole_body_mpc]; each runs only for its `approach`."""
    sequential = LaunchConfigurationEquals("approach", "sequential")
    mpc = LaunchConfigurationEquals("approach", "mpc")

    def demo(executable: str, condition) -> Node:
        return Node(package="ranger_sim", executable=executable, name=DEMO_NODE_NAME,
                    output="screen", condition=condition,
                    parameters=[_demo_parameters(scene, execute, use_sim_time)])

    pick_overrides = {
        "q_nominal": scenes.from_scene(scene, "grasp_joints"),
        "base_box": scenes.from_scene(scene, "mpc_base_box"),
        "position_tolerance": MPC_PICK_POSITION_TOLERANCE_M,
    }
    return [
        demo(DEMO_EXECUTABLES[0], sequential),
        demo(DEMO_EXECUTABLES[1], mpc),
        common.mpc_node(use_sim_time, overrides=pick_overrides, condition=mpc),
    ]


def end_with_the_demo(nodes: list[Node], message: str = "demo finished") -> list:
    """
    Return handlers that shut the launch down when a demo of `nodes` exits.

    whole_body_mpc spins until it is stopped: without them a launch with
    approach:=mpc would never end.
    """
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[LogInfo(msg=message),
                     EmitEvent(event=Shutdown(reason="pick_place_demo finished"))],
        ))
        for node in nodes if node.node_executable in DEMO_EXECUTABLES
    ]


def video_title() -> PythonExpression:
    """Return the recorder's title for the `approach`."""
    return PythonExpression([
        f"'{MPC_VIDEO_TITLE}' if '", LaunchConfiguration("approach"),
        f"' == 'mpc' else '{VIDEO_TITLE}'",
    ])
