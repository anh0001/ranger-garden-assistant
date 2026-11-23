#!/usr/bin/env python3
r"""
Run the pick-and-place demo against an already-running sim.

Prereq: `ros2 launch ranger_sim sim_bringup.launch.py` is up (Gazebo +
controllers + grasp_manager + MoveIt move_group), with the same `scene`.

Safe, validate planning only (no motion):
  ros2 launch ranger_sim pick_place_demo.launch.py                  # small_house
  ros2 launch ranger_sim pick_place_demo.launch.py scene:=garden

Actually run it in Gazebo (drives base + moves arm + gripper + grasp):
  ros2 launch ranger_sim pick_place_demo.launch.py execute:=true
  ros2 launch ranger_sim pick_place_demo.launch.py scene:=garden execute:=true

The pick done by the whole-body MPC instead (docs/sim_wbmpc_pick.md; the
sim needs the gate: sim_bringup.launch.py handover_gate:=true), and the
comparison runs from a deliberately wrong standoff:
  ros2 launch ranger_sim pick_place_demo.launch.py execute:=true approach:=mpc
  ros2 launch ranger_sim pick_place_demo.launch.py execute:=true approach:=mpc \
      approach_only:=true approach_error:="[-0.08, 0.06, 0.07]"

The demo picks the object at the scene's table, carries it along the place
route and sets it down at the place pose, then drives back (small_house:
dining table -> kitchen counter). The object, base approach, gripper
closing, hover/grasp waypoints (and the place's, if the scene has its own),
standoff, place pose and route come from the scene
(config/pick_place_scenes.yaml); a non-empty
object_name, base_approach_x or gripper_closed argument overrides the
scene's value.
"""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

_spec = importlib.util.spec_from_file_location(
    "pick_place_nodes", Path(__file__).with_name("pick_place_nodes.py"))
nodes = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(nodes)


def generate_launch_description():
    scene = LaunchConfiguration("scene")
    args = [
        DeclareLaunchArgument("scene", default_value="small_house",
                              description="Scene in config/pick_place_scenes.yaml (default "
                                          "small_house; garden needs no fetched assets)"),
        DeclareLaunchArgument("execute", default_value="false",
                              description="false = plan only (no motion); true = run in sim"),
        DeclareLaunchArgument("base_approach_x", default_value="",
                              description="World x the base drives to before grasping "
                                          "(empty = from scene)"),
        DeclareLaunchArgument("object_name", default_value="",
                              description="Graspable model, config/graspable_objects.yaml "
                                          "(empty = from scene)"),
        DeclareLaunchArgument("gripper_closed", default_value="",
                              description="joint7 travel (m) when closing on the object, "
                                          "its radius (empty = from scene)"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]

    # The grasp itself is served by grasp_manager (/grasp/set), started by
    # sim_bringup / spawn_robot, so this launch only runs the demo node (and,
    # for approach:=mpc, whole_body_mpc).
    demo_nodes = nodes.pick_place_nodes(
        scene, LaunchConfiguration("execute"), LaunchConfiguration("use_sim_time"))

    return LaunchDescription(
        args + nodes.approach_arguments() + demo_nodes + nodes.end_with_the_demo(demo_nodes)
    )
