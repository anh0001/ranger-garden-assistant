#!/usr/bin/env python3
"""
Run the pick-and-place demo against an already-running sim.

Prereq: `ros2 launch ranger_sim sim_bringup.launch.py` is up (Gazebo +
controllers + grasp_manager + MoveIt move_group), with the same `scene`.

Safe, validate planning only (no motion):
  ros2 launch ranger_sim pick_place_demo.launch.py
  ros2 launch ranger_sim pick_place_demo.launch.py scene:=small_house

Actually run it in Gazebo (drives base + moves arm + gripper + grasp):
  ros2 launch ranger_sim pick_place_demo.launch.py execute:=true
  ros2 launch ranger_sim pick_place_demo.launch.py scene:=small_house execute:=true

The object, base approach, gripper closing and hover/grasp waypoints come
from the scene (config/pick_place_scenes.yaml); a non-empty object_name,
base_approach_x or gripper_closed argument overrides the scene's value.
"""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_spec = importlib.util.spec_from_file_location(
    "pick_place_scenes", Path(__file__).with_name("pick_place_scenes.py"))
scenes = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(scenes)


def generate_launch_description():
    scene = LaunchConfiguration("scene")
    args = [
        DeclareLaunchArgument("scene", default_value="garden",
                              description="Scene in config/pick_place_scenes.yaml"),
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
    # sim_bringup / spawn_robot, so this launch only runs the demo node.
    demo = Node(
        package="ranger_sim",
        executable="pick_place_demo.py",
        name="pick_place_demo",
        output="screen",
        parameters=[{
            "execute": LaunchConfiguration("execute"),
            "object_name": scenes.from_scene(scene, "object_name", str),
            "base_approach_x": scenes.from_scene(scene, "base_approach_x", float),
            "gripper_closed": scenes.from_scene(scene, "gripper_closed", float),
            "hover_joints": scenes.from_scene(scene, "hover_joints"),
            "grasp_joints": scenes.from_scene(scene, "grasp_joints"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription(args + [demo])
