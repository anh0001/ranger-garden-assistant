#!/usr/bin/env python3
"""
Run the pick-and-place demo against an already-running sim.

Prereq: `ros2 launch ranger_sim sim_bringup.launch.py` is up (Gazebo +
controllers + grasp_manager + MoveIt move_group).

Safe, validate planning only (no motion):
  ros2 launch ranger_sim pick_place_demo.launch.py

Actually run it in Gazebo (drives base + moves arm + gripper + grasp):
  ros2 launch ranger_sim pick_place_demo.launch.py execute:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("execute", default_value="false",
                              description="false = plan only (no motion); true = run in sim"),
        DeclareLaunchArgument("base_approach_x", default_value="0.52",
                              description="World x the base drives to before grasping"),
        DeclareLaunchArgument("object_name", default_value="cup",
                              description="Graspable model (config/graspable_objects.yaml)"),
        DeclareLaunchArgument("gripper_closed", default_value="0.025",
                              description="joint7 travel (m) when closing on the object: "
                                          "the cup radius"),
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
            "base_approach_x": LaunchConfiguration("base_approach_x"),
            "object_name": LaunchConfiguration("object_name"),
            "gripper_closed": LaunchConfiguration("gripper_closed"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription(args + [demo])
