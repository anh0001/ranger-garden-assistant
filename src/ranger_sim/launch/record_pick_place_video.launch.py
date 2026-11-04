#!/usr/bin/env python3
"""
Record the pick-and-place demo to an MP4 against an already-running sim.

Prereq: `ros2 launch ranger_sim sim_bringup.launch.py` is up (headless is
fine, but the cameras need GPU rendering like the robot's own sensors).

  ros2 launch ranger_sim record_pick_place_video.launch.py
  ros2 launch ranger_sim record_pick_place_video.launch.py output_path:=/tmp/demo.mp4
  ros2 launch ranger_sim record_pick_place_video.launch.py scene:=small_house

Spawns the scene's static camera rig (models/video_rig*.sdf: one wide
camera), bridges it, starts record_demo_video.py, which draws the robot's
wrist camera (bridged by spawn_robot via config/gz_bridge.yaml) as the inset,
and starts pick_place_demo (execute:=true: pick, transfer, place, return)
only once the recorder logs its first frame. ros_gz_sim create exits 0 even
when no Gazebo answers, so that first frame, which only a running sim with
the rig can produce, is what keeps this launch from moving a real arm. When
the demo exits the launch shuts down and the recorder finishes the file. If
the recorder gives up first (no frame within first_frame_timeout, encoder
error) everything shuts down and the demo never starts. The default output
is ~/.ros/videos/pick_place_<time>.mp4.

Each scene names its camera rig (config/pick_place_scenes.yaml); its world
name, object, base approach, gripper closing and waypoints come from the
scene too (as do the standoff, place pose and route), unless the matching
argument is non-empty. Run sim_bringup with the same `scene`.
"""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

_spec = importlib.util.spec_from_file_location(
    "pick_place_scenes", Path(__file__).with_name("pick_place_scenes.py"))
scenes = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(scenes)

# Must equal READY_MARKER in scripts/record_demo_video.py (asserted by
# test/test_video_rig_launch.py).
READY_MARKER = "video recording started"
# Time the recorder gets on shutdown to flush the encoder and close the MP4.
RECORDER_SIGTERM_TIMEOUT_SEC = "30"


def make_demo_gate(demo_actions):
    """Return an OnProcessIO callback that yields ``demo_actions`` on the first ready line."""
    started = False

    def gate(event):
        nonlocal started
        if started or READY_MARKER not in event.text.decode(errors="replace"):
            return None
        started = True
        return demo_actions

    return gate


def generate_launch_description():
    scene = LaunchConfiguration("scene")
    args = [
        DeclareLaunchArgument("scene", default_value="garden",
                              description="Scene in config/pick_place_scenes.yaml "
                                          "(world, object, waypoints, camera rig)"),
        DeclareLaunchArgument("world_name", default_value="",
                              description="<world name> of the running sim "
                                          "(empty = from scene)"),
        DeclareLaunchArgument("output_path", default_value="",
                              description="MP4 to write (default ~/.ros/videos/...)"),
        DeclareLaunchArgument("object_name", default_value="",
                              description="Object the demo picks and the overlay tracks "
                                          "(empty = from scene)"),
        DeclareLaunchArgument("base_approach_x", default_value="",
                              description="World x the base drives to before grasping "
                                          "(empty = from scene)"),
        DeclareLaunchArgument("gripper_closed", default_value="",
                              description="joint7 travel (m) when closing on the object, "
                                          "its radius (empty = from scene)"),
        DeclareLaunchArgument("first_frame_timeout", default_value="60.0",
                              description="Wall seconds to wait for the first camera frame"),
    ]

    spawn_rig = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", scenes.SceneValue(
                scene, "world_name", override=LaunchConfiguration("world_name")
            ),
            "-file", PathJoinSubstitution(
                [FindPackageShare("ranger_sim"), "models", scenes.SceneValue(scene, "rig_file")]
            ),
            "-name", "video_rig",
        ],
        output="screen",
    )
    rig_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="video_rig_bridge",
        arguments=["/video/wide@sensor_msgs/msg/Image[ignition.msgs.Image"],
        output="screen",
    )
    recorder = Node(
        package="ranger_sim",
        executable="record_demo_video.py",
        name="demo_video_recorder",
        output="screen",
        sigterm_timeout=RECORDER_SIGTERM_TIMEOUT_SEC,
        parameters=[{
            "output_path": ParameterValue(LaunchConfiguration("output_path"), value_type=str),
            "grasp_object": scenes.from_scene(scene, "object_name", str),
            "first_frame_timeout_sec": ParameterValue(
                LaunchConfiguration("first_frame_timeout"), value_type=float
            ),
            "use_sim_time": True,
        }],
    )
    demo = Node(
        package="ranger_sim",
        executable="pick_place_demo.py",
        name="pick_place_demo",
        output="screen",
        parameters=[{
            "execute": True,
            "object_name": scenes.from_scene(scene, "object_name", str),
            "base_approach_x": scenes.from_scene(scene, "base_approach_x", float),
            "gripper_closed": scenes.from_scene(scene, "gripper_closed", float),
            "hover_joints": scenes.from_scene(scene, "hover_joints"),
            "grasp_joints": scenes.from_scene(scene, "grasp_joints"),
            "standoff_m": scenes.from_scene(scene, "standoff_m", float),
            "place_base_pose": scenes.from_scene(scene, "place_base_pose"),
            "place_route": scenes.from_scene(scene, "place_route"),
            "use_sim_time": True,
        }],
    )

    gate = make_demo_gate([LogInfo(msg="first camera frame recorded; starting the demo"), demo])
    start_demo_on_first_frame = RegisterEventHandler(
        OnProcessIO(target_action=recorder, on_stdout=gate, on_stderr=gate)
    )
    stop_when_demo_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=demo,
            on_exit=[
                LogInfo(msg="demo finished; closing the video"),
                EmitEvent(event=Shutdown(reason="pick_place_demo finished")),
            ],
        )
    )
    stop_when_recorder_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=recorder,
            on_exit=[EmitEvent(event=Shutdown(reason="demo_video_recorder exited"))],
        )
    )

    return LaunchDescription(
        args + [
            spawn_rig,
            rig_bridge,
            recorder,
            start_demo_on_first_frame,
            stop_when_demo_exits,
            stop_when_recorder_exits,
        ]
    )
