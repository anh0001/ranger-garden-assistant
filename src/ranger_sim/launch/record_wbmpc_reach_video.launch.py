#!/usr/bin/env python3
"""
Record the whole-body MPC reach to an MP4 against an already-running sim.

Prereq: the sim with the base handover gate (and Nav2 for use_nav), headless
is fine but the cameras need GPU rendering:

  ros2 launch ranger_sim sim_bringup.launch.py handover_gate:=true [launch_nav:=true]
  ros2 launch ranger_sim record_wbmpc_reach_video.launch.py [use_nav:=true]
  ros2 launch ranger_sim record_wbmpc_reach_video.launch.py output_path:=/tmp/reach.mp4

Spawns the scene's camera rig and a visual-only marker at the target
(models/wbmpc_target.sdf, world z = target_z - 0.010), bridges the rig's
camera, starts record_demo_video.py (captions from wbmpc_reach_demo's log)
and only once the recorder logs its first frame starts whole_body_mpc and
wbmpc_reach_demo (execute). ros_gz_sim create exits 0 even when no Gazebo
answers, so that first frame, which only a running sim with the rig can
produce, is what keeps this launch from moving a real robot. The launch
shuts down when the demo or the recorder exits. Default output:
~/.ros/videos/pick_place_<time>.mp4 (the recorder's naming).
"""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_sibling(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(f"{name}.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


scenes = _load_sibling("pick_place_scenes")
common = _load_sibling("wbmpc_reach_common")

# Must equal READY_MARKER in scripts/record_demo_video.py (asserted by
# test/test_wbmpc_launch.py).
READY_MARKER = "video recording started"
# Time the recorder gets on shutdown to flush the encoder and close the MP4.
RECORDER_SIGTERM_TIMEOUT_SEC = "30"
TITLE_WITH_NAV = "Ranger Mini 3.0 + PiPER  |  Nav2 MPPI -> whole-body MPC"
TITLE_PARKED = "Ranger Mini 3.0 + PiPER  |  whole-body MPC reach (base + arm)"
# Gazebo rests base_footprint 10 mm below the ground: world z = odom z - 0.010.
FOOTPRINT_HEIGHT_M = -0.010


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
    world_name = scenes.SceneValue(scene, "world_name", override=LaunchConfiguration("world_name"))
    args = [
        DeclareLaunchArgument("scene", default_value="small_house",
                              description="Scene in config/pick_place_scenes.yaml (world, "
                                          "camera rig); run sim_bringup with the same one"),
        DeclareLaunchArgument("world_name", default_value="",
                              description="<world name> of the running sim (empty = from scene)"),
        DeclareLaunchArgument("output_path", default_value="",
                              description="MP4 to write (default ~/.ros/videos/...)"),
        DeclareLaunchArgument("first_frame_timeout", default_value="60.0",
                              description="Wall seconds to wait for the first camera frame"),
        *common.reach_arguments(),
    ]

    spawn_rig = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", world_name,
            "-file", PathJoinSubstitution(
                [FindPackageShare("ranger_sim"), "models", scenes.SceneValue(scene, "rig_file")]
            ),
            "-name", "video_rig",
        ],
        output="screen",
    )
    spawn_marker = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", world_name,
            "-file", PathJoinSubstitution(
                [FindPackageShare("ranger_sim"), "models", "wbmpc_target.sdf"]
            ),
            "-name", "wbmpc_target",
            "-x", LaunchConfiguration("target_x"),
            "-y", LaunchConfiguration("target_y"),
            "-z", PythonExpression(
                [LaunchConfiguration("target_z"), f" + ({FOOTPRINT_HEIGHT_M})"]
            ),
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
            "caption_node": common.DEMO_NODE_NAME,
            "title": PythonExpression([
                f"'{TITLE_WITH_NAV}' if '", LaunchConfiguration("use_nav"),
                f"' == 'true' else '{TITLE_PARKED}'",
            ]),
            "grasp_object": scenes.from_scene(scene, "object_name", str),
            "first_frame_timeout_sec": ParameterValue(
                LaunchConfiguration("first_frame_timeout"), value_type=float
            ),
            "use_sim_time": True,
        }],
    )
    mpc = common.mpc_node(True)
    demo = common.demo_node(True, True)

    gate = make_demo_gate([LogInfo(msg="first camera frame recorded; starting the reach"),
                           mpc, demo])
    return LaunchDescription(args + [
        spawn_rig,
        spawn_marker,
        rig_bridge,
        recorder,
        RegisterEventHandler(OnProcessIO(target_action=recorder, on_stdout=gate, on_stderr=gate)),
        RegisterEventHandler(OnProcessExit(target_action=demo, on_exit=[
            LogInfo(msg="demo finished; closing the video"),
            EmitEvent(event=Shutdown(reason="wbmpc_reach_demo finished")),
        ])),
        RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=[
            EmitEvent(event=Shutdown(reason="demo_video_recorder exited")),
        ])),
    ])
