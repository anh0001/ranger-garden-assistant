#!/usr/bin/env python3
"""
Handover baseline: launch arguments and nodes (a helper module, not a launch file).

handover_baseline.launch.py and test/test_handover_sim.py import it by path
(it is installed next to them; `ros2 launch` only lists *.launch.py):

    _spec = importlib.util.spec_from_file_location(
        "handover_common", Path(__file__).with_name("handover_common.py"))
    common = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(common)

Every node takes config/handover.yaml, which addresses them by name, plus
the episode's launch arguments; whole_body_mpc takes config/wbmpc.yaml with
the scene's mpc_base_box as its start box (the baseline sends the box of the
chosen standoff before each hand-over). See docs/sim_handover_baseline.md.
"""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(f"{name}.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


scenes = _load("pick_place_scenes")
reach = _load("wbmpc_reach_common")

PERSON_NODE = "person_proxy"
SAFETY_NODE = "handover_safety"
BASELINE_NODE = "handover_baseline"
RECORDER_NODE = "handover_recorder"
PERSON_MODEL, HAND_MODEL = "person_proxy", "person_hand"  # model names person_proxy.py moves
FAULT_COUNT = 12
# whole_body_mpc's REACHED tolerance for the offer and set-down (config/wbmpc.yaml: 5 mm).
# Near the reach limit the MPC settles 4.3-5.0 mm off on a kinematic plant (CARRY
# posture); in Gazebo it settled 9.1-9.7 mm off at the table place pose (2026-11-25:
# with the speed scale at 0.3 the base command decays under the 0.01 m/s deadband
# and the base parks first). 15 mm keeps a margin and is ample to hand over or set
# down; the recorder accepts a release within 30 mm of the place pose.
MPC_POSITION_TOLERANCE_M = 0.015
# person_proxy.py HAND_REST: forward, to the hand's side, world z.
HAND_REST = (0.25, 0.18, 0.62)
# (name, default, description) of the episode's arguments.
ARGUMENTS = (
    ("execute", "false", "false = plan only (no motion); true = run in sim"),
    ("use_sim_time", "true", "Use the sim's clock"),
    ("scene", "small_house", "Scene in config/pick_place_scenes.yaml (the house: the chair)"),
    ("world_name", "", "<world name> of the running sim (empty = from scene)"),
    # false by default: in Gazebo (2026-11-25) Nav2 did not reach the route's first,
    # reversing goal in 120 s (docs/sim_handover_baseline.md, Known limitations).
    ("use_nav", "false", "true: Nav2 drives the legs (sim_bringup launch_nav:=true); false: "
                         "the scripted pose controller"),
    ("person_x", "0.504", "Torso axis x in odom (ChairA_01_001)"),
    ("person_y", "-0.858", "Torso axis y in odom (ChairA_01_001)"),
    ("facing_yaw", "0.0", "Facing (rad, odom) the person turns to once the robot left the table"),
    ("hand_extends", "true", "false: the person never reaches (the set-down case)"),
    ("reaction_delay", "[0.5, 1.0]", "[min, max] s of the person's reaction delays"),
    ("seed", "0", "Seed of the person's delays and of the measurement noise"),
    ("facing_change", "0.0", "rad added to the facing 1 s after the robot stood still nearby"),
    ("faults", "[" + ", ".join(["0.0"] * FAULT_COUNT) + "]",
     "[bias_x, bias_y, noise_std, dropout_start, dropout_sec, stale_start, stale_sec, "
     "latency_sec, jump_at, jump_x, jump_y, trigger_distance] of the measured person"),
    ("episode_id", "0", "Episode number, copied into the results row"),
    ("results_file", "", "CSV the recorder appends to (empty = ~/.ros/handover_results/...)"),
)
HAND_SIDES = ("left", "right")


def config_file() -> PathJoinSubstitution:
    """Return config/handover.yaml of the installed package."""
    return PathJoinSubstitution([FindPackageShare("ranger_sim"), "config", "handover.yaml"])


def handover_arguments() -> list[DeclareLaunchArgument]:
    """Return the arguments of handover_baseline.launch.py."""
    return [
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in ARGUMENTS),
        DeclareLaunchArgument("hand_side", default_value="left", choices=list(HAND_SIDES),
                              description="The hand the person extends"),
    ]


def _float(name: str) -> ParameterValue:
    # float(): facing_yaw:=1 must still give a double, not an int, to the node.
    return ParameterValue(PythonExpression(["float(", LaunchConfiguration(name), ")"]),
                          value_type=float)


def _float_list(name: str) -> ParameterValue:
    # float(): faults:="[0, 0.1, ...]" must still give doubles, not a mixed list.
    return ParameterValue(
        PythonExpression(["[float(v) for v in ", LaunchConfiguration(name), "]"]))


def _typed(name: str, value_type) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def world_name(scene):
    """Return the running sim's world: the world_name argument, else the scene's."""
    return scenes.SceneValue(scene, "world_name", override=LaunchConfiguration("world_name"))


def person_spawns(scene) -> list[Node]:
    """Spawn the person (facing the table) and its hand at rest, by `ros_gz_sim create`."""
    x, y = LaunchConfiguration("person_x"), LaunchConfiguration("person_y")
    side = PythonExpression([f"{HAND_REST[1]} if '", LaunchConfiguration("hand_side"),
                             f"' == 'left' else {-HAND_REST[1]}"])

    def create(model: str, position: list) -> Node:
        return Node(
            package="ros_gz_sim", executable="create", output="screen",
            arguments=[
                "-world", world_name(scene),
                "-file", PathJoinSubstitution(
                    [FindPackageShare("ranger_sim"), "models", f"{model}.sdf"]),
                "-name", model,
                "-x", position[0], "-y", position[1], "-z", position[2], "-Y", "0.0",
            ],
        )

    return [
        create(PERSON_MODEL, [x, y, "0.0"]),
        create(HAND_MODEL, [PythonExpression([x, f" + {HAND_REST[0]}"]),
                            PythonExpression([y, " + ", side]), str(HAND_REST[2])]),
    ]


def _episode_parameters() -> dict:
    return {
        "facing_yaw": _float("facing_yaw"),
        "hand_side": _typed("hand_side", str),
        "hand_extends": _typed("hand_extends", bool),
        "reaction_delay": _float_list("reaction_delay"),
        "seed": _typed("seed", int),
        "facing_change": _float("facing_change"),
        "faults": _float_list("faults"),
    }


def person_node(scene, use_sim_time) -> Node:
    """Return the scripted person node (ground truth and measured)."""
    return Node(
        package="ranger_sim", executable="person_proxy.py", name=PERSON_NODE, output="screen",
        parameters=[config_file(), {
            "use_sim_time": use_sim_time,
            "person_x": _float("person_x"), "person_y": _float("person_y"),
            **_episode_parameters(),
            "world_name": ParameterValue(world_name(scene), value_type=str),
            "object_name": scenes.from_scene(scene, "object_name", str),
        }],
    )


def safety_node(use_sim_time) -> Node:
    """Return the node of the hard limits between the baseline and the MPC, gate, Nav2."""
    return Node(package="ranger_sim", executable="handover_safety.py", name=SAFETY_NODE,
                output="screen", parameters=[config_file(), {"use_sim_time": use_sim_time}])


def baseline_node(scene, use_sim_time) -> Node:
    """Return the rule baseline; the pick's values come from the scene (pick_place_nodes)."""
    s = scenes.from_scene
    return Node(
        package="ranger_sim", executable="handover_baseline.py", name=BASELINE_NODE,
        output="screen",
        parameters=[config_file(), {
            "use_sim_time": use_sim_time,
            "execute": _typed("execute", bool),
            "use_nav": _typed("use_nav", bool),
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
        }],
    )


def recorder_node(scene, use_sim_time) -> Node:
    """Return the recorder: the ground-truth metrics of the episode, one CSV row."""
    return Node(
        package="ranger_sim", executable="handover_recorder.py", name=RECORDER_NODE,
        output="screen",
        parameters=[config_file(), {
            "use_sim_time": use_sim_time,
            **_episode_parameters(),
            "episode_id": _typed("episode_id", int),
            "results_file": _typed("results_file", str),
            "object_name": scenes.from_scene(scene, "object_name", str),
        }],
    )


def mpc_node(scene, use_sim_time) -> Node:
    """Return whole_body_mpc: the scene's box as the start box, the handover's tolerance."""
    return reach.mpc_node(use_sim_time, overrides={
        "base_box": scenes.from_scene(scene, "mpc_base_box"),
        "position_tolerance": MPC_POSITION_TOLERANCE_M,
    })


def handover_nodes(scene, use_sim_time) -> list[Node]:
    """[person, safety, MPC, baseline, recorder]."""
    return [person_node(scene, use_sim_time), safety_node(use_sim_time),
            mpc_node(scene, use_sim_time), baseline_node(scene, use_sim_time),
            recorder_node(scene, use_sim_time)]


def end_with_the_baseline(nodes: list[Node]) -> list[RegisterEventHandler]:
    """
    Shut the launch down when the baseline or handover_safety exits.

    whole_body_mpc, person_proxy and the recorder spin until stopped; and a
    baseline without its safety layer must not go on.
    """
    enders = {f"{name}.py" for name in (BASELINE_NODE, SAFETY_NODE)}
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[LogInfo(msg=f"{node.node_executable} exited: shutting down"),
                     EmitEvent(event=Shutdown(reason=f"{node.node_executable} exited"))],
        ))
        for node in nodes if node.node_executable in enders
    ]
