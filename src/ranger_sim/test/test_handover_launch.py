"""
Launch wiring of the handover baseline (no simulator).

Checks launch/handover_baseline.launch.py and launch/handover_common.py
against config/handover.yaml and the nodes: arguments and defaults, node
names = YAML sections, every YAML key declared by its node, typed launch
parameters, the two person spawns, the start box of the MPC, and the
shutdown with the baseline. Needs the install space sourced (FindPackageShare).
"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest
import rclpy
from rclpy.parameter import Parameter
import yaml

PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG / "config" / "handover.yaml"
SCENES = yaml.safe_load((PKG / "config" / "pick_place_scenes.yaml").read_text())
LAUNCH = "handover_baseline.launch.py"
NODES = {
    "person_proxy.py": "person_proxy", "handover_safety.py": "handover_safety",
    "handover_baseline.py": "handover_baseline", "handover_recorder.py": "handover_recorder",
}
DEFAULTS = {
    "execute": "false", "use_sim_time": "true", "scene": "small_house", "world_name": "",
    "use_nav": "false", "person_x": "0.504", "person_y": "-0.858", "facing_yaw": "0.0",
    "hand_side": "left", "hand_extends": "true", "reaction_delay": "[0.5, 1.0]", "seed": "0",
    "facing_change": "0.0", "episode_id": "0", "results_file": "",
}


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _entities():
    module = _load("handover_baseline_launch", PKG / "launch" / LAUNCH)
    return module.generate_launch_description().entities


def _defaults(entities):
    return {e.name: perform_substitutions(LaunchContext(), e.default_value)
            for e in entities if isinstance(e, DeclareLaunchArgument)}


def _context(**configurations):
    context = LaunchContext()
    context.launch_configurations.update(configurations)
    return context


def _node(entities, executable):
    return next(e for e in entities if isinstance(e, Node) and e.node_executable == executable)


def _evaluated(node, context):
    """Return the node's parameters: dicts resolved in `context`, files as their paths."""
    from launch_ros.utilities import evaluate_parameters

    merged, files = {}, []
    for entry in evaluate_parameters(context, node._Node__parameters):
        if isinstance(entry, dict):
            merged.update(entry)
        else:
            files.append(str(entry))
    return merged, files


def _cmd(node, context):
    return [perform_substitutions(context, list(arg)) for arg in node.cmd]


def test_the_launch_declares_its_arguments_and_plans_only_by_default():
    defaults = _defaults(_entities())

    assert {k: v for k, v in defaults.items() if k != "faults"} == DEFAULTS
    assert [float(v) for v in yaml.safe_load(defaults["faults"])] == [0.0] * 12


def test_the_hand_side_is_left_or_right():
    entities = _entities()
    argument = next(e for e in entities
                    if isinstance(e, DeclareLaunchArgument) and e.name == "hand_side")

    assert argument.choices == ["left", "right"]


def test_the_nodes_carry_the_names_the_config_uses():
    entities = _entities()
    sections = set(yaml.safe_load(CONFIG.read_text())) - {"/**"}

    for executable, name in NODES.items():
        assert _node(entities, executable)._Node__node_name == name
    assert sections == set(NODES.values())


@pytest.mark.parametrize("executable", sorted(NODES))
def test_each_node_gets_the_config_and_sim_time(executable):
    entities = _entities()
    context = _context(**{**_defaults(entities), "use_sim_time": "true"})

    params, files = _evaluated(_node(entities, executable), context)

    assert any(f.endswith("config/handover.yaml") for f in files)
    assert params["use_sim_time"] is True


def test_launch_values_reach_the_nodes_typed():
    entities = _entities()
    context = _context(**{**_defaults(entities), "facing_yaw": "1", "seed": "7",
                          "faults": "[0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]",
                          "hand_extends": "false", "execute": "true"})

    person, _ = _evaluated(_node(entities, "person_proxy.py"), context)
    recorder, _ = _evaluated(_node(entities, "handover_recorder.py"), context)
    baseline, _ = _evaluated(_node(entities, "handover_baseline.py"), context)

    for params in (person, recorder):
        assert params["facing_yaw"] == 1.0 and isinstance(params["facing_yaw"], float)
        assert params["seed"] == 7
        assert params["faults"] == [0.0, 0.1] + [0.0] * 9 + [1.0]
        assert params["hand_extends"] is False
        assert params["object_name"] == "vitamin_bottle"
    assert person["world_name"] == "small_house"
    assert baseline["execute"] is True and baseline["use_nav"] is False
    assert baseline["base_approach_x"] == SCENES["small_house"]["base_approach_x"]


def test_the_mpc_starts_with_the_scenes_box():
    entities = _entities()
    context = _context(**_defaults(entities))

    params, files = _evaluated(_node(entities, "whole_body_mpc.py"), context)

    assert params["base_box"] == SCENES["small_house"]["mpc_base_box"]
    assert params["position_tolerance"] == 0.015
    assert any(f.endswith("config/wbmpc.yaml") for f in files)


def test_the_person_and_its_hand_are_spawned_where_the_node_moves_them():
    entities = _entities()
    context = _context(**{**_defaults(entities), "hand_side": "right"})
    creates = [_cmd(e, context) for e in entities
               if isinstance(e, Node) and e.node_executable == "create"]

    names = {cmd[cmd.index("-name") + 1]: cmd for cmd in creates}
    assert set(names) == {"person_proxy", "person_hand"}
    for name, cmd in names.items():
        assert cmd[cmd.index("-file") + 1].endswith(f"models/{name}.sdf")
        assert cmd[cmd.index("-world") + 1] == "small_house"
    hand = names["person_hand"]
    xyz = [float(hand[hand.index(flag) + 1]) for flag in ("-x", "-y", "-z")]
    assert xyz == pytest.approx([0.504 + 0.25, -0.858 - 0.18, 0.62])


def test_the_launch_ends_with_the_baseline_or_the_safety_node():
    handlers = [e.event_handler for e in _entities() if isinstance(e, RegisterEventHandler)
                and isinstance(e.event_handler, OnProcessExit)]

    targets = {h._OnActionEventBase__action_matcher.node_executable for h in handlers}
    assert targets == {"handover_baseline.py", "handover_safety.py"}


# ---- every YAML key is a parameter its node declares ------------------------------------


def _module(script):
    return _load(script[:-3], PKG / "scripts" / script)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.mark.parametrize("script, class_name", [
    ("person_proxy.py", "PersonProxy"), ("handover_safety.py", "HandoverSafety"),
    ("handover_baseline.py", "HandoverBaseline"), ("handover_recorder.py", "HandoverRecorder"),
])
def test_every_key_of_the_yaml_is_declared_by_its_node(ros, script, class_name):
    config = yaml.safe_load(CONFIG.read_text())
    name = NODES[script]
    values = {**config["/**"]["ros__parameters"], **config[name]["ros__parameters"]}
    overrides = [Parameter(key, value=value) for key, value in values.items()]

    node = getattr(_module(script), class_name)(parameter_overrides=overrides)
    try:
        undeclared = sorted(key for key in values if not node.has_parameter(key))
        assert undeclared == []
        for key, value in values.items():
            assert node.get_parameter(key).value == value, key
    finally:
        node.destroy_node()
