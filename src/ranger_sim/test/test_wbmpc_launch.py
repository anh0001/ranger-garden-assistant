"""
Launch wiring, controller config and experiment geometry of the whole-body MPC (no simulator).

Needs the install space sourced (FindPackageShare, xacro).
"""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
CONTROLLERS = PKG / "config" / "ros2_controllers_sim.yaml"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gate_mod = _load("base_handover_gate", PKG / "scripts" / "base_handover_gate.py")


def _entities(launch_name):
    module = _load(launch_name.replace(".", "_"), PKG / "launch" / launch_name)
    return module.generate_launch_description().entities


def _all_actions(entities):
    """Every entity, with the actions of timers and OnProcessExit handlers included."""
    for entity in entities:
        yield entity
        if isinstance(entity, TimerAction):
            yield from _all_actions(entity.actions)
        elif isinstance(entity, RegisterEventHandler):
            yield from _all_actions(entity.event_handler._OnActionEventBase__actions_on_event)


def _argument(entities, name):
    return next(e for e in entities if isinstance(e, DeclareLaunchArgument) and e.name == name)


def _node(entities, executable):
    return next(e for e in entities if isinstance(e, Node) and e.node_executable == executable)


def _context(**configurations):
    context = LaunchContext()
    context.launch_configurations.update(configurations)
    return context


def _parameter(node, name, context):
    """Resolve the launch_ros Node parameter `name` in `context`."""
    for params in node._Node__parameters:
        for key, value in params.items():
            if perform_substitutions(context, list(key)) == name:
                return perform_substitutions(context, list(value))
    raise KeyError(name)


# ---- handover gate (Task 3) ------------------------------------------------


@pytest.mark.parametrize("launch_name", ["spawn_robot.launch.py", "sim_bringup.launch.py"])
def test_launches_declare_handover_gate_off_by_default(launch_name):
    arg = _argument(_entities(launch_name), "handover_gate")

    assert arg.default_value[0].text == "false"


def test_default_spawn_keeps_the_adapter_on_cmd_vel_and_starts_no_gate():
    entities = _entities("spawn_robot.launch.py")
    context = _context(handover_gate="false", use_sim_time="true")

    topic = _parameter(_node(entities, "ranger_motion_adapter.py"), "input_topic", context)
    gate = _node(entities, "base_handover_gate.py")

    assert topic == "/cmd_vel"
    assert not gate.condition.evaluate(context)


@pytest.mark.parametrize("value", ["true", "True", "1"])
def test_gated_spawn_puts_the_gate_between_cmd_vel_and_the_adapter(value):
    entities = _entities("spawn_robot.launch.py")
    context = _context(handover_gate=value, use_sim_time="true")

    topic = _parameter(_node(entities, "ranger_motion_adapter.py"), "input_topic", context)
    gate = _node(entities, "base_handover_gate.py")

    assert topic == gate_mod.OUTPUT_TOPIC == "/cmd_vel_gated"
    assert gate_mod.NAV_TOPIC == "/cmd_vel"
    assert gate.condition.evaluate(context)


def test_sim_bringup_passes_handover_gate_to_the_spawn_launch():
    includes = [
        e for e in _all_actions(_entities("sim_bringup.launch.py"))
        if isinstance(e, IncludeLaunchDescription)
    ]
    passed = [
        dict((name, value) for name, value in include.launch_arguments)
        for include in includes
    ]
    spawn = next(arguments for arguments in passed if "grasp_detach_timeout" in arguments)

    assert "handover_gate" in spawn
    assert perform_substitutions(_context(handover_gate="true"), [spawn["handover_gate"]]) \
        == "true"


# ---- streamed arm setpoints (Task 5) ----------------------------------------


def _controllers():
    return yaml.safe_load(CONTROLLERS.read_text())


def test_stream_controller_drives_the_arm_joints_by_position():
    config = _controllers()
    declared = config["controller_manager"]["ros__parameters"]["piper_arm_stream_controller"]
    params = config["piper_arm_stream_controller"]["ros__parameters"]

    assert declared["type"] == "forward_command_controller/ForwardCommandController"
    assert params["interface_name"] == "position"
    assert params["joints"] == config["piper_arm_controller"]["ros__parameters"]["joints"]


def _literal_arguments(node):
    return [perform_substitutions(LaunchContext(), list(arg)) for arg in node.cmd
            if all(hasattr(s, "text") for s in arg)]


def test_spawn_loads_the_stream_controller_inactive():
    spawners = [
        e for e in _all_actions(_entities("spawn_robot.launch.py"))
        if isinstance(e, Node) and e.node_executable == "spawner"
    ]
    stream = [s for s in spawners if "piper_arm_stream_controller" in _literal_arguments(s)]

    assert len(stream) == 1
    assert "--inactive" in _literal_arguments(stream[0])


def test_moveit_keeps_seeing_only_the_trajectory_controller():
    moveit = Path(get_package_share_directory("ranger_piper_moveit"))
    text = "\n".join(p.read_text() for p in moveit.rglob("*controllers*.yaml"))

    assert "piper_arm_controller" in text
    assert "piper_arm_stream_controller" not in text


# ---- config, marker and experiment geometry (Task 10) ------------------------

WBMPC_CONFIG = PKG / "config" / "wbmpc.yaml"
MARKER = PKG / "models" / "wbmpc_target.sdf"
SCENES = yaml.safe_load((PKG / "config" / "pick_place_scenes.yaml").read_text())
# The experiment target in odom: the reach launch's target_x/y/z/yaw defaults.
TARGET = (0.95, 0.15, 1.00)
FOOTPRINT_HEIGHT_M = -0.010  # world z = odom z - 0.010 (test_pick_place_geometry.py)
PAD_HALF_HEIGHT_M = 0.028  # finger pads span +-this about the TCP, level gripper
OBJECT_CLEARANCE_RADIUS_M = 0.10  # no table object origin this close to the target
CHECK_RADIUS_M = 0.20  # table objects this close must stay under the pads
ABOVE_TABLE_M = 0.15
ARM_ONLY_SHORTFALL_M = 0.10


def _wbmpc(node_name):
    return yaml.safe_load(WBMPC_CONFIG.read_text())[node_name]["ros__parameters"]


def _geometry():
    return _load("test_pick_place_geometry", PKG / "test" / "test_pick_place_geometry.py")


def test_marker_is_a_visual_only_static_sphere():
    import xml.etree.ElementTree as ET

    model = ET.parse(MARKER).getroot().find("model")

    assert model.get("name") == "wbmpc_target"
    assert model.findtext("static").strip() == "true"
    assert model.find(".//collision") is None
    assert model.find(".//sensor") is None
    assert model.find(".//visual/geometry/sphere") is not None


INTEGER_PARAMETERS = {"horizon", "max_solver_failures"}


def test_config_floats_are_floats():
    # A ROS double parameter rejects a YAML integer at startup.
    for name in ("whole_body_mpc", "wbmpc_reach_demo", "base_handover_gate"):
        for key, value in _wbmpc(name).items():
            values = value if isinstance(value, list) else [value]
            if key in INTEGER_PARAMETERS:
                assert all(isinstance(v, int) for v in values), key
            else:
                assert all(isinstance(v, float) for v in values), key


def test_base_box_stays_short_of_every_scenes_approach():
    x_min, y_min, x_max, y_max = _wbmpc("whole_body_mpc")["base_box"]

    assert x_min < 0.0 < x_max and y_min < 0.0 < y_max
    assert all(x_max <= cfg["base_approach_x"] for cfg in SCENES.values())


def test_the_gate_never_scales_a_legal_mpc_command():
    base_speed = _wbmpc("whole_body_mpc")["base_speed"]
    legal = (base_speed ** 2 * 2) ** 0.5

    assert gate_mod.MANIP_MAX_SPEED_MPS >= legal
    assert _wbmpc("base_handover_gate")["manip_max_speed"] >= legal


def test_target_is_well_above_the_house_table():
    geometry = _geometry()
    table = geometry._surface(geometry._world_root("small_house_world.sdf"), "table")
    world_z = TARGET[2] + FOOTPRINT_HEIGHT_M

    assert table.contains(TARGET[0], TARGET[1])
    assert world_z - table.top_z >= ABOVE_TABLE_M


def _table_objects(geometry):
    """(name, x, y, top z) of every object resting on the house table."""
    root = geometry._world_root("small_house_world.sdf")
    table = geometry._surface(root, "table")
    world = root.find("world")
    objects = []
    for element in world:
        if element.tag not in ("model", "include"):
            continue
        name = element.get("name") or element.findtext("name")
        x, y, z = geometry._xyz(element)
        if abs(z - table.top_z) > 0.01 or not table.contains(x, y):
            continue
        objects.append((name, x, y, element))
    return root, objects


def test_no_table_object_stands_at_the_target():
    _, objects = _table_objects(_geometry())

    assert objects, "the house table has no objects: the world changed"
    for name, x, y, _ in objects:
        distance = ((x - TARGET[0]) ** 2 + (y - TARGET[1]) ** 2) ** 0.5
        assert distance > OBJECT_CLEARANCE_RADIUS_M, name


def test_finger_pads_pass_above_every_nearby_table_object():
    geometry = _geometry()
    root, objects = _table_objects(geometry)
    pads_bottom = TARGET[2] + FOOTPRINT_HEIGHT_M - PAD_HALF_HEIGHT_M

    near = [o for o in objects
            if ((o[1] - TARGET[0]) ** 2 + (o[2] - TARGET[1]) ** 2) ** 0.5 <= CHECK_RADIUS_M]
    assert near, "expected the bottle and the jar near the target"
    for name, *_ in near:
        try:
            _, _, centre_z, _, height = geometry._object(root, name)
        except Exception as exc:  # noqa: BLE001 - name the model whose size is unknown
            pytest.fail(f"cannot read the size of '{name}': {exc}")
        assert pads_bottom > centre_z + height / 2, name


def _level_reach(model, core, y, z):
    """Largest TCP x (base frame) of a level gripper at (y, z) within the joint limits."""
    import casadi as ca

    best = -1.0
    for seed in ([0.0, 0.719, -1.496, 0.0, 0.864, 0.0], [0.0, 1.5, -1.0, 0.0, 0.0, 0.0]):
        opti = ca.Opti()
        q = opti.variable(6)
        position, rotation = model.fk(q)
        opti.minimize(-position[0])
        opti.subject_to(position[1] == y)
        opti.subject_to(position[2] == z)
        opti.subject_to(ca.sumsqr(rotation - core.LEVEL_ROTATION) <= 1e-6)
        opti.subject_to(opti.bounded(model.lower, q, model.upper))
        opti.set_initial(q, seed)
        opti.solver("ipopt", {"print_time": False}, {"print_level": 0})
        try:
            best = max(best, float(opti.solve().value(position[0])))
        except RuntimeError:
            continue
    return best


def test_the_target_needs_the_base_and_the_box_allows_it():
    pytest.importorskip("casadi")
    import xacro

    core = _load("whole_body_mpc_core", PKG / "scripts" / "whole_body_mpc_core.py")
    urdf = xacro.process_file(str(PKG / "urdf" / "ranger_gazebo.urdf.xacro")).toxml()
    model = core.build_arm_model(urdf)
    x_min, y_min, x_max, y_max = _wbmpc("whole_body_mpc")["base_box"]

    from_spawn = _level_reach(model, core, TARGET[1], TARGET[2])
    from_box_edge = _level_reach(model, core, 0.0, TARGET[2])

    assert TARGET[0] - from_spawn >= ARM_ONLY_SHORTFALL_M
    assert y_min <= TARGET[1] <= y_max
    assert TARGET[0] - x_max <= from_box_edge


# ---- reach launch (Task 12) -------------------------------------------------------

REACH_ARGUMENTS = {"target_x", "target_y", "target_z", "target_yaw", "use_nav"}


def _evaluated(node, context):
    """Return the node's parameters (dicts and parameter files) resolved in `context`."""
    from launch_ros.utilities import evaluate_parameters

    merged = {}
    for entry in evaluate_parameters(context, node._Node__parameters):
        if isinstance(entry, dict):
            merged.update(entry)
    return merged


def _defaults(entities):
    return {
        e.name: perform_substitutions(LaunchContext(), e.default_value)
        for e in entities if isinstance(e, DeclareLaunchArgument)
    }


def test_reach_launch_declares_its_arguments_and_plans_only_by_default():
    defaults = _defaults(_entities("wbmpc_reach_demo.launch.py"))

    assert set(defaults) == REACH_ARGUMENTS | {"execute", "use_sim_time"}
    assert defaults["execute"] == "false"
    assert defaults["use_nav"] == "false"
    assert tuple(float(defaults[n]) for n in ("target_x", "target_y", "target_z")) == TARGET
    assert float(defaults["target_yaw"]) == 0.0


def test_reach_nodes_carry_the_names_the_config_uses():
    entities = _entities("wbmpc_reach_demo.launch.py")
    config = yaml.safe_load(WBMPC_CONFIG.read_text())

    for executable, name in (("whole_body_mpc.py", "whole_body_mpc"),
                             ("wbmpc_reach_demo.py", "wbmpc_reach_demo")):
        node = _node(entities, executable)
        assert node._Node__node_name == name
        assert name in config


def test_reach_demo_gets_the_target_as_floats():
    entities = _entities("wbmpc_reach_demo.launch.py")
    context = _context(**{**_defaults(entities), "target_x": "1", "execute": "true"})

    params = _evaluated(_node(entities, "wbmpc_reach_demo.py"), context)

    assert params["target_position"] == [1.0, 0.15, 1.0]
    assert params["target_yaw"] == 0.0
    assert params["execute"] is True


def test_reach_launch_ends_with_the_demo():
    from launch.event_handlers import OnProcessExit

    handlers = [
        e.event_handler for e in _entities("wbmpc_reach_demo.launch.py")
        if isinstance(e, RegisterEventHandler) and isinstance(e.event_handler, OnProcessExit)
    ]

    assert len(handlers) == 1


# ---- record launch (Task 13) ------------------------------------------------------

RECORD_LAUNCH = "record_wbmpc_reach_video.launch.py"


def _record_module():
    return _load("record_wbmpc_reach_video_launch", PKG / "launch" / RECORD_LAUNCH)


def _recorder():
    return _load("record_demo_video", PKG / "scripts" / "record_demo_video.py")


def _cmd(node, context):
    return [perform_substitutions(context, list(arg)) for arg in node.cmd]


def test_record_launch_declares_its_arguments():
    defaults = _defaults(_entities(RECORD_LAUNCH))

    assert set(defaults) == REACH_ARGUMENTS | {
        "scene", "world_name", "output_path", "first_frame_timeout"}
    assert defaults["scene"] == "small_house"


def test_record_launch_starts_neither_the_mpc_nor_the_demo_up_front():
    # They move the robot: only the first camera frame (a running sim) starts them.
    executables = {e.node_executable for e in _entities(RECORD_LAUNCH) if isinstance(e, Node)}

    assert "whole_body_mpc.py" not in executables
    assert "wbmpc_reach_demo.py" not in executables
    assert "record_demo_video.py" in executables


def test_record_launch_waits_for_the_recorders_ready_line():
    assert _record_module().READY_MARKER == _recorder().READY_MARKER


def test_record_gate_fires_once_on_the_ready_line():
    from types import SimpleNamespace

    module = _record_module()
    gate = module.make_demo_gate(["demo"])
    ready = SimpleNamespace(text=f"[INFO] {module.READY_MARKER}: /x.mp4\n".encode())

    assert gate(SimpleNamespace(text=b"[INFO] waiting for the first frame")) is None
    assert gate(ready) == ["demo"]
    assert gate(ready) is None


def test_recorder_captions_follow_the_demo():
    entities = _entities(RECORD_LAUNCH)
    context = _context(**_defaults(entities))

    params = _evaluated(_node(entities, "record_demo_video.py"), context)

    assert params["caption_node"] == "wbmpc_reach_demo"
    assert params["grasp_object"] == SCENES["small_house"]["object_name"]


@pytest.mark.parametrize("use_nav, nav_in_title", [("true", True), ("false", False)])
def test_video_title_names_nav2_only_when_it_drives(use_nav, nav_in_title):
    entities = _entities(RECORD_LAUNCH)
    context = _context(**{**_defaults(entities), "use_nav": use_nav})

    title = _evaluated(_node(entities, "record_demo_video.py"), context)["title"]

    assert "whole-body MPC" in title
    assert ("Nav2" in title) is nav_in_title


def test_marker_is_spawned_at_the_target_in_world_z():
    entities = _entities(RECORD_LAUNCH)
    context = _context(**_defaults(entities))
    creates = [e for e in entities if isinstance(e, Node) and e.node_executable == "create"]

    (marker,) = [c for c in creates if "wbmpc_target" in _cmd(c, context)]
    cmd = _cmd(marker, context)

    position = [float(cmd[cmd.index(flag) + 1]) for flag in ("-x", "-y", "-z")]
    assert position == pytest.approx([TARGET[0], TARGET[1], TARGET[2] + FOOTPRINT_HEIGHT_M])
    assert cmd[cmd.index("-file") + 1].endswith("models/wbmpc_target.sdf")
