"""
Unit tests for scripts/wbmpc_pick_place_demo.py (no simulator, no MoveIt, no MPC).

The pick-and-place transfer with the pick done by the whole-body MPC: step
order, the three MPC targets per scene, hand-over and hand-back around the
pick, the failure and Ctrl-C paths (restore), approach_only and plan-only.
Step actions are replaced by recorders where the order is under test.
"""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.parameter import Parameter
import yaml

PKG = Path(__file__).resolve().parents[1]
SCENES = yaml.safe_load((PKG / "config" / "pick_place_scenes.yaml").read_text())
START = (0.0, 0.0, 0.0)
VIDEO_CAPTION_MAX_CHARS = 60  # scripts/record_demo_video.py CAPTION_MAX_CHARS


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


mpc_pick = _load("wbmpc_pick_place_demo", PKG / "scripts" / "wbmpc_pick_place_demo.py")
scenes = _load("pick_place_scenes", PKG / "launch" / "pick_place_scenes.py")
pick = mpc_pick.pick

PREPARE = ["release object", "open gripper", "'home'", "carry pose"]
MPC_PICK = [
    "drive to the table", "base to the whole-body MPC", "arm to streamed setpoints",
    "MPC: pre-grasp", "MPC: slide in", "attach object", "close gripper", "MPC: lift",
    "arm back to MoveIt", "base back to navigation", "carry pose",
]
MPC_PUT_BACK = [
    "MPC: set down", "open gripper", "release object", "MPC: back off",
    "arm back to MoveIt", "base back to navigation", "carry pose",
]
FINISH = ["return to start", "retract 'home'"]
# Design targets in odom: grasp = (base_approach_x + grasp_tcp x, 0, grasp_tcp z),
# pre-grasp standoff_m (0.15) back along the pick heading, lift up to hover_tcp z.
TARGETS = {
    "small_house": ((0.815, 0.0, 0.871), (0.965, 0.0, 0.871), (0.965, 0.0, 0.962)),
    "garden": ((0.800, 0.0, 0.842), (0.950, 0.0, 0.842), (0.950, 0.0, 0.935)),
}


def _scene_parameters(name):
    keys = ("object_name", "base_approach_x", "gripper_closed", "hover_joints",
            "grasp_joints", "place_hover_joints", "place_grasp_joints", "standoff_m",
            "place_base_pose", "place_route", "grasp_tcp", "hover_tcp")
    return {key: scenes.scene_value(SCENES[name], key) for key in keys}


@pytest.fixture
def make_node():
    rclpy.init()
    nodes = []

    def make(**parameters):
        overrides = [Parameter(name, value=value) for name, value in parameters.items()]
        nodes.append(mpc_pick.WbmpcPickPlaceDemo(parameter_overrides=overrides))
        return nodes[-1]

    yield make
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def recorded(make_node, monkeypatch):
    """Return a factory of (node, calls): nodes whose actions only record (kind, label, value)."""

    def make(scene="small_house", fail_at=None, raise_at=None, **parameters):
        node = make_node(**{**_scene_parameters(scene), **parameters})
        calls = []

        def record(kind, label, value=None):
            calls.append((kind, label, value))
            if len(calls) == raise_at:
                raise KeyboardInterrupt
            return len(calls) != fail_at

        patch = monkeypatch.setattr
        patch(node, "_wait_servers", lambda: None)
        patch(node, "_wait_reach_servers", lambda: None)
        patch(node, "_wait_for_odom_pose", lambda: START)
        patch(node, "move_to_joint_state", lambda label, q: record("arm", label, tuple(q)))
        patch(node, "drive_to_pose",
              lambda target, label, speed=None: record("drive", label, (target, speed)))
        patch(node, "set_gripper", lambda position, label: record("gripper", label))
        patch(node, "grasp", lambda attach: record("grasp", attach))
        patch(node, "set_base_owner", lambda manipulate: record("base", manipulate))
        patch(node, "switch_arm", lambda stream, strict=True: record("stream", stream))
        patch(node, "reach_pose",
              lambda position, yaw, label: record("reach", label, (position, yaw)))
        patch(node, "slide", lambda start, end, yaw, speed, label: record(
            "slide", label, (start, end, yaw, speed)))
        patch(node, "stop_base", lambda: record("stop", None))
        return node, calls

    return make


def _captions(node):
    return [caption for caption, _ in node.sequence(node.transfer_plan(), START)]


def _sequential_captions(scene):
    """Captions of the scripted pick_place_demo for `scene` (the transfer the MPC pick joins)."""
    node = pick.PickPlaceDemo(parameter_overrides=[
        Parameter(k, value=v) for k, v in _scene_parameters(scene).items()])
    try:
        return [caption for caption, _ in node.sequence(node.transfer_plan(), START)]
    finally:
        node.destroy_node()


# ---- node and step order ----------------------------------------------------------


def test_the_node_is_named_pick_place_demo(make_node):
    # The video recorder takes its captions from the logs of this node name.
    assert make_node().get_name() == "pick_place_demo"


@pytest.mark.parametrize("scene", sorted(SCENES))
def test_the_mpc_pick_replaces_only_the_scripted_pick(recorded, scene):
    node, _ = recorded(scene)
    sequential = _sequential_captions(scene)
    scripted_pick = len(PREPARE) + 7  # drive, look, approach, attach, close, lift, carry

    assert _captions(node) == PREPARE + MPC_PICK + sequential[scripted_pick:]


def test_approach_only_sets_the_object_back_with_the_mpc(recorded):
    node, _ = recorded(approach_only=True)

    assert _captions(node) == PREPARE + MPC_PICK[:-3] + MPC_PUT_BACK + FINISH


@pytest.mark.parametrize("approach_only", [False, True])
def test_captions_fit_the_video(recorded, approach_only):
    node, _ = recorded(approach_only=approach_only)

    assert all(c.isascii() and len(c) <= VIDEO_CAPTION_MAX_CHARS for c in _captions(node))


# ---- targets ---------------------------------------------------------------------


@pytest.mark.parametrize("scene", sorted(SCENES))
def test_the_mpc_targets_follow_the_scene(recorded, scene):
    node, calls = recorded(scene)
    node.run()

    pre_grasp, grasp, lift = TARGETS[scene]
    targets = {label: value for kind, label, value in calls if kind in ("reach", "slide")}
    assert targets["MPC: pre-grasp"] == (pytest.approx(pre_grasp), 0.0)
    speed = mpc_pick.SLIDE_SPEED_MPS
    assert targets["MPC: slide in"] == (pytest.approx(pre_grasp), pytest.approx(grasp), 0.0,
                                        speed)
    assert targets["MPC: lift"] == (pytest.approx(grasp), pytest.approx(lift), 0.0, speed)


def test_pick_targets_move_back_along_a_turned_pick_heading():
    plan = SimpleNamespace(pick=(1.0, 2.0, math.pi / 2), pick_standoff=(1.0, 1.85, math.pi / 2),
                           grasp_target=(1.0, 2.43, 0.871), hover_tcp_z=0.962)

    targets = mpc_pick.pick_targets(plan)

    assert targets.pre_grasp == pytest.approx((1.0, 2.28, 0.871))
    assert targets.lift == pytest.approx((1.0, 2.43, 0.962))


def test_put_back_reverses_the_lift_and_the_slide(recorded):
    node, calls = recorded(approach_only=True)
    node.run()

    pre_grasp, grasp, lift = TARGETS["small_house"]
    slides = {label: value[:2] for kind, label, value in calls if kind == "slide"}
    assert slides["MPC: set down"] == (pytest.approx(lift), pytest.approx(grasp))
    assert slides["MPC: back off"] == (pytest.approx(grasp), pytest.approx(pre_grasp))


def test_an_approach_error_moves_only_the_first_leg(recorded):
    # The MPC picks from wherever the base stopped: no corrective leg.
    node, calls = recorded(approach_error=[-0.08, 0.06, 0.07])
    node.run()

    plan = node.transfer_plan()
    drives = [(label, value[0]) for kind, label, value in calls if kind == "drive"]
    assert drives[0] == ("drive to the table", plan.pick_start)
    assert plan.pick_start != plan.pick_standoff
    assert "correct the standoff" not in _captions(node)
    assert all(pose != plan.pick_standoff for _, pose in drives)


# ---- order of hand-over, grasp and hand-back -------------------------------------


def _events(calls):
    return [(kind, label) for kind, label, _ in calls]


def _with_tcp(node):
    """Fake the TCP lookup: 1.9 mm short of the house grasp target (no TF here)."""
    node._tcp_in_odom = lambda: (0.9631, 0.0002, 0.8707)
    return node


def test_hand_over_follows_the_base_leg_and_hand_back_precedes_moveit(recorded):
    node, calls = recorded(execute=True)
    _with_tcp(node).run()

    events = _events(calls)
    first_leg = events.index(("drive", "drive to the table"))
    assert events[first_leg + 1:first_leg + 3] == [("base", True), ("stream", True)]
    lift = events.index(("slide", "MPC: lift"))
    assert events[lift + 1:lift + 4] == [
        ("stream", False), ("base", False), ("arm", "carry pose")]


def test_attach_comes_after_the_slide_and_before_the_gripper_closes(recorded):
    node, calls = recorded()
    node.run()

    events = _events(calls)
    slide_in = events.index(("slide", "MPC: slide in"))
    assert events[slide_in + 1:slide_in + 3] == [("grasp", True), ("gripper", "close")]


# Calls up to the slide in: 4 prepare steps, the leg, base, stream, pre-grasp, slide.
SLIDE_IN_CALL = 9


def test_a_failed_slide_aborts_before_the_attach_and_restores(recorded):
    node, calls = recorded(execute=True, fail_at=SLIDE_IN_CALL)
    node.run()

    assert calls[SLIDE_IN_CALL - 1][:2] == ("slide", "MPC: slide in")
    assert ("grasp", True, None) not in calls
    assert calls[-3:] == [("base", False, None), ("stream", False, None), ("stop", None, None)]


def test_ctrl_c_during_the_slide_still_restores(recorded):
    node, calls = recorded(execute=True, raise_at=SLIDE_IN_CALL)

    with pytest.raises(KeyboardInterrupt):
        node.run()

    assert calls[-3:] == [("base", False, None), ("stream", False, None), ("stop", None, None)]


def test_run_reports_whether_the_sequence_completed(recorded):
    done, _ = recorded(execute=True)
    failed, _ = recorded(execute=True, fail_at=6)

    assert _with_tcp(done).run() is True
    assert failed.run() is False


def test_the_result_line_says_mpc(recorded, monkeypatch):
    node, _ = recorded(execute=True)
    lines = []
    logger = node.get_logger()
    monkeypatch.setattr(node, "get_logger", lambda: SimpleNamespace(
        info=lines.append, warn=logger.warn, error=logger.error))

    _with_tcp(node).run()

    (result,) = [line for line in lines if line.startswith("approach result:")]
    assert result.startswith("approach result: mode=mpc ")
    assert result.endswith("aligned=yes")


def test_bad_values_fail_before_any_wait(recorded, monkeypatch):
    node, calls = recorded(approach_error=[0.0, 0.1])
    monkeypatch.setattr(node, "_wait_servers", lambda: pytest.fail("waited"))

    with pytest.raises(ValueError, match="approach_error"):
        node.run()
    assert calls == []


# ---- plan-only and main ---------------------------------------------------------


@pytest.mark.parametrize("approach_only", [False, True])
def test_plan_only_run_commands_nothing(make_node, monkeypatch, approach_only):
    node = make_node(**_scene_parameters("small_house"), execute=False,
                     approach_error=[-0.08, 0.06, 0.07], approach_only=approach_only)
    planned = []

    def forbidden(*_args, **_kwargs):
        raise AssertionError("plan-only mode must not command anything")

    patch = monkeypatch.setattr
    patch(node, "_wait_servers", lambda: None)
    patch(node, "_wait_reach_servers", forbidden)
    patch(node, "_wait_for_odom_pose", lambda: None)  # no sim at all
    patch(node, "_cmd_vel", SimpleNamespace(publish=forbidden))
    patch(node, "_target_pub", SimpleNamespace(publish=forbidden))
    patch(node, "_tcp_in_odom", forbidden)
    for client in (node._gate_client, node._switch_client, node._grasp):
        patch(client, "call_async", forbidden)
    for action in (node._gripper, node._nav):
        patch(action, "send_goal_async", forbidden)
    patch(node, "move_to_joint_state", lambda label, q: planned.append(label) or True)

    assert node.run() is True
    assert planned[-1] == "retract 'home'"


class _FakeDemo:
    """Stands in for WbmpcPickPlaceDemo in main()."""

    destroyed = []

    def run(self):
        raise RuntimeError("no /move_action")

    def get_logger(self):
        return SimpleNamespace(info=lambda *_: None, warn=lambda *_: None,
                               error=lambda *_: None)

    def destroy_node(self):
        self.destroyed.append(True)


def test_main_runs_the_mpc_pick_demo_and_exits_non_zero_on_failure(monkeypatch):
    fake = type("Fake", (_FakeDemo,), {"destroyed": []})
    monkeypatch.setattr(mpc_pick, "WbmpcPickPlaceDemo", fake)

    with pytest.raises(SystemExit) as stopped:
        mpc_pick.main()

    assert stopped.value.code == 1
    assert fake.destroyed == [True]
