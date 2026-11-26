"""
Unit tests for scripts/handover_baseline.py (no simulator, no MoveIt, no MPC).

The motion and service actions are replaced by recorders; a fake world
supplies the node clock and the measured person, advancing on every
spin_once. Covers the episode's branches (hand, no hand, offer-only
standoff, facing change, re-choice limit, take timeout), the hand-over
order, the release permit, plan-only, the bypass guard (no publisher on a
governed topic), the failure path and the injected decisions.
"""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.parameter import Parameter

PKG = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("handover_baseline",
                                               PKG / "scripts" / "handover_baseline.py")
baseline = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(baseline)
core = baseline.core
CFG = core.default_config()

PERSON_XY = (0.504, -0.858)
SPIN_SEC = 0.05
PICK_PREFIX = [
    "release object", "open gripper", "'home'", "carry pose", "drive to the table",
    "look at the object", "approach the object", "attach object", "close gripper", "lift",
    "carry pose",
]
APPROACH = ["MPC base box", "base to the whole-body MPC", "arm to streamed setpoints"]
RETREAT = ["MPC: back off", "arm back to MoveIt", "base back to navigation", "carry pose",
           "back to the pre-standoff", "drive away"]
GOVERNED_TOPICS = {"/whole_body_mpc/target", "/whole_body_mpc/speed_scale",
                   "/base_handover_gate/speed_scale", "/speed_limit"}


class _World:
    """Node clock and measured person; spin_once advances it."""

    def __init__(self, facing_deg=0.0, extend_at=None, take_at=None):
        self.t = 0.0
        self.facing = math.radians(facing_deg)
        self.extend_at, self.take_at = extend_at, take_at
        self.on_spin = []

    def spin(self, *args, **kwargs):
        self.t = round(self.t + SPIN_SEC, 6)
        for hook in self.on_spin:
            hook(self)

    def person(self):
        extended = self.extend_at is not None and self.t >= self.extend_at
        take = self.take_at is not None and self.t >= self.take_at
        return core.PersonState(*PERSON_XY, self.facing, (0.0, 0.0, 0.0), extended, take, self.t)


@pytest.fixture
def make_node():
    rclpy.init()
    nodes = []

    def make(decisions=None, **parameters):
        overrides = [Parameter(name, value=value) for name, value in parameters.items()]
        nodes.append(baseline.HandoverBaseline(decisions, parameter_overrides=overrides))
        return nodes[-1]

    yield make
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def recorded(make_node, monkeypatch):
    """Return a factory of (node, calls, world): executing nodes whose actions only record."""

    def make(world=None, fail=(), decisions=None, permit=True, **parameters):
        world = world or _World()
        node = make_node(decisions, execute=True, **parameters)
        calls = []

        def record(kind, value=None):
            calls.append((kind, value))
            return (kind, value) not in fail and kind not in fail

        def take_now(w):
            if w.take_at is not None and w.t >= w.take_at:
                node._take = True

        world.on_spin.append(take_now)
        monkeypatch.setattr(baseline.rclpy, "spin_once", world.spin)
        monkeypatch.setattr(node, "_clock_sec", lambda: world.t)
        monkeypatch.setattr(node, "person", world.person)
        monkeypatch.setattr(node, "_wait_servers", lambda: None)
        monkeypatch.setattr(node, "_wait_reach_servers", lambda: None)
        monkeypatch.setattr(node, "_log_approach", lambda plan: None)
        monkeypatch.setattr(node, "move_to_joint_state", lambda label, q: record("arm", label))
        monkeypatch.setattr(node, "set_gripper", lambda position, label: record("gripper", label))
        monkeypatch.setattr(node, "grasp", lambda attach: record("grasp", attach))
        monkeypatch.setattr(node, "drive_to_pose",
                            lambda pose, label, speed=None: record("drive", label))
        monkeypatch.setattr(node, "navigate", lambda pose, label: record("nav", label))
        monkeypatch.setattr(node, "set_base_owner", lambda manipulate: record("base", manipulate))
        monkeypatch.setattr(node, "switch_arm",
                            lambda stream, strict=True: record("stream", stream))
        monkeypatch.setattr(node, "reach_pose",
                            lambda position, yaw, label: record("target", label))
        monkeypatch.setattr(node, "slide",
                            lambda start, end, yaw, speed, label: record("slide", label))
        monkeypatch.setattr(node, "stop_base", lambda: record("stop"))
        monkeypatch.setattr(node, "_cancel_nav_goal", lambda: None)
        real_box = node.publish_box
        monkeypatch.setattr(node, "publish_box",
                            lambda standoff: record("box", standoff.box) and real_box(standoff))
        answer = SimpleNamespace(success=permit, message="permit" if permit else "no take")
        monkeypatch.setattr(node, "_call", lambda client, request, label: (
            record("permit", label) and answer))
        phases = []
        node._phase_pub = SimpleNamespace(publish=lambda msg: phases.append(msg.data))
        node.phases = phases
        return node, calls, world

    return make


def _kinds(calls):
    return [kind for kind, _ in calls]


# ---- episodes ---------------------------------------------------------------------------


def test_an_extended_hand_gets_the_object_handed_over(recorded):
    node, calls, _ = recorded(_World(extend_at=1.0, take_at=5.0))

    assert node.run()

    assert node.captions == [*PICK_PREFIX, "drive away", "drive to the pre-standoff", *APPROACH,
                             "MPC: offer", "release", *RETREAT]
    assert node.phases == ["PICK", "AWAY", "TRANSIT", "APPROACH", "OFFER", "WAIT", "RELEASE",
                           "RETREAT", "DONE: handover"]
    standoff = node._episode.standoff
    assert (round(math.degrees(standoff.bearing)), standoff.distance) == (-60, 0.7)


def test_no_hand_sets_the_object_down_from_a_standoff_that_reaches_the_table(recorded):
    node, calls, world = recorded(_World())

    assert node.run()

    assert "MPC: set down" in node.captions and "MPC: offer" not in node.captions
    assert node.phases[-1] == "DONE: set_down"
    assert node._episode.waited_sec == pytest.approx(CFG.hand_wait_sec, abs=SPIN_SEC)
    assert node._episode.rechoices == 0


def test_no_hand_at_an_offer_only_standoff_rechooses_for_the_table(recorded):
    node, calls, _ = recorded(_World(facing_deg=90.0))

    assert node.run()

    # Facing away from the table: no standoff reaches it, so the re-choice finds none.
    assert node._episode.rechoices == 1
    assert node._episode.need_place
    assert node.phases[-1] == "DONE: no_standoff"
    assert node.captions == [*PICK_PREFIX, "drive away", "drive to the pre-standoff"]


class _Scripted:
    """Decisions that pick an offer-only standoff first, then one that reaches the table."""

    def __init__(self):
        self.calls = []

    def choose_standoff(self, person, options, need_place):
        self.calls.append(("choose", need_place))
        wanted = [s for s in options if s.offer_ok and s.place_ok == need_place]
        return wanted[0] if wanted else None

    def request_scale(self, clearance_m):
        self.calls.append(("scale", clearance_m))
        return 0.42

    def wait_or_go(self, waited_sec, hand_extended):
        self.calls.append(("wait", hand_extended))
        return core.PLACE if waited_sec >= 1.0 else core.WAIT

    def should_rechoose(self, facing_at_choice, facing_now, rechoices):
        self.calls.append(("rechoose", rechoices))
        return False


def test_the_set_down_hovers_lowers_and_ends_on_the_place_topic(recorded, monkeypatch):
    node, calls, _ = recorded(_World())
    sent = []
    monkeypatch.setattr(node, "reach_pose", lambda position, yaw, label: (
        sent.append((label, tuple(position), node._target_pub)) or True))
    monkeypatch.setattr(node, "slide", lambda start, end, yaw, speed, label: (
        sent.append((label, tuple(end), node._target_pub)) or True))

    assert node.set_down(((1.0, -1.08, 0.871), 1.64))

    assert [(label, xyz) for label, xyz, _ in sent] == [
        ("MPC: above the place", (1.0, -1.08, CFG.place_hover_height)),
        ("MPC: lower to the table", (1.0, -1.08, 0.871)),
        ("MPC: set down", (1.0, -1.08, 0.871)),
    ]
    assert [pub for _, _, pub in sent] == [node._offer_pub, node._offer_pub, node._place_pub]
    assert node._target_pub is node._offer_pub


def test_the_hover_is_under_the_height_cap():
    assert CFG.place_hover_height < CFG.tcp_height_cap


def test_an_offer_only_standoff_backs_off_and_rechooses_with_need_place(recorded):
    decisions = _Scripted()
    node, calls, _ = recorded(_World(), decisions=decisions)

    assert node.run()

    assert [c for c in decisions.calls if c[0] == "choose"] == [("choose", False),
                                                                ("choose", True)]
    assert node._episode.rechoices == 1
    assert node.captions.count("drive to the pre-standoff") == 2
    assert node.phases[-1] == "DONE: set_down"


def test_a_facing_change_while_waiting_rechooses(recorded):
    world = _World(extend_at=3.0, take_at=8.0)

    def turn(w):
        if 1.0 <= w.t < 1.0 + SPIN_SEC:
            w.facing += 0.8

    world.on_spin.append(turn)
    node, calls, _ = recorded(world)

    assert node.run()

    assert node._episode.rechoices == 1
    assert node.captions.count("drive to the pre-standoff") == 2
    assert node.phases.count("TRANSIT") == 2  # on every entry: the recorder counts them
    assert node.phases[-1] == "DONE: handover"


def test_the_rechoice_limit_ends_the_rechoosing(recorded):
    world = _World(extend_at=None)
    node_ref = []

    def keep_turning(w):
        chosen = node_ref[0]._episode.facing_at_choice
        if abs(core.wrap_angle(w.facing - chosen)) < 0.7:
            w.facing = core.wrap_angle(chosen + 0.8)

    node, calls, _ = recorded(world)
    node_ref.append(node)
    world.on_spin.append(keep_turning)

    node.run()

    turns = node._episode.rechoices - (1 if node._episode.need_place else 0)
    assert turns == CFG.max_rechoices
    assert node.captions.count("drive to the pre-standoff") >= CFG.max_rechoices + 1


def test_no_take_within_the_timeout_keeps_the_object(recorded):
    node, calls, _ = recorded(_World(extend_at=1.0))

    assert node.run()

    assert "release" not in node.captions
    assert ("permit", "release permit") not in calls
    assert ("gripper", "open") not in calls[len(PICK_PREFIX):]
    assert node.phases[-3:] == ["WAIT", "RETREAT", "DONE: timeout"]
    assert node.captions[-len(RETREAT):] == RETREAT


def test_the_hand_over_order_is_box_gate_arm_target(recorded):
    node, calls, _ = recorded(_World(extend_at=1.0, take_at=5.0))

    node.run()

    kinds = _kinds(calls)
    box = kinds.index("box")
    assert box < kinds.index("base") < kinds.index("stream") < kinds.index("target")
    assert calls[kinds.index("base")] == ("base", True)
    assert calls[kinds.index("stream")] == ("stream", True)


def test_the_gripper_opens_only_after_a_granted_permit(recorded):
    node, calls, _ = recorded(_World(extend_at=1.0, take_at=5.0))

    node.run()

    permit = calls.index(("permit", "release permit"))
    assert calls[permit + 1:permit + 3] == [("gripper", "open"), ("grasp", False)]
    after_pick = calls.index(("arm", "carry pose"), len(PICK_PREFIX) - 2)
    assert ("gripper", "open") not in calls[after_pick:permit]


def test_a_refused_permit_fails_the_release_and_never_opens(recorded):
    node, calls, _ = recorded(_World(extend_at=1.0, take_at=5.0), permit=False)

    assert not node.run()

    permit = calls.index(("permit", "release permit"))
    assert ("gripper", "open") not in calls[permit:]
    assert node.phases[-1] == "FAILED: release"


def test_a_failed_step_restores_and_says_failed(recorded):
    node, calls, _ = recorded(_World(extend_at=1.0), fail=[("target", "MPC: offer")])

    assert not node.run()

    assert node.phases[-1] == "FAILED: MPC: offer"
    restore = _kinds(calls)[_kinds(calls).index("target") + 1:]
    assert restore == ["base", "stream", "stop"]


def test_a_failure_under_a_safety_stop_is_stopped(recorded, monkeypatch):
    node, calls, _ = recorded(_World(), fail=[("drive", "drive to the pre-standoff")])
    node._safety_state = "STOP: person data stale"
    lines = []
    real = node.get_logger()
    monkeypatch.setattr(node, "get_logger", lambda: SimpleNamespace(
        info=lines.append, warn=real.warn, error=real.error))

    node.run()

    assert any("outcome=stopped" in line for line in lines)


def test_ctrl_c_still_restores_and_marks_the_episode_failed(recorded, monkeypatch):
    node, calls, _ = recorded(_World())

    def interrupt(*args, **kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(node, "navigate", interrupt)
    monkeypatch.setattr(node, "drive_to_pose", interrupt)
    with pytest.raises(KeyboardInterrupt):
        node.run()

    assert node.phases[-1].startswith("FAILED: ")
    assert _kinds(calls)[-3:] == ["base", "stream", "stop"]


def test_a_drive_goes_around_the_person_via_the_route(recorded, monkeypatch):
    node, calls, _ = recorded(_World())
    monkeypatch.setattr(node, "_odom_pose", lambda: (-2.0, 0.0, math.pi))
    person = node.person()
    pre = core.RuleDecisions(CFG).choose_standoff(person, core.candidates(person, CFG),
                                                  False).pre_pose

    assert node.drive(pre, "drive to the pre-standoff")

    legs = core.route((-2.0, 0.0, math.pi), pre, person, CFG)
    assert len(legs) == 2
    assert calls == [("drive", "drive to the pre-standoff (via)"),
                     ("drive", "drive to the pre-standoff")]


def test_use_nav_drives_the_legs_with_nav2(recorded):
    node, calls, _ = recorded(_World(extend_at=1.0, take_at=5.0), use_nav=True)

    node.run()

    navs = [label for kind, label in calls if kind == "nav"]
    assert navs == ["drive away", "drive to the pre-standoff", "back to the pre-standoff",
                    "drive away"]


# ---- decisions and topics -----------------------------------------------------------------


def test_the_injected_decisions_are_the_only_source_of_the_four_decisions(recorded):
    decisions = _Scripted()
    node, calls, _ = recorded(_World(), decisions=decisions)
    published = []
    node._scale_pub = SimpleNamespace(publish=lambda msg: published.append(msg.data))
    node._odom = SimpleNamespace(position=SimpleNamespace(x=-2.0, y=0.0),
                                 orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    node._body = (*PERSON_XY, 0.0, 0.0)

    node._request_scale()
    node.run()

    assert published == [0.42]
    assert {kind for kind, _ in decisions.calls} == {"scale", "choose", "wait", "rechoose"}


def test_the_rule_decisions_are_the_default(make_node):
    node = make_node()

    assert isinstance(node.decisions, core.RuleDecisions)


def test_the_scale_request_follows_the_ramp_and_is_zero_without_a_person(make_node):
    node = make_node(execute=True)
    published = []
    node._scale_pub = SimpleNamespace(publish=lambda msg: published.append(msg.data))
    node._odom = SimpleNamespace(position=SimpleNamespace(x=-2.0, y=0.0),
                                 orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))

    node._request_scale()
    node._body = (*PERSON_XY, 0.0, 0.0)
    node._request_scale()

    assert published == [0.0, 1.0]  # far away: the ramp's 1.0


def test_the_node_has_no_publisher_on_a_governed_topic(make_node):
    node = make_node()

    topics = {name for name, _ in node.get_publisher_names_and_types_by_node(
        node.get_name(), node.get_namespace())}

    assert not topics & GOVERNED_TOPICS
    assert {"/handover_safety/scale_request", "/handover_safety/target_request",
            "/handover_safety/place_target_request", "/whole_body_mpc/base_box"} <= topics


def test_plan_only_publishes_nothing_and_calls_no_service(make_node, monkeypatch):
    node = make_node()
    world = _World()
    monkeypatch.setattr(baseline.rclpy, "spin_once", world.spin)
    monkeypatch.setattr(node, "_wait_servers", lambda: None)
    monkeypatch.setattr(node, "move_to_joint_state", lambda label, q: True)
    monkeypatch.setattr(node, "person", world.person)

    def forbidden(*args, **kwargs):
        raise AssertionError("plan-only must not publish or call")

    for name in ("_scale_pub", "_box_pub", "_offer_pub", "_place_pub", "_phase_pub",
                 "_cmd_vel"):
        monkeypatch.setattr(node, name, SimpleNamespace(publish=forbidden))
    for name in ("_release_client", "_gate_client", "_switch_client", "_grasp"):
        monkeypatch.setattr(node, name, SimpleNamespace(call_async=forbidden))
    monkeypatch.setattr(node._gripper, "send_goal_async", forbidden)
    monkeypatch.setattr(node._nav, "send_goal_async", forbidden)

    node._request_scale()
    assert node.run()


def test_the_baseline_waits_for_the_gate_to_be_governed(make_node, monkeypatch):
    node = make_node(execute=True)
    waited = []

    def spin_until(done, timeout_sec):
        waited.append(done())
        return True

    for client in (node._gate_client, node._switch_client, node._release_client):
        monkeypatch.setattr(client, "wait_for_service", lambda timeout_sec: True)
    monkeypatch.setattr(node, "_spin_until", spin_until)
    node._mpc_state, node._body = "IDLE", (*PERSON_XY, 0.0, 0.0)
    node._safety_state = baseline.SAFETY_UNGOVERNED

    node._wait_reach_servers()

    assert False in waited  # the governed wait saw the ungoverned state


def test_a_failure_before_the_episode_still_ends_with_a_phase(recorded, monkeypatch):
    node, calls, _ = recorded(_World())

    def no_moveit():
        raise RuntimeError("MoveGroup server /move_action not available")

    monkeypatch.setattr(node, "_wait_servers", no_moveit)
    with pytest.raises(RuntimeError):
        node.run()

    assert node.phases == ["FAILED: start"]


@pytest.mark.parametrize("name", ["scale_rate", "retreat_distance", "retreat_speed"])
def test_non_positive_parameters_fail_at_start(make_node, name):
    with pytest.raises(ValueError, match=name):
        make_node(**{name: 0.0})


def test_the_scripted_legs_get_the_longer_timeout(make_node):
    assert make_node().drive_timeout_sec == baseline.DRIVE_TIMEOUT_SEC
    assert make_node(drive_timeout=45.0).drive_timeout_sec == 45.0


def test_a_failed_attach_is_retried_once_and_a_release_is_not(make_node, monkeypatch):
    node = make_node(execute=True)
    answers, calls = [False, True, False, False], []

    def grasp(self, attach):
        calls.append(attach)
        return answers.pop(0)

    monkeypatch.setattr(baseline.pick.PickPlaceDemo, "grasp", grasp)

    assert node.grasp(True)
    assert not node.grasp(False)
    assert calls == [True, True, False]


def test_main_runs_the_baseline(monkeypatch):
    seen = []
    monkeypatch.setattr(baseline.reach, "main", seen.append)

    baseline.main()

    assert seen == [baseline.HandoverBaseline]
