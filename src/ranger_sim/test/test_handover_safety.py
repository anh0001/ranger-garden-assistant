"""
Unit tests for scripts/handover_safety.py (no simulator).

SafetyRules is pure; the node is driven through its callbacks and timer
with a fake clock and recording publishers (nothing spins).
"""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

PKG = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("handover_safety",
                                               PKG / "scripts" / "handover_safety.py")
safety = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(safety)
core = safety.core
CFG = core.default_config()

PERSON_XY = (0.504, -0.858)
CLEAR = (0.504, 0.436, -math.pi / 2)  # footprint 0.9 m from the torso axis, clearance 0.68
CLOSE = (0.504, -0.208, -math.pi / 2)  # footprint front 0.256 m from the torso: inside


def _rules():
    return safety.SafetyRules(CFG, request_timeout=0.5, take_fresh_sec=1.0)


def _person(t, x=PERSON_XY[0], y=PERSON_XY[1]):
    return core.PersonState(x, y, 0.0, (0.0, 0.0, 0.0), False, False, t)


def _robot(rules, now, base=CLEAR, tcp_z=0.95):
    """Fresh odometry and TCP at `now`."""
    rules.update_base(base, now)
    rules.update_tcp_z(tcp_z, now)


def _ready(now=10.0, base=CLEAR, request=1.0):
    rules = _rules()
    rules.update_person(_person(now), now)
    _robot(rules, now, base)
    rules.update_request(request, now)
    return rules


# ---- SafetyRules (pure) ---------------------------------------------------------


def test_the_request_passes_when_nothing_limits():
    assert _ready(request=0.7).final_scale(10.0) == (0.7, "")


@pytest.mark.parametrize("value, expected", [(1.5, 1.0), (-0.2, 0.0), (math.nan, 0.0)])
def test_the_request_is_clamped(value, expected):
    assert _ready(request=value).final_scale(10.0)[0] == expected


def test_an_old_request_gives_zero_and_names_the_request():
    rules = _ready()
    rules.update_person(_person(10.55), 10.55)
    _robot(rules, 10.55)

    assert rules.final_scale(10.6) == (0.0, safety.REASON_REQUEST)


def test_the_stop_distance_gives_zero():
    rules = _ready(base=CLOSE)

    assert rules.clearance() < CFG.stop_distance
    assert rules.final_scale(10.0) == (0.0, safety.REASON_DISTANCE)


def test_no_odometry_gives_zero():
    rules = _ready(base=None)

    assert rules.final_scale(10.0) == (0.0, safety.REASON_NO_ODOM)


def test_person_data_old_by_its_stamp_gives_zero():
    rules = _ready()
    rules.update_person(_person(9.6), 10.0)  # stamped 0.4 s ago, received now

    assert rules.final_scale(10.0) == (0.0, safety.REASON_STALE)


def test_person_data_old_by_its_receipt_gives_zero():
    rules = _ready()
    rules.update_request(1.0, 10.35)

    assert rules.final_scale(10.35) == (0.0, safety.REASON_STALE)


def test_person_data_from_the_future_counts_as_stale():
    rules = _ready()
    _robot(rules, 9.0)

    assert rules.final_scale(9.0)[1] == safety.REASON_STALE


def test_no_person_sample_gives_zero():
    rules = _rules()
    _robot(rules, 10.0)
    rules.update_request(1.0, 10.0)

    assert rules.final_scale(10.0) == (0.0, safety.REASON_STALE)


def test_a_jump_gives_zero_for_the_hold_time_then_recovers():
    rules = _ready()
    rules.update_person(_person(10.05, x=PERSON_XY[0] + 0.2), 10.05)

    def at(t):
        rules.update_person(_person(t, x=PERSON_XY[0] + 0.2), t)
        _robot(rules, t)
        rules.update_request(1.0, t)
        return rules.final_scale(t)

    assert at(10.1) == (0.0, safety.REASON_JUMP)
    assert at(11.0) == (0.0, safety.REASON_JUMP)
    assert at(11.1) == (1.0, "")


def test_a_repeated_sample_is_not_a_jump():
    rules = _ready()
    rules.update_person(_person(10.0), 10.05)

    assert rules.final_scale(10.05) == (1.0, "")


def test_the_tcp_above_the_cap_gives_zero_without_any_person_data():
    rules = _rules()
    rules.update_tcp_z(1.01, 10.0)

    assert rules.final_scale(10.0) == (0.0, safety.REASON_HEIGHT)


def test_the_tcp_under_the_cap_does_not_limit():
    rules = _ready()
    rules.update_tcp_z(0.95, 10.0)

    assert rules.final_scale(10.0) == (1.0, "")


def test_the_height_cap_is_named_before_the_other_reasons():
    rules = _ready(base=CLOSE)
    rules.update_tcp_z(1.2, 10.0)

    assert rules.final_scale(10.0)[1] == safety.REASON_HEIGHT


@pytest.mark.parametrize("frame, position, refused", [
    ("odom", (0.5, -0.4, 1.01), "height cap"), ("odom", (0.5, -0.4, 0.99), None),
    ("map", (0.5, -0.4, 0.8), "frame"), ("odom", (math.nan, -0.4, 0.8), "non-finite"),
    ("/odom", (0.5, -0.4, 0.8), None),
])
def test_target_check(frame, position, refused):
    reason = _rules().check_target(frame, position)

    if refused is None:
        assert reason is None
    else:
        assert refused in reason


@pytest.mark.parametrize("take_seen, place_reached, allowed", [
    (9.5, False, True), (8.9, False, False), (None, True, True), (None, False, False),
    (10.5, False, False),
])
def test_release_permit(take_seen, place_reached, allowed):
    assert _ready().release_allowed(10.0, take_seen, place_reached)[0] is allowed


# ---- node -------------------------------------------------------------------------


class _Future:
    def __init__(self, success=True):
        self._result = SimpleNamespace(success=success)

    def done(self):
        return True

    def result(self):
        return self._result


class _GovernClient:
    def __init__(self):
        self.requests, self.ready = [], True

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request.data)
        return _Future()


class _Logger:
    def __init__(self):
        self.infos, self.warnings, self.errors = [], [], []

    def info(self, text):
        self.infos.append(text)

    def warn(self, text):
        self.warnings.append(text)

    def error(self, text):
        self.errors.append(text)


class _Harness:
    def __init__(self, node):
        self.node = node
        self.clock = [100.0]
        self.log = _Logger()
        self.targets, self.mpc, self.gate, self.limits, self.states = [], [], [], [], []
        node._now = lambda: self.clock[0]
        node.get_logger = lambda: self.log
        node._tcp = lambda: (self.tcp_z, self.clock[0])
        self.tcp_z = 0.95
        node._governed = True  # the gate answered (see test_an_ungoverned_gate_...)
        node._target_pub = SimpleNamespace(publish=self.targets.append)
        node._mpc_scale_pub = SimpleNamespace(publish=self.mpc.append)
        node._gate_scale_pub = SimpleNamespace(publish=self.gate.append)
        node._limit_pub = SimpleNamespace(publish=self.limits.append)
        node._state_pub = SimpleNamespace(publish=self.states.append)
        self.client = _GovernClient()
        node._govern_client = self.client

    def person(self, x=PERSON_XY[0], stamp=None):
        msg = PoseStamped()
        msg.header.frame_id = "odom"
        t = self.clock[0] if stamp is None else stamp
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
        msg.pose.position.x, msg.pose.position.y = x, PERSON_XY[1]
        msg.pose.orientation.w = 1.0
        self.node._on_body(msg)

    def odom(self, pose=CLEAR):
        msg = Odometry()
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[0], pose[1]
        msg.pose.pose.orientation.z = math.sin(pose[2] / 2)
        msg.pose.pose.orientation.w = math.cos(pose[2] / 2)
        self.node._on_odom(msg)

    def request(self, scale):
        self.node._on_scale_request(Float64(data=float(scale)))

    def target(self, z, place=False, frame="odom", xy=(0.5, -0.4)):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (*xy, z)
        msg.pose.orientation.w = 1.0
        (self.node._on_place_target if place else self.node._on_offer_target)(msg)

    def release(self):
        return self.node._on_release(Trigger.Request(), Trigger.Response())

    def tick(self, advance=0.05, fresh=True):
        self.clock[0] = round(self.clock[0] + advance, 6)
        if fresh:
            self.person()
            self.odom()
            self.request(1.0)
        self.node._tick()

    @property
    def state(self):
        return self.states[-1].data if self.states else None


@pytest.fixture
def make_node():
    rclpy.init()
    nodes = []

    def make(**params):
        overrides = [Parameter(name, value=value) for name, value in params.items()]
        nodes.append(safety.HandoverSafety(parameter_overrides=overrides))
        return _Harness(nodes[-1])

    try:
        yield make
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def test_both_scale_topics_carry_the_same_value_every_tick(make_node):
    sim = make_node()

    for scale in (1.0, 0.4):
        sim.clock[0] += 0.05
        sim.person()
        sim.odom()
        sim.request(scale)
        sim.node._tick()

    assert [m.data for m in sim.mpc] == [m.data for m in sim.gate] == [1.0, 0.4]
    assert sim.state == "OK"


def test_the_gate_is_governed_once_answered(make_node):
    sim = make_node()
    sim.node._governed = False

    sim.tick()
    sim.tick()
    sim.tick(advance=1.1)

    assert sim.client.requests == [True]
    assert sim.node._governed


def test_a_stop_names_its_reason_and_scales_to_zero(make_node):
    sim = make_node()
    sim.tick()

    sim.clock[0] += 0.05
    sim.person()
    sim.odom(CLOSE)
    sim.request(1.0)
    sim.node._tick()

    assert sim.mpc[-1].data == sim.gate[-1].data == 0.0
    assert sim.state == "STOP: stop distance"


def test_a_quiet_decision_maker_is_zero_but_not_a_stop(make_node):
    sim = make_node()

    sim.clock[0] += 0.05
    sim.person()
    sim.odom()
    sim.node._tick()

    assert sim.mpc[-1].data == 0.0
    assert sim.state == "OK"


def test_a_target_over_the_cap_is_not_forwarded_and_the_state_says_so(make_node):
    sim = make_node()

    sim.target(1.01)
    sim.target(0.81)

    assert [round(m.pose.position.z, 3) for m in sim.targets] == [0.81]
    assert sim.states[0].data.startswith("REJECTED: ")
    assert "height cap" in sim.states[0].data


def test_the_speed_limit_is_never_zero_while_running_and_five_percent_at_a_stop(make_node):
    sim = make_node()
    sim.tick()
    sim.clock[0] += 0.05
    sim.person()
    sim.odom(CLOSE)
    sim.node._tick()

    percentages = [m.speed_limit for m in sim.limits]
    assert all(m.percentage for m in sim.limits)
    assert percentages == [100.0, 5.0]


def test_the_speed_limit_repeats_once_per_period(make_node):
    sim = make_node()

    for _ in range(30):
        sim.tick()

    assert len(sim.limits) == 2  # first tick, then after 1 s


def test_release_with_a_fresh_take(make_node):
    sim = make_node()
    sim.node._on_take(Bool(data=True))
    sim.clock[0] += 0.5
    sim.person()  # fresh person data

    assert sim.release().success


def test_release_at_a_reached_place_target_but_not_at_a_reached_offer(make_node):
    sim = make_node()
    sim.node._on_mpc_state(String(data="REACHED"))

    sim.target(0.81)
    offer = sim.release().success
    sim.target(0.871, place=True, xy=(1.0, -0.64))  # on the table
    place = sim.release().success

    assert not offer
    assert place


def test_shutdown_stops_removes_the_limit_and_ungoverns(make_node, monkeypatch):
    sim = make_node()
    sim.tick()
    monkeypatch.setattr(safety.rclpy, "spin_until_future_complete", lambda *a, **k: None)

    sim.node.release_gate()

    assert sim.mpc[-1].data == sim.gate[-1].data == 0.0
    assert sim.limits[-1].speed_limit == 0.0 and sim.limits[-1].percentage
    assert sim.client.requests[-1] is False


@pytest.mark.parametrize("name", ["rate", "request_timeout", "take_fresh_sec"])
def test_non_positive_parameters_fail_at_start(make_node, name):
    with pytest.raises(ValueError, match=name):
        make_node(**{name: 0.0})


# ---- fail closed (code review, 2026-11-25) ----------------------------------------


def test_an_unknown_tcp_gives_zero():
    rules = _ready()
    rules.update_tcp_z(None)

    assert rules.final_scale(10.0) == (0.0, safety.REASON_TCP_UNKNOWN)


def test_a_stale_tcp_transform_gives_zero():
    rules = _ready()
    rules.update_tcp_z(0.95, 9.4)  # a frozen TF tree

    assert rules.final_scale(10.0) == (0.0, safety.REASON_TCP_UNKNOWN)


def test_stale_odometry_gives_zero():
    rules = _ready()
    rules.update_base(CLEAR, 9.4)

    assert rules.final_scale(10.0) == (0.0, safety.REASON_ODOM_STALE)


@pytest.mark.parametrize("position, refused", [
    ((1.00, -0.64, 0.871), False), ((0.60, -0.40, 0.871), True),  # off the table
    ((1.00, -0.64, 0.95), True),  # above the place height
])
def test_a_place_target_must_be_on_the_table_at_the_place_height(position, refused):
    reason = _rules().check_target("odom", position, place=True)

    assert (reason is not None) is refused


def test_an_offer_target_is_not_held_to_the_table():
    assert _rules().check_target("odom", (0.60, -0.40, 0.81)) is None


def test_a_take_with_stale_person_data_does_not_release():
    rules = _ready()

    assert not rules.release_allowed(10.6, 10.5, False)[0]  # person sample from 10.0
    assert rules.release_allowed(10.2, 10.1, False)[0]


def test_a_take_during_a_jump_hold_does_not_release():
    rules = _ready()
    rules.update_person(_person(10.05, x=PERSON_XY[0] + 0.3), 10.05)

    assert not rules.release_allowed(10.1, 10.08, False)[0]


def test_an_ungoverned_gate_keeps_the_scale_at_zero(make_node):
    sim = make_node()
    sim.node._governed = False
    sim.client.ready = False  # no gate answers

    sim.tick()

    assert sim.mpc[-1].data == 0.0
    assert sim.state == "STOP: gate not governed"
