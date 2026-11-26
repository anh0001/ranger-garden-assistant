"""
Unit tests for scripts/person_proxy.py and its models (no simulator).

PersonScript and FaultInjector are pure and take the time as an argument;
the node is driven through its callbacks and timer with a fake clock,
recording publishers and a fake set_pose client, as in test_base_handover_gate.py.
"""

import dataclasses
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from ros_gz_interfaces.msg import Entity
from std_msgs.msg import String

PKG = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("person_proxy", PKG / "scripts" / "person_proxy.py")
person = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(person)
core = person.core

PERSON_XY = (0.504, -0.858)
DT = 0.05
FAR = (-2.0, 0.0, math.pi)  # away pose: footprint 2.2 m from the torso axis
NEAR = (0.504, 0.336, -math.pi / 2)  # a pre-standoff to the person's left
OFFER_TCP = (0.504, -0.408, 0.81)


def _settings(**changes):
    defaults = dict(
        x=PERSON_XY[0], y=PERSON_XY[1], start_facing=0.0, facing_yaw=math.radians(30.0),
        facing_change=0.0, facing_change_delay=1.0, hand_side=1.0, hand_extends=True,
        reaction_delay=(0.5, 1.0), turn_distance=1.5, notice_distance=1.5, still_speed=0.02,
        still_sec=0.5, hand_speed=0.5, hand_rest=(0.25, 0.18, 0.62),
        hand_extended=(0.40, 0.20, 0.80), take_reach=0.65, shoulder_height=1.00,
        footprint=(-0.34, -0.25, 0.394, 0.25),
    )
    return person.ScriptSettings(**{**defaults, **changes})


def _view(base=NEAR, speed=0.0, tcp=None, tcp_speed=math.inf, detached=False):
    return person.RobotView(base, speed, tcp, tcp_speed, detached)


def _run(script, view, start, end):
    """Step `script` from `start` to `end` (s) with `view(t)`; return the states."""
    states, t = [], start
    while t < end - 1e-9:
        states.append(script.step(t, view(t)))
        t = round(t + DT, 6)
    return states


def _turned(script, t=0.0):
    """Drive the robot away so the person turns; return the time reached."""
    _run(script, lambda _: _view(FAR), t, t + 0.2)
    return t + 0.2


def _first(states, predicate):
    return next((s.stamp_sec for s in states if predicate(s)), None)


# ---- PersonScript -------------------------------------------------------------------


def test_the_person_faces_the_table_until_the_robot_was_far():
    script = person.PersonScript(_settings(), seed=0)

    near = _run(script, lambda _: _view((0.385, 0.0, 0.0)), 0.0, 1.0)
    far = _run(script, lambda _: _view(FAR), 1.0, 1.2)
    back = _run(script, lambda _: _view((0.385, 0.0, 0.0)), 1.2, 1.5)

    assert {s.facing for s in near} == {0.0}
    assert far[-1].facing == pytest.approx(math.radians(30.0))
    assert back[-1].facing == pytest.approx(math.radians(30.0))  # stays turned


def test_the_person_does_not_turn_before_the_robot_was_seen():
    # Gazebo, 2026-11-26: before the first /odom the robot's distance was
    # unknown, read as far, and the person turned into the pick's path.
    script = person.PersonScript(_settings(facing_yaw=math.radians(60.0)), seed=0)

    unknown = _run(script, lambda _: _view(base=None), 0.0, 1.0)
    pick = _run(script, lambda _: _view((0.385, 0.0, 0.0)), 1.0, 2.0)

    assert {s.facing for s in unknown + pick} == {0.0}


def test_the_hand_extends_only_near_and_still_after_a_delay_in_the_range():
    script = person.PersonScript(_settings(), seed=3)
    t = _turned(script)
    moving = _run(script, lambda _: _view(NEAR, speed=0.1), t, t + 3.0)
    t += 3.0

    still = _run(script, lambda _: _view(NEAR), t, t + 5.0)

    assert not any(s.hand_extended for s in moving)
    extended_at = _first(still, lambda s: s.hand_extended)
    # still_sec, then the reaction delay, then 0.5 m/s over ~0.25 m.
    assert t + 0.5 + 0.5 <= extended_at <= t + 0.5 + 1.0 + 0.6 + 2 * DT
    rest = _run(person.PersonScript(_settings(), seed=3), lambda _: _view(NEAR), 0.0, 0.1)[0]
    assert math.dist(still[-1].hand, rest.hand) > 0.1


def test_the_hand_does_not_extend_before_the_person_turned():
    script = person.PersonScript(_settings(), seed=0)

    states = _run(script, lambda _: _view((0.385, 0.0, 0.0)), 0.0, 5.0)

    assert not any(s.hand_extended for s in states)


def _extension_time(seed):
    script = person.PersonScript(_settings(), seed=seed)
    t = _turned(script)
    return _first(_run(script, lambda _: _view(NEAR), t, t + 5.0), lambda s: s.hand_extended)


def test_the_same_seed_gives_the_same_delays_and_another_seed_others():
    assert _extension_time(1) == _extension_time(1)
    assert len({_extension_time(seed) for seed in range(6)}) > 1


def test_a_person_who_never_reaches_never_extends_nor_takes():
    script = person.PersonScript(_settings(hand_extends=False), seed=0)
    t = _turned(script)

    states = _run(script, lambda _: _view(NEAR, tcp=OFFER_TCP, tcp_speed=0.0), t, t + 10.0)

    assert not any(s.hand_extended or s.take for s in states)


def _extended(script):
    t = _turned(script)
    states = _run(script, lambda _: _view(NEAR), t, t + 4.0)
    assert states[-1].hand_extended
    return t + 4.0


def test_the_take_needs_a_still_tcp_within_reach():
    script = person.PersonScript(_settings(), seed=0)
    t = _extended(script)
    far_tcp = (0.504, 0.20, 0.95)  # 1 m from the shoulder
    out_of_reach = _run(script, lambda _: _view(NEAR, tcp=far_tcp, tcp_speed=0.0), t, t + 3.0)
    t += 3.0
    moving = _run(script, lambda _: _view(NEAR, tcp=OFFER_TCP, tcp_speed=0.1), t, t + 3.0)
    t += 3.0

    still = _run(script, lambda _: _view(NEAR, tcp=OFFER_TCP, tcp_speed=0.0), t, t + 4.0)

    assert not any(s.take for s in out_of_reach + moving)
    take_at = _first(still, lambda s: s.take)
    assert take_at is not None and take_at >= t + 0.5 + 0.5
    taking = next(s for s in still if s.take)
    assert math.dist(taking.hand, OFFER_TCP) <= person.CONTACT_M


def test_after_the_detach_the_hand_returns_and_the_take_ends():
    script = person.PersonScript(_settings(), seed=0)
    t = _extended(script)
    _run(script, lambda _: _view(NEAR, tcp=OFFER_TCP, tcp_speed=0.0), t, t + 4.0)
    t += 4.0

    after = _run(script, lambda _: _view(NEAR, tcp=OFFER_TCP, tcp_speed=0.0, detached=True),
                 t, t + 3.0)

    assert script.taken
    assert not after[-1].take and not after[-1].hand_extended
    rest = script._local(script._s.hand_rest)
    assert math.dist(after[-1].hand, rest) < 1e-9


def test_the_facing_change_is_applied_once():
    script = person.PersonScript(_settings(facing_change=0.8), seed=0)
    t = _turned(script)

    states = _run(script, lambda _: _view(NEAR), t, t + 4.0)

    facings = sorted({round(s.facing, 6) for s in states})
    assert facings == pytest.approx([math.radians(30.0), math.radians(30.0) + 0.8])
    changed_at = _first(states, lambda s: s.facing > math.radians(30.0) + 0.1)
    assert changed_at == pytest.approx(t + 0.5 + 1.0, abs=2 * DT)


def test_the_hand_side_mirrors_the_hand():
    pick_slot = (0.385, 0.0, 0.0)  # not turned yet: facing 0, the side is world y
    left = person.PersonScript(_settings(), seed=0).step(0.0, _view(pick_slot))
    right = person.PersonScript(_settings(hand_side=-1.0), seed=0).step(0.0, _view(pick_slot))

    assert left.hand[1] - PERSON_XY[1] == pytest.approx(-(right.hand[1] - PERSON_XY[1]))


@pytest.mark.parametrize("delay", [[1.0, 0.5], [-0.1, 1.0], [0.5], [0.5, math.nan]])
def test_a_malformed_reaction_delay_is_rejected(delay):
    with pytest.raises(ValueError, match="reaction_delay"):
        person.check_delay(delay)


# ---- FaultInjector -------------------------------------------------------------------


def _sample(t, x=PERSON_XY[0]):
    return core.PersonState(x, PERSON_XY[1], 0.3, (x + 0.4, -0.6, 0.81), True, False, t)


def _faults(**values):
    numbers = {name: 0.0 for name in person.FAULT_FIELDS}
    numbers.update(values)
    return person.parse_faults([numbers[name] for name in person.FAULT_FIELDS])


def _feed(injector, start, end, distance=lambda _: 5.0):
    out, t = [], start
    while t < end - 1e-9:
        out.append((t, injector.step(_sample(t), t, distance(t))))
        t = round(t + DT, 6)
    return out


def test_no_faults_return_the_ground_truth_sample_itself():
    injector = person.FaultInjector(_faults(), seed=0)
    sample = _sample(1.0)

    assert injector.step(sample, 1.0, 5.0) is sample
    assert injector.active(1.0) == ""


def test_a_bias_shifts_body_and_hand():
    injector = person.FaultInjector(_faults(bias_x=0.1, bias_y=-0.2), seed=0)

    measured = injector.step(_sample(1.0), 1.0, 5.0)

    assert (measured.x, measured.y) == pytest.approx((PERSON_XY[0] + 0.1, PERSON_XY[1] - 0.2))
    assert measured.hand == pytest.approx((PERSON_XY[0] + 0.5, -0.8, 0.81))
    assert measured.stamp_sec == 1.0


def test_noise_is_seeded():
    one = [s.x for _, s in _feed(person.FaultInjector(_faults(noise_std=0.05), 4), 0.0, 1.0)]
    two = [s.x for _, s in _feed(person.FaultInjector(_faults(noise_std=0.05), 4), 0.0, 1.0)]

    assert one == two
    assert len(set(one)) > 1


def test_a_dropout_publishes_nothing_inside_its_window_only():
    injector = person.FaultInjector(_faults(dropout_start=1.0, dropout_sec=0.5), seed=0)

    out = _feed(injector, 0.0, 3.0)

    missing = [t for t, s in out if s is None]
    assert missing == pytest.approx([1.0 + DT * i for i in range(10)])


def test_stale_repeats_the_first_sample_of_its_window_with_its_stamp():
    injector = person.FaultInjector(_faults(stale_start=1.0, stale_sec=0.5), seed=0)

    out = dict(_feed(injector, 0.0, 2.0))

    assert {out[t].stamp_sec for t in (1.0, 1.2, 1.45)} == {1.0}
    assert out[1.5].stamp_sec == 1.5
    assert out[0.95].stamp_sec == 0.95


def test_a_jump_shifts_from_jump_at_on():
    injector = person.FaultInjector(_faults(jump_at=1.0, jump_x=0.3), seed=0)

    out = dict(_feed(injector, 0.0, 2.0))

    assert out[0.95].x == pytest.approx(PERSON_XY[0])
    assert out[1.0].x == pytest.approx(PERSON_XY[0] + 0.3)
    assert out[1.9].x == pytest.approx(PERSON_XY[0] + 0.3)
    assert injector.active(1.9) == "jump"


def test_latency_delays_delivery_and_keeps_the_stamp():
    injector = person.FaultInjector(_faults(latency_sec=0.3), seed=0)

    out = _feed(injector, 0.0, 1.0)

    assert all(s is None for t, s in out if t < 0.3 - 1e-9)
    for t, s in out:
        if t >= 0.3 - 1e-9:
            assert s.stamp_sec == pytest.approx(t - 0.3)


def test_windows_count_from_the_trigger_distance():
    injector = person.FaultInjector(
        _faults(dropout_start=0.2, dropout_sec=0.3, trigger_distance=1.2), seed=0)

    out = _feed(injector, 0.0, 3.0, distance=lambda t: 3.0 - t)  # under 1.2 m from t = 1.8

    missing = [t for t, s in out if s is None]
    assert missing[0] == pytest.approx(2.05) and missing[-1] == pytest.approx(2.3)
    assert injector.active(2.1) == "dropout"
    assert injector.active(2.6) == ""


@pytest.mark.parametrize("values, match", [
    ([0.0] * 11, "12 values"), ([0.0] * 4 + [-1.0] + [0.0] * 7, "dropout_sec"),
    ([0.0] * 2 + [math.nan] + [0.0] * 9, "noise_std"), ([0.0] * 11 + [-0.5], "trigger"),
])
def test_malformed_faults_are_rejected(values, match):
    with pytest.raises(ValueError, match=match):
        person.parse_faults(values)


# ---- node ---------------------------------------------------------------------------------


class _Future:
    def __init__(self):
        self._done = False

    def done(self):
        return self._done

    def finish(self):
        self._done = True


class _PoseClient:
    srv_name = "/world/small_house/set_pose"

    def __init__(self):
        self.requests, self.futures, self.ready = [], [], True

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        self.futures.append(_Future())
        return self.futures[-1]


class _Logger:
    def __init__(self):
        self.infos, self.warnings = [], []

    def info(self, text):
        self.infos.append(text)

    def warn(self, text):
        self.warnings.append(text)


class _Harness:
    def __init__(self, node):
        self.node = node
        self.clock = [100.0]
        self.log = _Logger()
        self.published = {}
        node._now = lambda: self.clock[0]
        node.get_logger = lambda: self.log
        node._tcp = lambda now: (None, math.inf)
        for prefix, pubs in node._pubs.items():
            for name in pubs:
                record = self.published.setdefault((prefix, name), [])
                pubs[name] = SimpleNamespace(publish=record.append)
        self.faults = []
        node._fault_pub = SimpleNamespace(publish=self.faults.append)
        self.client = _PoseClient()
        node._pose_client = self.client

    def odom(self, pose, speed=0.0):
        msg = Odometry()
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[0], pose[1]
        msg.pose.pose.orientation.z = math.sin(pose[2] / 2)
        msg.pose.pose.orientation.w = math.cos(pose[2] / 2)
        msg.twist.twist.linear.x = speed
        self.node._on_odom(msg)

    def tick(self, count=1):
        for _ in range(count):
            self.clock[0] = round(self.clock[0] + DT, 6)
            self.node._tick()
            for future in self.client.futures:
                future.finish()

    def topic(self, prefix, name):
        return self.published[(prefix, name)]


@pytest.fixture
def make_node():
    rclpy.init()
    nodes = []

    def make(**params):
        overrides = [Parameter(name, value=value) for name, value in params.items()]
        nodes.append(person.PersonProxy(parameter_overrides=overrides))
        return _Harness(nodes[-1])

    try:
        yield make
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def _pose(msg):
    p = msg.pose.position
    return p.x, p.y, p.z, msg.header.stamp.sec, msg.header.stamp.nanosec


def test_without_faults_the_two_topic_sets_are_equal(make_node):
    sim = make_node()
    sim.odom(FAR)

    sim.tick(20)

    for name in ("body_pose", "hand_pose"):
        truth = [_pose(m) for m in sim.topic(person.GROUND_TRUTH, name)]
        assert truth == [_pose(m) for m in sim.topic(person.MEASURED, name)]
        assert len(truth) == 20
    for name in ("hand_extended", "take"):
        assert ([m.data for m in sim.topic(person.GROUND_TRUTH, name)]
                == [m.data for m in sim.topic(person.MEASURED, name)])


def test_the_body_pose_is_in_odom_at_seat_height_facing_the_table(make_node):
    sim = make_node()
    sim.odom((0.385, 0.0, 0.0))

    sim.tick()

    body = sim.topic(person.GROUND_TRUTH, "body_pose")[-1]
    assert body.header.frame_id == "odom"
    assert _pose(body)[:3] == pytest.approx((*PERSON_XY, 0.435 + 0.010))
    assert body.pose.orientation.w == pytest.approx(1.0)


def test_a_dropout_silences_only_the_measured_topics(make_node):
    faults = [0.0] * 12
    faults[3], faults[4] = 0.5, 1.0  # dropout_start, dropout_sec
    sim = make_node(faults=faults)
    sim.odom(FAR)

    sim.tick(40)

    assert len(sim.topic(person.GROUND_TRUTH, "body_pose")) == 40
    assert len(sim.topic(person.MEASURED, "body_pose")) == 20
    assert len(sim.topic(person.MEASURED, "take")) == 20


def test_fault_active_is_published_once_per_edge(make_node):
    faults = [0.0] * 12
    faults[3], faults[4] = 0.5, 0.5
    sim = make_node(faults=faults)
    sim.odom(FAR)

    sim.tick(40)

    assert [m.data for m in sim.faults] == ["", "dropout", ""]


def test_set_pose_moves_the_models_in_world_z(make_node):
    sim = make_node()
    sim.odom(FAR)

    sim.tick()

    requests = {r.entity.name: r for r in sim.client.requests}
    assert set(requests) == {"person_hand", "person_proxy"}
    assert all(r.entity.type == Entity.MODEL for r in requests.values())
    hand = sim.topic(person.GROUND_TRUTH, "hand_pose")[-1].pose.position
    assert requests["person_hand"].pose.position.z == pytest.approx(hand.z - 0.010)
    assert requests["person_proxy"].pose.position.z == 0.0


def test_a_pending_set_pose_is_not_doubled_and_an_unmoved_model_not_resent(make_node):
    sim = make_node()
    sim.odom(FAR)
    sim.node._tick()  # no finish: the request stays pending
    sim.clock[0] += DT

    sim.node._tick()
    pending = len(sim.client.requests)
    sim.tick(5)

    assert pending == 2
    assert len(sim.client.requests) == 2  # nothing moved since


def test_without_the_set_pose_service_nothing_is_sent_and_it_warns_once(make_node):
    sim = make_node()
    sim.client.ready = False
    sim.odom(FAR)

    sim.tick(5)

    assert sim.client.requests == []
    assert len([w for w in sim.log.warnings if "set_pose" in w]) == 1


def test_the_taken_object_is_moved_once_after_the_detach(make_node):
    sim = make_node()
    sim.node._script._taken = True

    sim.node._on_grasp(String(data="detached"))
    sim.odom(FAR)
    sim.tick(5)

    moved = [r for r in sim.client.requests if r.entity.name == "vitamin_bottle"]
    assert len(moved) == 1
    assert (moved[0].pose.position.x, moved[0].pose.position.y, moved[0].pose.position.z) \
        == pytest.approx(tuple(person.TAKEN_OBJECT_POSE))


@pytest.mark.parametrize("name, value, match", [
    ("hand_side", "middle", "hand_side"), ("faults", [0.0] * 3, "faults"),
    ("reaction_delay", [2.0, 1.0], "reaction_delay"), ("rate", 0.0, "rate"),
    ("hand_rest", [0.1, 0.2], "hand_rest"), ("stop_distance", -1.0, "stop_distance"),
])
def test_bad_parameters_fail_at_start(make_node, name, value, match):
    with pytest.raises(ValueError, match=match):
        make_node(**{name: value})


# ---- models ------------------------------------------------------------------------------------


@pytest.mark.parametrize("name", ["person_proxy", "person_hand"])
def test_the_person_models_are_static_visual_only(name):
    model = ET.parse(PKG / "models" / f"{name}.sdf").getroot().find("model")

    assert model.get("name") == name
    assert model.findtext("static").strip() == "true"
    assert model.find(".//collision") is None
    assert model.find(".//sensor") is None
    assert model.find(".//visual") is not None


def test_the_injector_is_identity_for_the_default_faults():
    injector = person.FaultInjector(person.parse_faults([0.0] * 12), seed=0)
    sample = _sample(2.0)

    assert dataclasses.astuple(injector.step(sample, 2.0, 0.5)) == dataclasses.astuple(sample)


def test_faults_arm_only_after_the_robot_was_away():
    # The pick slot is ~1 m from the person: a start below the trigger must not fire.
    injector = person.FaultInjector(
        _faults(bias_x=0.3, dropout_start=0.2, dropout_sec=0.3, trigger_distance=1.2), seed=0)

    def distance(t):
        return 1.0 if t < 1.0 else (2.5 if t < 2.0 else 1.0)  # pick, away, approach

    out = dict(_feed(injector, 0.0, 3.0, distance))

    assert out[0.5].x == pytest.approx(PERSON_XY[0])  # unarmed during the pick
    assert out[1.5].x == pytest.approx(PERSON_XY[0] + 0.3)  # armed while away: bias on
    missing = [t for t, s in out.items() if s is None]
    assert missing[0] == pytest.approx(2.2) and missing[-1] == pytest.approx(2.45)
