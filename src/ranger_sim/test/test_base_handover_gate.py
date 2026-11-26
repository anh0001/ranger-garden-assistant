"""Unit tests for scripts/base_handover_gate.py (no simulator required)."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "base_handover_gate.py"
_spec = importlib.util.spec_from_file_location("base_handover_gate", SCRIPT)
gate_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gate_mod)

Mode = gate_mod.Mode
SETTLE_SEC = 0.3
CMD_TIMEOUT_SEC = 0.5
MANIP_MAX_SPEED_MPS = 0.15
TICK_SEC = 0.02
STILL = (0.0, 0.0)
MOVING = (0.05, 0.0)
NAV = (0.2, 0.0, 0.5)
MANIP = (0.05, 0.03, 0.0)


def _gate():
    return gate_mod.HandoverGate(
        settle_sec=SETTLE_SEC,
        still_linear_mps=0.005,
        still_angular_rps=0.01,
        cmd_timeout_sec=CMD_TIMEOUT_SEC,
        manip_max_speed_mps=MANIP_MAX_SPEED_MPS,
    )


def _settle(gate, now, speed=STILL, duration=SETTLE_SEC + 2 * TICK_SEC):
    """Feed `speed` every tick for `duration` from `now`; return the time reached."""
    end = now + duration
    while now < end:
        now += TICK_SEC
        gate.update(speed, now)
    return now


# ---- HandoverGate (pure) ---------------------------------------------------


def test_gate_starts_in_navigate_and_passes_a_fresh_nav_twist_unchanged():
    gate = _gate()

    twist = gate.select((NAV, 10.0), None, 10.1)

    assert gate.mode is Mode.NAVIGATE
    assert twist == NAV


@pytest.mark.parametrize("stamp", [10.0 - CMD_TIMEOUT_SEC - 0.01, 10.5])
def test_gate_treats_a_stale_or_future_nav_twist_as_a_stop(stamp):
    # 10.5 > now: stamped before a sim-time jump backwards.
    gate = _gate()

    twist = gate.select((NAV, stamp), None, 10.0)

    assert twist == gate_mod.ZERO_TWIST


def test_gate_without_any_twist_outputs_zero():
    gate = _gate()

    assert gate.select(None, None, 10.0) == gate_mod.ZERO_TWIST


def test_request_enters_settling_and_outputs_zero_for_every_input():
    gate = _gate()

    target = gate.request(True)
    twist = gate.select((NAV, 10.0), (MANIP, 10.0), 10.0)

    assert target is Mode.MANIPULATE
    assert gate.mode is Mode.SETTLING
    assert twist == gate_mod.ZERO_TWIST


def test_settling_ends_after_the_base_was_still_for_settle_sec():
    gate = _gate()
    gate.request(True)
    gate.update(STILL, 10.0)

    early = gate.update(STILL, 10.0 + SETTLE_SEC - TICK_SEC)
    late = gate.update(STILL, 10.0 + SETTLE_SEC + TICK_SEC)

    assert early is Mode.SETTLING
    assert late is Mode.MANIPULATE


def test_settling_waits_while_the_base_moves():
    gate = _gate()
    gate.request(True)

    now = _settle(gate, 10.0, speed=MOVING, duration=2.0)

    assert gate.mode is Mode.SETTLING
    assert gate.update(STILL, now) is Mode.SETTLING


def test_settling_requires_the_angular_speed_still_too():
    gate = _gate()
    gate.request(True)

    _settle(gate, 10.0, speed=(0.0, 0.05), duration=1.0)

    assert gate.mode is Mode.SETTLING


@pytest.mark.parametrize("gap", [MOVING, None])
def test_a_moving_or_missing_sample_restarts_the_settle_wait(gap):
    gate = _gate()
    gate.request(True)
    now = _settle(gate, 10.0, duration=0.2)

    gate.update(gap, now + TICK_SEC)
    now = _settle(gate, now + TICK_SEC, duration=0.2)
    waiting = gate.mode
    _settle(gate, now, duration=0.2)

    assert waiting is Mode.SETTLING
    assert gate.mode is Mode.MANIPULATE


def test_settling_without_odom_waits_forever_and_outputs_zero():
    gate = _gate()
    gate.request(True)

    _settle(gate, 10.0, speed=None, duration=5.0)

    assert gate.mode is Mode.SETTLING
    assert gate.select((NAV, 15.0), (MANIP, 15.0), 15.0) == gate_mod.ZERO_TWIST


def test_a_clock_jump_back_restarts_the_settle_wait():
    gate = _gate()
    gate.request(True)
    gate.update(STILL, 10.0)

    jumped = gate.update(STILL, 5.0)
    later = gate.update(STILL, 5.0 + SETTLE_SEC - TICK_SEC)

    assert jumped is Mode.SETTLING
    assert later is Mode.SETTLING


def _manipulating(now=10.0):
    gate = _gate()
    gate.request(True)
    _settle(gate, now)
    assert gate.mode is Mode.MANIPULATE
    return gate


def test_manipulate_passes_the_manip_twist_without_angular_z():
    gate = _manipulating()

    twist = gate.select((NAV, 11.0), ((0.05, 0.03, 0.4), 11.0), 11.0)

    assert twist == (0.05, 0.03, 0.0)


def test_manipulate_scales_a_fast_twist_to_the_cap_keeping_its_direction():
    gate = _manipulating()

    vx, vy, wz = gate.select(None, ((0.3, 0.3, 0.4), 11.0), 11.0)

    assert math.hypot(vx, vy) == pytest.approx(MANIP_MAX_SPEED_MPS)
    assert vx == pytest.approx(0.106066, abs=1e-6)
    assert vy == pytest.approx(vx)
    assert wz == 0.0


def test_manipulate_ignores_nav_twists():
    gate = _manipulating()

    twist = gate.select((NAV, 11.0), None, 11.0)

    assert twist == gate_mod.ZERO_TWIST


def test_manipulate_treats_a_stale_manip_twist_as_a_stop():
    gate = _manipulating()

    twist = gate.select(None, (MANIP, 11.0), 11.0 + CMD_TIMEOUT_SEC + 0.01)

    assert twist == gate_mod.ZERO_TWIST


@pytest.mark.parametrize("bad", [(math.nan, 0.0, 0.0), (0.0, math.inf, 0.0)])
def test_non_finite_twists_are_zero(bad):
    navigating = _gate()
    manipulating = _manipulating()

    assert navigating.select((bad, 11.0), None, 11.0) == gate_mod.ZERO_TWIST
    assert manipulating.select(None, (bad, 11.0), 11.0) == gate_mod.ZERO_TWIST


def test_release_goes_through_settling_back_to_navigate():
    gate = _manipulating()

    target = gate.request(False)
    settling = gate.mode
    now = _settle(gate, 11.0)

    assert target is Mode.NAVIGATE
    assert settling is Mode.SETTLING
    assert gate.mode is Mode.NAVIGATE
    assert gate.select((NAV, now), None, now) == NAV


@pytest.mark.parametrize("manipulate", [False, True])
def test_a_request_for_the_current_owner_changes_nothing(manipulate):
    gate = _manipulating() if manipulate else _gate()
    before = gate.mode

    target = gate.request(manipulate)

    assert target is before
    assert gate.mode is before


def test_reverting_a_request_during_settling_settles_towards_the_previous_owner():
    gate = _gate()
    gate.request(True)

    target = gate.request(False)
    settling = gate.mode
    _settle(gate, 10.0)

    assert target is Mode.NAVIGATE
    assert settling is Mode.SETTLING
    assert gate.mode is Mode.NAVIGATE


def test_a_twist_from_before_the_change_of_owner_does_not_count():
    gate = _gate()
    gate.request(True)
    early = (MANIP, 10.01)  # sent while the base was still settling

    now = _settle(gate, 10.0)
    stale = gate.select(None, early, now)
    fresh = gate.select(None, (MANIP, now), now)

    assert gate.mode is Mode.MANIPULATE
    assert stale == gate_mod.ZERO_TWIST
    assert fresh == MANIP


def test_a_clock_jump_back_after_a_change_of_owner_does_not_block_new_twists():
    gate = _manipulating(now=500.0)

    twist = gate.select(None, (MANIP, 5.0), 5.0)  # sim time reset

    assert twist == MANIP


def test_gate_rejects_non_positive_settings():
    with pytest.raises(ValueError):
        gate_mod.HandoverGate(0.0, 0.005, 0.01, 0.5, 0.15)


# ---- node ------------------------------------------------------------------


class _Logger:
    def __init__(self):
        self.infos, self.warnings = [], []

    def info(self, text):
        self.infos.append(text)

    def warn(self, text):
        self.warnings.append(text)


class _Harness:
    """A BaseHandoverGate with recording publishers, a fake clock and logger."""

    def __init__(self, node):
        self.node = node
        self.clock = [100.0]
        self.log = _Logger()
        self.twists, self.modes = [], []
        node._now = lambda: self.clock[0]
        node.get_logger = lambda: self.log
        node._cmd_pub = SimpleNamespace(publish=self.twists.append)
        node._mode_pub = SimpleNamespace(publish=self.modes.append)

    @staticmethod
    def _twist(vx, vy, wz):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        return msg

    def nav(self, vx=0.0, vy=0.0, wz=0.0):
        self.node._on_nav(self._twist(vx, vy, wz))

    def manip(self, vx=0.0, vy=0.0, wz=0.0):
        self.node._on_manip(self._twist(vx, vy, wz))

    def odom(self, vx=0.0, vy=0.0, wz=0.0):
        msg = Odometry()
        msg.twist.twist = self._twist(vx, vy, wz)
        self.node._on_odom(msg)

    def request(self, manipulate):
        request = SetBool.Request()
        request.data = manipulate
        return self.node._on_set_manipulate(request, SetBool.Response())

    def govern(self, governed):
        request = SetBool.Request()
        request.data = governed
        return self.node._on_set_governed(request, SetBool.Response())

    def scale(self, value):
        self.node._on_speed_scale(Float64(data=float(value)))

    def tick(self, advance_sec=TICK_SEC):
        self.clock[0] += advance_sec
        self.node._tick()
        twist = self.twists[-1]
        return twist.linear.x, twist.linear.y, twist.angular.z

    def settle(self, seconds=SETTLE_SEC + 3 * TICK_SEC):
        for _ in range(round(seconds / TICK_SEC)):
            self.odom()
            self.tick()


@pytest.fixture
def make_node():
    rclpy.init()
    nodes = []

    def make(**params):
        overrides = [Parameter(name, value=value) for name, value in params.items()]
        nodes.append(gate_mod.BaseHandoverGate(parameter_overrides=overrides))
        return _Harness(nodes[-1])

    try:
        yield make
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def test_node_publishes_a_twist_every_tick_zeros_included(make_node):
    sim = make_node()

    first = sim.tick()
    second = sim.tick()

    assert first == second == (0.0, 0.0, 0.0)
    assert len(sim.twists) == 2


def test_node_passes_nav_through_in_navigate(make_node):
    sim = make_node()
    sim.nav(0.2, 0.0, 0.5)

    assert sim.tick() == (0.2, 0.0, 0.5)


def test_node_publishes_the_mode_on_the_first_tick_and_once_per_change(make_node):
    sim = make_node()

    sim.tick()
    sim.tick()
    sim.request(True)
    sim.tick()
    sim.settle()

    assert [msg.data for msg in sim.modes] == ["NAVIGATE", "SETTLING", "MANIPULATE"]
    assert sim.log.infos[-1] == "Base owner: MANIPULATE"


def test_node_hands_the_base_to_the_manip_input_once_still(make_node):
    sim = make_node()
    sim.request(True)

    sim.settle()
    sim.nav(0.2)
    sim.manip(0.05, 0.03, 0.4)

    assert sim.tick() == (0.05, 0.03, 0.0)


def test_node_does_not_hand_over_while_the_base_moves(make_node):
    sim = make_node()
    sim.request(True)

    for _ in range(50):
        sim.odom(vx=0.05)
        sim.manip(0.05)
        twist = sim.tick()

    assert twist == (0.0, 0.0, 0.0)
    assert sim.modes[-1].data == "SETTLING"


def test_node_treats_stale_odom_as_missing(make_node):
    sim = make_node(odom_timeout=0.5)
    sim.request(True)
    sim.odom()

    for _ in range(50):
        sim.tick()

    assert sim.modes[-1].data == "SETTLING"
    assert any("odom" in text for text in sim.log.warnings)


@pytest.mark.parametrize("manipulate, mode", [(True, "MANIPULATE"), (False, "NAVIGATE")])
def test_service_answers_with_the_mode_it_settles_towards(make_node, manipulate, mode):
    sim = make_node()
    if not manipulate:
        sim.request(True)
        sim.settle()

    response = sim.request(manipulate)

    assert response.success
    assert mode in response.message


def test_node_warns_once_about_a_non_finite_twist(make_node):
    sim = make_node()

    sim.nav(math.nan)
    first = sim.tick()
    sim.nav(math.nan)
    sim.tick()

    assert first == (0.0, 0.0, 0.0)
    assert len([w for w in sim.log.warnings if "non-finite" in w]) == 1


@pytest.mark.parametrize(
    "name", ["publish_rate", "cmd_timeout", "odom_timeout", "settle_sec", "manip_max_speed"]
)
def test_node_rejects_non_positive_parameters(make_node, name):
    with pytest.raises(ValueError, match=name):
        make_node(**{name: 0.0})


# ---- governed speed scale (docs/sim_handover_baseline.md) ---------------------------


def _settled_in_manipulate(now=10.0):
    gate = _gate()
    gate.request(True)
    return gate, _settle(gate, now)


def test_select_without_a_scale_is_the_ungoverned_select():
    gate, now = _gate(), 10.0
    fast_manip = (0.3, 0.2, 0.4)
    cases = [((NAV, now), None, now + 0.1), ((NAV, now), None, now + 1.0), (None, None, now)]
    for nav, manip, at in cases:
        assert gate.select(nav, manip, at, None) == gate.select(nav, manip, at)
    gate.request(True)
    assert gate.select((NAV, now), None, now, None) == gate.select((NAV, now), None, now)
    gate, now = _settled_in_manipulate()
    for manip in ((MANIP, now), (fast_manip, now), (MANIP, now - 1.0)):
        assert gate.select(None, manip, now + 0.01, None) == gate.select(None, manip, now + 0.01)


def test_a_zero_scale_stops_every_mode():
    gate, now = _gate(), 10.0
    assert gate.select((NAV, now), None, now, 0.0) == gate_mod.ZERO_TWIST
    gate.request(True)
    assert gate.select((NAV, now), (MANIP, now), now, 0.0) == gate_mod.ZERO_TWIST
    gate, now = _settled_in_manipulate()
    assert gate.select(None, (MANIP, now), now, 0.0) == gate_mod.ZERO_TWIST
    assert gate.select(None, (MANIP, now), now, math.nan) == gate_mod.ZERO_TWIST


def test_a_scaled_nav_twist_is_clamped_with_its_rotation():
    gate, now = _gate(), 10.0

    twist = gate.select(((0.26, 0.0, 0.5), now), None, now, 0.5)

    assert twist == pytest.approx((0.13, 0.0, 0.25))


def test_a_nav_twist_under_the_scaled_limit_passes_unchanged():
    gate, now = _gate(), 10.0

    assert gate.select(((0.1, 0.0, 0.3), now), None, now, 0.5) == (0.1, 0.0, 0.3)


def test_a_pure_rotation_passes_any_positive_scale_and_stops_at_zero():
    gate, now = _gate(), 10.0
    turn = ((0.0, 0.0, 1.4), now)

    assert gate.select(turn, None, now, 0.5) == (0.0, 0.0, 1.4)
    assert gate.select(turn, None, now, 0.0) == gate_mod.ZERO_TWIST


def test_a_manip_twist_planned_under_the_scaled_bound_passes_unchanged():
    gate, now = _settled_in_manipulate()

    assert gate.select(None, ((0.05, 0.05, 0.0), now), now, 0.5) == (0.05, 0.05, 0.0)


def test_a_fast_manip_twist_is_capped_at_the_scaled_speed():
    gate, now = _settled_in_manipulate()

    vx, vy, _ = gate.select(None, ((0.3, 0.0, 0.0), now), now, 0.5)

    assert (vx, vy) == pytest.approx((MANIP_MAX_SPEED_MPS * 0.5, 0.0))


def test_the_nav_limit_must_be_positive():
    with pytest.raises(ValueError):
        gate_mod.HandoverGate(0.3, 0.005, 0.01, 0.5, 0.15, nav_max_speed_mps=0.0)


def test_an_ungoverned_node_ignores_the_speed_scale(make_node):
    sim = make_node()
    sim.scale(0.0)
    sim.nav(0.2, 0.0, 0.5)

    assert sim.tick() == (0.2, 0.0, 0.5)


def test_a_governed_node_without_a_scale_stops_and_warns_once(make_node):
    sim = make_node()
    response = sim.govern(True)
    sim.nav(0.2, 0.0, 0.5)

    first = sim.tick()
    sim.nav(0.2, 0.0, 0.5)
    second = sim.tick()

    assert response.success and "governed" in response.message
    assert first == second == (0.0, 0.0, 0.0)
    assert len([w for w in sim.log.warnings if "set_governed" in w]) == 1


def test_a_governed_node_with_a_fresh_full_scale_passes_nav(make_node):
    sim = make_node()
    sim.govern(True)
    sim.scale(1.0)
    sim.nav(0.2, 0.0, 0.5)

    assert sim.tick() == (0.2, 0.0, 0.5)


def test_a_governed_node_clamps_nav_by_the_scale(make_node):
    sim = make_node()
    sim.govern(True)
    sim.scale(0.5)
    sim.nav(0.26, 0.0, 0.5)

    assert sim.tick() == pytest.approx((0.13, 0.0, 0.25))


def test_a_stale_scale_stops_a_governed_node(make_node):
    sim = make_node()
    sim.govern(True)
    sim.scale(1.0)

    sim.nav(0.2)
    sim.tick(advance_sec=0.6)

    assert sim.twists[-1].linear.x == 0.0


def test_ungoverning_restores_the_plain_gate(make_node):
    sim = make_node()
    sim.govern(True)
    sim.nav(0.2)
    sim.tick()

    response = sim.govern(False)
    sim.nav(0.2)

    assert response.success and "ungoverned" in response.message
    assert sim.tick() == (0.2, 0.0, 0.0)


@pytest.mark.parametrize("name", ["nav_max_speed", "speed_scale_timeout"])
def test_node_rejects_non_positive_governance_parameters(make_node, name):
    with pytest.raises(ValueError, match=name):
        make_node(**{name: 0.0})
