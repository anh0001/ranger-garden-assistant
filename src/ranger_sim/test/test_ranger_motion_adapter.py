"""Unit tests for scripts/ranger_motion_adapter.py (no simulator required)."""

import importlib.util
import itertools
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "ranger_motion_adapter.py"
_spec = importlib.util.spec_from_file_location("ranger_motion_adapter", SCRIPT)
adapter = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(adapter)

STRAIGHT = [0.0, 0.0, 0.0, 0.0]
# Twists covering every mode, both signs and the speed caps.
TWIST_GRID = [
    twist
    for twist in itertools.product(
        (-2.0, -0.3, 0.0, 0.05, 0.4), (-0.2, 0.0, 0.1), (-6.0, -1.0, 0.0, 0.2, 0.7)
    )
    if any(twist)
]


def _wheel_velocity(heading, speed_rps):
    """Ground velocity (x, y) of a wheel steered to `heading`, rolling at `speed_rps`."""
    rim = speed_rps * adapter.WHEEL_RADIUS_M
    return rim * math.cos(heading), rim * math.sin(heading)


# ---- mode selection and wheel kinematics -----------------------------------


def test_forward_twist_selects_dual_ackermann_and_keeps_wheels_straight():
    mode, body = adapter.select_mode(0.3, 0.0, 0.0)

    targets = adapter.wheel_targets(body, STRAIGHT)

    assert mode == adapter.Mode.DUAL_ACKERMANN == 0
    assert body == (0.3, 0.0, 0.0)
    assert [heading for heading, _ in targets] == [0.0] * 4
    assert [speed for _, speed in targets] == pytest.approx([0.3 / 0.09] * 4)


def test_lateral_twist_selects_parallel_and_drops_the_angular_part():
    mode, body = adapter.select_mode(0.2, 0.1, 0.5)

    targets = adapter.wheel_targets(body, STRAIGHT)

    assert mode == adapter.Mode.PARALLEL == 1
    assert body == (0.2, 0.1, 0.0)
    assert [heading for heading, _ in targets] == pytest.approx([math.atan2(0.1, 0.2)] * 4)
    assert [speed for _, speed in targets] == pytest.approx([math.hypot(0.2, 0.1) / 0.09] * 4)


def test_pure_side_slip_steers_ninety_degrees():
    mode, body = adapter.select_mode(0.0, 0.1, 0.0)

    targets = adapter.wheel_targets(body, STRAIGHT)

    assert mode == adapter.Mode.PARALLEL
    assert [abs(heading) for heading, _ in targets] == pytest.approx([math.pi / 2] * 4)


def test_side_slip_reversal_flips_wheel_speed_not_steering():
    _, body = adapter.select_mode(0.0, -0.1, 0.0)

    targets = adapter.wheel_targets(body, [math.pi / 2] * 4)

    assert [heading for heading, _ in targets] == pytest.approx([math.pi / 2] * 4)
    assert all(speed < 0.0 for _, speed in targets)


def test_side_slip_follows_the_previous_heading_on_either_side():
    _, body = adapter.select_mode(0.0, 0.1, 0.0)

    targets = adapter.wheel_targets(body, [-math.pi / 2] * 4)

    assert [heading for heading, _ in targets] == pytest.approx([-math.pi / 2] * 4)
    assert all(speed < 0.0 for _, speed in targets)


def test_tight_turn_selects_spinning_and_drops_the_linear_part():
    mode, body = adapter.select_mode(0.2, 0.0, 1.0)

    assert mode == adapter.Mode.SPINNING == 2
    assert body == (0.0, 0.0, 1.0)


def test_wide_turn_stays_dual_ackermann():
    mode, body = adapter.select_mode(0.5, 0.0, 1.0)

    fl, fr, rl, rr = (heading for heading, _ in adapter.wheel_targets(body, STRAIGHT))

    assert mode == adapter.Mode.DUAL_ACKERMANN
    assert body == (0.5, 0.0, 1.0)
    assert fl > 0.0 and fr > 0.0
    assert rl < 0.0 and rr < 0.0
    assert abs(fl) > abs(fr)


@pytest.mark.parametrize(
    "radius_offset, expected",
    [(-1e-4, adapter.Mode.SPINNING), (1e-4, adapter.Mode.DUAL_ACKERMANN)],
)
def test_turn_radius_boundary_matches_the_driver(radius_offset, expected):
    wz = 0.8
    vx = (adapter.MIN_TURN_RADIUS_M + radius_offset) * wz

    mode, _ = adapter.select_mode(vx, 0.0, wz)

    assert mode == expected


def test_spinning_wheels_form_an_x_and_roll_tangentially():
    _, body = adapter.select_mode(0.0, 0.0, 1.0)

    targets = adapter.wheel_targets(body, STRAIGHT)

    (fl_heading, fl_speed), (fr_heading, fr_speed) = targets[0], targets[1]
    assert fl_heading == pytest.approx(-math.atan2(0.25, 0.19))
    assert fl_speed < 0.0
    assert fr_heading == pytest.approx(math.atan2(0.25, 0.19))
    assert fr_speed > 0.0
    for (heading, speed), (lx, ly) in zip(targets, adapter.WHEEL_POSITIONS_M):
        assert abs(heading) <= adapter.MAX_STEER_RAD
        assert abs(heading) == pytest.approx(math.atan2(abs(lx), abs(ly)))
        assert abs(speed) * adapter.WHEEL_RADIUS_M == pytest.approx(math.hypot(lx, ly))


@pytest.mark.parametrize("twist", TWIST_GRID)
def test_headings_never_exceed_the_steering_limit(twist):
    _, body = adapter.select_mode(*twist)

    targets = adapter.wheel_targets(body, STRAIGHT)

    assert all(abs(heading) <= adapter.MAX_STEER_RAD for heading, _ in targets)


@pytest.mark.parametrize("twist", TWIST_GRID)
def test_wheel_velocity_matches_the_body_velocity_at_the_wheel(twist):
    _, body = adapter.select_mode(*twist)
    vx, vy, wz = body

    targets = adapter.wheel_targets(body, STRAIGHT)

    for (heading, speed), (lx, ly) in zip(targets, adapter.WHEEL_POSITIONS_M):
        expected = (vx - wz * ly, vy + wz * lx)
        assert _wheel_velocity(heading, speed) == pytest.approx(expected, abs=1e-9)


def test_zero_wheel_speed_keeps_the_previous_heading():
    previous = [0.3, -0.2, 0.1, 1.2]

    targets = adapter.wheel_targets((0.0, 0.0, 0.0), previous)

    assert targets == [(0.3, 0.0), (-0.2, 0.0), (0.1, 0.0), (1.2, 0.0)]


def test_wheel_targets_does_not_mutate_the_previous_headings():
    previous = [0.3, -0.2, 0.1, 1.2]

    adapter.wheel_targets((0.0, 0.1, 0.0), previous)

    assert previous == [0.3, -0.2, 0.1, 1.2]


def test_linear_speed_is_capped():
    _, body = adapter.select_mode(3.0, 0.0, 0.0)

    assert body == pytest.approx((1.5, 0.0, 0.0))


def test_diagonal_speed_is_capped_without_changing_direction():
    _, body = adapter.select_mode(3.0, 4.0, 0.0)

    assert math.hypot(body[0], body[1]) == pytest.approx(adapter.MAX_LINEAR_SPEED_MPS)
    assert math.atan2(body[1], body[0]) == pytest.approx(math.atan2(4.0, 3.0))


def test_capped_ackermann_turn_keeps_its_radius():
    # The driver sets the steering from the turn radius and the speed
    # separately: capping the speed must not tighten the turn.
    mode, body = adapter.select_mode(3.0, 0.0, 2.0)

    assert mode == adapter.Mode.DUAL_ACKERMANN
    assert body[0] == pytest.approx(adapter.MAX_LINEAR_SPEED_MPS)
    assert body[0] / body[2] == pytest.approx(1.5)


def test_angular_speed_is_capped_when_spinning():
    mode, body = adapter.select_mode(0.0, 0.0, -9.0)

    assert mode == adapter.Mode.SPINNING
    assert body == (0.0, 0.0, -adapter.MAX_ANGULAR_SPEED_RPS)


def test_zero_twist_is_dual_ackermann_at_rest():
    mode, body = adapter.select_mode(0.0, 0.0, 0.0)

    assert mode == adapter.Mode.DUAL_ACKERMANN
    assert body == (0.0, 0.0, 0.0)


def test_steering_joint_position_inverts_the_heading():
    # The URDF steering axis is 0 0 -1: a left (+z) heading is a negative joint angle.
    assert adapter.steering_joint_position(0.5) == -0.5
    assert adapter.steering_joint_position(-1.2) == 1.2


def test_topic_names_follow_the_joint_names():
    assert adapter.steering_topic("fl_steering_joint") == "/ranger/fl_steering_joint/cmd_pos"
    assert adapter.wheel_topic("rr_wheel") == "/ranger/rr_wheel/cmd_vel"
    assert adapter.BODY_TOPIC == "/cmd_vel_body"


# ---- steering gate ---------------------------------------------------------


def _gate():
    return adapter.SteeringGate(enter_rad=0.20, exit_rad=0.05, timeout_sec=3.0)


def test_gate_allows_driving_when_steering_is_on_target():
    gate = _gate()

    driving = gate.update([0.5] * 4, [0.49] * 4, now_sec=0.0)

    assert driving
    assert not gate.aligning


def test_gate_blocks_driving_until_steering_arrives():
    gate = _gate()

    blocked = gate.update([1.0, 0.0, 0.0, 0.0], [0.0] * 4, now_sec=0.0)
    released = gate.update([1.0, 0.0, 0.0, 0.0], [0.96, 0.0, 0.0, 0.0], now_sec=0.5)

    assert not blocked
    assert released
    assert not gate.timed_out


def test_gate_hysteresis_ignores_small_steering_changes():
    gate = _gate()

    driving = gate.update([0.1] * 4, [0.0] * 4, now_sec=0.0)

    assert driving


def test_gate_stays_blocked_between_exit_and_enter_thresholds():
    gate = _gate()
    gate.update([1.0] * 4, [0.0] * 4, now_sec=0.0)

    driving = gate.update([1.0] * 4, [0.9] * 4, now_sec=0.5)

    assert not driving
    assert gate.aligning


def test_gate_times_out_and_reports_it():
    gate = _gate()
    gate.update([1.0] * 4, [0.0] * 4, now_sec=0.0)

    before = gate.update([1.0] * 4, [0.0] * 4, now_sec=2.9)
    after = gate.update([1.0] * 4, [0.0] * 4, now_sec=3.1)

    assert not before
    assert after
    assert gate.timed_out


def test_gate_stays_open_after_a_timeout_until_steering_catches_up():
    # A stuck steering joint must not turn the base into a 3 s stop-and-go.
    gate = _gate()
    gate.update([1.0] * 4, [0.0] * 4, now_sec=0.0)
    gate.update([1.0] * 4, [0.0] * 4, now_sec=3.1)

    still_stuck = gate.update([1.0] * 4, [0.0] * 4, now_sec=3.2)
    caught_up = gate.update([1.0] * 4, [1.0] * 4, now_sec=4.0)
    new_alignment = gate.update([-1.0] * 4, [1.0] * 4, now_sec=4.1)

    assert still_stuck
    assert caught_up
    assert not new_alignment
    assert not gate.timed_out


def test_gate_timeout_flag_resets_when_a_new_alignment_starts():
    gate = _gate()
    gate.update([1.0] * 4, [0.0] * 4, now_sec=0.0)
    gate.update([1.0] * 4, [0.0] * 4, now_sec=3.1)
    gate.update([1.0] * 4, [1.0] * 4, now_sec=3.2)

    gate.update([0.0] * 4, [1.0] * 4, now_sec=3.3)

    assert gate.aligning
    assert not gate.timed_out


def test_gate_passes_through_without_measurements():
    gate = _gate()

    driving = gate.update([1.0] * 4, None, now_sec=0.0)

    assert driving
    assert not gate.aligning


def test_gate_uses_only_the_measured_joints():
    gate = _gate()

    on_target = gate.update([1.0, 0.5, 0.0, 0.0], [None, 0.5, None, None], now_sec=0.0)
    off_target = gate.update([1.0, 0.5, 0.0, 0.0], [None, 0.0, None, None], now_sec=0.1)

    assert on_target
    assert not off_target


def test_gate_with_no_measured_joint_passes_through():
    gate = _gate()

    driving = gate.update([1.0] * 4, [None] * 4, now_sec=0.0)

    assert driving


# ---- node ------------------------------------------------------------------


class _Logger:
    def __init__(self):
        self.infos, self.warnings = [], []

    def info(self, text):
        self.infos.append(text)

    def warn(self, text):
        self.warnings.append(text)


class _Harness:
    """A RangerMotionAdapter with recording publishers, a fake clock and logger."""

    def __init__(self, node):
        self.node = node
        self.clock = [100.0]
        self.log = _Logger()
        self.body, self.steering, self.wheels = [], [[] for _ in range(4)], [[] for _ in range(4)]
        node._now = lambda: self.clock[0]
        node.get_logger = lambda: self.log
        node._body_pub = SimpleNamespace(publish=self.body.append)
        node._steering_pubs = [SimpleNamespace(publish=sent.append) for sent in self.steering]
        node._wheel_pubs = [SimpleNamespace(publish=sent.append) for sent in self.wheels]

    def command(self, vx=0.0, vy=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.node._on_cmd(msg)

    def joints(self, names, positions):
        msg = JointState()
        msg.name, msg.position = list(names), list(positions)
        self.node._on_joint_states(msg)

    def tick(self, advance_sec=0.02):
        self.clock[0] += advance_sec
        self.node._tick()
        body = self.body[-1]
        return (
            (body.linear.x, body.linear.y, body.angular.z),
            [sent[-1].data for sent in self.steering],
            [sent[-1].data for sent in self.wheels],
        )


@pytest.fixture
def make_node():
    rclpy.init()
    nodes = []

    def make(**params):
        overrides = [Parameter(name, value=value) for name, value in params.items()]
        nodes.append(adapter.RangerMotionAdapter(parameter_overrides=overrides))
        return _Harness(nodes[-1])

    try:
        yield make
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def _joint_targets(body, previous=STRAIGHT):
    targets = adapter.wheel_targets(body, previous)
    joints = [adapter.steering_joint_position(heading) for heading, _ in targets]
    return joints, [speed for _, speed in targets]


def test_node_publishes_zeros_every_tick_before_any_command(make_node):
    sim = make_node()

    first = sim.tick()
    second = sim.tick()

    assert first == second == ((0.0, 0.0, 0.0), [0.0] * 4, [0.0] * 4)
    assert len(sim.body) == 2


def test_node_steers_first_and_withholds_body_and_wheel_motion(make_node):
    sim = make_node()
    sim.joints(adapter.STEERING_JOINTS, STRAIGHT)
    sim.command(wz=1.0)

    body, steering, wheels = sim.tick()

    assert steering == pytest.approx(_joint_targets((0.0, 0.0, 1.0))[0])
    assert body == (0.0, 0.0, 0.0)
    assert wheels == [0.0] * 4


def test_node_drives_once_steering_reports_on_target(make_node):
    sim = make_node()
    sim.joints(adapter.STEERING_JOINTS, STRAIGHT)
    sim.command(wz=1.0)
    _, steering, _ = sim.tick()
    sim.joints(adapter.STEERING_JOINTS, steering)
    sim.command(wz=1.0)

    body, _, wheels = sim.tick()

    assert body == (0.0, 0.0, 1.0)
    assert wheels == pytest.approx(_joint_targets((0.0, 0.0, 1.0))[1])
    assert all(speed != 0.0 for speed in wheels)


def test_node_stops_when_cmd_vel_goes_quiet(make_node):
    sim = make_node()
    sim.command(vx=0.3)
    moving, _, _ = sim.tick()

    stopped = sim.tick(advance_sec=0.6)
    sim.tick()

    assert moving == (0.3, 0.0, 0.0)
    assert stopped == ((0.0, 0.0, 0.0), [0.0] * 4, [0.0] * 4)
    assert len(sim.log.warnings) == 2  # no joint states, then /cmd_vel quiet: once each


def test_node_holds_steering_on_a_zero_twist(make_node):
    sim = make_node()
    sim.command(vy=0.1)
    _, sideways, _ = sim.tick()
    sim.command()

    body, steering, wheels = sim.tick()

    assert sideways == pytest.approx([-math.pi / 2] * 4)
    assert steering == sideways
    assert body == (0.0, 0.0, 0.0)
    assert wheels == [0.0] * 4


def test_node_straightens_on_a_zero_twist_when_asked(make_node):
    sim = make_node(straighten_on_stop=True)
    sim.command(vy=0.1)
    sim.tick()
    sim.command()

    _, steering, _ = sim.tick()

    assert steering == [0.0] * 4


def test_node_drives_without_joint_states_and_warns_once(make_node):
    sim = make_node()
    sim.command(wz=1.0)

    first, _, wheels = sim.tick()
    second, _, _ = sim.tick()

    assert first == second == (0.0, 0.0, 1.0)
    assert all(speed != 0.0 for speed in wheels)
    assert len(sim.log.warnings) == 1


def test_node_ignores_joints_it_does_not_own(make_node):
    sim = make_node()
    sim.joints(["piper_joint1", "fl_steering_joint", "fr_steering_joint"], [0.7, 0.0, 0.0])
    sim.command(wz=1.0)

    body, _, _ = sim.tick()

    assert body == (0.0, 0.0, 0.0)  # the two reported joints are off target
    assert sim.log.warnings == []


def test_node_treats_a_non_finite_twist_as_zero_and_warns_once(make_node):
    sim = make_node()
    sim.command(vx=math.nan)
    first = sim.tick()
    sim.command(wz=math.inf)

    second = sim.tick()

    assert first == second == ((0.0, 0.0, 0.0), [0.0] * 4, [0.0] * 4)
    assert len(sim.log.warnings) == 1


def test_node_warns_once_when_the_steering_times_out(make_node):
    sim = make_node(align_timeout=1.0)
    sim.joints(adapter.STEERING_JOINTS, STRAIGHT)
    for _ in range(4):
        sim.command(wz=1.0)
        body, _, _ = sim.tick(advance_sec=0.4)

    assert body == (0.0, 0.0, 1.0)
    assert len(sim.log.warnings) == 1


def test_node_logs_each_mode_change_once(make_node):
    sim = make_node()
    for twist in [(0.3, 0.0, 0.0), (0.3, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, 1.0)]:
        sim.command(*twist)
        sim.tick()

    modes = [text for text in sim.log.infos if "mode" in text]

    assert len(modes) == 3  # DUAL_ACKERMANN, PARALLEL, SPINNING


def test_node_rejects_an_exit_threshold_above_the_enter_threshold(make_node):
    with pytest.raises(ValueError, match="align_exit_rad"):
        make_node(align_enter_rad=0.1, align_exit_rad=0.2)


def test_node_does_not_rewarn_about_missing_joint_states_after_a_stop(make_node):
    sim = make_node()
    sim.command(vx=0.3)
    sim.tick()
    sim.command()
    sim.tick()
    sim.command(vx=0.3)

    sim.tick()

    assert len(sim.log.warnings) == 1


def test_reverse_lateral_twist_selects_parallel_and_rolls_backwards():
    # The driver's steer formula differs here (it negates atan(vy/vx) for
    # vx < 0); the adapter steers the wheels to move the commanded way.
    mode, body = adapter.select_mode(-0.1, 0.1, 0.3)

    targets = adapter.wheel_targets(body, STRAIGHT)

    assert mode == adapter.Mode.PARALLEL
    assert body == (-0.1, 0.1, 0.0)
    assert [heading for heading, _ in targets] == pytest.approx([-math.pi / 4] * 4)
    assert all(speed < 0.0 for _, speed in targets)


def test_gate_ignores_non_finite_steering_readings():
    gate = _gate()

    driving = gate.update([1.0] * 4, [math.nan, 0.0, 0.0, 0.0], now_sec=0.0)

    assert not driving


def test_node_drives_until_the_watchdog_timeout_and_no_longer(make_node):
    sim = make_node()
    sim.command(vx=0.3)

    inside, _, _ = sim.tick(advance_sec=0.49)
    outside, _, _ = sim.tick(advance_sec=0.02)

    assert inside == (0.3, 0.0, 0.0)
    assert outside == (0.0, 0.0, 0.0)


def test_node_stops_when_the_clock_jumps_back(make_node):
    # A sim reset sends sim time backwards: the last command is not fresh.
    sim = make_node()
    sim.command(vx=0.3)
    sim.tick()

    sim.clock[0] = 1.0
    stopped, _, _ = sim.tick()

    assert stopped == (0.0, 0.0, 0.0)


def test_node_keeps_quiet_after_the_steering_timeout_warning(make_node):
    sim = make_node(align_timeout=1.0)
    sim.joints(adapter.STEERING_JOINTS, STRAIGHT)
    for _ in range(10):
        sim.command(wz=1.0)
        sim.tick(advance_sec=0.4)

    assert len(sim.log.warnings) == 1


def test_node_warns_again_when_the_steering_times_out_after_catching_up(make_node):
    sim = make_node(align_timeout=1.0)
    sim.joints(adapter.STEERING_JOINTS, STRAIGHT)
    for _ in range(4):
        sim.command(wz=1.0)
        _, spin_angles, _ = sim.tick(advance_sec=0.4)
    sim.joints(adapter.STEERING_JOINTS, spin_angles)  # caught up
    sim.command(wz=1.0)
    sim.tick(advance_sec=0.4)

    for _ in range(4):  # now asked to go sideways; the steering stays put
        sim.command(vy=0.1)
        sim.tick(advance_sec=0.4)

    assert len(sim.log.warnings) == 2
