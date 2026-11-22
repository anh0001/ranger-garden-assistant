"""
Unit tests for scripts/whole_body_mpc.py, the node around the MPC core (no simulator).

The node's callbacks and timer are called directly with a fake clock,
logger and publishers; nothing spins. Needs the install space sourced
(xacro) and casadi; skipped without casadi.
"""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
import xacro
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String

pytest.importorskip("casadi")

PKG = Path(__file__).resolve().parents[1]
SIM_XACRO = PKG / "urdf" / "ranger_gazebo.urdf.xacro"
SCRIPT = PKG / "scripts" / "whole_body_mpc.py"
_spec = importlib.util.spec_from_file_location("whole_body_mpc", SCRIPT)
node_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(node_mod)
core = node_mod.core

BOX = [-0.40, -0.20, 0.52, 0.30]
CARRY = [0.0, 0.719, -1.496, 0.0, 0.864, 0.0]
FAR_TARGET = (0.95, 0.15, 1.00)
PERIOD_SEC = 0.05  # 20 Hz


@pytest.fixture(scope="module")
def urdf_xml():
    return xacro.process_file(str(SIM_XACRO)).toxml()


@pytest.fixture(scope="module")
def model(urdf_xml):
    return core.build_arm_model(urdf_xml)


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
    """A WholeBodyMpcNode with recording publishers, a fake clock and logger."""

    def __init__(self, node, urdf_xml):
        self.node = node
        self.urdf_xml = urdf_xml
        self.clock = [100.0]
        self.log = _Logger()
        self.twists, self.arm, self.states, self.errors = [], [], [], []
        node._now = lambda: self.clock[0]
        node.get_logger = lambda: self.log
        node._cmd_pub = SimpleNamespace(publish=self.twists.append)
        node._arm_pub = SimpleNamespace(publish=self.arm.append)
        node._state_pub = SimpleNamespace(publish=self.states.append)
        node._error_pub = SimpleNamespace(publish=self.errors.append)
        self.base = (0.0, 0.0, 0.0)
        self.q = list(CARRY)

    def description(self):
        self.node._on_robot_description(String(data=self.urdf_xml))

    def odom(self, x=None, y=None, yaw=None):
        bx, by, byaw = self.base
        self.base = (bx if x is None else x, by if y is None else y,
                     byaw if yaw is None else yaw)
        msg = Odometry()
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.base[:2]
        msg.pose.pose.orientation.z = math.sin(self.base[2] / 2)
        msg.pose.pose.orientation.w = math.cos(self.base[2] / 2)
        self.node._on_odom(msg)

    def joints(self, q=None):
        self.q = list(self.q if q is None else q)
        msg = JointState()
        msg.name = list(core.ARM_JOINTS) + ["piper_joint7"]
        msg.position = self.q + [0.0]
        self.node._on_joint_states(msg)

    def mode(self, value):
        self.node._on_mode(String(data=value))

    def target(self, position=FAR_TARGET, yaw=0.0, frame="odom"):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        qx, qy, qz, qw = core.level_quaternion(yaw)
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self.node._on_target(msg)

    def tick(self, advance_sec=PERIOD_SEC, fresh=True):
        self.clock[0] += advance_sec
        if fresh:
            self.odom()
            self.joints()
        self.node._tick()

    @property
    def state(self):
        return self.states[-1].data if self.states else None

    def ready(self, position=FAR_TARGET):
        """Model, fresh odom and joints, MANIPULATE and a target."""
        self.description()
        self.odom()
        self.joints()
        self.mode("MANIPULATE")
        self.target(position)


@pytest.fixture
def make_node(urdf_xml):
    rclpy.init()
    nodes = []

    def make(**params):
        params = {"base_box": BOX, **params}
        overrides = [Parameter(name, value=value) for name, value in params.items()]
        nodes.append(node_mod.WholeBodyMpcNode(parameter_overrides=overrides))
        return _Harness(nodes[-1], urdf_xml)

    try:
        yield make
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def _tcp(model, sim):
    bx, by, yaw = sim.base
    return core.tcp_pose(model, (bx, by), yaw, sim.q)[0]


# ---- IDLE --------------------------------------------------------------------


def test_node_without_a_robot_model_stays_idle_and_commands_nothing(make_node):
    sim = make_node()
    sim.odom()
    sim.joints()
    sim.mode("MANIPULATE")
    sim.target()

    sim.tick()

    assert sim.state == "IDLE"
    assert sim.twists == [] and sim.arm == []


def test_node_without_manipulate_stays_idle(make_node):
    sim = make_node()
    sim.ready()
    sim.mode("NAVIGATE")

    sim.tick()

    assert sim.state == "IDLE"
    assert sim.twists == [] and sim.arm == []


def test_node_without_a_target_stays_idle(make_node):
    sim = make_node()
    sim.description()
    sim.mode("MANIPULATE")

    sim.tick()

    assert sim.state == "IDLE"
    assert sim.twists == [] and sim.arm == []


def test_a_target_outside_odom_is_ignored_with_one_warning(make_node):
    sim = make_node()
    sim.description()
    sim.mode("MANIPULATE")

    sim.target(frame="map")
    sim.target(frame="map")
    sim.tick()

    assert sim.state == "IDLE"
    assert len([w for w in sim.log.warnings if "map" in w]) == 1


# ---- TRACKING ------------------------------------------------------------------


def test_first_tick_tracks_with_a_planar_twist_and_six_joint_positions(make_node, model):
    sim = make_node()
    sim.ready()

    sim.tick()

    assert sim.state == "TRACKING"
    twist = sim.twists[-1]
    assert twist.angular.z == 0.0 and twist.linear.z == 0.0
    assert math.hypot(twist.linear.x, twist.linear.y) > 0.0
    command = np.array(sim.arm[-1].data)
    assert command.shape == (6,)
    assert np.all(np.isfinite(command))
    assert np.all(command >= model.lower) and np.all(command <= model.upper)
    # q_cmd starts from the measured joints and moves at most one joint step.
    assert np.max(np.abs(command - CARRY)) <= 0.6 * PERIOD_SEC + 1e-9


def test_tracking_publishes_the_tracking_error(make_node):
    sim = make_node()
    sim.ready()

    sim.tick()

    position_m, angle_rad = sim.errors[-1].data
    assert position_m > 0.1
    assert angle_rad >= 0.0


def _fixed_solution(u0):
    return core.Solution(np.array(u0, dtype=float), np.zeros((21, 8)), True, 1.0)


def test_a_base_speed_below_the_deadband_is_sent_as_zero(make_node, monkeypatch):
    sim = make_node()
    sim.ready()
    sim.tick()
    u0 = [0.005, 0.005, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
    monkeypatch.setattr(sim.node._mpc, "solve", lambda *args: _fixed_solution(u0))
    before = list(sim.arm[-1].data)

    sim.tick()

    twist = sim.twists[-1]
    assert (twist.linear.x, twist.linear.y) == (0.0, 0.0)
    assert sim.arm[-1].data[0] == pytest.approx(before[0] + 0.1 * PERIOD_SEC)


# ---- REACHED -------------------------------------------------------------------


def test_reached_latches_and_only_reengages_beyond_three_tolerances(make_node, model):
    sim = make_node()
    sim.description()
    sim.odom()
    sim.joints()
    sim.mode("MANIPULATE")
    sim.target(tuple(_tcp(model, sim) + [0.004, 0.0, 0.0]))  # 4 mm ahead

    sim.tick()
    reached = sim.state
    held = list(sim.arm[-1].data)
    sim.odom(x=-0.006)  # 10 mm
    sim.tick()
    still = sim.state
    sim.odom(x=-0.012)  # 16 mm
    sim.tick()

    assert reached == still == "REACHED"
    assert held == pytest.approx(CARRY)
    assert sim.state == "TRACKING"


def test_reached_logs_the_solve_times_and_the_base_travel(make_node, model):
    sim = make_node()
    sim.description()
    sim.odom()
    sim.joints()
    sim.mode("MANIPULATE")
    sim.target(tuple(_tcp(model, sim) + [0.03, 0.0, 0.0]))  # 30 mm: one solve, then reached
    sim.tick()

    sim.odom(x=0.03)
    sim.tick()

    reached = [text for text in sim.log.infos if "-> REACHED" in text]
    assert len(reached) == 1
    assert "1 solves" in reached[0] and "median" in reached[0]
    assert "base moved 0.030 m" in reached[0]


def test_reached_holds_the_base_and_the_arm(make_node, model):
    sim = make_node()
    sim.description()
    sim.odom()
    sim.joints()
    sim.mode("MANIPULATE")
    sim.target(tuple(_tcp(model, sim)))

    for _ in range(3):
        sim.tick()

    assert sim.state == "REACHED"
    assert all(t.linear.x == t.linear.y == 0.0 for t in sim.twists)
    assert all(list(a.data) == pytest.approx(CARRY) for a in sim.arm)
    assert any("REACHED" in text and "mm" in text for text in sim.log.infos)


# ---- FAULT ---------------------------------------------------------------------


def _assert_fault(sim, reason, held):
    assert sim.state.startswith("FAULT: ")
    assert reason in sim.state
    twist = sim.twists[-1]
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)
    assert list(sim.arm[-1].data) == pytest.approx(held)


def test_stale_odom_is_a_fault(make_node):
    sim = make_node()
    sim.ready()
    sim.tick()
    held = list(sim.arm[-1].data)

    sim.clock[0] += 0.4
    sim.joints()
    sim.node._tick()

    _assert_fault(sim, "odom", held)


def test_stale_joint_states_are_a_fault(make_node):
    sim = make_node()
    sim.ready()
    sim.tick()
    held = list(sim.arm[-1].data)

    sim.clock[0] += 0.4
    sim.odom()
    sim.node._tick()

    _assert_fault(sim, "joint", held)


def _failed_solution():
    return core.Solution(np.zeros(8), np.zeros((21, 8)), False, 1.0)


def test_three_failed_solves_in_a_row_are_a_fault(make_node, monkeypatch):
    sim = make_node()
    sim.ready()
    sim.tick()
    held = list(sim.arm[-1].data)
    monkeypatch.setattr(sim.node._mpc, "solve", lambda *args: _failed_solution())

    sim.tick()
    sim.tick()
    after_two = sim.state
    sim.tick()

    assert after_two == "TRACKING"
    _assert_fault(sim, "solver", held)


def test_one_failed_solve_keeps_tracking_with_a_stop(make_node, monkeypatch):
    sim = make_node()
    sim.ready()
    sim.tick()
    monkeypatch.setattr(sim.node._mpc, "solve", lambda *args: _failed_solution())

    sim.tick()

    assert sim.state == "TRACKING"
    twist = sim.twists[-1]
    assert (twist.linear.x, twist.linear.y) == (0.0, 0.0)


def test_command_lag_is_a_fault_that_holds_the_arm_where_it_is(make_node, model):
    sim = make_node()
    sim.ready()
    sim.tick()
    measured = [v + 0.2 for v in CARRY]  # far from the command: e.g. the arm is blocked

    sim.joints(measured)
    sim.tick(fresh=False)

    # Holding the old command would keep pushing the arm into whatever stopped it.
    _assert_fault(sim, "lag", list(np.clip(measured, model.lower, model.upper)))


def test_non_finite_joint_states_are_never_commanded(make_node):
    sim = make_node()
    sim.description()
    sim.odom()
    sim.joints([math.nan] * 6)
    sim.mode("MANIPULATE")
    sim.target()

    sim.tick(fresh=False)

    assert sim.state.startswith("FAULT: ") and "joint" in sim.state
    assert all(all(math.isfinite(v) for v in a.data) for a in sim.arm)


def test_non_finite_odom_is_a_fault(make_node):
    sim = make_node()
    sim.description()
    sim.odom(x=math.nan)
    sim.joints()
    sim.mode("MANIPULATE")
    sim.target()

    sim.node._tick()

    assert sim.state.startswith("FAULT: ") and "odom" in sim.state


def test_a_target_without_a_rotation_is_ignored_with_a_warning(make_node):
    sim = make_node()
    sim.description()
    sim.mode("MANIPULATE")
    msg = PoseStamped()
    msg.header.frame_id = "odom"
    msg.pose.position.x = 0.9
    msg.pose.orientation.w = 0.0  # all-zero quaternion (the default is the identity)

    sim.node._on_target(msg)
    sim.tick()

    assert sim.state == "IDLE"
    assert any("quaternion" in w for w in sim.log.warnings)


def test_a_new_target_reengages_inside_the_latch(make_node, model):
    sim = make_node()
    sim.description()
    sim.odom()
    sim.joints()
    sim.mode("MANIPULATE")
    tcp = _tcp(model, sim)
    sim.target(tuple(tcp))
    sim.tick()
    reached = sim.state

    sim.target(tuple(tcp + [0.010, 0.0, 0.0]))  # 10 mm: inside 3x the tolerance
    sim.tick()

    assert reached == "REACHED"
    assert sim.state == "TRACKING"


def test_an_unusable_description_keeps_the_node_idle(make_node):
    sim = make_node()
    sim.node._on_robot_description(String(data="<robot name='x'/>"))
    sim.mode("MANIPULATE")
    sim.target()

    sim.tick()

    assert sim.state == "IDLE"
    assert any("robot description unusable" in e for e in sim.log.errors)


def test_a_fault_holds_while_manipulate_lasts(make_node):
    sim = make_node()
    sim.ready()
    sim.tick()
    sim.clock[0] += 0.4
    sim.node._tick()

    sim.tick()  # fresh odom and joints again

    assert sim.state.startswith("FAULT: ")


def test_leaving_manipulate_clears_the_fault_and_the_target(make_node):
    sim = make_node()
    sim.ready()
    sim.tick()
    sim.clock[0] += 0.4
    sim.node._tick()

    sim.mode("NAVIGATE")
    sim.tick()
    idle = sim.state
    sim.mode("MANIPULATE")
    published = len(sim.twists)
    sim.tick()

    assert idle == "IDLE"
    assert sim.state == "IDLE"
    assert len(sim.twists) == published  # no target: nothing commanded


# ---- parameters and main ---------------------------------------------------------


@pytest.mark.parametrize("box", [[0.0, 1.0], [0.5, -0.2, 0.0, 0.3]])
def test_a_bad_base_box_fails_at_start(make_node, box):
    with pytest.raises(ValueError, match="base_box"):
        make_node(base_box=box)


def test_a_missing_base_box_fails_at_start():
    rclpy.init()
    try:
        with pytest.raises(ValueError, match="base_box"):
            node_mod.WholeBodyMpcNode()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize("name", ["rate", "odom_timeout", "max_command_lag"])
def test_non_positive_settings_fail_at_start(make_node, name):
    with pytest.raises(ValueError, match=name):
        make_node(**{name: 0.0})


def test_main_without_casadi_exits_with_the_install_hint(monkeypatch):
    monkeypatch.setattr(node_mod, "core", None)

    with pytest.raises(SystemExit) as stopped:
        node_mod.main()

    assert "pip3 install --user casadi" in str(stopped.value.code)
