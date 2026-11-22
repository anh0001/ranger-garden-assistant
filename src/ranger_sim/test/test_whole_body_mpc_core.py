"""
Unit tests for scripts/whole_body_mpc_core.py: kinematics, OCP and helpers (no simulator).

The closed-loop tests run the MPC against a kinematic plant: the base
translates at the commanded body velocity for one control period (heading
fixed, as in the driver's PARALLEL mode), the arm joints integrate the
commanded joint velocities. Needs the install space sourced (xacro) and
casadi (pip3 install --user casadi); skipped without casadi.
"""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest
import xacro

casadi = pytest.importorskip("casadi")

PKG = Path(__file__).resolve().parents[1]
SIM_XACRO = PKG / "urdf" / "ranger_gazebo.urdf.xacro"
CONTROL_PERIOD_SEC = 0.05  # the node's 20 Hz timer
TARGET = np.array([0.95, 0.15, 1.00])  # the experiment target in odom (config/wbmpc.yaml)
BOX = np.array([-0.40, -0.20, 0.52, 0.30])  # [x_min, y_min, x_max, y_max] in odom
POSITION_TOLERANCE_M = 0.005
ANGLE_TOLERANCE_RAD = math.radians(2.0)
CARRY = (0.0, 0.719, -1.496, 0.0, 0.864, 0.0)
HOUSE_HOVER = (0.0, 0.965, -1.576, 0.0, 0.698, 0.0)
HOUSE_GRASP = (0.0, 0.777, -1.081, 0.0, 0.392, 0.0)
GRASP = (0.0, 0.732, -0.943, 0.0, 0.298, 0.0)
HOME = (0.0, 1.2, -0.2, 0.0, -0.8, 0.0)


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


core = _load("whole_body_mpc_core", PKG / "scripts" / "whole_body_mpc_core.py")
geometry = _load("test_pick_place_geometry", PKG / "test" / "test_pick_place_geometry.py")


@pytest.fixture(scope="module")
def urdf_xml():
    return xacro.process_file(str(SIM_XACRO)).toxml()


@pytest.fixture(scope="module")
def robot(urdf_xml):
    return geometry.URDF.from_xml_string(urdf_xml)


@pytest.fixture(scope="module")
def model(urdf_xml):
    return core.build_arm_model(urdf_xml)


@pytest.fixture(scope="module")
def mpc(model):
    return core.WholeBodyMpc(model, core.MpcConfig())


@pytest.fixture
def fresh_mpc(mpc):
    mpc.reset()
    return mpc


def _rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _random_configurations(robot, count=20, seed=7):
    rng = np.random.default_rng(seed)
    limits = [robot.joint_map[name].limit for name in core.ARM_JOINTS]
    lower = np.array([limit.lower for limit in limits])
    upper = np.array([limit.upper for limit in limits])
    return [tuple(rng.uniform(lower, upper)) for _ in range(count)]


def _target(yaw=0.0, position=TARGET):
    return core.Target(position=np.array(position), rotation=core.level_rotation(yaw))


# ---- kinematics --------------------------------------------------------------


@pytest.mark.parametrize("q", [CARRY, GRASP, HOME, HOUSE_HOVER, HOUSE_GRASP])
def test_fk_equals_the_numpy_fk_at_the_waypoints(model, robot, q):
    expected = geometry._fk(robot, core.TCP_LINK, q)

    position, rotation = core.tcp_pose(model, (0.0, 0.0), 0.0, q)

    np.testing.assert_allclose(position, expected[:3, 3], atol=1e-9)
    np.testing.assert_allclose(rotation, expected[:3, :3], atol=1e-9)


def test_fk_equals_the_numpy_fk_for_random_configurations(model, robot):
    for q in _random_configurations(robot):
        expected = geometry._fk(robot, core.TCP_LINK, q)

        position, rotation = core.tcp_pose(model, (0.0, 0.0), 0.0, q)

        np.testing.assert_allclose(position, expected[:3, 3], atol=1e-9)
        np.testing.assert_allclose(rotation, expected[:3, :3], atol=1e-9)


def test_base_pose_moves_and_turns_the_tcp(model):
    local_p, local_r = core.tcp_pose(model, (0.0, 0.0), 0.0, CARRY)

    position, rotation = core.tcp_pose(model, (0.3, -0.2), 0.7, CARRY)

    np.testing.assert_allclose(position, _rz(0.7) @ local_p + [0.3, -0.2, 0.0], atol=1e-12)
    np.testing.assert_allclose(rotation, _rz(0.7) @ local_r, atol=1e-12)


def test_model_limits_are_the_urdf_limits(model, robot):
    limits = [robot.joint_map[name].limit for name in core.ARM_JOINTS]

    np.testing.assert_array_equal(model.lower, [limit.lower for limit in limits])
    np.testing.assert_array_equal(model.upper, [limit.upper for limit in limits])
    np.testing.assert_array_equal(model.velocity, [limit.velocity for limit in limits])


def test_model_arrays_are_read_only(model):
    with pytest.raises(ValueError):
        model.lower[0] = 0.0


def test_model_needs_every_arm_joint(urdf_xml):
    broken = urdf_xml.replace('name="piper_joint4"', 'name="piper_joint4_renamed"')

    with pytest.raises(ValueError, match="piper_joint4"):
        core.build_arm_model(broken)


def test_level_rotation_is_the_carry_gripper(model):
    _, carry_rotation = core.tcp_pose(model, (0.0, 0.0), 0.0, CARRY)

    np.testing.assert_allclose(core.level_rotation(0.0), carry_rotation, atol=1e-3)
    np.testing.assert_allclose(core.level_rotation(0.0), core.LEVEL_ROTATION)


def test_level_rotation_turns_with_the_yaw():
    np.testing.assert_allclose(core.level_rotation(0.4), _rz(0.4) @ core.level_rotation(0.0))


@pytest.mark.parametrize("yaw", [-2.0, 0.0, 0.7, math.pi])
def test_level_quaternion_round_trips(yaw):
    quaternion = core.level_quaternion(yaw)

    assert np.linalg.norm(quaternion) == pytest.approx(1.0)
    np.testing.assert_allclose(
        core.quaternion_to_rotation(*quaternion), core.level_rotation(yaw), atol=1e-12
    )


# ---- pure helpers ------------------------------------------------------------


def test_tracking_error_is_zero_at_the_target():
    target = _target()

    error = core.tracking_error(target.position, target.rotation, target)

    assert error == pytest.approx((0.0, 0.0), abs=1e-7)


def test_tracking_error_returns_distance_and_angle():
    target = _target()

    distance, angle = core.tracking_error(
        target.position + [0.03, 0.0, 0.04], _rz(0.2) @ target.rotation, target
    )

    assert distance == pytest.approx(0.05)
    assert angle == pytest.approx(0.2)


def test_effective_box_keeps_a_box_that_contains_the_base():
    box = core.effective_box(BOX, (0.1, 0.0))

    np.testing.assert_array_equal(box, BOX)


def test_effective_box_grows_only_the_violated_side():
    box = core.effective_box(BOX, (0.15, -0.25))

    np.testing.assert_array_equal(box, [-0.40, -0.25, 0.52, 0.30])


def test_effective_box_does_not_mutate_its_input():
    box = BOX.copy()

    core.effective_box(box, (0.9, 0.9))

    np.testing.assert_array_equal(box, BOX)


def test_deadband_zeroes_slow_base_commands():
    assert core.apply_deadband(0.006, 0.006, 0.01) == (0.0, 0.0)
    assert core.apply_deadband(0.02, -0.01, 0.01) == (0.02, -0.01)


def test_integrate_command_clips_at_the_limits_without_mutating(model):
    q = np.array(model.upper) - 0.01

    result = core.integrate_command(q, np.full(6, 1.0), 0.05, model.lower, model.upper)

    np.testing.assert_array_equal(result, model.upper)
    np.testing.assert_array_equal(q, np.array(model.upper) - 0.01)


def test_config_rejects_non_positive_values():
    with pytest.raises(ValueError):
        core.MpcConfig(horizon=0)
    with pytest.raises(ValueError):
        core.MpcConfig(base_speed=-0.1)


# ---- OCP ----------------------------------------------------------------------


def test_one_solve_respects_every_bound(fresh_mpc, model):
    config = core.MpcConfig()

    solution = fresh_mpc.solve((0.0, 0.0), 0.0, CARRY, _target(), np.zeros(8), CARRY, BOX)

    assert solution.ok
    assert solution.states.shape == (config.horizon + 1, core.NX)
    assert np.all(np.abs(solution.u0[:2]) <= config.base_speed + 1e-6)
    joint_speed = np.minimum(model.velocity, config.joint_speed)
    assert np.all(np.abs(solution.u0[2:]) <= joint_speed + 1e-6)
    assert np.all(solution.states[:, 2:] >= model.lower - 1e-6)
    assert np.all(solution.states[:, 2:] <= model.upper + 1e-6)
    assert np.all(solution.states[:, :2] >= BOX[:2] - 1e-6)
    assert np.all(solution.states[:, :2] <= BOX[2:] + 1e-6)


def _closed_loop(mpc, model, start, box, steps, target=None):
    """Run the MPC on a kinematic plant; return (base path, last tracking error)."""
    target = target or _target()
    bx, by, yaw = start
    base = np.array([bx, by])
    q = np.array(CARRY)
    u_prev = np.zeros(core.NU)
    path, error = [base.copy()], None
    for _ in range(steps):
        solution = mpc.solve(base, yaw, q, target, u_prev, CARRY, core.effective_box(box, base))
        u = solution.u0
        base = base + CONTROL_PERIOD_SEC * (_rz(yaw)[:2, :2] @ u[:2])
        q = core.integrate_command(q, u[2:], CONTROL_PERIOD_SEC, model.lower, model.upper)
        u_prev = u
        path.append(base.copy())
        error = core.tracking_error(*core.tcp_pose(model, base, yaw, q), target)
        if error[0] < POSITION_TOLERANCE_M and error[1] < ANGLE_TOLERANCE_RAD:
            break
    return np.array(path), error


def test_base_and_arm_together_reach_the_target(fresh_mpc, model):
    path, (distance, angle) = _closed_loop(fresh_mpc, model, (0.0, 0.0, 0.0), BOX, steps=300)

    assert distance < POSITION_TOLERANCE_M
    assert angle < ANGLE_TOLERANCE_RAD
    assert np.linalg.norm(path[-1] - path[0]) > 0.15
    assert np.all(path >= BOX[:2] - 1e-6)
    assert np.all(path <= BOX[2:] + 1e-6)


def test_the_arm_alone_cannot_reach_the_target(fresh_mpc, model):
    collapsed = np.array([0.0, 0.0, 0.0, 0.0])

    path, (distance, _) = _closed_loop(fresh_mpc, model, (0.0, 0.0, 0.0), collapsed, steps=200)

    assert distance > 0.10
    np.testing.assert_allclose(path[-1], [0.0, 0.0], atol=1e-6)


@pytest.mark.parametrize("start", [(-0.25, 0.0, 0.0), (-0.20, 0.20, 0.10), (0.15, -0.25, -0.10)])
def test_the_reach_converges_from_where_nav2_may_stop(fresh_mpc, model, start):
    path, (distance, angle) = _closed_loop(fresh_mpc, model, start, BOX, steps=300)

    assert distance < POSITION_TOLERANCE_M
    assert angle < ANGLE_TOLERANCE_RAD
    # A start outside the box (y -0.25 < -0.20) only ever moves back in.
    assert np.all(path[:, 1] >= min(start[1], BOX[1]) - 1e-6)
    assert np.all(path[:, 0] <= BOX[2] + 1e-6)


def test_an_unreachable_target_drives_the_base_to_the_box_edge(fresh_mpc, model):
    far = _target(position=(3.5, 0.0, 1.0))

    path, (distance, _) = _closed_loop(fresh_mpc, model, (0.0, 0.0, 0.0), BOX, 150, far)

    assert distance > 1.0
    assert path[-1][0] == pytest.approx(BOX[2], abs=0.01)


def test_reset_drops_the_warm_start(model, mpc):
    other = core.WholeBodyMpc(model, core.MpcConfig())
    mpc.reset()
    mpc.solve((0.0, 0.0), 0.0, CARRY, _target(), np.zeros(8), CARRY, BOX)
    mpc.reset()

    warm = mpc.solve((0.1, 0.0), 0.0, CARRY, _target(), np.zeros(8), CARRY, BOX)
    cold = other.solve((0.1, 0.0), 0.0, CARRY, _target(), np.zeros(8), CARRY, BOX)

    np.testing.assert_allclose(warm.u0, cold.u0, atol=1e-9)


def test_solve_does_not_mutate_its_inputs(fresh_mpc):
    q, u_prev, box = np.array(CARRY), np.zeros(8), BOX.copy()

    fresh_mpc.solve((0.0, 0.0), 0.0, q, _target(), u_prev, CARRY, box)

    np.testing.assert_array_equal(q, CARRY)
    np.testing.assert_array_equal(u_prev, np.zeros(8))
    np.testing.assert_array_equal(box, BOX)


def test_a_solver_failure_returns_a_zero_command(fresh_mpc, monkeypatch):
    def fail():
        raise RuntimeError("Infeasible_Problem_Detected")

    monkeypatch.setattr(fresh_mpc._opti, "solve", fail)

    solution = fresh_mpc.solve((0.0, 0.0), 0.0, CARRY, _target(), np.zeros(8), CARRY, BOX)

    assert not solution.ok
    np.testing.assert_array_equal(solution.u0, np.zeros(core.NU))


@pytest.mark.parametrize("bad", ["base", "q", "target"])
def test_non_finite_inputs_fail_the_solve_without_raising(fresh_mpc, bad):
    base = (math.nan, 0.0) if bad == "base" else (0.0, 0.0)
    q = [math.nan] * 6 if bad == "q" else CARRY
    target = _target(position=(math.nan, 0.0, 1.0)) if bad == "target" else _target()

    solution = fresh_mpc.solve(base, 0.0, q, target, np.zeros(8), CARRY, BOX)

    assert not solution.ok
    np.testing.assert_array_equal(solution.u0, np.zeros(core.NU))


def test_a_zero_quaternion_is_rejected():
    with pytest.raises(ValueError, match="quaternion"):
        core.quaternion_to_rotation(0.0, 0.0, 0.0, 0.0)
