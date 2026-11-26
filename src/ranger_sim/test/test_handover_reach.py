"""
Reach check of the handover standoffs against the whole-body MPC (no simulator).

Go/no-go of docs/sim_handover_baseline.md: from a standoff next to the person
on ChairA_01_001, does the MPC (scripts/whole_body_mpc_core.py, default
MpcConfig, posture CARRY) bring the level gripper to the offer pose and to the
table place pose? Each case runs from two starts: the standoff itself, and
the corner of its base box nearest the navigation goal with the yaw 0.10 rad
off (where Nav2 may stop). The plant is kinematic, as in
test_whole_body_mpc_core.py: the base translates at the commanded body
velocity with the heading held, the joints integrate the commanded speeds.

The last test guards the reach proxy of scripts/handover_core.py: every
candidate it flags offer_ok / place_ok must be reached here (no false
positive). Needs the install space sourced (xacro) and casadi; slow (about a
hundred closed loops of 1-4 s).
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
CARRY = (0.0, 0.719, -1.496, 0.0, 0.864, 0.0)  # q_nominal and the start joints
POSITION_TOLERANCE_M = 0.005
ANGLE_TOLERANCE_RAD = math.radians(2.0)
STEPS = 300
STALL_STEPS = 40  # stop when the error has not improved by STALL_IMPROVEMENT_M in this many
STALL_IMPROVEMENT_M = 0.001
UNREACHED_M = 0.020  # "not reachable": the error stays above this
START_YAW_OFFSET_RAD = 0.10
PERSON_XY = (0.504, -0.858)  # ChairA_01_001


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


core = _load("whole_body_mpc_core", PKG / "scripts" / "whole_body_mpc_core.py")
handover = _load("handover_core", PKG / "scripts" / "handover_core.py")
CFG = handover.default_config()


@pytest.fixture(scope="module")
def model():
    return core.build_arm_model(xacro.process_file(str(SIM_XACRO)).toxml())


@pytest.fixture(scope="module")
def mpc(model):
    return core.WholeBodyMpc(model, core.MpcConfig())


def _person(facing_deg: float):
    return handover.PersonState(*PERSON_XY, math.radians(facing_deg), (0.0, 0.0, 0.0),
                                False, False, 0.0)


def _standoff(facing_deg: float, bearing_deg: float, distance: float):
    person = _person(facing_deg)
    pose = handover.standoff_pose(person, math.radians(bearing_deg), distance, 0.0, CFG)
    return person, pose


def _starts(person, pose):
    """Return the standoff, and the box corner nearest the pre-standoff with the yaw off."""
    box = handover.standoff_box(pose, CFG)
    pre = handover.pre_standoff_pose(person, pose, CFG)
    corners = [(box[0], box[1]), (box[2], box[1]), (box[2], box[3]), (box[0], box[3])]
    corner = min(corners, key=lambda c: math.dist(c, pre[:2]))
    return [pose, (corner[0], corner[1], pose[2] + START_YAW_OFFSET_RAD)]


def _closed_loop(mpc, model, start, box, target):
    """Run the MPC on a kinematic plant; return the last (distance, angle) error."""
    mpc.reset()
    bx, by, yaw = start
    base = np.array([bx, by])
    q = np.array(CARRY)
    u_prev = np.zeros(core.NU)
    best, best_step, error = math.inf, 0, (math.inf, math.inf)
    for step in range(STEPS):
        solution = mpc.solve(base, yaw, q, target, u_prev, CARRY, core.effective_box(box, base))
        u = solution.u0
        c, s = math.cos(yaw), math.sin(yaw)
        base = base + CONTROL_PERIOD_SEC * np.array([c * u[0] - s * u[1], s * u[0] + c * u[1]])
        q = core.integrate_command(q, u[2:], CONTROL_PERIOD_SEC, model.lower, model.upper)
        u_prev = u
        error = core.tracking_error(*core.tcp_pose(model, base, yaw, q), target)
        if error[0] < POSITION_TOLERANCE_M and error[1] < ANGLE_TOLERANCE_RAD:
            return error
        if error[0] < best - STALL_IMPROVEMENT_M:
            best, best_step = error[0], step
        elif step - best_step >= STALL_STEPS:
            return error
    return error


def _offer_target(person, pose):
    position, yaw = handover.offer_pose(person, pose, CFG)
    return core.Target(np.array(position), core.level_rotation(yaw))


def _place_target(person, pose):
    place = handover.place_pose(person, pose, CFG)
    assert place is not None, "no free table point within the person's reach"
    position, yaw = place
    return core.Target(np.array(position), core.level_rotation(yaw))


def _reached(error) -> bool:
    return error[0] < POSITION_TOLERANCE_M and error[1] < ANGLE_TOLERANCE_RAD


def _assert_reached(mpc, model, person, pose, target, label):
    box = handover.standoff_box(pose, CFG)
    for start in _starts(person, pose):
        error = _closed_loop(mpc, model, start, box, target)
        assert _reached(error), (
            f"{label} from {np.round(start, 3)}: {error[0] * 1e3:.1f} mm, "
            f"{math.degrees(error[1]):.1f} deg"
        )


@pytest.mark.parametrize("facing, bearing_deg, distance", [
    (0, 90, 0.5), (0, 90, 0.7), (0, -60, 0.7), (0, -90, 0.6),
    (60, 35, 0.8), (90, 0, 0.9), (90, 35, 0.9),
])
def test_the_offer_pose_is_reached(mpc, model, facing, bearing_deg, distance):
    person, pose = _standoff(facing, bearing_deg, distance)

    _assert_reached(mpc, model, person, pose, _offer_target(person, pose),
                    f"offer, facing {facing}, ({bearing_deg} deg, {distance} m)")


@pytest.mark.parametrize("facing, bearing_deg, distance", [
    (0, 90, 0.5), (0, -60, 0.7), (0, -60, 0.8), (30, -90, 0.7),
])
def test_the_table_place_pose_is_reached(mpc, model, facing, bearing_deg, distance):
    person, pose = _standoff(facing, bearing_deg, distance)

    _assert_reached(mpc, model, person, pose, _place_target(person, pose),
                    f"place, facing {facing}, ({bearing_deg} deg, {distance} m)")


@pytest.mark.parametrize("facing, bearing_deg, distance", [(0, 90, 0.8), (90, 0, 0.9)])
def test_the_table_place_pose_is_out_of_reach(mpc, model, facing, bearing_deg, distance):
    person, pose = _standoff(facing, bearing_deg, distance)
    box = handover.standoff_box(pose, CFG)

    error = _closed_loop(mpc, model, pose, box, _place_target(person, pose))

    assert error[0] > UNREACHED_M


def _flagged():
    cases = []
    for facing in (0, 30, 60, 90):
        person = _person(facing)
        for standoff in handover.candidates(person, CFG):
            label = f"{facing}/{math.degrees(standoff.bearing):.0f}/{standoff.distance}"
            if standoff.offer_ok:
                cases.append(pytest.param(facing, standoff, "offer", id=f"offer-{label}"))
            if standoff.place_ok:
                cases.append(pytest.param(facing, standoff, "place", id=f"place-{label}"))
    return cases


@pytest.mark.parametrize("facing, standoff, kind", _flagged())
def test_the_reach_proxy_has_no_false_positive(mpc, model, facing, standoff, kind):
    person = _person(facing)
    target = (_offer_target if kind == "offer" else _place_target)(person, standoff.pose)

    _assert_reached(mpc, model, person, standoff.pose, target, f"{kind} {facing}")
