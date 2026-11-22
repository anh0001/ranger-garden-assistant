"""
Kinematic whole-body MPC of the Ranger base (translation only) and the PiPER arm.

State x = [bx, by, q1..q6]: base position in odom and the six arm joints.
Input u = [vx, vy, qd1..qd6]: base velocity in the BODY frame and the joint
velocities. The heading psi is measured every cycle and held constant over
the horizon,

    x[k+1] = x[k] + dt * [R2(psi) @ u[:2], u[2:]],

which is what the Ranger driver executes in its PARALLEL mode (any linear.y
selects it and drops angular.z; ranger_messenger.cpp TwistCmdCallback). A
model with a commanded yaw rate would plan motions the driver cannot do.

The optimal control problem (CasADi Opti + IPOPT, built once, warm-started
from the previous solution shifted by one stage) tracks a TCP target pose:
position, the gripper's approach (z) and finger (x) axes, a posture term
towards q_nominal, velocity and input-change penalties, with bounds on the
base speed, the joint speeds and positions (URDF) and a base box in odom.
Forward kinematics come from the robot URDF, base_footprint -> piper_tcp.

This module imports casadi and numpy, never rclpy: whole_body_mpc.py wraps
it in a node, test/test_whole_body_mpc_core.py tests it without ROS.
Simulation only: nothing here reaches a CAN bus or a real-robot topic.
"""

import dataclasses
import math
import time

import casadi as ca
import numpy as np
from urdf_parser_py.urdf import URDF

ARM_JOINTS = tuple(f"piper_joint{i}" for i in range(1, 7))
BASE_FRAME = "base_footprint"
TCP_LINK = "piper_tcp"
NQ = len(ARM_JOINTS)
NX = NU = 2 + NQ
# Level gripper in base_footprint, Ry(+90 deg): approach (tcp z) along +x,
# fingers closing along y, tcp x pointing down (the demo's CARRY/GRASP).
LEVEL_ROTATION = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
_LEVEL_QUATERNION = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))  # (x, y, z, w)
_REVOLUTE = ("revolute", "continuous")
_URDF_DEFAULT_AXIS = (1.0, 0.0, 0.0)  # a joint without <axis>
MIN_QUATERNION_NORM = 1e-6


@dataclasses.dataclass(frozen=True)
class ArmModel:
    """TCP kinematics of the arm on the base, and its URDF joint limits (read-only)."""

    fk: ca.Function  # q (6) -> position (3), rotation (3x3) of TCP_LINK in BASE_FRAME
    lower: np.ndarray  # rad
    upper: np.ndarray  # rad
    velocity: np.ndarray  # rad/s


@dataclasses.dataclass(frozen=True)
class MpcConfig:
    """Horizon, bounds, weights and solver budget (values from the planning spike)."""

    horizon: int = 20
    dt: float = 0.1  # s per stage
    base_speed: float = 0.10  # m/s per body axis
    joint_speed: float = 0.6  # rad/s cap below the URDF's 3-5 rad/s
    w_position: float = 200.0
    w_rotation: float = 20.0
    w_posture: float = 0.05
    w_base_velocity: float = 4.0
    w_joint_velocity: float = 0.2
    w_input_change: float = 2.0
    w_terminal_position: float = 2000.0
    w_terminal_rotation: float = 200.0
    max_iter: int = 60
    tol: float = 1e-4

    def __post_init__(self) -> None:
        if min(self.horizon, self.dt, self.base_speed, self.joint_speed, self.max_iter) <= 0:
            raise ValueError("horizon, dt, base_speed, joint_speed and max_iter must be positive")


@dataclasses.dataclass(frozen=True)
class Target:
    """TCP pose to reach, in odom."""

    position: np.ndarray  # (3,)
    rotation: np.ndarray  # (3, 3)


@dataclasses.dataclass(frozen=True)
class Solution:
    """First input to apply, the predicted states and how the solve went."""

    u0: np.ndarray  # (NU,): vx, vy (m/s, body frame), qd1..qd6 (rad/s)
    states: np.ndarray  # (horizon + 1, NX)
    ok: bool
    solve_ms: float


# ---- rotations ------------------------------------------------------------------


def yaw_rotation(yaw: float) -> np.ndarray:
    """Return the rotation about +z by `yaw` (rad)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def level_rotation(yaw: float) -> np.ndarray:
    """Return the level gripper rotation approaching along heading `yaw` (odom)."""
    return yaw_rotation(yaw) @ LEVEL_ROTATION


def level_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return level_rotation(yaw) as a quaternion (x, y, z, w)."""
    s, c = math.sin(yaw / 2), math.cos(yaw / 2)
    x, y, z, w = _LEVEL_QUATERNION
    # (0, 0, s, c) * (x, y, z, w), Hamilton product.
    return (c * x - s * y, c * y + s * x, c * z + s * w, c * w - s * z)


def quaternion_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return the rotation matrix of quaternion (x, y, z, w), normalised first."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if not n > MIN_QUATERNION_NORM:  # also NaN
        raise ValueError(f"quaternion ({x}, {y}, {z}, {w}) has no rotation (norm {n})")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _origin_transform(origin) -> np.ndarray:
    """4x4 transform of a URDF <origin> (fixed-axis rpy: R = Rz @ Ry @ Rx)."""
    t = np.eye(4)
    if origin is None:
        return t
    roll, pitch, yaw = origin.rpy or (0.0, 0.0, 0.0)
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    t[:3, :3] = yaw_rotation(yaw) @ ry @ rx
    t[:3, 3] = origin.xyz or (0.0, 0.0, 0.0)
    return t


def _axis_rotation(axis, angle: ca.SX) -> ca.SX:
    """Symbolic 4x4 rotation by `angle` about the unit-normalised `axis` (Rodrigues)."""
    k = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    kx = ca.DM(np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]))
    t = ca.SX.eye(4)
    t[:3, :3] = ca.SX.eye(3) + ca.sin(angle) * kx + (1 - ca.cos(angle)) * ca.mtimes(kx, kx)
    return t


# ---- kinematics -------------------------------------------------------------------


def _chain(robot: URDF) -> list:
    """Joints from BASE_FRAME to TCP_LINK, root first."""
    by_child = {joint.child: joint for joint in robot.joints}
    chain, link = [], TCP_LINK
    while link != BASE_FRAME:
        if link not in by_child:
            raise ValueError(f"{TCP_LINK} is not connected to {BASE_FRAME} in the URDF")
        chain.append(by_child[link])
        link = by_child[link].parent
    return chain[::-1]


def build_arm_model(urdf_xml: str) -> ArmModel:
    """Build the TCP forward kinematics and joint limits from the robot URDF."""
    robot = URDF.from_xml_string(urdf_xml)
    chain = _chain(robot)
    missing = [name for name in ARM_JOINTS if name not in {j.name for j in chain}]
    if missing:
        raise ValueError(f"arm joints missing between {BASE_FRAME} and {TCP_LINK}: {missing}")
    q = ca.SX.sym("q", NQ)
    t = ca.SX.eye(4)
    for joint in chain:
        t = ca.mtimes(t, ca.DM(_origin_transform(joint.origin)))
        # Other movable joints on the chain (none today) are held at zero.
        if joint.name in ARM_JOINTS and joint.type in _REVOLUTE:
            axis = joint.axis or _URDF_DEFAULT_AXIS
            t = ca.mtimes(t, _axis_rotation(axis, q[ARM_JOINTS.index(joint.name)]))
    limits = [robot.joint_map[name].limit for name in ARM_JOINTS]
    if any(limit is None or None in (limit.lower, limit.upper) for limit in limits):
        raise ValueError(f"every arm joint needs a <limit> with lower and upper: {ARM_JOINTS}")

    def frozen(values: list[float]) -> np.ndarray:
        array = np.array(values, dtype=float)
        array.setflags(write=False)
        return array

    return ArmModel(
        fk=ca.Function("fk", [q], [t[:3, 3], t[:3, :3]]),
        lower=frozen([limit.lower for limit in limits]),
        upper=frozen([limit.upper for limit in limits]),
        velocity=frozen([limit.velocity for limit in limits]),
    )


def tcp_pose(model: ArmModel, base_xy, yaw: float, q) -> tuple[np.ndarray, np.ndarray]:
    """Return the TCP (position, rotation) in odom for the base at (x, y, yaw) and joints q."""
    position, rotation = model.fk(np.asarray(q, dtype=float))
    turn = yaw_rotation(yaw)
    offset = np.array([base_xy[0], base_xy[1], 0.0])
    return offset + turn @ np.array(position).ravel(), turn @ np.array(rotation)


def tracking_error(position, rotation, target: Target) -> tuple[float, float]:
    """Return (distance m, rotation angle rad) from the TCP pose to `target`."""
    distance = float(np.linalg.norm(np.asarray(position) - target.position))
    relative = target.rotation.T @ np.asarray(rotation)
    skew = relative - relative.T
    sine = math.sqrt(skew[2, 1] ** 2 + skew[0, 2] ** 2 + skew[1, 0] ** 2) / 2
    return distance, math.atan2(sine, (np.trace(relative) - 1) / 2)


# ---- command helpers ----------------------------------------------------------------


def effective_box(box, base_xy) -> np.ndarray:
    """Return `box` grown to contain the base: from outside it may only move in or stay."""
    x_min, y_min, x_max, y_max = (float(v) for v in box)
    x, y = float(base_xy[0]), float(base_xy[1])
    return np.array([min(x_min, x), min(y_min, y), max(x_max, x), max(y_max, y)])


def apply_deadband(vx: float, vy: float, deadband: float) -> tuple[float, float]:
    """Return (0, 0) for a base speed below `deadband`, else (vx, vy) unchanged."""
    return (0.0, 0.0) if math.hypot(vx, vy) < deadband else (vx, vy)


def integrate_command(q_cmd, qd, period: float, lower, upper) -> np.ndarray:
    """Return the commanded joints advanced by `qd` for `period`, clipped to the limits."""
    return np.clip(np.asarray(q_cmd, dtype=float) + period * np.asarray(qd), lower, upper)


# ---- MPC ------------------------------------------------------------------------------


class WholeBodyMpc:
    """The OCP of the module docstring, built once and solved with new parameters."""

    def __init__(self, model: ArmModel, config: MpcConfig) -> None:
        self._model = model
        self._config = config
        self._opti = ca.Opti()
        horizon = config.horizon
        self._xs = [self._opti.variable(NX) for _ in range(horizon + 1)]
        self._us = [self._opti.variable(NU) for _ in range(horizon)]
        o = self._opti
        self._p = {
            "x0": o.parameter(NX), "psi": o.parameter(), "p_ref": o.parameter(3),
            "r_ref": o.parameter(3, 3), "u_prev": o.parameter(NU),
            "q_nom": o.parameter(NQ), "box": o.parameter(4),
        }
        self._add_constraints()
        o.minimize(self._cost())
        o.solver(
            "ipopt",
            {"expand": True, "print_time": False},
            {"print_level": 0, "max_iter": config.max_iter, "tol": config.tol,
             "warm_start_init_point": "yes", "mu_init": 1e-3, "linear_solver": "mumps"},
        )
        self._guess: tuple[list, list] | None = None

    def _turn(self) -> ca.MX:
        psi = self._p["psi"]
        c, s = ca.cos(psi), ca.sin(psi)
        return ca.vertcat(ca.horzcat(c, -s, 0), ca.horzcat(s, c, 0), ca.horzcat(0, 0, 1))

    def _add_constraints(self) -> None:
        o, p, cfg, model = self._opti, self._p, self._config, self._model
        joint_speed = np.minimum(model.velocity, cfg.joint_speed)
        turn = self._turn()[:2, :2]
        o.subject_to(self._xs[0] == p["x0"])
        for x, u, x_next in zip(self._xs, self._us, self._xs[1:]):
            o.subject_to(x_next == x + cfg.dt * ca.vertcat(ca.mtimes(turn, u[:2]), u[2:]))
            o.subject_to(o.bounded(-cfg.base_speed, u[:2], cfg.base_speed))
            o.subject_to(o.bounded(-joint_speed, u[2:], joint_speed))
            o.subject_to(o.bounded(model.lower, x_next[2:], model.upper))
            o.subject_to(o.bounded(p["box"][:2], x_next[:2], p["box"][2:]))

    def _cost(self) -> ca.MX:
        p, cfg = self._p, self._config
        turn = self._turn()
        cost = 0
        last = len(self._us) - 1
        for k, (u, x_next) in enumerate(zip(self._us, self._xs[1:])):
            local_p, local_r = self._model.fk(x_next[2:])
            position = ca.vertcat(x_next[:2], 0) + ca.mtimes(turn, local_p)
            rotation = ca.mtimes(turn, local_r)
            w_p, w_r = ((cfg.w_terminal_position, cfg.w_terminal_rotation) if k == last
                        else (cfg.w_position, cfg.w_rotation))
            cost += w_p * ca.sumsqr(position - p["p_ref"])
            cost += w_r * (ca.sumsqr(rotation[:, 2] - p["r_ref"][:, 2])
                           + ca.sumsqr(rotation[:, 0] - p["r_ref"][:, 0]))
            cost += cfg.w_posture * ca.sumsqr(x_next[2:] - p["q_nom"])
            cost += cfg.w_base_velocity * ca.sumsqr(u[:2])
            cost += cfg.w_joint_velocity * ca.sumsqr(u[2:])
            previous = p["u_prev"] if k == 0 else self._us[k - 1]
            cost += cfg.w_input_change * ca.sumsqr(u - previous)
        return cost

    def reset(self) -> None:
        """Drop the warm start: the next solve starts from its current state."""
        self._guess = None

    def solve(self, base_xy, yaw: float, q, target: Target, u_prev, q_nominal,
              box) -> Solution:
        """Solve from base (x, y, yaw) and joints q; a failure returns ok=False and u0 = 0."""
        model, o = self._model, self._opti
        x0 = np.concatenate([np.asarray(base_xy, dtype=float)[:2],
                             np.clip(np.asarray(q, dtype=float), model.lower, model.upper)])
        values = {"x0": x0, "psi": float(yaw), "p_ref": target.position,
                  "r_ref": target.rotation, "u_prev": np.asarray(u_prev, dtype=float),
                  "q_nom": np.asarray(q_nominal, dtype=float), "box": np.asarray(box, float)}
        if not all(np.all(np.isfinite(value)) for value in values.values()):
            # CasADi asserts (it does not raise RuntimeError) on non-finite parameters.
            self.reset()
            return Solution(np.zeros(NU), np.tile(x0, (len(self._xs), 1)), False, 0.0)
        for name, value in values.items():
            o.set_value(self._p[name], value)
        guess_x, guess_u = self._guess or ([x0] * len(self._xs), [np.zeros(NU)] * len(self._us))
        for variable, value in zip(self._xs + self._us, guess_x + guess_u):
            o.set_initial(variable, value)
        start = time.perf_counter()
        try:
            solved = o.solve()
        except RuntimeError:
            # Never warm-start from a failed iterate: the next solve starts afresh.
            self.reset()
            states = np.array(guess_x)
            return Solution(np.zeros(NU), states, False, (time.perf_counter() - start) * 1e3)
        solve_ms = (time.perf_counter() - start) * 1e3
        states = np.array([np.asarray(solved.value(x)).ravel() for x in self._xs])
        inputs = np.array([np.asarray(solved.value(u)).ravel() for u in self._us])
        self._guess = (list(states[1:]) + [states[-1]], list(inputs[1:]) + [inputs[-1]])
        return Solution(inputs[0].copy(), states, True, solve_ms)
