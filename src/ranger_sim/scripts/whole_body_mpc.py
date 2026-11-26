#!/usr/bin/env python3
"""
Whole-body MPC node: the Ranger base (translation) and the PiPER arm reach a TCP target together.

Wraps whole_body_mpc_core.py (loaded by path from next to this file). Once
base_handover_gate has handed the base over (its ~/mode is MANIPULATE) and a
target pose arrives on ~/target (geometry_msgs/PoseStamped in odom), a 20 Hz
timer solves the MPC and publishes

  * cmd_topic (/cmd_vel_manip): the base twist, linear.x / linear.y only
    (the driver's PARALLEL mode); a speed under base_deadband is sent as zero
  * arm_command_topic (/piper_arm_stream_controller/commands): the six
    commanded joint positions (std_msgs/Float64MultiArray, joint 1-6)

The MPC's arm state is the COMMANDED position q_cmd: the measured joints at
activation, then q_cmd + qd * period every cycle, clipped to the URDF limits.
Closing the loop on the measured joints would feed the position interface's
lag back into the plan; the measured joints give the tracking error and the
lag fault instead.

Runtime inputs of a governor (docs/sim_handover_baseline.md); a node that
never receives them behaves as without them:
  * ~/speed_scale (std_msgs/Float64): multiplies the MPC's base and joint
    speed bounds, clamped to [0, 1], NaN = 0. From the first message on the
    node is governed: a scale older than speed_scale_timeout counts as 0.
  * ~/base_box (std_msgs/Float64MultiArray, [x_min, y_min, x_max, y_max] in
    odom): replaces base_box while the node is not TRACKING, REACHED or
    PAUSED; a malformed box is ignored with a warning.

States on ~/state (std_msgs/String, transient local, on change):
  IDLE      not in MANIPULATE, no target or no robot model: publishes nothing
  TRACKING  solving and commanding
  PAUSED    governed with a speed scale under SPEED_SCALE_PAUSE: zero twist,
            q_cmd held, no solve; the warm start is dropped for the resume
  REACHED   error under position_tolerance and angle_tolerance: zero twist,
            q_cmd held; back to TRACKING only beyond reengage_factor times them
  FAULT: <reason>  odom or joint states older than their timeout, too many
            failed solves in a row, or q_cmd more than max_command_lag from
            the measured joints: zero twist, q_cmd held, until MANIPULATE ends
Leaving MANIPULATE clears the target and any fault. ~/tracking_error carries
[position m, angle rad] every tick that computes it.

Simulation only: nothing here reaches a CAN bus or a real-robot topic. The
node writes /cmd_vel_manip (read by base_handover_gate in the sim) and the
sim's stream controller topic.
"""

import importlib.util
import math
from pathlib import Path
import sys

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String

# resolve(): with --symlink-install this file is a link into the source tree.
CORE_FILE = Path(__file__).resolve().with_name("whole_body_mpc_core.py")
INSTALL_HINT = "pip3 install --user casadi"


def _load_core():
    spec = importlib.util.spec_from_file_location("whole_body_mpc_core", CORE_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


try:
    core = _load_core()
    CORE_ERROR = ""
except ImportError as exc:  # casadi missing: main() exits with INSTALL_HINT
    core = None
    CORE_ERROR = str(exc)

RATE_HZ = 20.0
# pick_place_demo CARRY: level gripper retracted over the base (posture term).
Q_NOMINAL = [0.0, 0.719, -1.496, 0.0, 0.864, 0.0]
POSITION_TOLERANCE_M = 0.005
ANGLE_TOLERANCE_RAD = 0.035  # 2 deg
REENGAGE_FACTOR = 3.0  # REACHED -> TRACKING only beyond this many tolerances
BASE_DEADBAND_MPS = 0.01  # slower base commands are zero: no re-steering for noise
ODOM_TIMEOUT_SEC = 0.3
JOINT_TIMEOUT_SEC = 0.3
# Measured 0.046 rad worst in the planning spike (position lag ~0.1 s).
MAX_COMMAND_LAG_RAD = 0.15
MAX_SOLVER_FAILURES = 3
SPEED_SCALE_TIMEOUT_SEC = 0.5  # a governed scale older than this counts as 0
SPEED_SCALE_PAUSE = 0.05  # under this scale the node holds instead of solving
TARGET_FRAME = "odom"
MANIPULATE = "MANIPULATE"  # base_handover_gate Mode.MANIPULATE
IDLE, TRACKING, REACHED, PAUSED = "IDLE", "TRACKING", "REACHED", "PAUSED"
FAULT = "FAULT: "
LATCHED_QOS = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _check_box(box) -> np.ndarray:
    """Return `box` as an array; ValueError unless [x_min, y_min, x_max, y_max] with min < max."""
    if box is None or len(box) != 4:
        raise ValueError(f"base_box must be [x_min, y_min, x_max, y_max] in odom, got {box}")
    x_min, y_min, x_max, y_max = (float(v) for v in box)
    if not (x_min < x_max and y_min < y_max):
        raise ValueError(f"base_box needs x_min < x_max and y_min < y_max, got {list(box)}")
    return np.array([x_min, y_min, x_max, y_max])


class WholeBodyMpcNode(Node):
    """Track ~/target with the base and the arm while the gate is in MANIPULATE."""

    def __init__(self, **kwargs) -> None:
        super().__init__("whole_body_mpc", **kwargs)
        self._declare_parameters()
        self._model = self._mpc = self._target = self._mode = self._state = None
        self._odom: tuple[float, float, float] | None = None  # x, y, yaw in odom
        self._odom_sec: float | None = None
        self._joints: dict[str, float] = {}
        self._joints_sec: float | None = None
        self._q_cmd: np.ndarray | None = None
        self._u_prev = np.zeros(core.NU)
        self._failures = 0
        self._solve_ms: list[float] = []  # this activation's solve times
        self._start_xy = (0.0, 0.0)  # base position at activation
        self._target_fresh = False  # a new target: REACHED must be earned again
        self._scale: tuple[float, float] | None = None  # (value, receipt time): governed
        self._warned = {"frame": False, "rotation": False, "slow": False, "scale": False,
                        "box": False}
        self._cmd_pub = self.create_publisher(Twist, self._topic("cmd_topic"), 10)
        self._arm_pub = self.create_publisher(
            Float64MultiArray, self._topic("arm_command_topic"), 10)
        self._state_pub = self.create_publisher(String, "~/state", LATCHED_QOS)
        self._error_pub = self.create_publisher(Float64MultiArray, "~/tracking_error", 10)
        sub = self.create_subscription
        sub(Odometry, self._topic("odom_topic"), self._on_odom, 10)
        sub(JointState, self._topic("joint_state_topic"), self._on_joint_states, 20)
        sub(String, self._topic("mode_topic"), self._on_mode, LATCHED_QOS)
        sub(String, self._topic("robot_description_topic"), self._on_robot_description,
            LATCHED_QOS)
        sub(PoseStamped, "~/target", self._on_target, 10)
        sub(Float64, "~/speed_scale", self._on_speed_scale, 10)
        sub(Float64MultiArray, "~/base_box", self._on_base_box, 10)
        self._timer = self.create_timer(self._period, self._tick)
        self.get_logger().info(
            f"Whole-body MPC at {1.0 / self._period:.0f} Hz, base box {self._box.tolist()}: "
            f"waiting for the robot description on {self._topic('robot_description_topic')}"
        )

    def _declare_parameters(self) -> None:
        """Declare and check the parameters (ValueError for a bad one)."""
        p = self.declare_parameter
        self._config = core.MpcConfig(
            horizon=p("horizon", 20).value, dt=p("dt", 0.1).value,
            base_speed=p("base_speed", 0.10).value, joint_speed=p("joint_speed", 0.6).value,
        )
        self._box = _check_box(p("base_box", Parameter.Type.DOUBLE_ARRAY).value)
        self._q_nominal = np.array(p("q_nominal", Q_NOMINAL).value, dtype=float)
        if self._q_nominal.shape != (core.NQ,):
            raise ValueError(f"q_nominal must be {core.NQ} joint positions")
        positive = {
            "rate": p("rate", RATE_HZ).value,
            "position_tolerance": p("position_tolerance", POSITION_TOLERANCE_M).value,
            "angle_tolerance": p("angle_tolerance", ANGLE_TOLERANCE_RAD).value,
            "base_deadband": p("base_deadband", BASE_DEADBAND_MPS).value,
            "odom_timeout": p("odom_timeout", ODOM_TIMEOUT_SEC).value,
            "joint_timeout": p("joint_timeout", JOINT_TIMEOUT_SEC).value,
            "max_command_lag": p("max_command_lag", MAX_COMMAND_LAG_RAD).value,
            "max_solver_failures": p("max_solver_failures", MAX_SOLVER_FAILURES).value,
            "speed_scale_timeout": p("speed_scale_timeout", SPEED_SCALE_TIMEOUT_SEC).value,
        }
        for name, value in positive.items():
            if value <= 0:
                raise ValueError(f"{name} must be positive, got {value}")
        self._reengage = p("reengage_factor", REENGAGE_FACTOR).value
        if self._reengage < 1.0:
            raise ValueError(f"reengage_factor must be at least 1, got {self._reengage}")
        self._period = 1.0 / positive["rate"]
        self._limits = positive
        for name, default in (
            ("odom_topic", "/odom"), ("joint_state_topic", "/joint_states"),
            ("mode_topic", "/base_handover_gate/mode"), ("cmd_topic", "/cmd_vel_manip"),
            ("arm_command_topic", "/piper_arm_stream_controller/commands"),
            ("robot_description_topic", "/robot_description"),
        ):
            p(name, default)

    def _topic(self, name: str) -> str:
        return self.get_parameter(name).value

    def _now(self) -> float:
        """Node clock (sim time in the sim) in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _warn_once(self, key: str, active: bool, text: str) -> None:
        """Warn when degraded state `key` begins; re-arm once it ends."""
        if active and not self._warned[key]:
            self.get_logger().warn(text)
        self._warned[key] = active

    # ---- inputs ------------------------------------------------------------------
    def _on_robot_description(self, msg: String) -> None:
        if self._model is not None:
            return
        try:
            model = core.build_arm_model(msg.data)
            mpc = core.WholeBodyMpc(model, self._config)
        except Exception as exc:  # noqa: BLE001 - any unusable URDF: stay IDLE, say why
            self.get_logger().error(f"robot description unusable: {exc}")
            return
        # Pay IPOPT's slow first solve now, not on the first command.
        position, rotation = core.tcp_pose(model, (0.0, 0.0), 0.0, self._q_nominal)
        cold = mpc.solve((0.0, 0.0), 0.0, self._q_nominal,
                         core.Target(position + [0.1, 0.0, 0.0], rotation),
                         np.zeros(core.NU), self._q_nominal, self._box)
        mpc.reset()
        self._model, self._mpc = model, mpc
        log = self.get_logger().info if cold.ok else self.get_logger().warn
        log(f"robot model ready ({core.BASE_FRAME} -> {core.TCP_LINK}), first solve "
            f"{cold.solve_ms:.0f} ms{'' if cold.ok else ' FAILED'}")

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        odom = (pose.position.x, pose.position.y, _yaw(pose.orientation))
        if not all(map(math.isfinite, odom)):
            return  # never stamped: goes stale, a FAULT while tracking
        self._odom, self._odom_sec = odom, self._now()

    def _on_joint_states(self, msg: JointState) -> None:
        arm = {n: q for n, q in zip(msg.name, msg.position) if n in core.ARM_JOINTS}
        if not all(map(math.isfinite, arm.values())):
            return  # never stamped: goes stale, a FAULT while tracking
        self._joints.update(arm)
        if len(arm) == core.NQ:
            self._joints_sec = self._now()

    def _on_mode(self, msg: String) -> None:
        if self._mode == MANIPULATE and msg.data != MANIPULATE:
            self._target = None  # the next hand-over needs a new target
        self._mode = msg.data

    def _on_target(self, msg: PoseStamped) -> None:
        frame = msg.header.frame_id.lstrip("/")
        self._warn_once(
            "frame", frame != TARGET_FRAME,
            f"target in frame '{frame}' ignored: targets must be in '{TARGET_FRAME}'",
        )
        position = msg.pose.position
        o = msg.pose.orientation
        values = (position.x, position.y, position.z, o.x, o.y, o.z, o.w)
        if frame != TARGET_FRAME or not all(map(math.isfinite, values)):
            return
        try:
            rotation = core.quaternion_to_rotation(o.x, o.y, o.z, o.w)
        except ValueError as exc:
            self._warn_once("rotation", True, f"target ignored: {exc}")
            return
        self._warn_once("rotation", False, "")
        self._target = core.Target(np.array(values[:3]), rotation)
        self._target_fresh = True

    def _on_speed_scale(self, msg: Float64) -> None:
        value = msg.data
        self._scale = (min(1.0, max(0.0, value)) if math.isfinite(value) else 0.0, self._now())

    def _speed_scale(self, now: float) -> float:
        """1.0 until the first scale arrives; then the scale while fresh, else 0."""
        if self._scale is None:
            return 1.0
        value, stamp = self._scale
        timeout = self._limits["speed_scale_timeout"]
        stale = not 0.0 <= now - stamp <= timeout  # a negative age (sim-time reset) counts
        self._warn_once("scale", stale, f"speed scale older than {timeout} s: paused")
        return 0.0 if stale else value

    def _on_base_box(self, msg: Float64MultiArray) -> None:
        try:
            box = _check_box(list(msg.data))
        except ValueError as exc:
            self._warn_once("box", True, f"base box ignored: {exc}")
            return
        if self._state in (TRACKING, REACHED, PAUSED):
            self._warn_once("box", True, f"base box ignored while {self._state}: "
                                         "it changes only between targets")
            return
        self._warn_once("box", False, "")
        self._box = box
        self.get_logger().info(f"base box {box.tolist()}")

    # ---- cycle ---------------------------------------------------------------------
    def _set_state(self, state: str, detail: str = "") -> None:
        if state == self._state:
            return
        self.get_logger().info(f"{self._state or 'START'} -> {state}{detail}")
        self._state = state
        self._state_pub.publish(String(data=state))

    def _publish(self, vx: float, vy: float) -> None:
        twist = Twist()
        twist.linear.x, twist.linear.y = float(vx), float(vy)
        self._cmd_pub.publish(twist)
        if self._q_cmd is not None:
            self._arm_pub.publish(Float64MultiArray(data=[float(v) for v in self._q_cmd]))

    def _hold(self) -> None:
        """Stop the base and keep the arm at q_cmd."""
        self._u_prev = np.zeros(core.NU)
        self._publish(0.0, 0.0)

    def _fault(self, reason: str) -> None:
        self._set_state(FAULT + reason)
        self._hold()

    def _stale_reason(self, now: float) -> str | None:
        """Name the input that is too old (a negative age, a sim-time reset, counts)."""
        for name, stamp, timeout in (
            ("odom", self._odom_sec, self._limits["odom_timeout"]),
            ("joint states", self._joints_sec, self._limits["joint_timeout"]),
        ):
            if stamp is None or not 0.0 <= now - stamp <= timeout:
                return f"{name} older than {timeout} s"
        return None

    def _tick(self) -> None:
        if self._mode != MANIPULATE or self._model is None or self._target is None:
            self._q_cmd, self._failures = None, 0
            self._set_state(IDLE)
            return
        if (self._state or "").startswith(FAULT):
            self._hold()
            return
        now = self._now()
        reason = self._stale_reason(now)
        if reason:
            self._fault(reason)
            return
        q_meas = np.array([self._joints[name] for name in core.ARM_JOINTS])
        x, y, yaw = self._odom
        if self._q_cmd is None:  # activation
            self._q_cmd, self._u_prev = q_meas.copy(), np.zeros(core.NU)
            self._solve_ms, self._start_xy = [], (x, y)
            self._mpc.reset()
        lag = float(np.max(np.abs(self._q_cmd - q_meas)))
        if not lag <= self._limits["max_command_lag"]:
            # Hold where the arm is: holding q_cmd would keep pushing a blocked arm.
            self._q_cmd = np.clip(q_meas, self._model.lower, self._model.upper)
            self._fault(f"command lag {lag:.3f} rad")
            return
        error = core.tracking_error(*core.tcp_pose(self._model, (x, y), yaw, q_meas),
                                    self._target)
        self._error_pub.publish(Float64MultiArray(data=list(error)))
        scale = self._speed_scale(now)
        paused = scale < SPEED_SCALE_PAUSE
        if self._reached(error, (x, y), paused):
            self._hold()
            return
        if paused:
            self._hold()
            self._mpc.reset()  # resume from the current state, not the old plan
            return
        self._track(x, y, yaw, scale)

    def _progress(self, base_xy: tuple[float, float]) -> str:
        """Solve times and base travel of this activation, for the REACHED log line."""
        moved = math.hypot(base_xy[0] - self._start_xy[0], base_xy[1] - self._start_xy[1])
        if not self._solve_ms:
            return f"; base moved {moved:.3f} m"
        return (f"; {len(self._solve_ms)} solves, median {np.median(self._solve_ms):.0f} ms, "
                f"max {max(self._solve_ms):.0f} ms; base moved {moved:.3f} m")

    def _reached(self, error: tuple[float, float], base_xy: tuple[float, float],
                 paused: bool = False) -> bool:
        """Update TRACKING / PAUSED / REACHED from the measured error; True while REACHED."""
        tolerance = (self._limits["position_tolerance"], self._limits["angle_tolerance"])
        within = all(e < t for e, t in zip(error, tolerance))
        beyond = any(e > self._reengage * t for e, t in zip(error, tolerance))
        detail = f" ({error[0] * 1e3:.1f} mm, {math.degrees(error[1]):.1f} deg"
        latched = self._state == REACHED and not self._target_fresh
        self._target_fresh = False
        if within or (latched and not beyond):
            self._set_state(REACHED, f"{detail}{self._progress(base_xy)})")
            return True
        if self._state == REACHED:
            self._mpc.reset()  # the warm start is from before the hold
        # Here, not after: on change only, so the state does not flip every tick.
        self._set_state(PAUSED if paused else TRACKING, f"{detail})")
        return False

    def _track(self, x: float, y: float, yaw: float, scale: float = 1.0) -> None:
        solution = self._mpc.solve((x, y), yaw, self._q_cmd, self._target, self._u_prev,
                                   self._q_nominal, core.effective_box(self._box, (x, y)),
                                   scale)
        self._solve_ms.append(solution.solve_ms)
        self._warn_once(
            "slow", solution.solve_ms > self._period * 1e3,
            f"solve took {solution.solve_ms:.0f} ms, longer than the "
            f"{self._period * 1e3:.0f} ms period",
        )
        if not solution.ok:
            self._failures += 1
            if self._failures >= self._limits["max_solver_failures"]:
                self._fault(f"{self._failures} solver failures in a row")
            else:
                self._hold()
            return
        self._failures = 0
        u0 = solution.u0
        vx, vy = core.apply_deadband(u0[0], u0[1], self._limits["base_deadband"])
        self._q_cmd = core.integrate_command(
            self._q_cmd, u0[2:], self._period, self._model.lower, self._model.upper)
        self._u_prev = np.concatenate([[vx, vy], u0[2:]])
        self._publish(vx, vy)


def main() -> None:
    if core is None:
        sys.exit(f"cannot load {CORE_FILE.name} ({CORE_ERROR}): {INSTALL_HINT}")
    rclpy.init()
    node = WholeBodyMpcNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
