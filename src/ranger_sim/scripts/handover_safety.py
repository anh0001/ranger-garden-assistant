#!/usr/bin/env python3
"""
Hard limits of the handover study, outside the decision logic (docs/sim_handover_baseline.md).

A software interlock in a simulation: not a safety-rated function, no
redundancy, and nothing here may be reused on hardware as a safety measure.

The decision maker (handover_baseline, later a learned policy) only
REQUESTS a speed scale (~/scale_request, std_msgs/Float64) and MPC targets
(~/target_request, ~/place_target_request, PoseStamped in odom). Every
1 / rate s this node publishes the final scale

    min(request fresher than request_timeout, else 0;  the limits below)

on /whole_body_mpc/speed_scale and /base_handover_gate/speed_scale, and a
Nav2 SpeedLimit on /speed_limit (percentage, max(scale, 0.05) x 100: Nav2
reads 0.0 as "no limit", so the stop itself is the gate's). The limits, in
the order ~/state names them:

  (a) height cap: 0 while the TCP (TF odom -> piper_tcp) is above
      tcp_height_cap, or unknown, or older than robot_state_timeout; a target
      request above it is rejected, not forwarded
  (b) watchdog: 0 while the newest measured body pose is older than
      person_timeout, by its stamp or by its receipt
  (c) jump: 0 for jump_hold_sec after the measured body moved more than
      max_body_jump between two samples
  (d) stop distance: 0 while the base footprint (/odom) is within
      stop_distance of the measured person capsule (no /odom, or /odom older
      than robot_state_timeout: 0 too)
  and 0 until base_handover_gate has confirmed it is governed.

Release permit ~/release (std_srvs/Trigger): success only when the measured
take was true within take_fresh_sec while the person data is fresh and not
jumping, or when the last forwarded target came on ~/place_target_request
(which must lie on the table top at place_tcp_height +- place_height_tolerance)
and whole_body_mpc reports REACHED. The gripper
and /grasp/set are not behind this node: the permit is checked by the
decision maker and audited by handover_recorder (viol_release).

~/state (String, latched, on change): OK, STOP: <first reason>, or
REJECTED: <why> for a refused target. A quiet decision maker (no fresh
request) gives scale 0 but is not a STOP. At start the node asks
base_handover_gate to be governed (~/set_governed, retried once per second);
on exit (Ctrl-C, SIGTERM, failure; the signals only set a flag) it
publishes scale 0 once, removes the Nav2 limit (0.0) and ungoverns the gate.

Simulation only: nothing here reaches a CAN bus or a real-robot topic.
"""

import importlib.util
import math
from pathlib import Path
import signal
import sys
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def _load_core():
    # resolve(): with --symlink-install this file is a link into the source tree.
    path = Path(__file__).resolve().with_name("handover_core.py")
    spec = importlib.util.spec_from_file_location("handover_core", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


core = _load_core()

# Defaults of the node parameters of the same name (config/handover.yaml).
RATE_HZ = 20.0
REQUEST_TIMEOUT_SEC = 0.5
TAKE_FRESH_SEC = 1.0
ROBOT_STATE_TIMEOUT_SEC = 0.5  # older TCP (TF) or /odom counts as unknown: a stop
PLACE_HEIGHT_TOLERANCE_M = 0.05  # a place target's z off place_tcp_height
SPEED_LIMIT_PERIOD_SEC = 1.0
MIN_SPEED_LIMIT_SCALE = 0.05  # Nav2 cannot express a stop: 0.0 is "no limit"
GOVERN_RETRY_SEC = 1.0
RELEASE_WAIT_SEC = 2.0  # for the gate's answer on exit
TARGET_FRAME = "odom"
TCP_FRAME = "piper_tcp"
MPC_TARGET_TOPIC = "/whole_body_mpc/target"
MPC_SCALE_TOPIC = "/whole_body_mpc/speed_scale"
MPC_STATE_TOPIC = "/whole_body_mpc/state"
GATE_SCALE_TOPIC = "/base_handover_gate/speed_scale"
GATE_GOVERN_SERVICE = "/base_handover_gate/set_governed"
SPEED_LIMIT_TOPIC = "/speed_limit"
REACHED = "REACHED"
OK, STOP, REJECTED = "OK", "STOP: ", "REJECTED: "
REASON_HEIGHT = "tcp above the height cap"
REASON_TCP_UNKNOWN = "tcp position unknown or stale"
REASON_ODOM_STALE = "stop distance (odometry stale)"
REASON_UNGOVERNED = "gate not governed"
REASON_STALE = "person data stale"
REASON_JUMP = "person data jumped"
REASON_DISTANCE = "stop distance"
REASON_NO_ODOM = "stop distance (no odometry)"
REASON_REQUEST = "no scale request"
LATCHED_QOS = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
)
# Matches a volatile and a transient-local subscriber alike.
SPEED_LIMIT_QOS = QoSProfile(
    depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
)


def _fresh(stamp: Optional[float], now: float, timeout: float) -> bool:
    """Whether `stamp` is at most `timeout` old (a negative age, a sim-time reset, is not)."""
    return stamp is not None and 0.0 <= now - stamp <= timeout


class SafetyRules:
    """The hard limits; time comes in as arguments (no clock, no ROS)."""

    def __init__(self, cfg, request_timeout: float, take_fresh_sec: float,
                 robot_state_timeout: float = ROBOT_STATE_TIMEOUT_SEC,
                 place_height_tolerance: float = PLACE_HEIGHT_TOLERANCE_M) -> None:
        self._cfg = cfg
        self._request_timeout = request_timeout
        self._take_fresh = take_fresh_sec
        self._robot_timeout = robot_state_timeout
        self._place_tolerance = place_height_tolerance
        self._person: Optional[core.PersonState] = None
        self._person_receipt: Optional[float] = None
        self._jump_until: Optional[float] = None
        self._base: Optional[tuple[core.Pose2D, float]] = None  # (pose, receipt)
        self._tcp: Optional[tuple[float, float]] = None  # (z, TF stamp)
        self._request: Optional[tuple[float, float]] = None  # (scale, receipt)

    def update_person(self, sample: core.PersonState, receipt_sec: float) -> None:
        """Take a measured body sample, received at `receipt_sec`."""
        previous = self._person
        if previous is not None and sample.stamp_sec != previous.stamp_sec:
            moved = math.hypot(sample.x - previous.x, sample.y - previous.y)
            if moved > self._cfg.max_body_jump:
                self._jump_until = receipt_sec + self._cfg.jump_hold_sec
        self._person, self._person_receipt = sample, receipt_sec

    def update_base(self, pose: Optional[core.Pose2D], receipt_sec: float) -> None:
        """Take the base pose from /odom, received at `receipt_sec` (None: unusable)."""
        self._base = None if pose is None else (pose, receipt_sec)

    def update_tcp_z(self, z: Optional[float], stamp_sec: float = 0.0) -> None:
        """Take the TCP height in odom and its TF stamp (None: no transform)."""
        self._tcp = None if z is None else (z, stamp_sec)

    def update_request(self, scale: float, now: float) -> None:
        """Take the requested scale: clamped to [0, 1], NaN = 0."""
        value = min(1.0, max(0.0, scale)) if math.isfinite(scale) else 0.0
        self._request = (value, now)

    def clearance(self) -> Optional[float]:
        """Footprint to measured capsule (m), or None without base or person."""
        if self._base is None or self._person is None:
            return None
        return core.footprint_clearance(self._base[0], self._cfg.footprint, self._person,
                                        self._cfg)

    def _jumping(self, now: float) -> bool:
        return self._jump_until is not None and now <= self._jump_until

    def _person_stale(self, now: float) -> bool:
        if self._person is None:
            return True
        timeout = self._cfg.person_timeout
        return not (_fresh(self._person.stamp_sec, now, timeout)
                    and _fresh(self._person_receipt, now, timeout))

    def final_scale(self, now: float) -> tuple[float, str]:
        """Return the scale to publish and the first active reason ('' if only the request)."""
        if self._tcp is None or not _fresh(self._tcp[1], now, self._robot_timeout):
            return 0.0, REASON_TCP_UNKNOWN
        if self._tcp[0] > self._cfg.tcp_height_cap:
            return 0.0, REASON_HEIGHT
        if self._person_stale(now):
            return 0.0, REASON_STALE
        if self._jumping(now):
            return 0.0, REASON_JUMP
        if self._base is not None and not _fresh(self._base[1], now, self._robot_timeout):
            return 0.0, REASON_ODOM_STALE
        clearance = self.clearance()
        if clearance is None:
            return 0.0, REASON_NO_ODOM
        if clearance < self._cfg.stop_distance:
            return 0.0, REASON_DISTANCE
        if self._request is None or not _fresh(self._request[1], now, self._request_timeout):
            return 0.0, REASON_REQUEST
        return self._request[0], ""

    def check_target(self, frame: str, position, place: bool = False) -> Optional[str]:
        """Why a target request is refused, or None to forward it."""
        if frame.lstrip("/") != TARGET_FRAME:
            return f"target in frame '{frame}', not '{TARGET_FRAME}'"
        if not all(math.isfinite(v) for v in position):
            return "non-finite target"
        if position[2] > self._cfg.tcp_height_cap:
            return (f"target z {position[2]:.3f} above the height cap "
                    f"{self._cfg.tcp_height_cap:.2f}")
        if place:
            on_table = core.point_rect_distance(position[:2], self._cfg.table_top) == 0.0
            level = abs(position[2] - self._cfg.place_tcp_height) <= self._place_tolerance
            if not (on_table and level):
                return (f"place target ({position[0]:.3f}, {position[1]:.3f}, "
                        f"{position[2]:.3f}) is not on the table top at the place height")
        return None

    def release_allowed(self, now: float, take_seen_sec: Optional[float],
                        place_reached: bool) -> tuple[bool, str]:
        """Grant the release permit on a fresh take, or at a reached table place pose."""
        person_ok = not (self._person_stale(now) or self._jumping(now))
        if _fresh(take_seen_sec, now, self._take_fresh) and person_ok:
            return True, "the person takes the object"
        if place_reached:
            return True, "set down at the table place pose"
        return False, (f"no take within the last {self._take_fresh:.1f} s with fresh "
                       "person data, and not at a place pose")


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class HandoverSafety(Node):
    """The only route from a decision maker to the speed scale, the MPC target and a release."""

    def __init__(self, **kwargs) -> None:
        super().__init__("handover_safety", **kwargs)
        period = self._declare_parameters()
        self._take_seen: Optional[float] = None
        self._mpc_state: Optional[str] = None
        self._place_target = False  # the last forwarded target was a table place pose
        self._state: Optional[str] = None
        self._limit: Optional[tuple[float, float]] = None  # (percentage, time sent)
        self._governed = False
        self._govern_future = None
        self._govern_sent: Optional[float] = None
        self._rejected = 0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._target_pub = self.create_publisher(PoseStamped, MPC_TARGET_TOPIC, 10)
        self._mpc_scale_pub = self.create_publisher(Float64, MPC_SCALE_TOPIC, 10)
        self._gate_scale_pub = self.create_publisher(Float64, GATE_SCALE_TOPIC, 10)
        self._limit_pub = self.create_publisher(SpeedLimit, SPEED_LIMIT_TOPIC, SPEED_LIMIT_QOS)
        self._state_pub = self.create_publisher(String, "~/state", LATCHED_QOS)
        sub = self.create_subscription
        sub(PoseStamped, "/person/measured/body_pose", self._on_body, 10)
        sub(Bool, "/person/measured/take", self._on_take, 10)
        sub(Odometry, "/odom", self._on_odom, 10)
        sub(Float64, "~/scale_request", self._on_scale_request, 10)
        sub(PoseStamped, "~/target_request", self._on_offer_target, 10)
        sub(PoseStamped, "~/place_target_request", self._on_place_target, 10)
        sub(String, MPC_STATE_TOPIC, self._on_mpc_state, LATCHED_QOS)
        self.create_service(Trigger, "~/release", self._on_release)
        self._govern_client = self.create_client(SetBool, GATE_GOVERN_SERVICE)
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"Handover safety at {1.0 / period:.0f} Hz: stop distance "
            f"{self._cfg.stop_distance} m, TCP cap {self._cfg.tcp_height_cap} m, person "
            f"timeout {self._cfg.person_timeout} s (software interlock, simulation only)"
        )

    def _declare_parameters(self) -> float:
        """Declare and check the parameters; returns the timer period (s)."""
        p = self.declare_parameter
        shared = {key: p(key, value).value for key, value in core.SHARED_DEFAULTS.items()}
        self._cfg = core.config_from_parameters(shared)
        positive = {
            "rate": p("rate", RATE_HZ).value,
            "request_timeout": p("request_timeout", REQUEST_TIMEOUT_SEC).value,
            "take_fresh_sec": p("take_fresh_sec", TAKE_FRESH_SEC).value,
            "speed_limit_period": p("speed_limit_period", SPEED_LIMIT_PERIOD_SEC).value,
            "robot_state_timeout": p("robot_state_timeout", ROBOT_STATE_TIMEOUT_SEC).value,
            "place_height_tolerance": p("place_height_tolerance",
                                        PLACE_HEIGHT_TOLERANCE_M).value,
        }
        for name, value in positive.items():
            if not value > 0.0:
                raise ValueError(f"{name} must be positive, got {value}")
        self._limit_period = positive["speed_limit_period"]
        self._rules = SafetyRules(self._cfg, positive["request_timeout"],
                                  positive["take_fresh_sec"], positive["robot_state_timeout"],
                                  positive["place_height_tolerance"])
        return 1.0 / positive["rate"]

    def _now(self) -> float:
        """Node clock (sim time in the sim) in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- inputs ---------------------------------------------------------------------
    def _on_body(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        facing = _yaw_of(msg.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg.header.frame_id.lstrip("/") != TARGET_FRAME or not all(
                map(math.isfinite, (p.x, p.y, facing))):
            return  # dropped: the watchdog sees silence
        self._rules.update_person(
            core.PersonState(p.x, p.y, facing, (0.0, 0.0, 0.0), False, False, stamp),
            self._now())

    def _on_take(self, msg: Bool) -> None:
        if msg.data:
            self._take_seen = self._now()

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        base = (pose.position.x, pose.position.y, _yaw_of(pose.orientation))
        self._rules.update_base(base if all(map(math.isfinite, base)) else None, self._now())

    def _on_scale_request(self, msg: Float64) -> None:
        self._rules.update_request(msg.data, self._now())

    def _on_mpc_state(self, msg: String) -> None:
        self._mpc_state = msg.data

    def _on_offer_target(self, msg: PoseStamped) -> None:
        self._forward(msg, place=False)

    def _on_place_target(self, msg: PoseStamped) -> None:
        self._forward(msg, place=True)

    def _forward(self, msg: PoseStamped, place: bool) -> None:
        p = msg.pose.position
        reason = self._rules.check_target(msg.header.frame_id, (p.x, p.y, p.z), place)
        if reason is not None:
            self._rejected += 1
            self.get_logger().warn(f"target refused ({self._rejected} so far): {reason}")
            self._set_state(REJECTED + reason)
            return
        self._place_target = place
        self._target_pub.publish(msg)

    def _on_release(self, request: Trigger.Request,
                    response: Trigger.Response) -> Trigger.Response:
        place_reached = self._place_target and self._mpc_state == REACHED
        response.success, response.message = self._rules.release_allowed(
            self._now(), self._take_seen, place_reached)
        log = self.get_logger().info if response.success else self.get_logger().warn
        log(f"release {'permitted' if response.success else 'refused'}: {response.message}")
        return response

    def _tcp(self) -> tuple[Optional[float], float]:
        """Return the TCP z in odom and the transform's stamp; (None, 0) without one."""
        try:
            transform = self._tf_buffer.lookup_transform(TARGET_FRAME, TCP_FRAME, Time())
        except TransformException:
            return None, 0.0
        stamp = transform.header.stamp
        return transform.transform.translation.z, stamp.sec + stamp.nanosec * 1e-9

    # ---- outputs ----------------------------------------------------------------------
    def _set_state(self, state: str) -> None:
        if state == self._state:
            return
        self.get_logger().info(f"{self._state or 'START'} -> {state}")
        self._state = state
        self._state_pub.publish(String(data=state))

    def _publish_scale(self, scale: float) -> None:
        self._mpc_scale_pub.publish(Float64(data=float(scale)))
        self._gate_scale_pub.publish(Float64(data=float(scale)))

    def _publish_limit(self, percentage: float, now: float) -> None:
        msg = SpeedLimit()
        msg.header.frame_id = TARGET_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage, msg.speed_limit = True, float(percentage)
        self._limit_pub.publish(msg)
        self._limit = (percentage, now)

    def _govern(self, now: float) -> None:
        """Ask the gate to be governed until it answered (async, once per GOVERN_RETRY_SEC)."""
        if self._governed:
            return
        future = self._govern_future
        if future is not None and future.done():
            result = future.result()
            self._governed = result is not None and result.success
            self._govern_future = None
            if self._governed:
                self.get_logger().info("base_handover_gate governed")
                return
        if self._govern_future is not None or not self._govern_client.service_is_ready():
            return
        if self._govern_sent is not None and 0.0 <= now - self._govern_sent < GOVERN_RETRY_SEC:
            return
        self._govern_future = self._govern_client.call_async(SetBool.Request(data=True))
        self._govern_sent = now

    def _tick(self) -> None:
        now = self._now()
        self._govern(now)
        self._rules.update_tcp_z(*self._tcp())
        scale, reason = self._rules.final_scale(now)
        if not self._governed:
            # The gate ignores the scale until it is governed: nothing may move yet.
            scale, reason = 0.0, REASON_UNGOVERNED
        self._publish_scale(scale)
        percentage = 100.0 * max(scale, MIN_SPEED_LIMIT_SCALE)
        if (self._limit is None or self._limit[0] != percentage
                or not 0.0 <= now - self._limit[1] < self._limit_period):
            self._publish_limit(percentage, now)
        # A request timeout is not a stop of the safety layer: the decision maker is quiet.
        self._set_state(STOP + reason if reason and reason != REASON_REQUEST else OK)

    def release_gate(self) -> None:
        """Stop once, remove the Nav2 limit and ungovern the gate (exit path, best effort)."""
        self._publish_scale(0.0)
        self._publish_limit(0.0, self._now())
        if not self._govern_client.service_is_ready():
            self.get_logger().warn(f"{GATE_GOVERN_SERVICE} not available: gate left as it is")
            return
        future = self._govern_client.call_async(SetBool.Request(data=False))
        rclpy.spin_until_future_complete(self, future, timeout_sec=RELEASE_WAIT_SEC)
        if future.result() is None:
            self.get_logger().error(
                f"gate not ungoverned: call {GATE_GOVERN_SERVICE} with data false by hand")
        else:
            self.get_logger().info("base_handover_gate ungoverned, Nav2 speed limit removed")


SPIN_PERIOD_SEC = 0.1


def main() -> None:
    # The signals only set a flag (handover_core.run_until_signal): release_gate()
    # still has to call the gate's service on an intact executor.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = HandoverSafety()
    failed = False
    try:
        core.run_until_signal(lambda: rclpy.spin_once(node, timeout_sec=SPIN_PERIOD_SEC),
                              rclpy.ok)
    except ExternalShutdownException:
        pass
    except Exception as exc:  # noqa: BLE001 - release the gate whatever happened
        node.get_logger().error(f"handover_safety failed: {exc!r}")
        failed = True
    finally:
        # A second Ctrl-C or SIGTERM must not cut the release short (launch
        # escalates to SIGKILL after its timeout anyway).
        held = {sig: signal.signal(sig, signal.SIG_IGN) for sig in (signal.SIGINT, signal.SIGTERM)}
        if rclpy.ok():
            try:
                node.release_gate()
            except Exception as exc:  # noqa: BLE001 - exit path
                node.get_logger().error(f"release failed: {exc!r}")
        for sig, handler in held.items():
            signal.signal(sig, handler)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
