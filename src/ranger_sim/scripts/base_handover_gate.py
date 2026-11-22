#!/usr/bin/env python3
"""
Hand the sim base back and forth between navigation and the whole-body MPC.

Two writers want the base: navigation (Nav2, or a scripted demo) on
/cmd_vel, and whole_body_mpc on /cmd_vel_manip. This node is the only
publisher of the motion adapter's input (/cmd_vel_gated when
spawn_robot.launch.py runs with handover_gate:=true) and passes exactly one
of them through:

  NAVIGATE    /cmd_vel unchanged
  SETTLING    zeros, until /odom reports the base still (linear and angular
              speed under still_linear / still_angular) for settle_sec
  MANIPULATE  /cmd_vel_manip, translation only: angular.z is dropped and the
              linear part is scaled down to manip_max_speed

A change of owner (service ~/set_manipulate, std_srvs/SetBool: true =
MANIPULATE) always goes through SETTLING, so neither writer is handed a
moving base. The translation-only rule is the Ranger driver's PARALLEL mode
(ranger_messenger.cpp TwistCmdCallback: any linear.y drops angular.z); it
holds here even if the MPC node misbehaves. A twist older than cmd_timeout
(or stamped in the future after a sim-time jump back) or with a non-finite
value is a stop. The twist is published every tick, zeros included: the
adapter stops a quiet input, Gazebo's VelocityControl would not. The mode
is published on ~/mode (transient local) when it changes.

Simulation only: nothing here reaches a CAN bus or a real-robot topic.
"""

import enum
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

# Defaults of the node parameters of the same name (no unit suffix there).
SETTLE_SEC = 0.3  # still this long before the new owner gets the base
STILL_LINEAR_MPS = 0.005  # /odom linear speed below which the base is still
STILL_ANGULAR_RPS = 0.01  # /odom angular speed below which the base is still
CMD_TIMEOUT_SEC = 0.5  # the motion adapter's cmd_timeout
ODOM_TIMEOUT_SEC = 0.5  # older /odom is missing: the settle wait restarts
# Above hypot(0.10, 0.10) = 0.141 m/s, the MPC's per-axis base speed bound
# (config/wbmpc.yaml base_speed): a legal MPC command is never scaled.
MANIP_MAX_SPEED_MPS = 0.15
PUBLISH_RATE_HZ = 50.0  # the motion adapter's publish_rate
NAV_TOPIC = "/cmd_vel"  # Nav2 / scripted demos
MANIP_TOPIC = "/cmd_vel_manip"  # whole_body_mpc
# The motion adapter's input_topic with handover_gate:=true (spawn_robot.launch.py).
OUTPUT_TOPIC = "/cmd_vel_gated"

Twist2D = tuple[float, float, float]  # (vx m/s, vy m/s, wz rad/s), body frame
ZERO_TWIST: Twist2D = (0.0, 0.0, 0.0)
Stamped = tuple[Twist2D, float]  # (twist, receipt time in s)
LATCHED_QOS = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
)


class Mode(enum.Enum):
    """Owner of the base command."""

    NAVIGATE = "NAVIGATE"
    SETTLING = "SETTLING"
    MANIPULATE = "MANIPULATE"


def _finite(twist: Twist2D) -> bool:
    return all(math.isfinite(value) for value in twist)


class HandoverGate:
    """
    Which input owns the base, and the twist it may send.

    request() starts a change of owner through SETTLING; update() ends the
    settle wait once the base has been still for `settle_sec` without a gap
    (a moving or missing sample, or a clock jump back, restarts it);
    select() returns the twist of the current owner, counting only twists
    received after it took over. Time is passed in, so the class needs no
    clock.
    """

    def __init__(
        self,
        settle_sec: float,
        still_linear_mps: float,
        still_angular_rps: float,
        cmd_timeout_sec: float,
        manip_max_speed_mps: float,
    ) -> None:
        if min(settle_sec, still_linear_mps, still_angular_rps, cmd_timeout_sec,
               manip_max_speed_mps) <= 0.0:
            raise ValueError("every HandoverGate setting must be positive")
        self._settle_sec = settle_sec
        self._still_linear = still_linear_mps
        self._still_angular = still_angular_rps
        self._cmd_timeout = cmd_timeout_sec
        self._max_speed = manip_max_speed_mps
        self._mode = Mode.NAVIGATE
        self._target = Mode.NAVIGATE  # owner after SETTLING
        self._still_since: float | None = None
        self._owner_since: float | None = None  # when the current owner took over

    @property
    def mode(self) -> Mode:
        """Return the current mode."""
        return self._mode

    def request(self, manipulate: bool) -> Mode:
        """Ask for MANIPULATE (True) or NAVIGATE; returns the owner it settles towards."""
        wanted = Mode.MANIPULATE if manipulate else Mode.NAVIGATE
        if self._mode is Mode.SETTLING:
            self._target = wanted
        elif self._mode is not wanted:
            self._mode, self._target = Mode.SETTLING, wanted
            self._still_since = None
        return self._target if self._mode is Mode.SETTLING else self._mode

    def update(self, base_speed: tuple[float, float] | None, now_sec: float) -> Mode:
        """
        Advance the settle wait with one base speed sample; returns the mode.

        `base_speed` is (linear m/s, angular rad/s) from odometry, or None
        when there is no fresh sample.
        """
        if self._mode is not Mode.SETTLING:
            return self._mode
        still = base_speed is not None and (
            base_speed[0] < self._still_linear and base_speed[1] < self._still_angular
        )
        if not still:
            self._still_since = None
        elif self._still_since is None or now_sec < self._still_since:
            self._still_since = now_sec
        elif now_sec - self._still_since >= self._settle_sec:
            self._mode, self._still_since = self._target, None
            self._owner_since = now_sec
        return self._mode

    def _fresh(self, entry: Stamped | None, now_sec: float) -> Twist2D:
        """Return the twist of `entry` if recent and finite, else a stop."""
        if entry is None:
            return ZERO_TWIST
        twist, stamp = entry
        if self._owner_since is not None and now_sec < self._owner_since:
            self._owner_since = None  # sim time jumped back
        # A twist sent before the current owner took over is a stale one.
        before_owner = self._owner_since is not None and stamp < self._owner_since
        if (before_owner or not (0.0 <= now_sec - stamp <= self._cmd_timeout)
                or not _finite(twist)):
            return ZERO_TWIST
        return twist

    def select(self, nav: Stamped | None, manip: Stamped | None, now_sec: float) -> Twist2D:
        """Twist for the motion adapter from the latest nav and manip twists."""
        if self._mode is Mode.NAVIGATE:
            return self._fresh(nav, now_sec)
        if self._mode is Mode.SETTLING:
            return ZERO_TWIST
        vx, vy, _ = self._fresh(manip, now_sec)
        speed = math.hypot(vx, vy)
        scale = 1.0 if speed <= self._max_speed else self._max_speed / speed
        return vx * scale, vy * scale, 0.0


class BaseHandoverGate(Node):
    """Publish the twist of the base's current owner on ``output_topic``."""

    def __init__(self, **kwargs) -> None:
        super().__init__("base_handover_gate", **kwargs)
        rate = self._declare_parameters()
        self._nav: Stamped | None = None
        self._manip: Stamped | None = None
        self._odom_sec: float | None = None
        self._published_mode: Mode | None = None
        self._warned = {"nav": False, "manip": False, "odom": False}
        self._cmd_pub = self.create_publisher(Twist, self._output_topic, 10)
        self._mode_pub = self.create_publisher(String, "~/mode", LATCHED_QOS)
        self.create_subscription(Twist, self._nav_topic, self._on_nav, 10)
        self.create_subscription(Twist, self._manip_topic, self._on_manip, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_service(SetBool, "~/set_manipulate", self._on_set_manipulate)
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Base handover gate: {self._nav_topic} (navigate) / {self._manip_topic} "
            f"(manipulate) -> {self._output_topic} at {rate} Hz"
        )

    def _declare_parameters(self) -> float:
        """Declare and check the parameters; returns the publish rate (Hz)."""
        p = self.declare_parameter
        rate = p("publish_rate", PUBLISH_RATE_HZ).value
        positive = {
            "publish_rate": rate,
            "cmd_timeout": p("cmd_timeout", CMD_TIMEOUT_SEC).value,
            "odom_timeout": p("odom_timeout", ODOM_TIMEOUT_SEC).value,
            "settle_sec": p("settle_sec", SETTLE_SEC).value,
            "still_linear": p("still_linear", STILL_LINEAR_MPS).value,
            "still_angular": p("still_angular", STILL_ANGULAR_RPS).value,
            "manip_max_speed": p("manip_max_speed", MANIP_MAX_SPEED_MPS).value,
        }
        self._nav_topic = p("nav_topic", NAV_TOPIC).value
        self._manip_topic = p("manip_topic", MANIP_TOPIC).value
        self._output_topic = p("output_topic", OUTPUT_TOPIC).value
        self._odom_topic = p("odom_topic", "/odom").value
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive, got {value}")
        self._odom_timeout = positive["odom_timeout"]
        self._gate = HandoverGate(
            positive["settle_sec"], positive["still_linear"], positive["still_angular"],
            positive["cmd_timeout"], positive["manip_max_speed"],
        )
        return rate

    def _now(self) -> float:
        """Node clock (sim time in the sim) in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _warn_once(self, key: str, active: bool, text: str) -> None:
        """Warn when degraded state `key` begins; re-arm once it ends."""
        if active and not self._warned[key]:
            self.get_logger().warn(text)
        self._warned[key] = active

    def _stamp(self, key: str, topic: str, msg: Twist) -> Stamped:
        twist = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._warn_once(
            key, not _finite(twist), f"non-finite twist on {topic}: treated as a stop"
        )
        return twist, self._now()

    def _on_nav(self, msg: Twist) -> None:
        self._nav = self._stamp("nav", self._nav_topic, msg)

    def _on_manip(self, msg: Twist) -> None:
        self._manip = self._stamp("manip", self._manip_topic, msg)

    def _on_odom(self, msg: Odometry) -> None:
        twist = msg.twist.twist
        speed = (math.hypot(twist.linear.x, twist.linear.y), abs(twist.angular.z))
        self._odom_sec = self._now()
        # Each real sample advances the settle wait; the tick only reports gaps.
        self._gate.update(speed if all(map(math.isfinite, speed)) else None, self._odom_sec)

    def _on_set_manipulate(self, request: SetBool.Request,
                           response: SetBool.Response) -> SetBool.Response:
        target = self._gate.request(request.data)
        response.success = True
        response.message = (
            f"base owner settles towards {target.value}"
            if self._gate.mode is Mode.SETTLING
            else f"base owner is {target.value}"
        )
        return response

    def _tick(self) -> None:
        now = self._now()
        # A negative age (sim time reset) counts as stale.
        stale = self._odom_sec is None or not (0.0 <= now - self._odom_sec <= self._odom_timeout)
        settling = self._gate.mode is Mode.SETTLING
        self._warn_once(
            "odom",
            stale and settling,
            f"no fresh {self._odom_topic} for {self._odom_timeout} s: base not handed over",
        )
        if stale:
            self._gate.update(None, now)
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = self._gate.select(
            self._nav, self._manip, now
        )
        self._cmd_pub.publish(twist)
        mode = self._gate.mode
        if mode is not self._published_mode:
            self._published_mode = mode
            self._mode_pub.publish(String(data=mode.value))
            self.get_logger().info(f"Base owner: {mode.value}")


def main() -> None:
    rclpy.init()
    node = BaseHandoverGate()
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
