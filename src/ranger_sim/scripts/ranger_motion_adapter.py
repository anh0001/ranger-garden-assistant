#!/usr/bin/env python3
"""
Turn /cmd_vel into Ranger Mini 3.0 steering, wheel and body commands for the sim.

The real Ranger Mini 3.0 is a four-wheel swerve base: each wheel turns about
z to an angle, then rolls. Its driver (ranger_ros2/ranger_base
ranger_messenger.cpp, TwistCmdCallback) never blends vx, vy and wz; it picks
one motion mode per Twist:

  * any linear.y      -> PARALLEL: every wheel at atan(vy/vx), angular.z dropped
  * turn radius |vx/wz| below min_turn_radius (0.476 m)
                      -> SPINNING: wheels in an X, linear part dropped
  * otherwise         -> DUAL_ACKERMANN: front and rear wheels counter-steer

In the Fortress sim the chassis is still moved kinematically by
VelocityControl (urdf/ranger_gazebo.urdf.xacro); wheel contact does not
propel it. This node sits between /cmd_vel and the sim and makes the
wheels behave like the real ones: it selects the mode as the driver does,
computes the four steering angles and wheel speeds that match the permitted
body twist, and withholds body twist and wheel speed until the steering
joints have reached their targets (steer, then drive). It publishes

  * BODY_TOPIC (/cmd_vel_body)            -> VelocityControl (body)
  * /ranger/<steering joint>/cmd_pos  x4   -> JointPositionController
  * /ranger/<wheel joint>/cmd_vel      x4   -> JointController

every tick, zeros included: VelocityControl keeps the last twist it received
forever, so a quiet /cmd_vel (cmd_timeout) stops the base here.

Simulation only: nothing here reaches a CAN bus or a real-robot topic.
"""

import enum
import math
from typing import Sequence

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Joint order shared by every list below (the xacro's steering_joints and
# wheel_joints properties).
STEERING_JOINTS = [
    "fl_steering_joint",
    "fr_steering_joint",
    "rl_steering_joint",
    "rr_steering_joint",
]
WHEEL_JOINTS = ["fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel"]
# (x, y) of each steering joint in base_link (ranger_description
# ranger_complete.urdf.xacro). The rr origin really differs from the other
# three in the URDF; test_pick_place_geometry.py pins these to the URDF.
WHEEL_POSITIONS_M = [(0.25, 0.19), (0.25, -0.19), (-0.25, 0.19), (-0.23, -0.206)]
WHEEL_RADIUS_M = 0.09  # wheel collision cylinder radius

# From ranger_ros2/ranger_base ranger_params.hpp, RangerMiniV3Params.
MIN_TURN_RADIUS_M = 0.47644  # min_turn_radius
MAX_LINEAR_SPEED_MPS = 1.5  # max_linear_speed
MAX_ANGULAR_SPEED_RPS = 4.8  # max_angular_speed
MAX_STEER_RAD = math.pi / 2  # max_steer_angle_parallel (1.570)
TWIST_EPS = 1e-6  # CalculateSteeringAngle: angular < 1e-6 means straight

# Wheel ground speed below which a wheel keeps its previous heading.
MIN_WHEEL_SPEED_MPS = 1e-4
# Within this of +-pi/2 both steering signs point the wheel the same way;
# the one nearer the previous heading wins (side slip does not flap).
SIDE_SLIP_BAND_RAD = 1e-3
# The URDF steering axis is "0 0 -1": joint position = -(heading about +z).
STEERING_AXIS_SIGN = -1.0
# SteeringGate defaults (parameters align_enter_rad / align_exit_rad): a
# steering error above ALIGN_ENTER_RAD stops the base until it is below
# ALIGN_EXIT_RAD.
ALIGN_ENTER_RAD = 0.20
ALIGN_EXIT_RAD = 0.05

BODY_TOPIC = "/cmd_vel_body"

Twist2D = tuple[float, float, float]  # (vx m/s, vy m/s, wz rad/s), body frame
ZERO_TWIST: Twist2D = (0.0, 0.0, 0.0)


class Mode(enum.IntEnum):
    """ugv_sdk motion modes (MotionState::MOTION_MODE_*) the V3 driver uses."""

    DUAL_ACKERMANN = 0
    PARALLEL = 1
    SPINNING = 2


def steering_topic(joint: str) -> str:
    """ROS topic of the position command for steering joint `joint`."""
    return f"/ranger/{joint}/cmd_pos"


def wheel_topic(joint: str) -> str:
    """ROS topic of the velocity command for wheel joint `joint`."""
    return f"/ranger/{joint}/cmd_vel"


def _linear_scale(vx: float, vy: float) -> float:
    """Factor that brings hypot(vx, vy) down to MAX_LINEAR_SPEED_MPS (1.0 if under)."""
    speed = math.hypot(vx, vy)
    return 1.0 if speed <= MAX_LINEAR_SPEED_MPS else MAX_LINEAR_SPEED_MPS / speed


def select_mode(vx: float, vy: float, wz: float) -> tuple[Mode, Twist2D]:
    """
    Return the driver's motion mode for a commanded twist and the body twist it permits.

    The mode choice mirrors TwistCmdCallback for the Ranger Mini V3:
    linear.y selects PARALLEL and drops wz; a turn radius under
    MIN_TURN_RADIUS_M selects SPINNING and drops the linear part; anything
    else is DUAL_ACKERMANN. The linear speed is capped at
    MAX_LINEAR_SPEED_MPS; a capped Ackermann turn keeps its radius (the
    driver steers from the radius and sends the speed apart). The wheel
    angles are not the driver's: wheel_targets() steers each wheel along the
    permitted body twist. The driver's PARALLEL steer formula negates
    atan(vy/vx) for vx < 0 and, for vx == 0, picks the sign from the last
    non-zero vx; how the firmware reads that is unverified.
    """
    if abs(vy) >= TWIST_EPS:
        scale = _linear_scale(vx, vy)
        return Mode.PARALLEL, (vx * scale, vy * scale, 0.0)
    if abs(wz) < TWIST_EPS:
        return Mode.DUAL_ACKERMANN, (vx * _linear_scale(vx, 0.0), 0.0, 0.0)
    if abs(vx) / abs(wz) < MIN_TURN_RADIUS_M:
        rate = max(-MAX_ANGULAR_SPEED_RPS, min(MAX_ANGULAR_SPEED_RPS, wz))
        return Mode.SPINNING, (0.0, 0.0, rate)
    scale = _linear_scale(vx, 0.0)
    return Mode.DUAL_ACKERMANN, (vx * scale, 0.0, wz * scale)


def _wheel_target(v_x: float, v_y: float, previous: float) -> tuple[float, float]:
    """(heading rad, wheel speed rad/s) that rolls a wheel at ground velocity (v_x, v_y)."""
    speed = math.hypot(v_x, v_y)
    if speed < MIN_WHEEL_SPEED_MPS:
        return previous, 0.0
    heading = math.atan2(v_y, v_x)
    # Steer within +-MAX_STEER_RAD and roll backwards instead of turning further.
    if heading > MAX_STEER_RAD:
        heading, speed = heading - math.pi, -speed
    elif heading < -MAX_STEER_RAD:
        heading, speed = heading + math.pi, -speed
    if abs(heading) > MAX_STEER_RAD - SIDE_SLIP_BAND_RAD:
        flipped = heading - math.copysign(math.pi, heading)
        if abs(flipped - previous) < abs(heading - previous):
            heading, speed = flipped, -speed
    return heading, speed / WHEEL_RADIUS_M


def wheel_targets(body: Twist2D, previous_headings: Sequence[float]) -> list[tuple[float, float]]:
    """
    (heading rad, wheel speed rad/s) per wheel, in STEERING_JOINTS order.

    Swerve inverse kinematics: the wheel at (lx, ly) moves over the ground at
    (vx - wz*ly, vy + wz*lx); it steers to that direction (about +z, robot
    frame) and rolls at its magnitude. Headings stay within +-MAX_STEER_RAD
    (a side-slipping wheel may follow its previous heading up to
    SIDE_SLIP_BAND_RAD past it); a wheel that would not move keeps
    `previous_headings[i]`.
    """
    vx, vy, wz = body
    return [
        _wheel_target(vx - wz * ly, vy + wz * lx, previous)
        for (lx, ly), previous in zip(WHEEL_POSITIONS_M, previous_headings)
    ]


def steering_joint_position(heading: float) -> float:
    """Steering joint position (rad) that points a wheel at `heading` about +z."""
    return STEERING_AXIS_SIGN * heading


class SteeringGate:
    """
    Steer, then drive: whether the base may move given steering targets and angles.

    DRIVING until the worst steering error exceeds `enter_rad`, then ALIGNING
    (body and wheels held) until it is under `exit_rad`. The gap between the
    two keeps the continuous small changes of an Ackermann turn from stopping
    the base. An alignment that takes longer than `timeout_sec` releases the
    base anyway and sets `timed_out`; the gate then stays open until the
    steering catches up (error under `exit_rad`), so a stuck joint does not
    turn into a stop-and-go. `timed_out` clears when a new alignment starts.
    Time is passed in, so the class needs no clock.
    """

    def __init__(self, enter_rad: float, exit_rad: float, timeout_sec: float) -> None:
        self._enter_rad = enter_rad
        self._exit_rad = exit_rad
        self._timeout_sec = timeout_sec
        self._aligning = False
        self._since_sec = 0.0
        self._released = False  # timed out, waiting for the steering to catch up
        self.timed_out = False

    @property
    def aligning(self) -> bool:
        """Whether the base is held for the steering."""
        return self._aligning

    def update(
        self,
        targets: Sequence[float],
        measured: Sequence[float | None] | None,
        now_sec: float,
    ) -> bool:
        """
        Advance the gate; True when the base may drive.

        `targets` and `measured` are steering joint positions (rad) in the
        same order; unmeasured joints (None or non-finite, or `measured`
        None) are left out, and with none measured the gate passes
        everything through.
        """
        errors = [
            abs(target - angle)
            for target, angle in zip(targets, measured or ())
            if angle is not None and math.isfinite(angle)
        ]
        if not errors:
            self._aligning = False
            return True
        error = max(errors)
        if self._released:
            self._released = error >= self._exit_rad
            return True
        if not self._aligning:
            if error > self._enter_rad:
                self._aligning = True
                self._since_sec = now_sec
                self.timed_out = False
            return not self._aligning
        if error < self._exit_rad:
            self._aligning = False
        elif now_sec - self._since_sec > self._timeout_sec:
            self._aligning = False
            self._released = True
            self.timed_out = True
        return not self._aligning


class RangerMotionAdapter(Node):
    """Drive the sim's steering joints, wheel joints and body from ``input_topic``."""

    def __init__(self, **kwargs) -> None:
        super().__init__("ranger_motion_adapter", **kwargs)
        rate = self._declare_parameters()
        # Latest command (vx, vy, wz) and when it arrived (None: never).
        self._cmd: Twist2D = ZERO_TWIST
        self._cmd_sec: float | None = None
        self._steering_angles: dict[str, float] = {}  # replaced per message
        self._headings = [0.0] * len(STEERING_JOINTS)  # last steering targets about +z
        self._mode: Mode | None = None
        # Degraded states already warned about (warn once when one begins).
        self._warned = {"invalid": False, "quiet": False, "unmeasured": False, "timeout": False}
        self._body_pub = self.create_publisher(Twist, BODY_TOPIC, 10)
        self._steering_pubs = [
            self.create_publisher(Float64, steering_topic(joint), 10) for joint in STEERING_JOINTS
        ]
        self._wheel_pubs = [
            self.create_publisher(Float64, wheel_topic(joint), 10) for joint in WHEEL_JOINTS
        ]
        self.create_subscription(Twist, self._input_topic, self._on_cmd, 10)
        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_states, 10)
        # Publish on a timer, not on receipt: VelocityControl keeps the last
        # twist forever, so the zeros of a quiet /cmd_vel must be sent too.
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Steering and driving the sim base from {self._input_topic} at {rate} Hz "
            f"(body on {BODY_TOPIC}, steering angles from {self._joint_state_topic})"
        )

    def _declare_parameters(self) -> float:
        """Declare and check the parameters; returns the publish rate (Hz)."""
        rate = self.declare_parameter("publish_rate", 50.0).value
        self._cmd_timeout = self.declare_parameter("cmd_timeout", 0.5).value
        enter_rad = self.declare_parameter("align_enter_rad", ALIGN_ENTER_RAD).value
        exit_rad = self.declare_parameter("align_exit_rad", ALIGN_EXIT_RAD).value
        align_timeout = self.declare_parameter("align_timeout", 3.0).value
        self._straighten_on_stop = self.declare_parameter("straighten_on_stop", False).value
        self._input_topic = self.declare_parameter("input_topic", "/cmd_vel").value
        self._joint_state_topic = self.declare_parameter(
            "joint_state_topic", "/gz/base_joint_states"
        ).value
        if min(rate, self._cmd_timeout, exit_rad, align_timeout) <= 0.0:
            raise ValueError(
                "publish_rate, cmd_timeout, align_exit_rad and align_timeout must be positive"
            )
        if exit_rad >= enter_rad:
            raise ValueError(
                f"align_exit_rad ({exit_rad}) must be below align_enter_rad ({enter_rad})"
            )
        self._gate = SteeringGate(enter_rad, exit_rad, align_timeout)
        return rate

    def _now(self) -> float:
        """Node clock (sim time in the sim) in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _warn_once(self, key: str, active: bool, text: str) -> None:
        """Warn when degraded state `key` begins; re-arm once it ends."""
        if active and not self._warned[key]:
            self.get_logger().warn(text)
        self._warned[key] = active

    def _on_cmd(self, msg: Twist) -> None:
        cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        valid = all(math.isfinite(value) for value in cmd)
        self._warn_once(
            "invalid", not valid, f"non-finite twist on {self._input_topic}: treated as a stop"
        )
        self._cmd = cmd if valid else ZERO_TWIST
        self._cmd_sec = self._now()

    def _on_joint_states(self, msg: JointState) -> None:
        self._steering_angles = {
            name: position
            for name, position in zip(msg.name, msg.position)
            if name in STEERING_JOINTS
        }

    def _current_command(self) -> Twist2D:
        """Return the latest command, or a stop once older than cmd_timeout (watchdog)."""
        # A negative age (sim time reset) counts as quiet: the command is stale.
        quiet = self._cmd_sec is None or not (
            0.0 <= self._now() - self._cmd_sec <= self._cmd_timeout
        )
        self._warn_once(
            "quiet",
            quiet and self._cmd != ZERO_TWIST,
            f"no {self._input_topic} for {self._cmd_timeout} s: base stopped",
        )
        return ZERO_TWIST if quiet else self._cmd

    def _targets(self, body: Twist2D) -> tuple[list[float], list[float]]:
        """Steering headings (rad about +z) and wheel speeds (rad/s) for `body`."""
        if body == ZERO_TWIST:
            headings = [0.0] * len(STEERING_JOINTS) if self._straighten_on_stop else self._headings
            return list(headings), [0.0] * len(WHEEL_JOINTS)
        targets = wheel_targets(body, self._headings)
        return [heading for heading, _ in targets], [speed for _, speed in targets]

    def _tick(self) -> None:
        mode, body = select_mode(*self._current_command())
        if mode != self._mode:
            self.get_logger().info(f"Motion mode {mode.name}")
            self._mode = mode
        self._headings, speeds = self._targets(body)
        joints = [steering_joint_position(heading) for heading in self._headings]
        measured = [self._steering_angles.get(joint) for joint in STEERING_JOINTS]
        unmeasured = all(angle is None for angle in measured)
        if body != ZERO_TWIST or not unmeasured:
            # Warn at the first drive without angles; re-arm once they arrive.
            self._warn_once(
                "unmeasured",
                unmeasured,
                f"no steering angles on {self._joint_state_topic}: driving without "
                "waiting for the steering",
            )
        driving = self._gate.update(joints, measured, self._now())
        self._warn_once(
            "timeout",
            self._gate.timed_out,
            "steering did not reach its targets in time: driving anyway",
        )
        if not driving:
            body, speeds = ZERO_TWIST, [0.0] * len(WHEEL_JOINTS)
        self._publish(body, joints, speeds)

    def _publish(self, body: Twist2D, joints: list[float], speeds: list[float]) -> None:
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = body
        self._body_pub.publish(twist)
        for pub, position in zip(self._steering_pubs, joints):
            pub.publish(Float64(data=position))
        for pub, speed in zip(self._wheel_pubs, speeds):
            pub.publish(Float64(data=speed))


def main() -> None:
    rclpy.init()
    node = RangerMotionAdapter()
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
