#!/usr/bin/env python3
"""
Pick-and-place transfer demo for the Ranger Mini 3.0 + PiPER arm in Gazebo Fortress.

Pipeline
--------
  1. Release any held object, open the gripper, arm to 'home' and then to
     the level 'carry' pose -- BEFORE the base moves.
  2. Drive the base to the pick standoff, `standoff_m` short of the approach
     pose (pose controller on /odom).
  3. Bring the level gripper to grasp height ("look at the object"): the
     wrist camera now has the object centred. Slide the base in so the open
     finger pads straddle the object body.
  4. Attach the object (/grasp/set -> DetachableJoint), close on the body,
     lift it straight up, retract to the carry pose.
  5. Drive `place_route` (back off, turn, drive) to the place standoff, hover,
     slide in, set the object down, open, release, clear upward, back off.
  6. Drive the route back to where the base started, retract to 'home'.

Grasp
-----
The object is held by a simulator-native fixed joint (grasp_manager.py
serves /grasp/set on top of Gazebo's DetachableJoint), not by finger
friction. It is attached BEFORE the fingers close so the physics engine
never creates the joint while the fingers interpenetrate it; the fingers
then close to its radius, i.e. the pads rest on its surface. The gripper is
driven through control_msgs/GripperCommand on
/piper_gripper_controller/gripper_cmd, the same interface MoveIt and the
real PiPER bridge use. The attach keeps the object's pose relative to the
gripper, so going down to the grasp pose at the place sets it on a table
of the same height, the same distance ahead of the base as it was picked.

Why joint-space waypoints
-------------------------
The PiPER is a small arm on a low mobile base. Near the edge of its
workspace (a cup on a 0.82 m table) Cartesian pose goals frequently have
no IK solution and OMPL fails ("Unable to sample valid states for goal
tree"). The waypoints below were chosen from MoveIt FK so every TCP pose
is verified reachable; the objects are placed in the world to match the
grasp TCP. This is the robust, deterministic approach for a scripted demo.

Why the approach is along the wrist camera
------------------------------------------
The RealSense D405 rides on top of the gripper and, with the gripper level,
looks 30 deg down along the approach (optical axis (0.86, 0.06, -0.51) in
base_footprint). Coming down on the object from above, as this demo used
to, left the object 49-54 deg off that axis, outside the camera's 27.8 deg
half vertical field of view: it never saw what it grasped. And the old
carry pose (LIFT, approach axis (0.63, 0, 0.78)) pointed the gripper 51 deg
up, tilting the held cup back. Now the gripper goes to grasp height with the
base `standoff_m` = 0.15 m short of the object: the object is ~4 deg off the
optical axis at 0.20 m, centred in the wrist image, and it stays in view
(<= 19 deg) halfway through the slide-in. The carry pose is level.
test/test_pick_place_geometry.py checks this with FK.

Geometry (base_footprint frame; garden_world table top z = 0.77, front
edge at world x = 0.90; cup r = 0.025, h = 0.10, centre (0.915, 0, 0.82),
rim z = 0.87). Gazebo rests base_footprint 1 cm below the ground (the
wheel geometry says 2 cm; the waypoints are checked for both), so world
z = frame z - 0.01..0.02. HOVER, GRASP and CARRY keep the gripper level
(approach +x, fingers closing along y); HOVER and GRASP share link6
x = 0.290, so the finger pads span x 0.349..0.426 (cup centre at 0.395
with the base at world x = 0.52) and 5.6 cm in z:
  home  TCP (0.484, 0, 0.596)  BELOW the table top: only used away from
                               the tables, hence the back-off before it
  carry TCP (0.340, 0, 0.950)  level, retracted over the base: the held
                               object stays upright, behind the table
                               edge and >= 5 cm above the table tops
  hover TCP (0.430, 0, 0.935)  pads >= 1.7 cm above the rim: no contact
  grasp TCP (0.430, 0, 0.842)  pads inside the cup body (>= 2.4 cm above
                               the table, >= 1.0 cm below the rim), palm
                               >= 1.2 cm from the cup, 1 cm finger gap
                               per side when open
test/test_pick_place_geometry.py re-derives all of this from the URDF and
the world; tools/solve_level_waypoint.py solves new level waypoints.

Base motion
-----------
drive_to_pose() turns the base in place until its yaw is within
BASE_YAW_TOLERANCE_RAD of the target, then translates it holonomically (the
sim's VelocityControl takes vx, vy, wz), until it is within BASE_TOLERANCE_M
and the yaw tolerance (base_turning, base_command). While it translates it
only turns back for a yaw error beyond BASE_YAW_REALIGN_RAD: a /cmd_vel that
arrives late lets a turn overshoot the tolerance band, and stopping to turn
back every update made the base crawl (18 s instead of 6 s for 0.92 m with
one /odom update of latency; never at three). A last turn fixes the yaw once
the position is reached. The base is stopped when a leg ends for any reason
(Ctrl-C included) and whenever /odom is older than ODOM_STALE_SEC: it never
drives on a stale pose. The sim base skids: its
steering is locked straight, so turning in place and moving sideways drag
the wheels over the ground and nothing moves below ~0.13 rad/s or
~0.035 m/s sideways (measured; forward motion is exact). BASE_MIN_YAW_RATE
and BASE_MIN_LATERAL_SPEED_MPS are above those dead zones; a turn also drags
the base ~0.1 mm per degree, which the translation then removes. Base poses
are odom poses; odom is the world frame only because the robot spawns at the
world origin facing +x and the sim's OdometryPublisher has no noise. The
slide-ins at the tables run at BASE_APPROACH_SPEED_MPS: the open fingers
clear the object by only 8-10 mm per side.

Per-scene values (world, object, approach, waypoints, place pose and route)
live in config/pick_place_scenes.yaml; the constants here are the garden
defaults. small_house: table top 0.801, vitamin bottle r 0.027 h 0.096 at
(0.935, 0, 0.801), base at x 0.535, hover/grasp TCP z 0.962/0.871, placed
0.40 m along the same table.

Safety
------
`execute` defaults to **False** (plan-only). In plan-only mode the node
validates motion planning through MoveIt but commands NO motion, does NOT
drive the base and does not attach anything. Set `execute:=true` to
actually run it (Gazebo only).
"""

from dataclasses import dataclass
import math
import time
from typing import Callable, Optional

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from ranger_sim.srv import SetGrasp

ARM_JOINTS = [f"piper_joint{i}" for i in range(1, 7)]

Pose2D = tuple[float, float, float]  # x, y (m), yaw (rad) of the base in odom
Step = tuple[str, Callable[[], bool]]  # (caption, action); captions feed the video

# Joint configurations (rad); TCP from FK in the base_footprint frame
# (test/test_pick_place_geometry.py re-derives them from the URDF).
#   HOME  TCP ~ (0.484, 0, 0.596)  retract / stowed (BELOW the table top)
#   HOVER TCP ~ (0.430, 0, 0.935)  level gripper, pads clear of the cup rim
#   GRASP TCP ~ (0.430, 0, 0.842)  straight below HOVER; the finger pads
#                                  straddle the cup body
#   CARRY TCP ~ (0.340, 0, 0.950)  level, retracted over the base: the held
#                                  object stays upright, 7.9 cm above the
#                                  0.801 m house table, 10.8 cm above the garden's
#                                  (tools/solve_level_waypoint.py 0.20 0.95)
HOME = [0.0, 1.2, -0.2, 0.0, -0.8, 0.0]
HOVER = [0.0, 0.9, -1.413, 0.0, 0.601, 0.0]
GRASP = [0.0, 0.732, -0.943, 0.0, 0.298, 0.0]
CARRY = [0.0, 0.719, -1.496, 0.0, 0.864, 0.0]
BASE_APPROACH_X = 0.52  # world x the base drives to (garden scene)
# The base stops this far short of the approach (and of the place pose):
# the wrist camera sees the object centred, then the base slides in.
STANDOFF_M = 0.15
PLACE_BASE_POSE = [0.0, 1.07, 1.5708]  # garden: facing table_b
PLACE_ROUTE = [0.0, 0.0, 0.0, 0.0, 0.0, 1.5708]  # garden: back to the origin, turn left
# The grasp leaves ~1 cm between palm and cup: the base must stop within
# a few millimetres of the approach pose (a 3 cm band left the fingers on
# the near edge of the cup).
BASE_TOLERANCE_M = 0.005
BASE_YAW_TOLERANCE_RAD = 0.005  # 2 mm sideways at the fingers (0.4 m lever)
# While translating, turn back only beyond this yaw error (see "Base motion").
BASE_YAW_REALIGN_RAD = 3 * BASE_YAW_TOLERANCE_RAD
BASE_GAIN = 1.0  # 1/s, proportional drive gain
BASE_MIN_SPEED_MPS = 0.02  # speed floor so the last centimetres do not crawl
# Sideways and turning floors: above the sim's skid dead zones (0.035 m/s,
# 0.13 rad/s, see "Base motion"); per 50 Hz /odom update the sim base then
# moves ~1 mm / ~4 mrad, a base that does not skid 1.6 mm / 7 mrad.
BASE_MIN_LATERAL_SPEED_MPS = 0.08
BASE_SPEED_MPS = 0.25  # default speed ceiling (base_speed parameter)
BASE_APPROACH_SPEED_MPS = 0.08  # sliding the open fingers around the object
BASE_YAW_GAIN = 1.5  # 1/s
BASE_MAX_YAW_RATE = 0.5  # rad/s
BASE_MIN_YAW_RATE = 0.35  # rad/s
BASE_DRIVE_TIMEOUT_SEC = 30.0  # per leg
ODOM_WAIT_SEC = 5.0
ODOM_STALE_SEC = 0.5  # older /odom: stop the base, it never drives on a stale pose

GRIPPER_OPEN = 0.035  # joint7 travel (m): 70 mm opening, the PiPER stroke
GRIPPER_CLOSED = 0.025  # = cup radius: pads on the surface; the grasp joint holds it
GRIPPER_MAX_EFFORT = 5.0
SERVER_WAIT_SEC = 10.0
GRIPPER_RESULT_TIMEOUT_SEC = 10.0
GRASP_CALL_TIMEOUT_SEC = 5.0
# Humble: wait_for_server() can return before the goal-response reader is
# matched, so the first goal's acceptance may be lost. Give discovery a
# moment, and re-send gripper goals (idempotent position targets) once.
DISCOVERY_SETTLE_SEC = 1.0
GRIPPER_GOAL_ATTEMPTS = 2
GRIPPER_ACCEPT_TIMEOUT_SEC = 5.0
# The trajectory controller reports success when the trajectory time ends,
# while the joints may still be converging; MoveIt then rejects the next
# trajectory (start state off by > allowed_start_tolerance = 0.01 rad).
SETTLE_TIMEOUT_SEC = 3.0
SETTLE_STILL_RAD = 0.0005  # max joint change between samples = at rest
SETTLE_SAMPLE_SEC = 0.1


# ---- base pose control (pure) ----------------------------------------------


def wrap_angle(angle: float) -> float:
    """`angle` (rad) wrapped into (-pi, pi]."""
    wrapped = math.remainder(angle, 2.0 * math.pi)
    return math.pi if wrapped <= -math.pi else wrapped


def yaw_of(q) -> float:
    """Yaw (rad) of a geometry_msgs/Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_error(target: Pose2D, pose: Pose2D) -> Pose2D:
    """(dx, dy) of `target` in the body frame of `pose` and the wrapped yaw error."""
    x, y, yaw = pose
    ex, ey = target[0] - x, target[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return c * ex + s * ey, -s * ex + c * ey, wrap_angle(target[2] - yaw)


def saturate(err: float, gain: float, floor: float, ceiling: float) -> float:
    """`gain * err` with its magnitude clamped to [floor, ceiling], sign of `err`."""
    return math.copysign(max(floor, min(ceiling, gain * abs(err))), err)


def at_pose(target: Pose2D, pose: Pose2D) -> bool:
    """Whether `pose` is within the base position and yaw tolerances of `target`."""
    dx, dy, dyaw = pose_error(target, pose)
    return math.hypot(dx, dy) < BASE_TOLERANCE_M and abs(dyaw) < BASE_YAW_TOLERANCE_RAD


def base_turning(target: Pose2D, pose: Pose2D, turning: bool) -> bool:
    """
    Whether the base at `pose` should turn in place next (yaw hysteresis).

    A turn runs until the yaw is within BASE_YAW_TOLERANCE_RAD. While the base
    translates it turns back only beyond BASE_YAW_REALIGN_RAD, so a turn that
    a late /cmd_vel carried past the tolerance band does not stop it every
    update. Once the position is reached, a last turn takes the yaw into the
    tolerance.
    """
    dx, dy, dyaw = pose_error(target, pose)
    deadband = BASE_TOLERANCE_M / 2
    settled = abs(dx) < deadband and abs(dy) < deadband
    threshold = BASE_YAW_TOLERANCE_RAD if (turning or settled) else BASE_YAW_REALIGN_RAD
    return abs(dyaw) >= threshold


def base_command(target: Pose2D, pose: Pose2D, speed: float, turning: bool) -> Pose2D:
    """
    Body-frame (vx, vy, wz) that moves the base at `pose` towards `target`.

    Turning (see base_turning): in place, with a rate floor so the turn
    finishes. Otherwise translate, at most `speed` per axis, holding the yaw
    proportionally. An axis whose error is under half the position tolerance
    is left alone: no dithering at the speed floor, and once both are, the
    base is within the tolerance.
    """
    dx, dy, dyaw = pose_error(target, pose)
    if turning:
        return 0.0, 0.0, saturate(dyaw, BASE_YAW_GAIN, BASE_MIN_YAW_RATE, BASE_MAX_YAW_RATE)
    deadband = BASE_TOLERANCE_M / 2
    vx = saturate(dx, BASE_GAIN, BASE_MIN_SPEED_MPS, speed) if abs(dx) >= deadband else 0.0
    vy = (
        saturate(dy, BASE_GAIN, BASE_MIN_LATERAL_SPEED_MPS, speed) if abs(dy) >= deadband else 0.0
    )
    wz = max(-BASE_MAX_YAW_RATE, min(BASE_MAX_YAW_RATE, BASE_YAW_GAIN * dyaw))
    return vx, vy, wz


def route_triples(flat: list[float]) -> list[Pose2D]:
    """Split a flat [x0, y0, yaw0, x1, ...] list into poses; ValueError unless len % 3 == 0."""
    if len(flat) % 3:
        raise ValueError(
            f"place_route: {len(flat)} values is not a multiple of 3 (x, y, yaw per pose)"
        )
    return [tuple(float(v) for v in flat[i:i + 3]) for i in range(0, len(flat), 3)]


def standoff_pose(approach: Pose2D, standoff_m: float) -> Pose2D:
    """`approach` moved back by `standoff_m` along its own heading."""
    x, y, yaw = approach
    return x - standoff_m * math.cos(yaw), y - standoff_m * math.sin(yaw), yaw


def format_pose(pose: Pose2D) -> str:
    """Short ASCII form of a base pose for the log (and the video caption)."""
    x, y, yaw = pose
    return f"({x:.3f}, {y:.3f}, {math.degrees(yaw):.0f} deg)"


@dataclass(frozen=True)
class TransferPlan:
    """Arm waypoints and base poses of one pick -> transfer -> place run."""

    home: tuple[float, ...]
    hover: tuple[float, ...]
    grasp: tuple[float, ...]
    carry: tuple[float, ...]
    gripper_closed: float
    pick: Pose2D  # base at the pick approach: the fingers straddle the object
    pick_standoff: Pose2D
    place: Pose2D
    place_standoff: Pose2D
    route: tuple[Pose2D, ...]  # pick standoff -> place standoff, exclusive


class PickPlaceDemo(Node):
    """Scripted base pose control + MoveIt arm waypoints + /grasp/set."""

    def __init__(self, **node_kwargs) -> None:
        super().__init__("pick_place_demo", **node_kwargs)

        p = self.declare_parameter
        p("execute", False)  # False => plan only (safe)
        p("planning_group", "piper_arm")
        p("object_name", "cup")  # must be in config/graspable_objects.yaml
        p("base_approach_x", BASE_APPROACH_X)  # world x to drive base to
        p("base_speed", BASE_SPEED_MPS)  # m/s ceiling of the base legs
        p("home_joints", HOME)
        p("hover_joints", HOVER)
        p("grasp_joints", GRASP)
        p("carry_joints", CARRY)
        # Finger travel when "closed" on the attached object; object specific.
        p("gripper_closed", GRIPPER_CLOSED)
        p("standoff_m", STANDOFF_M)
        p("place_base_pose", PLACE_BASE_POSE)  # [x, y, yaw] in odom
        p("place_route", PLACE_ROUTE)  # flat [x, y, yaw, ...] in odom

        self.execute = self.get_parameter("execute").value
        self.group = self.get_parameter("planning_group").value
        self.object_name = self.get_parameter("object_name").value

        self._odom = None
        self._odom_time: Optional[float] = None  # wall clock of the last /odom
        self._joints: dict[str, float] = {}
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._joints_cb, 20)
        self._cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        self._move_group = ActionClient(self, MoveGroup, "/move_action")
        self._gripper = ActionClient(
            self, GripperCommand, "/piper_gripper_controller/gripper_cmd"
        )
        self._grasp = self.create_client(SetGrasp, "/grasp/set")

        mode = "EXECUTE" if self.execute else "PLAN-ONLY (no motion)"
        self.get_logger().info(f"PickPlaceDemo starting — mode: {mode}")

    # ---- helpers -------------------------------------------------------
    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = msg.pose.pose
        self._odom_time = time.time()

    def _joints_cb(self, msg: JointState) -> None:
        self._joints.update(zip(msg.name, msg.position))

    def _arm_positions(self) -> list[float] | None:
        if not all(name in self._joints for name in ARM_JOINTS):
            return None
        return [self._joints[name] for name in ARM_JOINTS]

    def _wait_until_settled(self, label: str) -> None:
        """Block until the arm joints stop moving (or the timeout ends)."""
        deadline = time.time() + SETTLE_TIMEOUT_SEC
        previous = None
        while rclpy.ok() and time.time() < deadline:
            sample_end = time.time() + SETTLE_SAMPLE_SEC
            while time.time() < sample_end:
                rclpy.spin_once(self, timeout_sec=0.02)
            current = self._arm_positions()
            if current is not None and previous is not None:
                if (
                    max(abs(a - b) for a, b in zip(current, previous))
                    < SETTLE_STILL_RAD
                ):
                    return
            previous = current
        self.get_logger().warn(
            f"{label}: arm still moving after {SETTLE_TIMEOUT_SEC:.0f} s"
        )

    def _wait_servers(self) -> None:
        self.get_logger().info("Waiting for /move_action ...")
        if not self._move_group.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("MoveGroup server /move_action not available")
        gripper_ok = self._gripper.wait_for_server(timeout_sec=SERVER_WAIT_SEC)
        grasp_ok = self._grasp.wait_for_service(timeout_sec=SERVER_WAIT_SEC)
        if self.execute and not gripper_ok:
            raise RuntimeError(
                "GripperCommand server /piper_gripper_controller/gripper_cmd not available"
            )
        if self.execute and not grasp_ok:
            raise RuntimeError("grasp service /grasp/set not available (grasp_manager)")
        if not (gripper_ok and grasp_ok):
            self.get_logger().warn(
                "gripper action or /grasp/set missing — fine for plan-only"
            )
        settle_end = time.time() + DISCOVERY_SETTLE_SEC
        while time.time() < settle_end:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ---- base motion ---------------------------------------------------
    def _odom_pose(self) -> Optional[Pose2D]:
        if self._odom is None:
            return None
        position = self._odom.position
        return position.x, position.y, yaw_of(self._odom.orientation)

    def _wait_for_odom_pose(self) -> Optional[Pose2D]:
        """Return the current odom pose, waiting briefly for the first message."""
        deadline = time.time() + ODOM_WAIT_SEC
        while rclpy.ok() and self._odom is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._odom_pose()

    def drive_to_pose(self, target: Pose2D, label: str, speed: Optional[float] = None) -> bool:
        """Drive the base to the odom pose `target`: turn in place, then translate."""
        if not self.execute:
            self.get_logger().info(
                f"[plan-only] {label}: would drive to {format_pose(target)} (skipped)"
            )
            return True
        if speed is None:
            speed = float(self.get_parameter("base_speed").value)
        self.get_logger().info(f"{label} -> {format_pose(target)}")
        try:
            reached = self._drive_leg(target, speed)
        finally:
            self.stop_base()  # also on Ctrl-C or an error mid-leg
        if not reached:
            self.get_logger().error(f"{label}: base did not reach {format_pose(target)}")
            return False
        self.get_logger().info(f"{label}: base at {format_pose(self._odom_pose())}")
        return True

    def _drive_leg(self, target: Pose2D, speed: float) -> bool:
        """Command /cmd_vel until the base is at `target`; False on timeout."""
        deadline = time.time() + BASE_DRIVE_TIMEOUT_SEC
        turning = stale = False
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            pose = self._odom_pose()
            if pose is None:
                continue
            if not self._odom_fresh():
                self.stop_base()
                if not stale:
                    self.get_logger().warn(f"/odom older than {ODOM_STALE_SEC} s: base stopped")
                stale = True
                continue
            stale = False
            if at_pose(target, pose):
                return True
            turning = base_turning(target, pose, turning)
            tw = Twist()
            tw.linear.x, tw.linear.y, tw.angular.z = base_command(target, pose, speed, turning)
            self._cmd_vel.publish(tw)
        return False

    def _odom_fresh(self) -> bool:
        return self._odom_time is not None and time.time() - self._odom_time <= ODOM_STALE_SEC

    def stop_base(self) -> None:
        """Publish a zero /cmd_vel (the sim's VelocityControl keeps the last one)."""
        self._cmd_vel.publish(Twist())

    # ---- arm motion ----------------------------------------------------
    def move_to_joint_state(self, label: str, positions: list[float]) -> bool:
        self.get_logger().info(f"Arm -> {label}")
        c = Constraints()
        for jn, val in zip(ARM_JOINTS, positions):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(val)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.group
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        req.goal_constraints.append(c)
        goal.planning_options.plan_only = not self.execute
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        fut = self._move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f"{label}: goal rejected")
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        code = rf.result().result.error_code.val
        ok = code == 1  # moveit_msgs/MoveItErrorCodes.SUCCESS
        if ok and self.execute:
            self._wait_until_settled(label)
        self.get_logger().info(
            f"{label}: {'executed' if self.execute else 'planned'} -> "
            f"{'SUCCESS' if ok else f'FAIL (code {code})'}"
        )
        return ok

    # ---- gripper -------------------------------------------------------
    def set_gripper(self, position: float, label: str) -> bool:
        if not self.execute:
            self.get_logger().info(f"[plan-only] would set gripper '{label}' (skipped)")
            return True
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = GRIPPER_MAX_EFFORT
        gh = None
        for attempt in range(1, GRIPPER_GOAL_ATTEMPTS + 1):
            fut = self._gripper.send_goal_async(goal)
            rclpy.spin_until_future_complete(
                self, fut, timeout_sec=GRIPPER_ACCEPT_TIMEOUT_SEC
            )
            gh = fut.result()
            if gh is not None:
                break
            self.get_logger().warn(
                f"gripper '{label}': no goal response (attempt {attempt}), re-sending"
            )
        if gh is None or not gh.accepted:
            self.get_logger().error(f"gripper '{label}': goal rejected or timed out")
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(
            self, rf, timeout_sec=GRIPPER_RESULT_TIMEOUT_SEC
        )
        wrapped = rf.result()
        # allow_stalling (ros2_controllers_sim.yaml) reports fingers that
        # stopped on an object as SUCCEEDED.
        ok = wrapped is not None and wrapped.status == GoalStatus.STATUS_SUCCEEDED
        if ok:
            self.get_logger().info(f"gripper -> {label}")
        else:
            self.get_logger().error(f"gripper '{label}' did not succeed")
        return ok

    # ---- grasp (DetachableJoint via /grasp/set) ------------------------
    def grasp(self, attach: bool) -> bool:
        verb = "attach" if attach else "release"
        if not self.execute:
            self.get_logger().info(
                f"[plan-only] would {verb} '{self.object_name}' (skipped)"
            )
            return True
        request = SetGrasp.Request()
        request.object = self.object_name
        request.attach = attach
        fut = self._grasp.call_async(request)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=GRASP_CALL_TIMEOUT_SEC)
        response = fut.result()
        if response is None:
            self.get_logger().error(f"/grasp/set {verb}: no response")
            return False
        if not response.success:
            self.get_logger().error(f"/grasp/set {verb}: {response.message}")
            return False
        self.get_logger().info(
            f"grasp -> {verb} '{self.object_name}' ({response.message})"
        )
        return True

    # ---- orchestration -------------------------------------------------
    def transfer_plan(self) -> TransferPlan:
        """Return the run's waypoints from the parameters; ValueError on a bad pose or route."""

        def value(name: str):
            return self.get_parameter(name).value

        place = tuple(float(v) for v in value("place_base_pose"))
        if len(place) != 3:
            raise ValueError(f"place_base_pose must be [x, y, yaw], got {list(place)}")
        standoff = float(value("standoff_m"))
        pick = (float(value("base_approach_x")), 0.0, 0.0)
        return TransferPlan(
            home=tuple(value("home_joints")),
            hover=tuple(value("hover_joints")),
            # Not `grasp`: that is the attach/release method.
            grasp=tuple(value("grasp_joints")),
            carry=tuple(value("carry_joints")),
            gripper_closed=float(value("gripper_closed")),
            pick=pick,
            pick_standoff=standoff_pose(pick, standoff),
            place=place,
            place_standoff=standoff_pose(place, standoff),
            route=tuple(route_triples(list(value("place_route")))),
        )

    def _arm_step(self, caption: str, joints: tuple[float, ...]) -> Step:
        return caption, lambda: self.move_to_joint_state(caption, joints)

    def _drive_step(self, caption: str, pose: Pose2D, speed: Optional[float] = None) -> Step:
        return caption, lambda: self.drive_to_pose(pose, caption, speed)

    def _pick_steps(self, plan: TransferPlan) -> list[Step]:
        return [
            self._drive_step("drive to the table", plan.pick_standoff),
            # Level gripper at grasp height, base short of the object: the
            # wrist camera has it centred (FK: ~4 deg off its optical axis).
            self._arm_step("look at the object", plan.grasp),
            # Slide the open fingers around the object (8-10 mm per side).
            self._drive_step("approach the object", plan.pick, BASE_APPROACH_SPEED_MPS),
            # Attach before closing so the fixed joint is never created
            # while the fingers interpenetrate the object.
            ("attach object", lambda: self.grasp(True)),
            ("close gripper", lambda: self.set_gripper(plan.gripper_closed, "close")),
            self._arm_step("lift", plan.hover),
            # Upright and behind the table edge before the base moves.
            self._arm_step("carry pose", plan.carry),
        ]

    def _place_steps(self, plan: TransferPlan) -> list[Step]:
        return [
            self._drive_step("drive to the place", plan.place_standoff),
            # The held object 9 cm above the table top, then over it.
            self._arm_step("hover above the place", plan.hover),
            self._drive_step("approach the place", plan.place, BASE_APPROACH_SPEED_MPS),
            # Straight down: GRASP sets the object on a table as high as the pick's.
            self._arm_step("place", plan.grasp),
            ("open gripper", lambda: self.set_gripper(GRIPPER_OPEN, "open")),
            ("release object", lambda: self.grasp(False)),
            self._arm_step("clear (hover)", plan.hover),
            # Open fingers above the rim: the base can back off under them.
            self._drive_step("back off", plan.place_standoff, BASE_APPROACH_SPEED_MPS),
            self._arm_step("carry pose", plan.carry),
        ]

    def sequence(self, plan: TransferPlan, start: Optional[Pose2D]) -> list[Step]:
        """Every step of the run in order; `start` is where the base returns (None: stay)."""
        return [
            # Known start state: object released, gripper open.
            ("release object", lambda: self.grasp(False)),
            ("open gripper", lambda: self.set_gripper(GRIPPER_OPEN, "open")),
            self._arm_step("'home'", plan.home),
            # Level and retracted BEFORE the base moves: at 'home' the
            # gripper would end up under the table top and jam against it.
            self._arm_step("carry pose", plan.carry),
            *self._pick_steps(plan),
            *(self._drive_step("drive to the place", pose) for pose in plan.route),
            *self._place_steps(plan),
            *(self._drive_step("return", pose) for pose in reversed(plan.route)),
            # Plan-only without a sim has no start pose to return to.
            ("return to start",
             lambda: start is None or self.drive_to_pose(start, "return to start")),
            # 'home' is below the table top: only away from the tables.
            self._arm_step("retract 'home'", plan.home),
        ]

    def run(self) -> None:
        plan = self.transfer_plan()  # malformed scene values fail before any wait
        self._wait_servers()
        start = self._wait_for_odom_pose()
        if self.execute and start is None:
            raise RuntimeError("no /odom received: cannot drive the base")

        for name, action in self.sequence(plan, start):
            if not action():
                self.get_logger().error(f"step '{name}' failed — aborting")
                return

        self.get_logger().info(
            "Pick-and-place sequence complete "
            f"({'executed' if self.execute else 'plan-only'})."
        )


def main() -> None:
    # Ctrl-C as a plain KeyboardInterrupt: rclpy's own SIGINT handler would
    # shut the context down first, and the stop below could not be sent.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PickPlaceDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted")
    except Exception as exc:  # noqa: BLE001 - demo: surface any failure clearly
        node.get_logger().error(f"demo failed: {exc}")
    finally:
        if node.execute:  # plan-only never touches /cmd_vel
            node.stop_base()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
