#!/usr/bin/env python3
"""
Pick-and-place demo for the Ranger Mini 3.0 + PiPER arm in Gazebo Fortress.

Pipeline
--------
  1. Release any held object, open the gripper, arm to 'home' and then to
     the high 'lift' carry pose -- BEFORE the base approaches the table.
  2. Drive the base to the approach pose (proportional control on /odom).
  3. Lower the level gripper to the hover pose above the cup, then descend
     straight down so the finger pads straddle the cup body (joint space,
     via the MoveIt MoveGroup action -> ranger_piper_moveit).
  4. Attach the cup (/grasp/set -> DetachableJoint), close on the body.
  5. Raise, lift, come back down the same way to set it on its spot, open,
     release, clear upward.
  6. Back the base away to where it started, then retract to 'home'.

Grasp
-----
The cup is held by a simulator-native fixed joint (grasp_manager.py serves
/grasp/set on top of Gazebo's DetachableJoint), not by finger friction. It
is attached BEFORE the fingers close so the physics engine never creates
the joint while the fingers interpenetrate the cup; the fingers then close
to the cup radius, i.e. the pads rest on its surface. The gripper is driven
through control_msgs/GripperCommand on /piper_gripper_controller/gripper_cmd,
the same interface MoveIt and the real PiPER bridge use.

Why joint-space waypoints
-------------------------
The PiPER is a small arm on a low mobile base. Near the edge of its
workspace (a cup on a 0.82 m table) Cartesian pose goals frequently have
no IK solution and OMPL fails ("Unable to sample valid states for goal
tree"). The waypoints below were chosen from MoveIt FK so every TCP pose
is verified reachable; the cup is placed in the world to match the grasp
TCP. This is the robust, deterministic approach for a scripted sim demo.

Geometry (base_footprint frame; garden_world table top z = 0.77, front
edge at world x = 0.90; cup r = 0.025, h = 0.10, centre (0.915, 0, 0.82),
rim z = 0.87). Gazebo rests base_footprint 1 cm below the ground (the
wheel geometry says 2 cm; the waypoints are checked for both), so world
z = frame z - 0.01..0.02. HOVER and GRASP keep the gripper level
(approach +x, fingers closing along y) at the same link6 x = 0.290; the
finger pads then span x 0.349..0.426 (cup centre at 0.395 with the base
at world x = 0.52) and 5.6 cm in z:
  home  TCP (0.484, 0, 0.596)  BELOW the table top: only used away from
                               the table, hence the back-off before it
  lift  TCP (0.347, 0, 1.111)  carry pose, clear above the table
  hover TCP (0.430, 0, 0.935)  pads >= 1.7 cm above the rim: no contact
  grasp TCP (0.430, 0, 0.842)  pads inside the cup body (>= 2.4 cm above
                               the table, >= 1.0 cm below the rim), palm
                               >= 1.2 cm from the cup, 1 cm finger gap
                               per side when open
The attach keeps the cup's current pose relative to the gripper, so
going back down to the grasp pose sets the cup on its original spot.
test/test_pick_place_geometry.py re-derives all of this from the URDF and
the world; re-solve `*_joints` / `base_approach_x` there if you move the
cup.

Safety
------
`execute` defaults to **False** (plan-only). In plan-only mode the node
validates motion planning through MoveIt but commands NO motion, does NOT
drive the base and does not attach anything. Set `execute:=true` to
actually run it (Gazebo only).
"""

import math
import time
from typing import Callable

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
from ranger_sim.srv import SetGrasp

ARM_JOINTS = [f"piper_joint{i}" for i in range(1, 7)]

# Joint configurations (rad); TCP from FK in the base_footprint frame
# (test/test_pick_place_geometry.py re-derives them from the URDF).
#   HOME  TCP ~ (0.484, 0, 0.596)  retract / stowed (BELOW the table top)
#   HOVER TCP ~ (0.430, 0, 0.935)  level gripper, pads clear of the cup rim
#   GRASP TCP ~ (0.430, 0, 0.842)  straight below HOVER; the finger pads
#                                  straddle the cup body
#   LIFT  TCP ~ (0.347, 0, 1.111)  carry pose, cup clear of the table
HOME = [0.0, 1.2, -0.2, 0.0, -0.8, 0.0]
HOVER = [0.0, 0.9, -1.413, 0.0, 0.601, 0.0]
GRASP = [0.0, 0.732, -0.943, 0.0, 0.298, 0.0]
LIFT = [0.0, 0.9, -1.4, 0.0, -0.3, 0.0]
# The grasp leaves ~1 cm between palm and cup: the base must stop within
# a few millimetres of the approach pose (a 3 cm band left the fingers on
# the near edge of the cup).
BASE_TOLERANCE_M = 0.005
BASE_GAIN = 1.0  # 1/s, proportional drive gain
BASE_MIN_SPEED_MPS = 0.02  # speed floor so the last centimetres do not crawl
BASE_DRIVE_TIMEOUT_SEC = 30.0
ODOM_WAIT_SEC = 5.0

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


class PickPlaceDemo(Node):
    """Scripted base approach + MoveIt arm waypoints + /grasp/set."""

    def __init__(self) -> None:
        super().__init__("pick_place_demo")

        p = self.declare_parameter
        p("execute", False)  # False => plan only (safe)
        p("planning_group", "piper_arm")
        p("object_name", "cup")  # must be in config/graspable_objects.yaml
        p("base_approach_x", 0.52)  # world x to drive base to
        p("base_speed", 0.25)  # m/s open-loop drive
        p("home_joints", HOME)
        p("hover_joints", HOVER)
        p("grasp_joints", GRASP)
        p("lift_joints", LIFT)
        # Finger travel when "closed" on the attached object; object specific.
        p("gripper_closed", GRIPPER_CLOSED)

        self.execute = self.get_parameter("execute").value
        self.group = self.get_parameter("planning_group").value
        self.object_name = self.get_parameter("object_name").value

        self._odom = None
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
    def _wait_for_odom(self) -> float | None:
        """Return the current odom x, waiting briefly for the first message."""
        deadline = time.time() + ODOM_WAIT_SEC
        while rclpy.ok() and self._odom is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return None if self._odom is None else self._odom.position.x

    def drive_base_to(self, world_x: float) -> bool:
        if not self.execute:
            self.get_logger().info(
                f"[plan-only] would drive base to world x={world_x:.2f} (skipped)"
            )
            return True
        self.get_logger().info(f"Driving base to world x={world_x:.2f} ...")
        deadline = time.time() + BASE_DRIVE_TIMEOUT_SEC
        speed = float(self.get_parameter("base_speed").value)
        reached = False
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._odom is None:
                continue
            err = world_x - self._odom.position.x
            if abs(err) < BASE_TOLERANCE_M:
                reached = True
                break
            tw = Twist()
            tw.linear.x = math.copysign(
                max(BASE_MIN_SPEED_MPS, min(speed, BASE_GAIN * abs(err))), err
            )
            self._cmd_vel.publish(tw)
        self._cmd_vel.publish(Twist())  # stop
        if not reached:
            self.get_logger().error(f"base did not reach world x={world_x:.2f}")
            return False
        self.get_logger().info("Base drive done.")
        return True

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
    def run(self) -> None:
        self._wait_servers()
        home = list(self.get_parameter("home_joints").value)
        hover = list(self.get_parameter("hover_joints").value)
        # Not `grasp`: that is the attach/release method.
        grasp_pose = list(self.get_parameter("grasp_joints").value)
        lift = list(self.get_parameter("lift_joints").value)
        closed = float(self.get_parameter("gripper_closed").value)

        approach_x = float(self.get_parameter("base_approach_x").value)
        start_x = self._wait_for_odom()
        if self.execute and start_x is None:
            raise RuntimeError("no /odom received: cannot drive the base")

        steps: list[tuple[str, Callable[[], bool]]] = [
            # Known start state: object released, gripper open.
            ("release object", lambda: self.grasp(False)),
            ("open gripper", lambda: self.set_gripper(GRIPPER_OPEN, "open")),
            ("'home'", lambda: self.move_to_joint_state("'home'", home)),
            # Raise the arm BEFORE approaching: at 'home' the gripper would
            # end up under the table top and jam against it.
            ("carry pose", lambda: self.move_to_joint_state("lift", lift)),
            ("approach table", lambda: self.drive_base_to(approach_x)),
            ("hover above cup", lambda: self.move_to_joint_state("hover", hover)),
            # Straight descent: HOVER and GRASP share link6 x and a level
            # approach, so the joint-space path stays vertical.
            ("descend to grasp", lambda: self.move_to_joint_state("grasp", grasp_pose)),
            # Attach before closing so the fixed joint is never created
            # while the fingers interpenetrate the cup (1 cm clearance per
            # side when open).
            ("attach cup", lambda: self.grasp(True)),
            ("close gripper", lambda: self.set_gripper(closed, "close")),
            ("raise", lambda: self.move_to_joint_state("hover", hover)),
            ("lift", lambda: self.move_to_joint_state("lift", lift)),
            # Place: back down the same way; the attach kept the cup's pose
            # relative to the wrist, so GRASP sets it on its original spot.
            ("hover above table", lambda: self.move_to_joint_state("hover", hover)),
            ("place", lambda: self.move_to_joint_state("place", grasp_pose)),
            ("open gripper", lambda: self.set_gripper(GRIPPER_OPEN, "open")),
            ("release cup", lambda: self.grasp(False)),
            ("clear (hover)", lambda: self.move_to_joint_state("hover", hover)),
            ("clear (lift)", lambda: self.move_to_joint_state("lift", lift)),
            # 'home' is below the table top: back away before retracting.
            ("back away", lambda: start_x is None or self.drive_base_to(start_x)),
            ("retract 'home'", lambda: self.move_to_joint_state("'home'", home)),
        ]
        for name, action in steps:
            if not action():
                self.get_logger().error(f"step '{name}' failed — aborting")
                return

        self.get_logger().info(
            "Pick-and-place sequence complete "
            f"({'executed' if self.execute else 'plan-only'})."
        )


def main() -> None:
    rclpy.init()
    node = PickPlaceDemo()
    try:
        node.run()
    except Exception as exc:  # noqa: BLE001 - demo: surface any failure clearly
        node.get_logger().error(f"demo failed: {exc}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
