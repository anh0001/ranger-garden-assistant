#!/usr/bin/env python3
"""
Pick-and-place demo for the Ranger Mini 3.0 + PiPER arm in Gazebo Fortress.

Pipeline
--------
  1. Release any held object, open the gripper, arm to 'home' and then to
     the high 'lift' carry pose -- BEFORE the base approaches the table.
  2. Drive the base to the approach pose (open-loop proportional control on
     /odom).
  3. Lower the arm to the pre-grasp hover pose just above the cup (joint
     space, via the MoveIt MoveGroup action -> ranger_piper_moveit).
  4. Attach the cup (/grasp/set -> DetachableJoint), then close the gripper.
  5. Lift, place it back (hover pose again), open, release, clear upward.
  6. Back the base away to where it started, then retract to 'home'.

Grasp
-----
The cup is held by a simulator-native fixed joint (grasp_manager.py serves
/grasp/set on top of Gazebo's DetachableJoint), not by finger friction. It
is attached BEFORE the fingers close so the physics engine never creates
the joint while the fingers interpenetrate the cup. The gripper is driven
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

Geometry (TCP in base_footprint frame; garden_world table top z = 0.77,
front edge at world x = 0.90; cup centre (0.915, 0, 0.82), top z = 0.87):
  home      -> (0.484, 0, 0.596)  BELOW the table top: only used away from
                                  the table, hence the back-off before it
  lift      -> (0.347, 0, 1.111)  carry pose, clear above the table
  pre-grasp -> (0.438, 0, 0.920)  hover: fingers ~5 cm above the cup top, so
                                  attach/release happen without contact
With base_approach_x = 0.52 the hovering gripper sits over the cup. The
attach keeps the cup's current pose relative to the gripper, so placing
back at the same hover pose returns the cup to its original spot. Tune
`*_joints` / `base_approach_x` together if you move the cup.

Safety
------
`execute` defaults to **False** (plan-only). In plan-only mode the node
validates motion planning through MoveIt but commands NO motion, does NOT
drive the base and does not attach anything. Set `execute:=true` to
actually run it (Gazebo only).
"""

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

# Joint configurations (rad); TCP from MoveIt FK / IK (base_footprint frame).
#   HOME      TCP ~ (0.484, 0, 0.596)  retract / stowed (away from the table)
#   PRE_GRASP TCP ~ (0.438, 0, 0.920)  hover above the cup (attach/release)
#   LIFT      TCP ~ (0.347, 0, 1.111)  carry pose, cup clear of the table
HOME = [0.0, 1.2, -0.2, 0.0, -0.8, 0.0]
PRE_GRASP = [0.0, 0.854, -1.23, 0.0, 0.376, 0.0]
LIFT = [0.0, 0.9, -1.4, 0.0, -0.3, 0.0]
BASE_TOLERANCE_M = 0.03
BASE_DRIVE_TIMEOUT_SEC = 30.0
ODOM_WAIT_SEC = 5.0

GRIPPER_OPEN = 0.035  # joint7 travel (m)
GRIPPER_CLOSED = 0.002  # cosmetic close: the cup is held by the grasp joint
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
        p("pre_grasp_joints", PRE_GRASP)
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
            tw.linear.x = max(-speed, min(speed, 1.0 * err))
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
        pre = list(self.get_parameter("pre_grasp_joints").value)
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
            ("pre-grasp (hover)", lambda: self.move_to_joint_state("pre-grasp", pre)),
            # Attach before closing so the fixed joint is never created
            # while the fingers interpenetrate the cup.
            ("attach cup", lambda: self.grasp(True)),
            ("close gripper", lambda: self.set_gripper(closed, "close")),
            ("lift", lambda: self.move_to_joint_state("lift", lift)),
            # Place: back to the hover pose, where the cup rests on the
            # table again, then open and release.
            ("place", lambda: self.move_to_joint_state("place", pre)),
            ("open gripper", lambda: self.set_gripper(GRIPPER_OPEN, "open")),
            ("release cup", lambda: self.grasp(False)),
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
