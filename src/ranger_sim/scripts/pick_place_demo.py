#!/usr/bin/env python3
"""
Pick-and-place demo for the Ranger Mini 3.0 + PiPER arm in Gazebo Fortress.

Pipeline
--------
  1. (optional) Drive the base forward to an approach pose near the table,
     using open-loop proportional control on /odom.
  2. Move the arm to 'home' and open the gripper.
  3. Move to a pre-grasp configuration above the cup, then down to the
     grasp configuration (joint-space, via the MoveIt MoveGroup action ->
     ranger_piper_moveit).
  4. Close the gripper.
  5. Lift back to pre-grasp, then return the arm to 'home'.

Why joint-space waypoints
-------------------------
The PiPER is a small arm on a low mobile base. Near the edge of its
workspace (a cup on a 0.82 m table) Cartesian pose goals frequently have
no IK solution and OMPL fails ("Unable to sample valid states for goal
tree"). The waypoints below were chosen from MoveIt FK so every TCP pose
is verified reachable; the cup is placed in the world to match the grasp
TCP. This is the robust, deterministic approach for a scripted sim demo.

Reachability geometry (TCP in base_footprint frame):
  home      -> (0.484, 0, 0.596)
  pre-grasp -> (0.438, 0, 0.851)
  grasp     -> (0.395, 0, 0.800)
With base_approach_x = 0.52 the cup at world (0.915, 0, 0.82) lands under
the grasp TCP. Tune `*_joints` / `base_approach_x` together if you move
the cup in worlds/garden_world.sdf.

Safety
------
`execute` defaults to **False** (plan-only). In plan-only mode the node
validates motion planning through MoveIt but commands NO motion and does
NOT drive the base. Set `execute:=true` to actually run it (Gazebo only).
"""

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from builtin_interfaces.msg import Duration as MsgDuration

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [f"piper_joint{i}" for i in range(1, 7)]

# Verified-reachable joint configurations (rad), from MoveIt FK.
#   HOME      TCP ~ (0.484, 0, 0.596)  retract / safe
#   PRE_GRASP TCP ~ (0.438, 0, 0.851)  above the cup, no contact (attach here)
#   LIFT      TCP ~ (0.347, 0, 1.111)  cup raised clear of the table
HOME = [0.0, 1.2, -0.2, 0.0, -0.8, 0.0]
PRE_GRASP = [0.0, 0.75, -0.9, 0.0, 0.15, 0.0]
LIFT = [0.0, 0.9, -1.4, 0.0, -0.3, 0.0]

GRIPPER_OPEN = 0.035   # joint7 travel (m)
GRIPPER_CLOSED = 0.0


class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__("pick_place_demo")

        p = self.declare_parameter
        p("execute", False)                   # False => plan only (safe)
        p("planning_group", "piper_arm")
        p("base_approach_x", 0.52)            # world x to drive base to
        p("base_speed", 0.25)                 # m/s open-loop drive
        p("home_joints", HOME)
        p("pre_grasp_joints", PRE_GRASP)
        p("lift_joints", LIFT)

        self.execute = self.get_parameter("execute").value
        self.group = self.get_parameter("planning_group").value

        self._odom = None
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self._cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        # Pose-follow virtual attach (grasp_magnet node listens here).
        self._grasp = self.create_publisher(Bool, "/gripper/grasp", 10)

        self._move_group = ActionClient(self, MoveGroup, "/move_action")
        self._gripper = ActionClient(
            self, FollowJointTrajectory,
            "/piper_gripper_controller/follow_joint_trajectory",
        )

        mode = "EXECUTE" if self.execute else "PLAN-ONLY (no motion)"
        self.get_logger().info(f"PickPlaceDemo starting — mode: {mode}")

    # ---- helpers -------------------------------------------------------
    def _odom_cb(self, msg):
        self._odom = msg.pose.pose

    def _wait_servers(self):
        self.get_logger().info("Waiting for /move_action ...")
        if not self._move_group.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("MoveGroup server /move_action not available")
        if not self._gripper.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn(
                "gripper action server not available — gripper steps skipped"
            )

    # ---- base motion ---------------------------------------------------
    def drive_base_to(self, world_x):
        if not self.execute:
            self.get_logger().info(
                f"[plan-only] would drive base to world x={world_x:.2f} (skipped)"
            )
            return
        self.get_logger().info(f"Driving base to world x={world_x:.2f} ...")
        deadline = time.time() + 30.0
        speed = float(self.get_parameter("base_speed").value)
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._odom is None:
                continue
            err = world_x - self._odom.position.x
            if abs(err) < 0.03:
                break
            tw = Twist()
            tw.linear.x = max(-speed, min(speed, 1.0 * err))
            self._cmd_vel.publish(tw)
        self._cmd_vel.publish(Twist())  # stop
        self.get_logger().info("Base approach done.")

    # ---- arm motion ----------------------------------------------------
    def move_to_joint_state(self, label, positions):
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
        self.get_logger().info(
            f"{label}: {'executed' if self.execute else 'planned'} -> "
            f"{'SUCCESS' if ok else f'FAIL (code {code})'}"
        )
        return ok

    # ---- gripper -------------------------------------------------------
    def set_gripper(self, position, label):
        if not self._gripper.server_is_ready():
            self.get_logger().warn(f"gripper not ready — skip '{label}'")
            return
        if not self.execute:
            self.get_logger().info(f"[plan-only] would set gripper '{label}' (skipped)")
            return
        traj = JointTrajectory()
        traj.joint_names = ["piper_joint7"]
        pt = JointTrajectoryPoint()
        pt.positions = [position]
        pt.time_from_start = MsgDuration(sec=1)
        traj.points.append(pt)
        g = FollowJointTrajectory.Goal()
        g.trajectory = traj
        fut = self._gripper.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh and gh.accepted:
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(self, rf)
        self.get_logger().info(f"gripper -> {label}")

    # ---- grasp-fix (pose-follow virtual attach) ------------------------
    def grasp_fix(self, hold, label):
        """Tell grasp_magnet to hold (True) or release (False) the cup."""
        if not self.execute:
            self.get_logger().info(f"[plan-only] would {label} cup (skipped)")
            return
        t0 = time.time()
        while (self._grasp.get_subscription_count() == 0
               and time.time() - t0 < 5.0):
            rclpy.spin_once(self, timeout_sec=0.1)
        msg = Bool()
        msg.data = hold
        for _ in range(5):  # ensure delivery
            self._grasp.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        self.get_logger().info(f"grasp-fix -> {label}")

    # ---- orchestration -------------------------------------------------
    def run(self):
        self._wait_servers()
        home = list(self.get_parameter("home_joints").value)
        pre = list(self.get_parameter("pre_grasp_joints").value)
        lift = list(self.get_parameter("lift_joints").value)

        # Ensure the magnet starts in the released state, gripper open.
        self.set_gripper(GRIPPER_OPEN, "open")
        self.grasp_fix(False, "release")

        self.drive_base_to(float(self.get_parameter("base_approach_x").value))

        steps = [
            ("'home'", lambda: self.move_to_joint_state("'home'", home)),
            ("pre-grasp (above cup)",
             lambda: self.move_to_joint_state("pre-grasp", pre)),
            ("close gripper",
             lambda: self.set_gripper(GRIPPER_CLOSED, "close") or True),
            # Magnet attaches the cup to the gripper link and follows it.
            ("grasp cup", lambda: self.grasp_fix(True, "hold") or True),
            ("lift", lambda: self.move_to_joint_state("lift", lift)),
            # Place: carry back over the table, set down, release so the
            # cup rests still (the magnet stops following on release).
            ("place (over table)",
             lambda: self.move_to_joint_state("place", pre)),
            ("open gripper",
             lambda: self.set_gripper(GRIPPER_OPEN, "open") or True),
            ("release cup", lambda: self.grasp_fix(False, "release") or True),
            # Retract upward first so the arm clears the just-placed cup
            # instead of sweeping through it on the way to 'home'.
            ("clear (lift)", lambda: self.move_to_joint_state("lift", lift)),
            ("retract 'home'", lambda: self.move_to_joint_state("'home'", home)),
        ]
        for name, action in steps:
            if action() is False:
                self.get_logger().error(f"step '{name}' failed — aborting")
                return

        self.get_logger().info(
            "Pick-and-place sequence complete "
            f"({'executed' if self.execute else 'plan-only'})."
        )


def main():
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
