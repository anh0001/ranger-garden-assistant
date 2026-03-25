#!/usr/bin/env python3
"""
PiPER MoveIt Servo Commander (Twist-based jog + gripper demo)

This script publishes TwistStamped commands to MoveIt Servo for quick, manual
validation of realtime Cartesian control on the PiPER arm. It also uses
MoveIt's MoveGroup action for discrete ready-pose and gripper open/close
commands.

Usage:
    # Terminal 1: Launch MoveIt + Servo
    ros2 launch robofi_bringup ranger_complete_bringup.launch.py

    # Terminal 2: Run this script
    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 scripts/piper_servo_commander.py

Design notes / debugging history
---------------------------------
1. Singularity thresholds must be raised in moveit_servo.yaml
   The PiPER arm has short link lengths (0.28 m, 0.25 m, 0.09 m), so the
   Jacobian condition number is naturally ~33 even at well-away-from-singular
   configurations.  MoveIt Servo's defaults (lower=17, hard_stop=30) fire
   HALT_FOR_SINGULARITY (status=2) at virtually every reachable pose.
   Fix: lower_singularity_threshold: 100, hard_stop_singularity_threshold: 200
   in src/ranger_piper_moveit/config/moveit_servo.yaml.
   NOTE: these thresholds are loaded once at node startup; ros2 param set has
   no effect — restart the servo node (or the whole bringup) after changing them.

2. Arm must NOT be at the all-zeros (home) pose when servo starts
   All joints = 0 is a true kinematic singularity (condition number → ∞).
   The script calls go_to_ready_pose() via the MoveGroup action and immediately
   calls start_servo() without any user-input delay in between, so the arm
   cannot drift back to zeros before servo is armed.

3. Executor pattern for sequential async ROS calls
   rclpy.spin_until_future_complete(node, future) creates a temporary executor
   internally and can silently fail when called multiple times on the same node
   (e.g. after go_to_ready_pose already used it for send_goal + get_result).
   Fix: a single SingleThreadedExecutor is created in __init__ and reused for
   all spin_until_future_complete / spin_once calls throughout the node's life.

4. start_servo now waits for service readiness (teleop-style)
   start_servo uses a Trigger client and checks wait_for_service() before
   sending the request, matching piper_servo_teleop.py behavior so startup is
   not attempted before /servo_node/start_servo is available.
"""

import time
from typing import Tuple

import rclpy
import rclpy.action
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    WorkspaceParameters,
)
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger


# A non-singular "ready" pose for Cartesian jogging (radians).
# joint4 and joint5 are offset to avoid the wrist singularity
# (joint4=0 + joint5~0 makes joints 4 & 6 collinear).
READY_JOINT_POSITIONS = {
    "piper_joint1": 0.0,
    "piper_joint2": 1.2,
    "piper_joint3": -0.2,
    "piper_joint4": 0.0,
    "piper_joint5": -0.8,
    "piper_joint6": 0.0,
}


class PiperServoCommander(Node):
    def __init__(self) -> None:
        super().__init__("piper_servo_commander")

        self.declare_parameter("servo_start_service", "/servo_node/start_servo")
        self.declare_parameter("servo_twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("move_group_wait_timeout_sec", 20.0)
        self.declare_parameter("ready_pose_group_name", "piper_arm")
        self.declare_parameter("gripper_group_name", "piper_gripper")
        self.declare_parameter("gripper_joint_name", "piper_joint7")
        self.declare_parameter("gripper_open_position", 0.065)
        self.declare_parameter("gripper_closed_position", 0.0)
        self.declare_parameter("gripper_cmd_action", "/piper_gripper_controller/gripper_cmd")
        self.declare_parameter("gripper_goal_timeout_sec", 15.0)
        self.declare_parameter("command_frame", "piper_base_link")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("default_duration_sec", 2.0)
        self.declare_parameter("linear_speed", 0.05)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("start_servo_timeout_sec", 12.0)

        self._servo_start_service = self.get_parameter("servo_start_service").value
        self._servo_twist_topic = self.get_parameter("servo_twist_topic").value
        self._move_group_action = str(self.get_parameter("move_group_action").value)
        self._move_group_wait_timeout_sec = max(
            0.5, float(self.get_parameter("move_group_wait_timeout_sec").value)
        )
        self._ready_pose_group_name = str(
            self.get_parameter("ready_pose_group_name").value
        )
        self._gripper_group_name = str(self.get_parameter("gripper_group_name").value)
        self._gripper_joint_name = str(self.get_parameter("gripper_joint_name").value)
        self._gripper_open_position = float(
            self.get_parameter("gripper_open_position").value
        )
        self._gripper_closed_position = float(
            self.get_parameter("gripper_closed_position").value
        )
        self._gripper_cmd_action = str(
            self.get_parameter("gripper_cmd_action").value
        )
        self._gripper_goal_timeout_sec = max(
            1.0, float(self.get_parameter("gripper_goal_timeout_sec").value)
        )
        self._command_frame = self.get_parameter("command_frame").value
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._default_duration_sec = float(
            self.get_parameter("default_duration_sec").value
        )
        self._linear_speed = float(self.get_parameter("linear_speed").value)
        self._angular_speed = float(self.get_parameter("angular_speed").value)
        self._start_servo_timeout_sec = max(
            0.1, float(self.get_parameter("start_servo_timeout_sec").value)
        )

        if self._publish_rate_hz <= 0.0:
            self._publish_rate_hz = 30.0
        if self._default_duration_sec <= 0.0:
            self._default_duration_sec = 0.5

        # Use a dedicated executor so spin_until_future_complete works
        # reliably across multiple sequential calls.
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self)

        self._twist_pub = self.create_publisher(
            TwistStamped, self._servo_twist_topic, 10
        )
        self._start_client = self.create_client(Trigger, self._servo_start_service)
        self._move_group_client = rclpy.action.ActionClient(
            self, MoveGroup, self._move_group_action
        )
        self._gripper_cmd_client = rclpy.action.ActionClient(
            self, GripperCommand, self._gripper_cmd_action
        )
        self.get_logger().info(f"Servo twist topic: {self._servo_twist_topic}")
        self.get_logger().info(f"Servo start service: {self._servo_start_service}")
        self.get_logger().info(f"MoveGroup action: {self._move_group_action}")
        self.get_logger().info(
            f"MoveGroup wait timeout: {self._move_group_wait_timeout_sec:.1f}s"
        )
        self.get_logger().info(
            f"Ready pose group: {self._ready_pose_group_name}"
        )
        self.get_logger().info(
            f"Gripper group/joint: {self._gripper_group_name}/{self._gripper_joint_name}"
        )
        self.get_logger().info(
            "Gripper positions open="
            f"{self._gripper_open_position:.3f} closed={self._gripper_closed_position:.3f}"
        )
        self.get_logger().info(
            f"Gripper goal timeout: {self._gripper_goal_timeout_sec:.1f}s"
        )
        self.get_logger().info(f"Command frame: {self._command_frame}")

    def start_servo(self) -> bool:
        """Call /start_servo after confirming the service is available."""
        if not self._start_client.wait_for_service(
            timeout_sec=self._start_servo_timeout_sec
        ):
            self.get_logger().warn(
                f"Service {self._servo_start_service} not available "
                f"(timeout={self._start_servo_timeout_sec:.1f}s)"
            )
            return False

        self.get_logger().info("Requesting MoveIt Servo start...")
        start_future = self._start_client.call_async(Trigger.Request())
        self._executor.spin_until_future_complete(
            start_future, timeout_sec=self._start_servo_timeout_sec
        )

        if not start_future.done():
            self.get_logger().warn(
                f"Timed out waiting for start_servo response "
                f"(timeout={self._start_servo_timeout_sec:.1f}s)"
            )
            return False

        try:
            result = start_future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"start_servo call failed: {exc}")
            return False

        if result is None:
            self.get_logger().warn("start_servo returned no response")
            return False

        if result.success:
            self.get_logger().info(f"Servo started: {result.message}")
            return True

        self.get_logger().warn(f"Servo start rejected: {result.message}")
        return False

    def _wait_for_move_group_server(self, timeout_sec: float | None = None) -> bool:
        timeout = (
            self._move_group_wait_timeout_sec
            if timeout_sec is None
            else max(0.1, timeout_sec)
        )
        if self._move_group_client.wait_for_server(timeout_sec=timeout):
            return True

        self.get_logger().error(
            f"MoveGroup action server {self._move_group_action} not available "
            f"(timeout={timeout:.1f}s)"
        )
        return False

    def _execute_joint_goal(
        self,
        joint_targets: dict[str, float],
        group_name: str,
        timeout_sec: float,
    ) -> bool:
        if not self._wait_for_move_group_server():
            return False

        joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=position,
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            )
            for name, position in joint_targets.items()
        ]

        request = MotionPlanRequest()
        request.group_name = group_name
        request.goal_constraints = [Constraints(joint_constraints=joint_constraints)]
        request.allowed_planning_time = 5.0
        request.num_planning_attempts = 3
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3
        workspace = WorkspaceParameters()
        workspace.min_corner.x = -2.0
        workspace.min_corner.y = -2.0
        workspace.min_corner.z = -2.0
        workspace.max_corner.x = 2.0
        workspace.max_corner.y = 2.0
        workspace.max_corner.z = 2.0
        request.workspace_parameters = workspace

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        send_future = self._move_group_client.send_goal_async(goal)
        self._executor.spin_until_future_complete(
            send_future, timeout_sec=self._move_group_wait_timeout_sec
        )
        if not send_future.done():
            self.get_logger().error(
                "Timed out waiting for MoveGroup goal acknowledgement"
            )
            return False
        send_error = send_future.exception()
        if send_error is not None:
            self.get_logger().error(f"Failed to send MoveGroup goal: {send_error}")
            return False
        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error("MoveGroup goal handle was not returned")
            return False

        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        execution_timeout = max(0.1, timeout_sec)
        self._executor.spin_until_future_complete(
            result_future, timeout_sec=execution_timeout
        )
        if not result_future.done():
            self.get_logger().error(
                f"MoveGroup execution timed out (timeout={execution_timeout:.1f}s)"
            )
            return False
        result_error = result_future.exception()
        if result_error is not None:
            self.get_logger().error(
                f"MoveGroup execution future raised an exception: {result_error}"
            )
            return False
        result_response = result_future.result()
        if result_response is None:
            self.get_logger().error("MoveGroup execution result was empty")
            return False

        status = result_response.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return True

        error_code = result_response.result.error_code.val
        self.get_logger().error(
            f"MoveGroup failed: status={status}, error_code={error_code}"
        )
        return False

    def go_to_ready_pose(self, timeout_sec: float = 15.0) -> bool:
        """Move arm to a non-singular pose before starting servo jogging."""
        self.get_logger().info("Moving arm to ready pose (away from singularity)...")
        if self._execute_joint_goal(
            READY_JOINT_POSITIONS, self._ready_pose_group_name, timeout_sec
        ):
            self.get_logger().info("Arm moved to ready pose")
            return True
        self.get_logger().error("Failed to move arm to ready pose")
        return False

    def _execute_gripper_cmd(self, position: float) -> bool:
        """Send a GripperCommand directly to the bridge, bypassing MoveIt planning."""
        t0 = time.monotonic()

        if not self._gripper_cmd_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"GripperCommand action {self._gripper_cmd_action} not available"
            )
            return False
        t_server = time.monotonic()
        self.get_logger().info(f"[TIMING] wait_for_server: {t_server - t0:.3f}s")

        goal = GripperCommand.Goal()
        goal.command = GripperCommandMsg(position=position, max_effort=0.0)

        send_future = self._gripper_cmd_client.send_goal_async(goal)
        self._executor.spin_until_future_complete(send_future, timeout_sec=5.0)
        if not send_future.done():
            self.get_logger().error("Timed out sending GripperCommand goal")
            return False
        t_sent = time.monotonic()
        self.get_logger().info(f"[TIMING] send_goal: {t_sent - t_server:.3f}s")

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("GripperCommand goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        self._executor.spin_until_future_complete(
            result_future, timeout_sec=self._gripper_goal_timeout_sec
        )
        if not result_future.done():
            self.get_logger().error("GripperCommand execution timed out")
            return False
        t_result = time.monotonic()
        self.get_logger().info(f"[TIMING] execution: {t_result - t_sent:.3f}s")

        result_response = result_future.result()
        if result_response is None:
            self.get_logger().error("GripperCommand result was empty")
            return False

        result = result_response.result
        stalled = bool(getattr(result, "stalled", False))
        self.get_logger().info(
            f"GripperCommand done: position={result.position:.4f}, "
            f"stalled={stalled}, reached_goal={result.reached_goal}"
        )
        self.get_logger().info(f"[TIMING] total gripper cmd: {t_result - t0:.3f}s")

        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                "GripperCommand failed with "
                f"status={result_response.status}, "
                f"position={result.position:.4f}, "
                f"stalled={stalled}, reached_goal={result.reached_goal}"
            )
            return False

        if stalled and not result.reached_goal:
            self.get_logger().warn(
                "GripperCommand reached a mechanical limit before the model target: "
                f"position={result.position:.4f}, "
                f"stalled={stalled}, reached_goal={result.reached_goal}"
            )

        return True

    def open_gripper(self) -> bool:
        """Open the gripper via GripperCommand action (no MoveIt planning)."""
        self.get_logger().info("Opening gripper...")
        if self._execute_gripper_cmd(self._gripper_open_position):
            self.get_logger().info("Gripper opened")
            return True
        self.get_logger().error("Failed to open gripper")
        return False

    def close_gripper(self) -> bool:
        """Close the gripper via GripperCommand action (no MoveIt planning)."""
        self.get_logger().info("Closing gripper...")
        if self._execute_gripper_cmd(self._gripper_closed_position):
            self.get_logger().info("Gripper closed")
            return True
        self.get_logger().error("Failed to close gripper")
        return False

    def _publish_twist(self, linear: Tuple[float, float, float],
                        angular: Tuple[float, float, float],
                        duration_sec: float) -> None:
        period = 1.0 / self._publish_rate_hz
        start_time = time.monotonic()

        twist = TwistStamped()
        twist.header.frame_id = self._command_frame
        twist.twist.linear.x = float(linear[0])
        twist.twist.linear.y = float(linear[1])
        twist.twist.linear.z = float(linear[2])
        twist.twist.angular.x = float(angular[0])
        twist.twist.angular.y = float(angular[1])
        twist.twist.angular.z = float(angular[2])

        while rclpy.ok() and (time.monotonic() - start_time) < duration_sec:
            twist.header.stamp = self.get_clock().now().to_msg()
            self._twist_pub.publish(twist)
            self._executor.spin_once(timeout_sec=0.0)
            time.sleep(period)

        self.publish_zero()

    def publish_zero(self) -> None:
        twist = TwistStamped()
        twist.header.frame_id = self._command_frame
        twist.header.stamp = self.get_clock().now().to_msg()
        self._twist_pub.publish(twist)

    def jog_linear(self, axis: str, duration_sec: float | None = None) -> None:
        duration = duration_sec if duration_sec is not None else self._default_duration_sec
        speed = self._linear_speed

        if axis == "x":
            linear = (speed, 0.0, 0.0)
        elif axis == "-x":
            linear = (-speed, 0.0, 0.0)
        elif axis == "y":
            linear = (0.0, speed, 0.0)
        elif axis == "-y":
            linear = (0.0, -speed, 0.0)
        elif axis == "z":
            linear = (0.0, 0.0, speed)
        elif axis == "-z":
            linear = (0.0, 0.0, -speed)
        else:
            self.get_logger().warn(f"Unknown linear axis: {axis}")
            return

        self._publish_twist(linear, (0.0, 0.0, 0.0), duration)

    def jog_angular(self, axis: str, duration_sec: float | None = None) -> None:
        duration = duration_sec if duration_sec is not None else self._default_duration_sec
        speed = self._angular_speed

        if axis == "roll":
            angular = (speed, 0.0, 0.0)
        elif axis == "-roll":
            angular = (-speed, 0.0, 0.0)
        elif axis == "pitch":
            angular = (0.0, speed, 0.0)
        elif axis == "-pitch":
            angular = (0.0, -speed, 0.0)
        elif axis == "yaw":
            angular = (0.0, 0.0, speed)
        elif axis == "-yaw":
            angular = (0.0, 0.0, -speed)
        else:
            self.get_logger().warn(f"Unknown angular axis: {axis}")
            return

        self._publish_twist((0.0, 0.0, 0.0), angular, duration)


def main() -> None:
    rclpy.init()
    commander = PiperServoCommander()

    try:
        input("\nPress Enter to move arm to ready pose and start servo...")
        if not commander.go_to_ready_pose():
            print("WARNING: Could not move to ready pose. Servo may not work correctly.")
        # Start servo immediately after moving to avoid the arm drifting
        # back to the all-zeros singularity.
        if not commander.start_servo():
            print("WARNING: Servo start request failed. Jog commands may be ignored.")

        input("\nPress Enter to jog +X...")
        commander.jog_linear("x")

        input("\nPress Enter to jog -X...")
        commander.jog_linear("-x")

        input("\nPress Enter to jog +Y...")
        commander.jog_linear("y")

        input("\nPress Enter to jog -Y...")
        commander.jog_linear("-y")

        input("\nPress Enter to jog +Z...")
        commander.jog_linear("z")

        input("\nPress Enter to jog -Z...")
        commander.jog_linear("-z")

        input("\nPress Enter to yaw +... ")
        commander.jog_angular("yaw")

        input("\nPress Enter to yaw -... ")
        commander.jog_angular("-yaw")

        input("\nPress Enter to open gripper fully...")
        commander.open_gripper()

        input("\nPress Enter to close gripper fully...")
        commander.close_gripper()

        commander.get_logger().info("\nServo jog demo completed")

    except KeyboardInterrupt:
        commander.get_logger().info("Interrupted by user")
    finally:
        commander.publish_zero()
        commander._executor.remove_node(commander)
        commander._executor.shutdown()
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
