#!/usr/bin/env python3
"""
PiPER MoveIt Servo Commander (Twist-based jog)

This script publishes TwistStamped commands to MoveIt Servo for quick, manual
validation of realtime Cartesian control on the PiPER arm.

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
"""

import time
from typing import Tuple

import rclpy
import rclpy.action
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    WorkspaceParameters,
)
from action_msgs.msg import GoalStatus


# A non-singular "ready" pose for Cartesian jogging (radians).
# joint4 and joint5 are offset to avoid the wrist singularity
# (joint4=0 + joint5~0 makes joints 4 & 6 collinear).
READY_JOINT_POSITIONS = {
    "piper_joint1": 0.0,
    "piper_joint2": 0.5,
    "piper_joint3": -1.0,
    "piper_joint4": 0.3,
    "piper_joint5": -0.8,
    "piper_joint6": 0.0,
}


class PiperServoCommander(Node):
    def __init__(self) -> None:
        super().__init__("piper_servo_commander")

        self.declare_parameter("servo_start_service", "/servo_node/start_servo")
        self.declare_parameter("servo_twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("command_frame", "piper_base_link")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("default_duration_sec", 0.8)
        self.declare_parameter("linear_speed", 0.04)
        self.declare_parameter("angular_speed", 0.4)

        self._servo_start_service = self.get_parameter("servo_start_service").value
        self._servo_twist_topic = self.get_parameter("servo_twist_topic").value
        self._command_frame = self.get_parameter("command_frame").value
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._default_duration_sec = float(
            self.get_parameter("default_duration_sec").value
        )
        self._linear_speed = float(self.get_parameter("linear_speed").value)
        self._angular_speed = float(self.get_parameter("angular_speed").value)

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
        self.get_logger().info(
            f"Servo twist topic: {self._servo_twist_topic}"
        )
        self.get_logger().info(
            f"Servo start service: {self._servo_start_service}"
        )
        self.get_logger().info(
            f"Command frame: {self._command_frame}"
        )

    def _call_trigger_service(self, service_name: str, timeout_sec: float = 5.0) -> bool:
        client = self.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f"Service {service_name} not available after {timeout_sec}s"
            )
            return False
        future = client.call_async(Trigger.Request())
        self._executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        if future.result() is None:
            self.get_logger().warn(f"Service call to {service_name} timed out")
            return False
        if not future.result().success:
            self.get_logger().warn(
                f"Service {service_name} returned failure: {future.result().message}"
            )
        return future.result().success

    def start_servo(self) -> None:
        if self._call_trigger_service(self._servo_start_service):
            self.get_logger().info("MoveIt Servo started")
        else:
            self.get_logger().warn("Could not start MoveIt Servo; continuing anyway")

    def go_to_ready_pose(self, timeout_sec: float = 15.0) -> bool:
        """Move arm to a non-singular pose before starting servo jogging."""
        self.get_logger().info("Moving arm to ready pose (away from singularity)...")

        client = rclpy.action.ActionClient(self, MoveGroup, "/move_action")
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveGroup action server not available")
            return False

        joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=pos,
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            )
            for name, pos in READY_JOINT_POSITIONS.items()
        ]

        request = MotionPlanRequest()
        request.group_name = "piper_arm"
        request.goal_constraints = [
            Constraints(joint_constraints=joint_constraints)
        ]
        request.allowed_planning_time = 5.0
        request.num_planning_attempts = 3
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3
        ws = WorkspaceParameters()
        ws.min_corner.x = -2.0
        ws.min_corner.y = -2.0
        ws.min_corner.z = -2.0
        ws.max_corner.x = 2.0
        ws.max_corner.y = 2.0
        ws.max_corner.z = 2.0
        request.workspace_parameters = ws

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        future = client.send_goal_async(goal)
        self._executor.spin_until_future_complete(future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error("Failed to send MoveGroup goal")
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        self._executor.spin_until_future_complete(result_future, timeout_sec=timeout_sec)

        if result_future.result() is None:
            self.get_logger().error("MoveGroup execution timed out")
            return False

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Arm moved to ready pose")
            return True
        else:
            error_code = result_future.result().result.error_code.val
            self.get_logger().error(
                f"MoveGroup failed: status={status}, error_code={error_code}"
            )
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
        commander.start_servo()

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
