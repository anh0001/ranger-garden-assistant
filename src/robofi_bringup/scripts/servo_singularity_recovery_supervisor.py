#!/usr/bin/env python3

"""Gate Servo command topics and auto-recover from singularity slowdown/stops."""

import time

import rclpy
import rclpy.action
from action_msgs.msg import GoalStatus
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, WorkspaceParameters
from rclpy.node import Node
from std_msgs.msg import Int8


INVALID = -1
NO_WARNING = 0
DECELERATE_FOR_APPROACHING_SINGULARITY = 1
HALT_FOR_SINGULARITY = 2
DECELERATE_FOR_COLLISION = 3
HALT_FOR_COLLISION = 4
JOINT_BOUND = 5
DECELERATE_FOR_LEAVING_SINGULARITY = 6


class ServoSingularityRecoverySupervisor(Node):
    def __init__(self) -> None:
        super().__init__("servo_singularity_recovery_supervisor")

        self.declare_parameter("twist_input_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("twist_output_topic", "/servo_node/delta_twist_cmds_internal")
        self.declare_parameter("joint_input_topic", "/servo_node/delta_joint_cmds")
        self.declare_parameter("joint_output_topic", "/servo_node/delta_joint_cmds_internal")
        self.declare_parameter("status_topic", "/servo_node/status")
        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("recovery_group_name", "piper_arm")
        self.declare_parameter("command_frame", "piper_base_link")
        self.declare_parameter("joint_prefix", "piper_")
        self.declare_parameter("recovery_timeout_sec", 15.0)
        self.declare_parameter("recovery_cooldown_sec", 2.0)
        self.declare_parameter("deceleration_persist_sec", 0.5)
        self.declare_parameter("move_group_wait_timeout_sec", 5.0)

        self._twist_input_topic = str(self.get_parameter("twist_input_topic").value)
        self._twist_output_topic = str(self.get_parameter("twist_output_topic").value)
        self._joint_input_topic = str(self.get_parameter("joint_input_topic").value)
        self._joint_output_topic = str(self.get_parameter("joint_output_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._move_group_action = str(self.get_parameter("move_group_action").value)
        self._recovery_group_name = str(self.get_parameter("recovery_group_name").value)
        self._command_frame = str(self.get_parameter("command_frame").value)
        self._joint_prefix = str(self.get_parameter("joint_prefix").value)
        self._recovery_timeout_sec = max(
            0.1, float(self.get_parameter("recovery_timeout_sec").value)
        )
        self._recovery_cooldown_sec = max(
            0.0, float(self.get_parameter("recovery_cooldown_sec").value)
        )
        self._deceleration_persist_sec = max(
            0.0, float(self.get_parameter("deceleration_persist_sec").value)
        )
        self._move_group_wait_timeout_sec = max(
            0.1, float(self.get_parameter("move_group_wait_timeout_sec").value)
        )

        prefix = self._joint_prefix
        self._recovery_targets = {
            f"{prefix}joint1": 0.0,
            f"{prefix}joint2": 1.2,
            f"{prefix}joint3": -0.2,
            f"{prefix}joint4": 0.0,
            f"{prefix}joint5": -0.8,
            f"{prefix}joint6": 0.0,
        }

        self._recovering = False
        self._commands_blocked_until = 0.0
        self._approaching_since = None
        self._last_status = NO_WARNING
        self._recovery_started_at = 0.0
        self._active_goal_handle = None

        self._twist_pub = self.create_publisher(TwistStamped, self._twist_output_topic, 10)
        self._joint_pub = self.create_publisher(JointJog, self._joint_output_topic, 10)
        self._twist_sub = self.create_subscription(
            TwistStamped, self._twist_input_topic, self._on_twist_command, 10
        )
        self._joint_sub = self.create_subscription(
            JointJog, self._joint_input_topic, self._on_joint_command, 10
        )
        self._status_sub = self.create_subscription(
            Int8, self._status_topic, self._on_servo_status, 10
        )

        self._move_group_client = rclpy.action.ActionClient(
            self, MoveGroup, self._move_group_action
        )
        self._recovery_watchdog = self.create_timer(0.2, self._check_recovery_timeout)

        self.get_logger().info(
            "Servo singularity recovery supervisor ready: "
            f"{self._twist_input_topic} -> {self._twist_output_topic}, "
            f"{self._joint_input_topic} -> {self._joint_output_topic}, "
            f"status={self._status_topic}"
        )

    def _commands_blocked(self) -> bool:
        return self._recovering or time.monotonic() < self._commands_blocked_until

    def _on_twist_command(self, msg: TwistStamped) -> None:
        if self._commands_blocked():
            return
        self._twist_pub.publish(msg)

    def _on_joint_command(self, msg: JointJog) -> None:
        if self._commands_blocked():
            return
        self._joint_pub.publish(msg)

    def _on_servo_status(self, msg: Int8) -> None:
        status = int(msg.data)
        now = time.monotonic()
        self._last_status = status

        if self._recovering:
            return

        if now < self._commands_blocked_until:
            return

        if status == DECELERATE_FOR_APPROACHING_SINGULARITY:
            if self._approaching_since is None:
                self._approaching_since = now
                return
            if (now - self._approaching_since) >= self._deceleration_persist_sec:
                self._start_recovery(
                    "persistent singularity slowdown reported by MoveIt Servo"
                )
            return

        self._approaching_since = None

        if status == HALT_FOR_SINGULARITY:
            self._start_recovery("Servo emergency stop near singularity")

    def _publish_zero_commands(self) -> None:
        zero_twist = TwistStamped()
        zero_twist.header.stamp = self.get_clock().now().to_msg()
        zero_twist.header.frame_id = self._command_frame
        self._twist_pub.publish(zero_twist)

        zero_joint = JointJog()
        zero_joint.header.stamp = zero_twist.header.stamp
        self._joint_pub.publish(zero_joint)

    def _start_recovery(self, reason: str) -> None:
        if self._recovering:
            return

        self._recovering = True
        self._approaching_since = None
        self._recovery_started_at = time.monotonic()
        self._active_goal_handle = None
        self._publish_zero_commands()
        self.get_logger().warn(
            f"Starting automatic singularity recovery: {reason}. "
            "Planning back to the servo-ready pose."
        )

        if not self._move_group_client.wait_for_server(
            timeout_sec=self._move_group_wait_timeout_sec
        ):
            self._finish_recovery(
                success=False,
                message=(
                    f"MoveGroup action server {self._move_group_action} not available "
                    f"(timeout={self._move_group_wait_timeout_sec:.1f}s)"
                ),
            )
            return

        send_future = self._move_group_client.send_goal_async(
            self._build_recovery_goal()
        )
        send_future.add_done_callback(self._on_goal_response)

    def _build_recovery_goal(self) -> MoveGroup.Goal:
        joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=position,
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            )
            for name, position in self._recovery_targets.items()
        ]

        request = MotionPlanRequest()
        request.group_name = self._recovery_group_name
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
        return goal

    def _on_goal_response(self, future) -> None:
        if not self._recovering:
            return

        exc = future.exception()
        if exc is not None:
            self._finish_recovery(False, f"Failed to send recovery goal: {exc}")
            return

        goal_handle = future.result()
        if goal_handle is None:
            self._finish_recovery(False, "MoveGroup returned no goal handle")
            return
        if not goal_handle.accepted:
            self._finish_recovery(False, "MoveGroup rejected the recovery goal")
            return

        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_recovery_result)

    def _on_recovery_result(self, future) -> None:
        if not self._recovering:
            return

        exc = future.exception()
        if exc is not None:
            self._finish_recovery(False, f"Recovery result raised an exception: {exc}")
            return

        result_response = future.result()
        if result_response is None:
            self._finish_recovery(False, "Recovery result was empty")
            return

        if result_response.status == GoalStatus.STATUS_SUCCEEDED:
            self._finish_recovery(True, "Arm returned to the servo-ready pose")
            return

        error_code = result_response.result.error_code.val
        self._finish_recovery(
            False,
            (
                "Recovery MoveGroup execution failed: "
                f"status={result_response.status}, error_code={error_code}"
            ),
        )

    def _check_recovery_timeout(self) -> None:
        if not self._recovering:
            return

        elapsed = time.monotonic() - self._recovery_started_at
        if elapsed <= self._recovery_timeout_sec:
            return

        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
            self._active_goal_handle = None

        self._finish_recovery(
            False,
            f"Recovery timed out after {self._recovery_timeout_sec:.1f}s",
        )

    def _finish_recovery(self, success: bool, message: str) -> None:
        self._publish_zero_commands()
        self._recovering = False
        self._active_goal_handle = None
        self._recovery_started_at = 0.0
        self._commands_blocked_until = time.monotonic() + self._recovery_cooldown_sec
        if success:
            self.get_logger().info(
                f"{message}. Servo commands will remain blocked for "
                f"{self._recovery_cooldown_sec:.1f}s."
            )
        else:
            self.get_logger().error(
                f"{message}. Servo commands will remain blocked for "
                f"{self._recovery_cooldown_sec:.1f}s."
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoSingularityRecoverySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
