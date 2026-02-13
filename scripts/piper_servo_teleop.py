#!/usr/bin/env python3
"""
Keyboard teleop for MoveIt Servo (PiPER arm).

This tool publishes ``geometry_msgs/msg/TwistStamped`` commands to MoveIt Servo
on ``/servo_node/delta_twist_cmds`` and can optionally call
``/servo_node/start_servo``.

Usage:
    # Terminal 1: start the robot + MoveIt Servo
    ros2 launch robofi_bringup ranger_complete_bringup.launch.py

    # Terminal 2: run keyboard teleop
    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 scripts/piper_servo_teleop.py

CRITICAL: Timer Priority and MoveGroup Actions
-----------------------------------------------
This script's 30 Hz publish timer causes MoveGroup actions (go_to_ready_pose,
gripper operations) to timeout when using SingleThreadedExecutor.

Root cause: rclpy's executor dispatches timers before subscriptions. During
spin_until_future_complete(), the timer fires every ~33ms and starves the
action client's goal-response/result callbacks, eventually timing out.

piper_servo_commander.py works because it has no timer — only the action
callbacks are processed during go_to_ready_pose().

Fix: _send_joint_goal() cancels the timer before MoveGroup operations and
restores it via try/finally to prevent callback starvation.
"""

import select
import sys
import termios
import time
import tty
from typing import Dict, Tuple

import rclpy
import rclpy.action
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, WorkspaceParameters
from rclpy.node import Node
from std_srvs.srv import Trigger


Vector3 = Tuple[float, float, float]
MotionBinding = Tuple[Vector3, Vector3, str]


MOTION_BINDINGS: Dict[str, MotionBinding] = {
    # Translation
    "w": ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), "+X"),
    "s": ((-1.0, 0.0, 0.0), (0.0, 0.0, 0.0), "-X"),
    "a": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0), "+Y"),
    "d": ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0), "-Y"),
    "r": ((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), "+Z"),
    "f": ((0.0, 0.0, -1.0), (0.0, 0.0, 0.0), "-Z"),
    # Rotation
    "i": ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), "+Roll"),
    "k": ((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0), "-Roll"),
    "j": ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), "+Pitch"),
    "l": ((0.0, 0.0, 0.0), (0.0, -1.0, 0.0), "-Pitch"),
    "u": ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), "+Yaw"),
    "o": ((0.0, 0.0, 0.0), (0.0, 0.0, -1.0), "-Yaw"),
}

READY_JOINT_POSITIONS = {
    "piper_joint1": 0.0,
    "piper_joint2": 1.2,
    "piper_joint3": -0.2,
    "piper_joint4": 0.0,
    "piper_joint5": -0.8,
    "piper_joint6": 0.0,
}


ARROW_TO_KEY = {
    "\x1b[A": "w",  # up
    "\x1b[B": "s",  # down
    "\x1b[D": "a",  # left
    "\x1b[C": "d",  # right
}


HELP_TEXT = """
MoveIt Servo keyboard teleop (PiPER)
------------------------------------
Move:
  w/s : +X / -X
  a/d : +Y / -Y
  r/f : +Z / -Z

Rotate:
  i/k : +Roll / -Roll
  j/l : +Pitch / -Pitch
  u/o : +Yaw / -Yaw

Other:
  Arrow keys : XY move (same as w/a/s/d)
  x/z : increase/decrease linear speed
  v/c : increase/decrease angular speed
  b/n : open/close gripper
  g   : call /servo_node/start_servo
  h   : print this help
  space: stop motion
  q   : quit

Tip: keep holding keys (keyboard auto-repeat) for continuous motion.
"""


class RawKeyboard:
    """Context manager for reading single key presses from a terminal."""

    def __init__(self) -> None:
        self._fd = sys.stdin.fileno()
        self._old_term = None

    def __enter__(self) -> "RawKeyboard":
        if not sys.stdin.isatty():
            raise RuntimeError("stdin is not a TTY. Run this script from a terminal.")
        self._old_term = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._old_term is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)

    def read_key(self, timeout_sec: float) -> str:
        readable, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if not readable:
            return ""

        key = sys.stdin.read(1)
        if key != "\x1b":
            return key

        # Arrow keys arrive as an escape sequence (3 bytes).
        seq = key
        for _ in range(2):
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                break
            seq += sys.stdin.read(1)
        return seq


class PiperServoTeleop(Node):
    def __init__(self) -> None:
        super().__init__("piper_servo_teleop")

        self.declare_parameter("servo_twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("servo_start_service", "/servo_node/start_servo")
        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("move_group_wait_timeout_sec", 20.0)
        self.declare_parameter("ready_pose_group_name", "piper_arm")
        self.declare_parameter("move_to_ready_pose", True)
        self.declare_parameter("ready_pose_timeout_sec", 15.0)
        self.declare_parameter("gripper_group_name", "piper_gripper")
        self.declare_parameter("gripper_joint_name", "piper_joint7")
        self.declare_parameter("gripper_open_position", 0.75)
        self.declare_parameter("gripper_closed_position", 0.0)
        self.declare_parameter("gripper_goal_timeout_sec", 15.0)
        self.declare_parameter("command_frame", "piper_base_link")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("command_hold_sec", 0.15)
        self.declare_parameter("linear_speed", 0.1)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("auto_start_servo", True)
        self.declare_parameter("start_servo_timeout_sec", 12.0)

        self._twist_topic = str(self.get_parameter("servo_twist_topic").value)
        self._start_service = str(self.get_parameter("servo_start_service").value)
        self._move_group_action = str(self.get_parameter("move_group_action").value)
        self._move_group_wait_timeout_sec = max(
            0.5, float(self.get_parameter("move_group_wait_timeout_sec").value)
        )
        self._ready_pose_group_name = str(
            self.get_parameter("ready_pose_group_name").value
        )
        self._move_to_ready_pose = bool(self.get_parameter("move_to_ready_pose").value)
        self._ready_pose_timeout_sec = max(
            1.0, float(self.get_parameter("ready_pose_timeout_sec").value)
        )
        self._gripper_group_name = str(self.get_parameter("gripper_group_name").value)
        self._gripper_joint_name = str(self.get_parameter("gripper_joint_name").value)
        self._gripper_open_position = float(
            self.get_parameter("gripper_open_position").value
        )
        self._gripper_closed_position = float(
            self.get_parameter("gripper_closed_position").value
        )
        self._gripper_goal_timeout_sec = max(
            1.0, float(self.get_parameter("gripper_goal_timeout_sec").value)
        )
        self._command_frame = str(self.get_parameter("command_frame").value)
        self._publish_rate_hz = max(
            1.0, float(self.get_parameter("publish_rate_hz").value)
        )
        self._command_hold_sec = max(
            0.01, float(self.get_parameter("command_hold_sec").value)
        )
        self._linear_speed = max(0.001, float(self.get_parameter("linear_speed").value))
        self._angular_speed = max(0.001, float(self.get_parameter("angular_speed").value))
        self._auto_start_servo = bool(self.get_parameter("auto_start_servo").value)
        self._start_servo_timeout = max(
            0.1, float(self.get_parameter("start_servo_timeout_sec").value)
        )

        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self)

        self._cmd_linear: Vector3 = (0.0, 0.0, 0.0)
        self._cmd_angular: Vector3 = (0.0, 0.0, 0.0)
        self._last_cmd_time = 0.0
        self._start_future = None

        self._twist_pub = self.create_publisher(TwistStamped, self._twist_topic, 10)
        self._start_client = self.create_client(Trigger, self._start_service)
        self._move_group_client = rclpy.action.ActionClient(
            self, MoveGroup, self._move_group_action
        )
        self._publish_timer = self.create_timer(
            1.0 / self._publish_rate_hz, self._on_publish
        )

        self.get_logger().info(f"Twist topic: {self._twist_topic}")
        self.get_logger().info(f"Start service: {self._start_service}")
        self.get_logger().info(f"MoveGroup action: {self._move_group_action}")
        self.get_logger().info(
            f"MoveGroup wait timeout: {self._move_group_wait_timeout_sec:.1f}s"
        )
        self.get_logger().info(f"Command frame: {self._command_frame}")
        self.get_logger().info(
            f"Gripper group/joint: {self._gripper_group_name}/{self._gripper_joint_name}"
        )
        self.get_logger().info(
            f"Gripper goal timeout: {self._gripper_goal_timeout_sec:.1f}s"
        )
        self.get_logger().info(
            f"Speeds linear={self._linear_speed:.3f} m/s angular={self._angular_speed:.3f} rad/s"
        )
        if self._gripper_goal_timeout_sec < 10.0:
            self.get_logger().warn(
                "gripper_goal_timeout_sec is below 10s; this may timeout before "
                "the gripper trajectory bridge reports completion."
            )

    @property
    def auto_start_servo(self) -> bool:
        return self._auto_start_servo

    @property
    def move_to_ready_pose(self) -> bool:
        return self._move_to_ready_pose

    def _on_publish(self) -> None:
        now = time.monotonic()
        active = (now - self._last_cmd_time) <= self._command_hold_sec

        if active:
            linear = tuple(v * self._linear_speed for v in self._cmd_linear)
            angular = tuple(v * self._angular_speed for v in self._cmd_angular)
        else:
            linear = (0.0, 0.0, 0.0)
            angular = (0.0, 0.0, 0.0)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._command_frame
        msg.twist.linear.x = linear[0]
        msg.twist.linear.y = linear[1]
        msg.twist.linear.z = linear[2]
        msg.twist.angular.x = angular[0]
        msg.twist.angular.y = angular[1]
        msg.twist.angular.z = angular[2]
        self._twist_pub.publish(msg)

    def set_motion(self, linear: Vector3, angular: Vector3) -> None:
        self._cmd_linear = linear
        self._cmd_angular = angular
        self._last_cmd_time = time.monotonic()

    def stop(self) -> None:
        self._cmd_linear = (0.0, 0.0, 0.0)
        self._cmd_angular = (0.0, 0.0, 0.0)
        self._last_cmd_time = time.monotonic()

    def scale_linear_speed(self, factor: float) -> None:
        self._linear_speed = max(0.001, self._linear_speed * factor)
        self.get_logger().info(f"Linear speed: {self._linear_speed:.3f} m/s")

    def scale_angular_speed(self, factor: float) -> None:
        self._angular_speed = max(0.001, self._angular_speed * factor)
        self.get_logger().info(f"Angular speed: {self._angular_speed:.3f} rad/s")

    def request_start_servo(self) -> None:
        if self._start_future is not None and not self._start_future.done():
            self.get_logger().info("start_servo request already in progress")
            return

        if not self._start_client.wait_for_service(timeout_sec=self._start_servo_timeout):
            self.get_logger().warn(
                f"Service {self._start_service} not available (timeout={self._start_servo_timeout:.1f}s)"
            )
            return

        self._start_future = self._start_client.call_async(Trigger.Request())
        self._start_future.add_done_callback(self._on_start_servo_response)
        self.get_logger().info("Requested servo start")

    def _on_start_servo_response(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"start_servo call failed: {exc}")
            self._start_future = None
            return

        if result.success:
            self.get_logger().info(f"Servo started: {result.message}")
        else:
            self.get_logger().warn(f"Servo start rejected: {result.message}")
        self._start_future = None

    def _send_joint_goal(
        self,
        joint_targets: Dict[str, float],
        group_name: str,
        timeout_sec: float,
    ) -> bool:
        # Suspend the 30 Hz twist-publish timer while MoveGroup is active.
        # The timer's callbacks compete with action-client subscriptions inside
        # SingleThreadedExecutor.spin_until_future_complete (timers have higher
        # priority), which can starve or delay the goal-response / result
        # callbacks and cause spurious timeouts.  The commander script works
        # because it has no timer at all during go_to_ready_pose().
        self._publish_timer.cancel()

        try:
            return self._execute_joint_goal(joint_targets, group_name, timeout_sec)
        finally:
            self._publish_timer.reset()

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
            f"(timeout={timeout:.1f}s). Is move_group fully started?"
        )
        return False

    def _execute_joint_goal(
        self,
        joint_targets: Dict[str, float],
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

    def go_to_ready_pose(self, timeout_sec: float | None = None) -> bool:
        """Move arm to a non-singular pose before servo teleoperation."""
        timeout = timeout_sec if timeout_sec is not None else self._ready_pose_timeout_sec
        self.get_logger().info("Moving arm to ready pose before teleop...")
        if self._send_joint_goal(
            READY_JOINT_POSITIONS, self._ready_pose_group_name, timeout
        ):
            self.get_logger().info("Arm moved to ready pose")
            return True
        self.get_logger().error("Failed to move arm to ready pose")
        return False

    def open_gripper(self) -> bool:
        self.get_logger().info("Opening gripper...")
        success = self._send_joint_goal(
            {self._gripper_joint_name: self._gripper_open_position},
            self._gripper_group_name,
            self._gripper_goal_timeout_sec,
        )
        if success:
            self.get_logger().info("Gripper opened")
        else:
            self.get_logger().error("Failed to open gripper")
        return success

    def close_gripper(self) -> bool:
        self.get_logger().info("Closing gripper...")
        success = self._send_joint_goal(
            {self._gripper_joint_name: self._gripper_closed_position},
            self._gripper_group_name,
            self._gripper_goal_timeout_sec,
        )
        if success:
            self.get_logger().info("Gripper closed")
        else:
            self.get_logger().error("Failed to close gripper")
        return success

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        self._executor.spin_once(timeout_sec=timeout_sec)

    def shutdown(self) -> None:
        self.stop()
        self._executor.remove_node(self)
        self._executor.shutdown()
        self.destroy_node()


def main() -> None:
    rclpy.init()
    node = PiperServoTeleop()

    print(HELP_TEXT)

    if node.move_to_ready_pose and not node.go_to_ready_pose():
        node.get_logger().warn(
            "Ready pose move failed. Teleop will continue but servo may halt near singularities."
        )

    if node.auto_start_servo:
        node.request_start_servo()

    try:
        with RawKeyboard() as keyboard:
            while rclpy.ok():
                node.spin_once(timeout_sec=0.01)
                key = keyboard.read_key(timeout_sec=0.02)
                if not key:
                    continue

                if key in ARROW_TO_KEY:
                    key = ARROW_TO_KEY[key]

                if key == "\x03" or key == "q":
                    break

                if key == " ":
                    node.stop()
                    continue

                if key == "h":
                    print(HELP_TEXT)
                    continue

                if key == "x":
                    node.scale_linear_speed(1.1)
                    continue
                if key == "z":
                    node.scale_linear_speed(0.9)
                    continue
                if key == "v":
                    node.scale_angular_speed(1.1)
                    continue
                if key == "c":
                    node.scale_angular_speed(0.9)
                    continue
                if key == "g":
                    node.request_start_servo()
                    continue
                if key == "b":
                    node.open_gripper()
                    continue
                if key == "n":
                    node.close_gripper()
                    continue

                binding = MOTION_BINDINGS.get(key)
                if binding is None:
                    continue

                linear, angular, label = binding
                node.set_motion(linear, angular)
                node.get_logger().info(f"Command: {label}")
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        # Publish a few zero twists before exit to ensure Servo halts.
        for _ in range(3):
            node.spin_once(timeout_sec=0.05)
        node.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
