#!/usr/bin/env python3

"""Bridge MoveIt Servo Float64 arrays to PiPER JointState command messages."""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class ServoArrayToPiperJointCmd(Node):
    def __init__(self):
        super().__init__("servo_array_to_piper_joint_cmd")

        self.declare_parameter("input_topic", "/moveit_servo/piper_arm_joint_targets")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("output_topic", "/piper/joint_cmd")
        self.declare_parameter("joint_prefix", "piper_")
        self.declare_parameter("expected_arm_dof", 6)

        self._input_topic = self.get_parameter("input_topic").value
        self._joint_states_topic = self.get_parameter("joint_states_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._joint_prefix = self.get_parameter("joint_prefix").value
        self._expected_arm_dof = int(self.get_parameter("expected_arm_dof").value)

        self._arm_joint_names = [
            f"{self._joint_prefix}joint1",
            f"{self._joint_prefix}joint2",
            f"{self._joint_prefix}joint3",
            f"{self._joint_prefix}joint4",
            f"{self._joint_prefix}joint5",
            f"{self._joint_prefix}joint6",
        ]
        self._gripper_joint_name = f"{self._joint_prefix}joint7"
        self._command_joint_names = self._arm_joint_names + [self._gripper_joint_name]

        self._latest_gripper_position = 0.0
        self._last_warn_time = 0.0

        self._cmd_pub = self.create_publisher(JointState, self._output_topic, 10)
        self._servo_sub = self.create_subscription(
            Float64MultiArray,
            self._input_topic,
            self._servo_callback,
            10,
        )
        self._joint_states_sub = self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._joint_states_callback,
            10,
        )

        self.get_logger().info(
            f"Servo array bridge ready: {self._input_topic} -> {self._output_topic}"
        )

    def _warn_throttle(self, message: str, period_sec: float = 2.0) -> None:
        now = time.monotonic()
        if now - self._last_warn_time >= period_sec:
            self._last_warn_time = now
            self.get_logger().warn(message)

    def _joint_states_callback(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name == self._gripper_joint_name:
                self._latest_gripper_position = float(pos)
                return

    def _servo_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < self._expected_arm_dof:
            self._warn_throttle(
                "Ignored Servo command: expected at least "
                f"{self._expected_arm_dof} arm values, got {len(msg.data)}"
            )
            return

        arm_targets = []
        for idx in range(self._expected_arm_dof):
            value = float(msg.data[idx])
            if not math.isfinite(value):
                self._warn_throttle(
                    f"Ignored Servo command: non-finite joint target at index {idx}"
                )
                return
            arm_targets.append(value)

        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(self._command_joint_names)
        cmd.position = arm_targets + [self._latest_gripper_position]

        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ServoArrayToPiperJointCmd()
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
