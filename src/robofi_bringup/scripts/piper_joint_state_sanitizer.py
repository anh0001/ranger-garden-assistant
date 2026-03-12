#!/usr/bin/env python3

"""Normalize JointState array lengths before forwarding to joint_state_publisher."""

import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PiperJointStateSanitizer(Node):
    def __init__(self) -> None:
        super().__init__("piper_joint_state_sanitizer")

        self.declare_parameter("input_topic", "/piper/joint_states")
        self.declare_parameter("output_topic", "/piper/joint_states_sanitized")

        self._input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        self._output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )

        self._sub = self.create_subscription(
            JointState,
            self._input_topic,
            self._on_joint_state,
            10,
        )
        self._pub = self.create_publisher(
            JointState,
            self._output_topic,
            10,
        )

        self.get_logger().info(
            f"Sanitizing JointState arrays: {self._input_topic} -> {self._output_topic}"
        )

    @staticmethod
    def _normalized(values, size: int):
        if len(values) == size:
            return list(values)
        if len(values) > size:
            return list(values[:size])
        return list(values) + [0.0] * (size - len(values))

    def _on_joint_state(self, msg: JointState) -> None:
        size = len(msg.name)
        if size == 0:
            return

        sanitized = copy.deepcopy(msg)
        sanitized.position = self._normalized(msg.position, size)
        sanitized.velocity = self._normalized(msg.velocity, size)
        sanitized.effort = self._normalized(msg.effort, size)
        self._pub.publish(sanitized)


def main() -> None:
    rclpy.init()
    node = PiperJointStateSanitizer()
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
