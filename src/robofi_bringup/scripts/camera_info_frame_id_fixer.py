#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


class CameraInfoFrameIdFixer(Node):
    def __init__(self) -> None:
        super().__init__("camera_info_frame_id_fixer")

        self.declare_parameter("frame_id", "")
        self.declare_parameter("only_if_empty", True)

        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._only_if_empty = (
            self.get_parameter("only_if_empty").get_parameter_value().bool_value
        )

        if not self._frame_id:
            self.get_logger().warn(
                "Parameter 'frame_id' is empty; camera_info headers will be forwarded unchanged."
            )

        self._subscriber = self.create_subscription(
            CameraInfo,
            "camera_info_raw",
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self._publisher = self.create_publisher(
            CameraInfo,
            "camera_info",
            qos_profile_sensor_data,
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if not self._frame_id:
            self._publisher.publish(msg)
            return

        if self._only_if_empty and msg.header.frame_id:
            self._publisher.publish(msg)
            return

        fixed = copy.deepcopy(msg)
        fixed.header.frame_id = self._frame_id
        self._publisher.publish(fixed)


def main() -> None:
    rclpy.init()
    node = CameraInfoFrameIdFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Only shutdown if context is still valid
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
