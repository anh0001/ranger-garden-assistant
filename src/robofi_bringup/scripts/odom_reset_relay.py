#!/usr/bin/env python3
"""Resettable odometry relay for the Ranger base.

The AgileX `ranger_base_node` integrates wheel odometry internally and exposes
no way to zero it (the driver lives in the `ranger_ros2` submodule and must not
be edited for feature work). This node sits in front of it: it consumes the
driver's raw odometry on `input_topic`, applies an SE(2) offset, and republishes
the corrected pose on `output_topic` (`/odom` by default). A `reset_odom`
service (std_srvs/Trigger) captures the current raw pose as the new origin, so
the published odometry — and, optionally, the `odom -> base` TF this node owns —
snaps back to zero on demand.

Only the pose is offset. The twist in nav_msgs/Odometry is expressed in the
base (child) frame, so it is forwarded unchanged.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    """Extract the planar yaw (rad) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    """Return (x, y, z, w) for a pure-yaw rotation."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OdomResetRelay(Node):
    def __init__(self):
        super().__init__("odom_reset_relay")

        self.input_topic = self.declare_parameter("input_topic", "odom_raw").value
        self.output_topic = self.declare_parameter("output_topic", "odom").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.publish_tf = self.declare_parameter("publish_tf", False).value
        # When true, the first raw message is treated as the origin so a fresh
        # launch starts at exactly zero.
        self.zero_on_start = self.declare_parameter("zero_on_start", True).value

        # SE(2) origin: the raw pose currently mapped to (0, 0, 0).
        self.off_x = 0.0
        self.off_y = 0.0
        self.off_yaw = 0.0
        self.have_offset = not self.zero_on_start
        self.last_raw = None  # (x, y, yaw) of the most recent raw odom

        # Match the typical sensor-stream QoS so we stay compatible with the driver.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(Odometry, self.output_topic, qos)
        self.sub = self.create_subscription(Odometry, self.input_topic, self.on_odom, qos)
        self.srv = self.create_service(Trigger, "reset_odom", self.on_reset)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.get_logger().info(
            f"odom_reset_relay: '{self.input_topic}' -> '{self.output_topic}' "
            f"(frame {self.odom_frame} -> {self.base_frame}, publish_tf={self.publish_tf}). "
            f"Call 'ros2 service call {self.resolve_service_name()} std_srvs/srv/Trigger' to zero."
        )

    def resolve_service_name(self):
        ns = self.get_namespace().rstrip("/")
        return f"{ns}/reset_odom" if ns else "/reset_odom"

    def on_odom(self, msg):
        raw_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        self.last_raw = (raw_x, raw_y, raw_yaw)

        if not self.have_offset:
            self.off_x, self.off_y, self.off_yaw = raw_x, raw_y, raw_yaw
            self.have_offset = True
            self.get_logger().info("Captured initial odom origin (zero_on_start).")

        # corrected = offset^-1 * raw, in SE(2): translate then rotate by -off_yaw.
        c = math.cos(self.off_yaw)
        s = math.sin(self.off_yaw)
        dx = raw_x - self.off_x
        dy = raw_y - self.off_y
        cx = c * dx + s * dy
        cy = -s * dx + c * dy
        cyaw = normalize_angle(raw_yaw - self.off_yaw)
        qx, qy, qz, qw = quaternion_from_yaw(cyaw)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = cx
        out.pose.pose.position.y = cy
        out.pose.pose.position.z = msg.pose.pose.position.z
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.pose.covariance = msg.pose.covariance
        # Twist is in the base frame; forward unchanged.
        out.twist = msg.twist
        self.pub.publish(out)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = cx
            tf.transform.translation.y = cy
            tf.transform.translation.z = msg.pose.pose.position.z
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

    def on_reset(self, request, response):
        if self.last_raw is None:
            response.success = False
            response.message = (
                f"No odometry received yet on '{self.input_topic}'; nothing to reset."
            )
            self.get_logger().warn(response.message)
            return response

        self.off_x, self.off_y, self.off_yaw = self.last_raw
        self.have_offset = True
        response.success = True
        response.message = (
            f"Odometry zeroed at raw pose "
            f"(x={self.off_x:.3f}, y={self.off_y:.3f}, yaw={self.off_yaw:.3f})."
        )
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = OdomResetRelay()
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
