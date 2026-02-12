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
"""

import time
from typing import Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger


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

        self._twist_pub = self.create_publisher(
            TwistStamped, self._servo_twist_topic, 10
        )
        self._start_client = self.create_client(Trigger, self._servo_start_service)

        self.get_logger().info(
            f"Servo twist topic: {self._servo_twist_topic}"
        )
        self.get_logger().info(
            f"Servo start service: {self._servo_start_service}"
        )
        self.get_logger().info(
            f"Command frame: {self._command_frame}"
        )

    def start_servo(self) -> None:
        if not self._start_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "Servo start service not available; continuing without explicit start"
            )
            return

        request = Trigger.Request()
        future = self._start_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.get_logger().warn("Servo start call failed or timed out")
            return

        response = future.result()
        if response.success:
            self.get_logger().info("MoveIt Servo started")
        else:
            self.get_logger().warn(
                f"MoveIt Servo start returned failure: {response.message}"
            )

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
            rclpy.spin_once(self, timeout_sec=0.0)
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
        input("\nPress Enter to start MoveIt Servo...")
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
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
