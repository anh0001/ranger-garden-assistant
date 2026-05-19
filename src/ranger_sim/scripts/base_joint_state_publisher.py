#!/usr/bin/env python3
"""
Publish static (zero) joint states for the Ranger base wheel/steering joints.

In Gazebo Fortress the PiPER arm runs through gz_ros2_control, so the
joint_state_broadcaster only publishes the 8 arm/gripper joints. The Ranger
base is driven by the DiffDrive plugin, which does not emit wheel/steering
joint states. Without them, MoveIt's planning_scene_monitor never gets a
complete robot state ("The complete state of the robot is not yet known ...
fl_wheel, fr_wheel, ...") and refuses to plan for the piper_arm group.

This node fills that gap by publishing the passive base joints at 0.0 on
/joint_states. robot_state_publisher and MoveIt merge joint states by name
across publishers, so this composes cleanly with the controller output.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Passive base joints declared in the URDF/SRDF that no controller publishes.
BASE_JOINTS = [
    "fl_wheel",
    "fr_wheel",
    "rl_wheel",
    "rr_wheel",
    "fl_steering_joint",
    "fr_steering_joint",
    "rl_steering_joint",
    "rr_steering_joint",
]


class BaseJointStatePublisher(Node):
    def __init__(self):
        super().__init__("base_joint_state_publisher")
        self.declare_parameter("publish_rate", 30.0)
        rate = self.get_parameter("publish_rate").value
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Publishing {len(BASE_JOINTS)} static base joint states at {rate} Hz"
        )

    def _tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = BASE_JOINTS
        msg.position = [0.0] * len(BASE_JOINTS)
        msg.velocity = [0.0] * len(BASE_JOINTS)
        msg.effort = [0.0] * len(BASE_JOINTS)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = BaseJointStatePublisher()
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
