#!/usr/bin/env python3
"""
Publish the Ranger base wheel/steering joint states on /joint_states.

In Gazebo Fortress the PiPER arm runs through gz_ros2_control, so the
joint_state_broadcaster only publishes the arm/gripper joints. The base is
driven kinematically (VelocityControl); its wheel and steering joints are
reported by Gazebo's JointStatePublisher (ranger_gazebo.urdf.xacro) and
bridged to `input_topic` (/gz/base_joint_states). This node relays them
under the URDF joint names so robot_state_publisher / MoveIt / RViz see a
complete, live robot state.

Until the first sim message arrives it publishes zeros: without a state for
these passive joints MoveIt's planning_scene_monitor never gets a complete
robot state ("The complete state of the robot is not yet known ...
fl_wheel, fr_wheel, ...") and refuses to plan for the piper_arm group.
robot_state_publisher and MoveIt merge joint states by name across
publishers, so this composes cleanly with the controller output.
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
    """Relay the base joints of `input_topic` to /joint_states at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("base_joint_state_publisher")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("input_topic", "/gz/base_joint_states")
        rate = self.get_parameter("publish_rate").value
        input_topic = self.get_parameter("input_topic").value
        # name -> (position, velocity); replaced as a whole per message.
        self._latest: dict[str, tuple[float, float]] = {}
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, input_topic, self._on_gz, 10)
        # Publish on a timer, not on receipt, so the rate does not depend on
        # the simulation's real-time factor.
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Relaying {len(BASE_JOINTS)} base joint states from {input_topic} "
            f"at {rate} Hz (zeros until the first message)"
        )

    def _on_gz(self, msg: JointState) -> None:
        # The gz message may carry no velocities.
        if len(msg.velocity) == len(msg.name):
            velocities = list(msg.velocity)
        else:
            velocities = [0.0] * len(msg.name)
        self._latest = {
            name: (position, velocity)
            for name, position, velocity in zip(msg.name, msg.position, velocities)
            if name in BASE_JOINTS
        }

    def _tick(self) -> None:
        latest = self._latest
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = BASE_JOINTS
        msg.position = [latest.get(name, (0.0, 0.0))[0] for name in BASE_JOINTS]
        msg.velocity = [latest.get(name, (0.0, 0.0))[1] for name in BASE_JOINTS]
        msg.effort = [0.0] * len(BASE_JOINTS)
        self._pub.publish(msg)


def main() -> None:
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
