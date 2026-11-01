#!/usr/bin/env python3
"""
Republish gz_ros2_control mimic-joint states under their real joint names.

gz_ros2_control 0.7.x (Humble) exports the interfaces of a mimic joint with
a "_mimic" suffix, so joint_state_broadcaster publishes "piper_joint8_mimic"
instead of "piper_joint8". Without the real name MoveIt's planning-scene
monitor never sees a complete robot state (piper_joint8 is a passive joint
in the SRDF and passive joints still need a state, the same reason
base_joint_state_publisher.py exists) and robot_state_publisher never
publishes the second finger's TF.

This node listens to ``input_topic`` (the local topic of the broadcaster that
owns the mimic interfaces, see ros2_controllers_sim.yaml) and, for every
joint whose name ends in the suffix, publishes the same
position/velocity/effort on /joint_states under the stripped name with the
original stamp. Should both topics be the same, its own messages carry no
suffixed names, so the relay cannot loop.
"""

from typing import Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

MIMIC_SUFFIX = "_mimic"


def _pick(values: Sequence[float], indices: list[int], n_names: int) -> list[float]:
    """Select ``indices`` from a JointState field; empty if the field is unset."""
    if len(values) != n_names:
        return []
    return [values[i] for i in indices]


def strip_mimic(msg: JointState, suffix: str = MIMIC_SUFFIX) -> JointState | None:
    """Return only the mimic joints of ``msg``, renamed; None if there are none."""
    indices = [i for i, name in enumerate(msg.name) if name.endswith(suffix)]
    if not indices:
        return None
    n_names = len(msg.name)
    relayed = JointState()
    relayed.header = msg.header
    relayed.name = [msg.name[i][: -len(suffix)] for i in indices]
    relayed.position = _pick(msg.position, indices, n_names)
    relayed.velocity = _pick(msg.velocity, indices, n_names)
    relayed.effort = _pick(msg.effort, indices, n_names)
    return relayed


class MimicJointStateRelay(Node):
    """Relay ``<joint>_mimic`` entries of ``input_topic`` to /joint_states as ``<joint>``."""

    def __init__(self) -> None:
        super().__init__("mimic_joint_state_relay")
        self.declare_parameter("suffix", MIMIC_SUFFIX)
        self.declare_parameter("input_topic", "/joint_states")
        self._suffix = self.get_parameter("suffix").value
        input_topic = self.get_parameter("input_topic").value
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self._sub = self.create_subscription(
            JointState, input_topic, self._on_joint_states, 10
        )

    def _on_joint_states(self, msg: JointState) -> None:
        relayed = strip_mimic(msg, self._suffix)
        if relayed is not None:
            self._pub.publish(relayed)


def main() -> None:
    rclpy.init()
    node = MimicJointStateRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
