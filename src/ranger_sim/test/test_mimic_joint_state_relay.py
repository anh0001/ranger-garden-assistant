"""Unit tests for scripts/mimic_joint_state_relay.py (no simulator required)."""

import importlib.util
from pathlib import Path

import rclpy
from sensor_msgs.msg import JointState

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "mimic_joint_state_relay.py"
_spec = importlib.util.spec_from_file_location("mimic_joint_state_relay", SCRIPT)
relay = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(relay)


def _joint_state(names, position, velocity=(), effort=()):
    msg = JointState()
    msg.header.stamp.sec = 42
    msg.header.frame_id = "base"
    msg.name = list(names)
    msg.position = list(position)
    msg.velocity = list(velocity)
    msg.effort = list(effort)
    return msg


def test_strip_mimic_renames_mimic_joints_and_keeps_stamp():
    msg = _joint_state(
        ["piper_joint7", "piper_joint8_mimic"], [0.035, -0.035], velocity=[0.1, -0.1]
    )

    relayed = relay.strip_mimic(msg)

    assert relayed.name == ["piper_joint8"]
    assert list(relayed.position) == [-0.035]
    assert list(relayed.velocity) == [-0.1]
    assert relayed.header.stamp.sec == 42
    assert relayed.header.frame_id == "base"


def test_strip_mimic_returns_none_without_mimic_joints():
    msg = _joint_state(["piper_joint7", "piper_joint8"], [0.0, 0.0])

    assert relay.strip_mimic(msg) is None


def test_strip_mimic_leaves_unset_fields_empty():
    # effort is unset (empty) on the input, so it must stay empty rather
    # than be indexed out of range.
    msg = _joint_state(["a_mimic", "b"], [1.0, 2.0], velocity=[0.5, 0.6])

    relayed = relay.strip_mimic(msg)

    assert list(relayed.effort) == []
    assert list(relayed.velocity) == [0.5]


def test_strip_mimic_honours_custom_suffix():
    msg = _joint_state(["finger_copy", "finger"], [0.2, 0.1])

    relayed = relay.strip_mimic(msg, suffix="_copy")

    assert relayed.name == ["finger"]
    assert list(relayed.position) == [0.2]


def test_node_publishes_only_messages_with_mimic_joints():
    rclpy.init()
    node = relay.MimicJointStateRelay()
    published = []
    node._pub.publish = published.append
    try:
        node._on_joint_states(_joint_state(["piper_joint7"], [0.0]))
        node._on_joint_states(_joint_state(["piper_joint8_mimic"], [-0.01]))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    assert [msg.name for msg in published] == [["piper_joint8"]]


def test_relayed_output_is_not_relayed_again():
    # The node publishes onto the topic it reads; its own output must not
    # trigger another publish.
    first = relay.strip_mimic(_joint_state(["j8_mimic"], [0.3]))

    assert relay.strip_mimic(first) is None
