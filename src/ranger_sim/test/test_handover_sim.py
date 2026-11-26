#!/usr/bin/env python3
"""
Headless Gazebo test of one handover baseline episode (docs/sim_handover_baseline.md).

Boots sim_bringup in the house (handover_gate:=true, MoveIt on, no Nav2)
and runs handover_baseline.launch.py execute:=true use_nav:=false with the
default person (facing the table, left hand, reacting in 0.5-1.0 s): the
scripted pick, away, the standoff, the hand-over to the whole-body MPC, the
offer, the take, the release, the retreat. Checks that the phase reaches
DONE: handover, that the object went attached then detached, that the gate
ends in NAVIGATE, that whole_body_mpc never faulted and that the ground-truth
footprint clearance to the person never fell under stop_distance.

Needs a GPU (gpu_lidar / camera sensors), the fetched house assets and
casadi, so it only runs with RANGER_SIM_GPU=1:
  RANGER_SIM_GPU=1 launch_test src/ranger_sim/test/test_handover_sim.py
Run it in an isolated ROS domain (e.g. ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1)
so it can never talk to a real robot on the network.
"""

import importlib.util
import math
import os
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped
import launch
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml

READY_DELAY_SEC = 10.0  # Humble's launch_testing wants ReadyToTest within 15 s
DEMO_START_SEC = 20.0  # MoveIt loaded, world unpaused (house)
EPISODE_TIMEOUT_SEC = 540.0  # pick ~60 s, away, transit, wait, MPC, take, retreat
SCENE = "small_house"

_SHARE = Path(get_package_share_directory("ranger_sim"))
_CFG = yaml.safe_load((_SHARE / "config" / "pick_place_scenes.yaml").read_text())[SCENE]
OBJECT = _CFG["object_name"]
LATCHED_QOS = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
)


def _core():
    # Installed next to the nodes under lib/ranger_sim.
    path = Path(get_package_prefix("ranger_sim")) / "lib" / "ranger_sim" / "handover_core.py"
    spec = importlib.util.spec_from_file_location("handover_core", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


core = _core()
CFG = core.default_config()


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


@unittest.skipIf(
    os.environ.get("RANGER_SIM_GPU") != "1",
    "headless Gazebo needs a GPU; set RANGER_SIM_GPU=1 to run",
)
def generate_test_description():
    """Launch the house sim with the gate, then one baseline episode."""
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(_SHARE / "launch" / "sim_bringup.launch.py")),
        launch_arguments={
            "scene": SCENE, "headless": "true", "handover_gate": "true",
            "launch_moveit": "true", "launch_nav": "false", "use_rviz": "false",
        }.items(),
    )
    episode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(_SHARE / "launch" / "handover_baseline.launch.py")),
        launch_arguments={"scene": SCENE, "execute": "true", "use_nav": "false",
                          "results_file": "/tmp/ranger_sim_handover_sim_test.csv"}.items(),
    )
    return launch.LaunchDescription([
        sim_bringup,
        TimerAction(period=DEMO_START_SEC, actions=[episode]),
        TimerAction(period=READY_DELAY_SEC, actions=[launch_testing.actions.ReadyToTest()]),
    ])


class TestHandoverEpisode(unittest.TestCase):
    """One episode, observed from the phase, state, grasp, odom and ground-truth topics."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("handover_sim_test")
        cls.phases, cls.states, cls.modes, cls.safety, cls.grasp = [], [], [], [], []
        cls.clearances = []
        cls.person = None
        sub = cls.node.create_subscription
        sub(String, "/handover_baseline/phase", lambda m: cls.phases.append(m.data), LATCHED_QOS)
        sub(String, "/handover_safety/state", lambda m: cls.safety.append(m.data), LATCHED_QOS)
        sub(String, "/whole_body_mpc/state", lambda m: cls.states.append(m.data), LATCHED_QOS)
        sub(String, "/base_handover_gate/mode", lambda m: cls.modes.append(m.data), LATCHED_QOS)
        sub(String, f"/grasp/{OBJECT}/state", lambda m: cls._on_grasp(m.data), 10)
        sub(PoseStamped, "/person/ground_truth/body_pose", cls._on_person, 10)
        sub(Odometry, "/odom", cls._on_odom, 50)

    @classmethod
    def _on_grasp(cls, state):
        if not cls.grasp or cls.grasp[-1] != state:
            cls.grasp.append(state)

    @classmethod
    def _on_person(cls, msg):
        p = msg.pose.position
        cls.person = core.PersonState(p.x, p.y, _yaw(msg.pose.orientation), (0.0, 0.0, 0.0),
                                      False, False, 0.0)

    @classmethod
    def _on_odom(cls, msg):
        p = msg.pose.pose.position
        # From TRANSIT on: the pick slot is next to the person by design.
        if cls.person is not None and cls.phases and cls.phases[-1] not in ("PICK", "AWAY"):
            pose = (p.x, p.y, _yaw(msg.pose.pose.orientation))
            cls.clearances.append(core.footprint_clearance(pose, CFG.footprint, cls.person, CFG))

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def _spin_until(self, done, timeout_sec):
        """Spin (collecting every topic) until done() or the timeout."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not done():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return done()

    def _ended(self):
        return bool(self.phases) and self.phases[-1].startswith(("DONE: ", "FAILED: "))

    def test_one_episode_hands_the_object_over(self):
        # One test that spins the whole time: every check needs the full record.
        self.assertTrue(self._spin_until(self._ended, EPISODE_TIMEOUT_SEC),
                        f"the episode did not end: {self.phases[-5:]} {self.safety[-3:]}")
        self.assertEqual(self.phases[-1], "DONE: handover", (self.phases, self.safety[-5:]))
        self._spin_until(lambda: self.modes and self.modes[-1] == "NAVIGATE", 10.0)
        self.assertEqual(self.modes[-1], "NAVIGATE", self.modes)
        self.assertIn("MANIPULATE", self.modes)
        self.assertIn("REACHED", self.states)
        self.assertFalse(any(s.startswith("FAULT") for s in self.states), self.states)
        self.assertEqual(self.grasp[-2:], ["attached", "detached"], self.grasp)
        self.assertTrue(self.clearances, "no ground-truth clearance sample")
        self.assertGreaterEqual(min(self.clearances), CFG.stop_distance, min(self.clearances))
