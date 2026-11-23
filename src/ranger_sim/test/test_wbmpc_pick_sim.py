#!/usr/bin/env python3
"""
Headless Gazebo test of the whole-body MPC pick from a wrong standoff (docs/sim_wbmpc_pick.md).

Boots sim_bringup (handover_gate:=true, MoveIt on, no Nav2) and runs
pick_place_demo.launch.py execute:=true approach:=mpc approach_only:=true
with the base's first leg ending APPROACH_ERROR off the pick standoff: the
MPC picks the object from there (pre-grasp, slide in, attach, close, lift),
sets it back and the base returns. Checks that the approach result line
reports aligned=yes, that the demo ends with its completion line, that the
object went attached and back to detached, that the gate ends in NAVIGATE,
that whole_body_mpc never faulted and that the base stayed in the scene's
mpc_base_box.

Needs a GPU (gpu_lidar / camera sensors), the fetched house assets and
casadi, so it only runs with RANGER_SIM_GPU=1:
  RANGER_SIM_GPU=1 launch_test src/ranger_sim/test/test_wbmpc_pick_sim.py
Run it in an isolated ROS domain (e.g. ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1)
so it can never talk to a real robot on the network.
"""

import os
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml

READY_DELAY_SEC = 10.0  # Humble's launch_testing wants ReadyToTest within 15 s
# MoveIt is loaded ~9 s and the world unpaused ~14 s after the launch starts
# (house); the demo itself waits up to 20 s more for /move_action.
DEMO_START_SEC = 20.0
RESULT_TIMEOUT_SEC = 240.0  # bringup + prepare + first leg + hand-over + MPC pick
DEMO_END_TIMEOUT_SEC = 150.0  # lift, set back, hand back, carry, return, 'home'
# 8 cm short, 6 cm left and 4 deg off the standoff: within Nav2's goal
# tolerances (config/nav2_sim.yaml), the stop the error stands in for.
APPROACH_ERROR = "[-0.08, 0.06, 0.07]"
BOX_MARGIN_M = 0.005  # the sim's kinematic base follows commands to the millimetre
SCENE = "small_house"
DEMO_NODE = "pick_place_demo"
COMPLETE = "Pick-and-place sequence complete (executed)."
RESULT_PREFIX = "approach result: mode=mpc "

_SHARE = Path(get_package_share_directory("ranger_sim"))
_CFG = yaml.safe_load((_SHARE / "config" / "pick_place_scenes.yaml").read_text())[SCENE]
BOX = _CFG["mpc_base_box"]
OBJECT = _CFG["object_name"]
LATCHED_QOS = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
)


@unittest.skipIf(
    os.environ.get("RANGER_SIM_GPU") != "1",
    "headless Gazebo needs a GPU; set RANGER_SIM_GPU=1 to run",
)
def generate_test_description():
    """Launch the sim with the gate, then the MPC pick (approach_only) from a wrong standoff."""
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(_SHARE / "launch" / "sim_bringup.launch.py")),
        launch_arguments={
            "scene": SCENE,
            "headless": "true",
            "handover_gate": "true",
            "launch_moveit": "true",
            "launch_nav": "false",
            "use_rviz": "false",
        }.items(),
    )
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(_SHARE / "launch" / "pick_place_demo.launch.py")),
        launch_arguments={
            "scene": SCENE,
            "execute": "true",
            "approach": "mpc",
            "approach_only": "true",
            "approach_error": APPROACH_ERROR,
        }.items(),
    )
    return launch.LaunchDescription([
        sim_bringup,
        TimerAction(period=DEMO_START_SEC, actions=[demo]),
        TimerAction(period=READY_DELAY_SEC, actions=[launch_testing.actions.ReadyToTest()]),
    ])


class TestWbmpcPick(unittest.TestCase):
    """The MPC pick in Gazebo, observed from /odom, /rosout and the state topics."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("wbmpc_pick_sim_test")
        cls.states, cls.modes, cls.poses, cls.grasp, cls.demo_log = [], [], [], [], []
        sub = cls.node.create_subscription
        sub(String, "/whole_body_mpc/state", lambda m: cls.states.append(m.data), LATCHED_QOS)
        sub(String, "/base_handover_gate/mode", lambda m: cls.modes.append(m.data), LATCHED_QOS)
        sub(String, f"/grasp/{OBJECT}/state", lambda m: cls._on_grasp(m.data), 10)
        sub(Odometry, "/odom", lambda m: cls.poses.append(
            (m.pose.pose.position.x, m.pose.pose.position.y)), 50)
        sub(Log, "/rosout", lambda m: m.name == DEMO_NODE and cls.demo_log.append(m.msg), 100)

    @classmethod
    def _on_grasp(cls, state):
        if not cls.grasp or cls.grasp[-1] != state:
            cls.grasp.append(state)

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

    def _result(self):
        return next((line for line in self.demo_log if line.startswith(RESULT_PREFIX)), None)

    def test_mpc_picks_from_a_wrong_standoff_and_hands_back(self):
        # One test that spins the whole time: every check needs the full record.
        self.assertTrue(self._spin_until(lambda: self._result(), RESULT_TIMEOUT_SEC),
                        f"no approach result: {self.demo_log[-5:]} {self.states[-3:]}")
        self.assertTrue(self._result().endswith("aligned=yes"), self._result())
        self.assertTrue(self._spin_until(lambda: COMPLETE in self.demo_log, DEMO_END_TIMEOUT_SEC),
                        f"the demo did not complete: {self.demo_log[-5:]}")
        self.assertEqual(self.modes[-1], "NAVIGATE", self.modes)
        self.assertIn("MANIPULATE", self.modes)
        self.assertIn("REACHED", self.states)
        self.assertFalse(any(s.startswith("FAULT") for s in self.states), self.states)
        self.assertEqual(self.grasp[-2:], ["attached", "detached"], self.grasp)
        x_min, y_min, x_max, y_max = BOX
        for x, y in self.poses:
            self.assertTrue(x_min - BOX_MARGIN_M <= x <= x_max + BOX_MARGIN_M, (x, y))
            self.assertTrue(y_min - BOX_MARGIN_M <= y <= y_max + BOX_MARGIN_M, (x, y))
        # The base was not driven back to the standoff before the pick.
        self.assertNotIn("correct the standoff", " ".join(self.demo_log))
