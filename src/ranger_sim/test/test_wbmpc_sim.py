#!/usr/bin/env python3
"""
Headless Gazebo test of the whole-body MPC reach with the base handover gate.

Boots sim_bringup (handover_gate:=true, MoveIt on, no Nav2) and runs
wbmpc_reach_demo (execute) with whole_body_mpc: carry pose, hand the base
and the arm to the MPC, reach the default target, hold, hand back. Checks
that the MPC reports REACHED, that the base slid (> 0.15 m) without turning
(< 0.02 rad) and stayed in the configured box, that the arm stayed inside
its joint limits and that the gate is back in NAVIGATE when the demo ends.

Needs a GPU (gpu_lidar / camera sensors) and casadi, so it only runs with
RANGER_SIM_GPU=1:
  RANGER_SIM_GPU=1 launch_test src/ranger_sim/test/test_wbmpc_sim.py
Run it in an isolated ROS domain (e.g. ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1)
so it can never talk to a real robot on the network.
"""

import importlib.util
import math
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
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from urdf_parser_py.urdf import URDF
import xacro
import yaml

READY_DELAY_SEC = 10.0  # Humble's launch_testing wants ReadyToTest within 15 s
# MoveIt is loaded ~9 s and the world unpaused ~14 s after the launch starts
# (house); the demo itself waits up to 20 s more for /move_action.
REACH_START_SEC = 20.0
REACHED_TIMEOUT_SEC = 240.0  # bringup + carry pose + hand-over + reach
DEMO_END_TIMEOUT_SEC = 90.0  # hold + hand back + carry pose
MIN_BASE_TRAVEL_M = 0.15
MAX_YAW_CHANGE_RAD = 0.02
BOX_MARGIN_M = 0.005  # the sim's kinematic base follows commands to the millimetre
JOINT_LIMIT_MARGIN_RAD = 0.01
COMPLETE = "Whole-body MPC reach complete (executed)."

_SHARE = Path(get_package_share_directory("ranger_sim"))
_HOUSE_ASSETS = _SHARE / "models" / "third_party" / "aws_small_house" / ".fetched"
SCENE = os.environ.get("RANGER_SIM_SCENE", "small_house" if _HOUSE_ASSETS.is_file() else "garden")
BOX = yaml.safe_load((_SHARE / "config" / "wbmpc.yaml").read_text())[
    "whole_body_mpc"]["ros__parameters"]["base_box"]
ARM_JOINTS = [f"piper_joint{i}" for i in range(1, 7)]
LATCHED_QOS = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
)

_spec = importlib.util.spec_from_file_location(
    "wbmpc_reach_common", _SHARE / "launch" / "wbmpc_reach_common.py")
common = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(common)


@unittest.skipIf(
    os.environ.get("RANGER_SIM_GPU") != "1",
    "headless Gazebo needs a GPU; set RANGER_SIM_GPU=1 to run",
)
def generate_test_description():
    """Launch the sim with the gate, then the MPC node and the demo (execute)."""
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
    mpc = common.mpc_node("true")
    demo = common.demo_node(True, "true")
    return launch.LaunchDescription([
        *common.reach_arguments(),
        sim_bringup,
        TimerAction(period=REACH_START_SEC, actions=[mpc, demo]),
        TimerAction(period=READY_DELAY_SEC, actions=[launch_testing.actions.ReadyToTest()]),
    ]), {"demo": demo}


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TestWbmpcReach(unittest.TestCase):
    """The reach in Gazebo, observed from /odom, /joint_states, /rosout and the state topics."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("wbmpc_sim_test")
        cls.states, cls.modes, cls.poses, cls.joints, cls.demo_log = [], [], [], [], []
        sub = cls.node.create_subscription
        sub(String, "/whole_body_mpc/state", lambda m: cls.states.append(m.data), LATCHED_QOS)
        sub(String, "/base_handover_gate/mode", lambda m: cls.modes.append(m.data), LATCHED_QOS)
        sub(Odometry, "/odom", lambda m: cls.poses.append(
            (m.pose.pose.position.x, m.pose.pose.position.y, _yaw(m.pose.pose.orientation))), 50)
        sub(JointState, "/joint_states", lambda m: cls.joints.append(
            {n: p for n, p in zip(m.name, m.position) if n in ARM_JOINTS}), 50)
        sub(Log, "/rosout", lambda m: m.name == common.DEMO_NODE_NAME
            and cls.demo_log.append(m.msg), 100)

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

    def test_reach_slides_the_base_and_hands_back(self, proc_output, demo):
        # One test that spins the whole time: every check needs the full record.
        self.assertTrue(self._spin_until(lambda: "REACHED" in self.states, REACHED_TIMEOUT_SEC),
                        f"whole_body_mpc never reported REACHED: {self.states}")
        self.assertTrue(self._spin_until(lambda: COMPLETE in self.demo_log, DEMO_END_TIMEOUT_SEC),
                        f"the demo did not complete: {self.demo_log[-5:]}")
        proc_output.assertWaitFor(COMPLETE, process=demo, timeout=10.0)
        self.assertEqual(self.modes[-1], "NAVIGATE", self.modes)
        self.assertIn("MANIPULATE", self.modes)
        self.assertFalse(any(s.startswith("FAULT") for s in self.states), self.states)
        self._check_base()
        self._check_arm()

    def _check_base(self):
        start = self.poses[0]
        travel = max(math.hypot(x - start[0], y - start[1]) for x, y, _ in self.poses)
        turn = max(abs(math.remainder(yaw - start[2], 2 * math.pi)) for _, _, yaw in self.poses)
        self.assertGreater(travel, MIN_BASE_TRAVEL_M)
        self.assertLess(turn, MAX_YAW_CHANGE_RAD)
        x_min, y_min, x_max, y_max = BOX
        for x, y, _ in self.poses:
            self.assertTrue(x_min - BOX_MARGIN_M <= x <= x_max + BOX_MARGIN_M, (x, y))
            self.assertTrue(y_min - BOX_MARGIN_M <= y <= y_max + BOX_MARGIN_M, (x, y))

    def _check_arm(self):
        robot = URDF.from_xml_string(
            xacro.process_file(str(_SHARE / "urdf" / "ranger_gazebo.urdf.xacro")).toxml())
        for name in ARM_JOINTS:
            limit = robot.joint_map[name].limit
            seen = [sample[name] for sample in self.joints if name in sample]
            self.assertTrue(seen, f"no {name} on /joint_states")
            self.assertGreaterEqual(min(seen), limit.lower - JOINT_LIMIT_MARGIN_RAD, name)
            self.assertLessEqual(max(seen), limit.upper + JOINT_LIMIT_MARGIN_RAD, name)
