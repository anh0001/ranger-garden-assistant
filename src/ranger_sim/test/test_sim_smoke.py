#!/usr/bin/env python3
"""
Headless smoke test for the Gazebo Fortress simulation.

Boots the sim, spawns the robot, and checks that core topics/actions are alive.
Needs a GPU (gpu_lidar / camera sensors), so it only runs with RANGER_SIM_GPU=1:
  RANGER_SIM_GPU=1 launch_test src/ranger_sim/test/test_sim_smoke.py
Run it in an isolated ROS domain (e.g. ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1)
so it can never talk to a real robot on the network.
"""

import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import rclpy
from rclpy.node import Node
from ranger_sim.srv import SetGrasp
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

# Humble's launch_testing aborts if ReadyToTest is not emitted within 15 s
# (hard-coded), so signal readiness early and let the per-test waits absorb
# the rest of the startup (paused start, startup detach, controller
# activation after the unpause).
READY_DELAY_SEC = 10.0
WAIT_SEC = 30.0


@unittest.skipIf(
    os.environ.get("RANGER_SIM_GPU") != "1",
    "headless Gazebo needs a GPU; set RANGER_SIM_GPU=1 to run",
)
def generate_test_description():
    """Launch the full sim stack in headless mode."""
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ranger_sim"), "launch", "sim_bringup.launch.py"
            ])
        ),
        launch_arguments={
            "headless": "true",
            "launch_moveit": "false",
            "launch_nav": "false",
            "use_rviz": "false",
        }.items(),
    )

    return launch.LaunchDescription([
        sim_bringup,
        # Gazebo + spawn have started; the tests wait for the rest.
        TimerAction(period=READY_DELAY_SEC, actions=[launch_testing.actions.ReadyToTest()]),
    ])


class TestSimSmoke(unittest.TestCase):
    """Verify core ROS interfaces are alive after sim bringup."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("sim_smoke_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for_topic(self, topic_name, timeout_sec=WAIT_SEC):
        """Wait until a topic has at least one publisher."""
        start = time.time()
        while time.time() - start < timeout_sec:
            topics = self.node.get_topic_names_and_types()
            for name, _ in topics:
                if name == topic_name:
                    info = self.node.get_publishers_info_by_topic(name)
                    if len(info) > 0:
                        return True
            rclpy.spin_once(self.node, timeout_sec=0.5)
        return False

    def _wait_for_service(self, service_name, timeout_sec=WAIT_SEC):
        """Wait until a service is advertised."""
        start = time.time()
        while time.time() - start < timeout_sec:
            names = [name for name, _ in self.node.get_service_names_and_types()]
            if service_name in names:
                return True
            rclpy.spin_once(self.node, timeout_sec=0.5)
        return False

    def test_clock_topic(self):
        """Gazebo must publish /clock."""
        self.assertTrue(
            self._wait_for_topic("/clock"),
            "/clock topic not found",
        )

    def test_joint_states_topic(self):
        """/joint_states must be published by joint_state_broadcaster."""
        self.assertTrue(
            self._wait_for_topic("/joint_states"),
            "/joint_states topic not found",
        )

    def test_robot_description(self):
        """/robot_description must be published."""
        self.assertTrue(
            self._wait_for_topic("/robot_description"),
            "/robot_description topic not found",
        )

    def test_tf_topic(self):
        """/tf must be published."""
        self.assertTrue(
            self._wait_for_topic("/tf"),
            "/tf topic not found",
        )

    def test_odom_topic(self):
        """/odom must be published by the DiffDrive bridge."""
        self.assertTrue(
            self._wait_for_topic("/odom"),
            "/odom topic not found",
        )

    def test_scan_topic(self):
        """/scan (lidar) must be published."""
        self.assertTrue(
            self._wait_for_topic("/scan"),
            "/scan topic not found",
        )

    def test_imu_topic(self):
        """/imu/data must be published."""
        self.assertTrue(
            self._wait_for_topic("/imu/data"),
            "/imu/data topic not found",
        )

    def test_camera_topic(self):
        """/camera/image_raw must be published."""
        self.assertTrue(
            self._wait_for_topic("/camera/image_raw"),
            "/camera/image_raw topic not found",
        )

    def test_grasp_service(self):
        """/grasp/set must be served once grasp_manager finished its startup detach."""
        self.assertTrue(
            self._wait_for_service("/grasp/set"),
            "/grasp/set service not found (grasp_manager)",
        )

    def _collect_joint_states(self, required, timeout_sec=WAIT_SEC):
        """Merge /joint_states messages until every name in `required` is seen."""
        seen = {}
        sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            lambda msg: seen.update(zip(msg.name, msg.position)),
            20,
        )
        start = time.time()
        while time.time() - start < timeout_sec and not required <= seen.keys():
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        return seen

    def test_joint_states_use_real_joint_names(self):
        """piper_joint8 is relayed from the mimic interface; no '*_mimic' names leak."""
        seen = self._collect_joint_states({"piper_joint7", "piper_joint8"})
        self.assertIn("piper_joint8", seen, "piper_joint8 missing (mimic relay)")
        self.assertFalse(
            [name for name in seen if name.endswith("_mimic")],
            "suffixed mimic joint names leaked into /joint_states",
        )

    def test_arm_starts_inside_joint_limits(self):
        """Joint2/3 must start off their 0 limit or DART ignores their servo."""
        seen = self._collect_joint_states({"piper_joint2", "piper_joint3"})
        self.assertGreater(seen.get("piper_joint2", -1.0), 0.0)
        self.assertLess(seen.get("piper_joint3", 1.0), 0.0)

    def test_simulation_is_running(self):
        """/clock must advance: grasp_manager unpaused the world after its detach."""
        stamps = []
        sub = self.node.create_subscription(
            Clock,
            "/clock",
            lambda msg: stamps.append(msg.clock.sec + msg.clock.nanosec * 1e-9),
            10,
        )
        start = time.time()
        while time.time() - start < WAIT_SEC and (not stamps or stamps[-1] - stamps[0] < 0.5):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        self.assertTrue(stamps, "no /clock message")
        self.assertGreaterEqual(
            stamps[-1] - stamps[0], 0.5, "simulation time is not advancing"
        )

    def _call_grasp(self, client, attach):
        request = SetGrasp.Request()
        request.object = "cup"
        request.attach = attach
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        return future.result()

    def test_grasp_round_trip_on_cup(self):
        """Attach and release the cup through /grasp/set against the real plugin."""
        self.assertTrue(self._wait_for_service("/grasp/set"), "/grasp/set not found")
        client = self.node.create_client(SetGrasp, "/grasp/set")
        self.assertTrue(client.wait_for_service(timeout_sec=WAIT_SEC))
        attached = self._call_grasp(client, True)
        released = self._call_grasp(client, False)
        self.node.destroy_client(client)
        self.assertIsNotNone(attached)
        self.assertTrue(attached.success, attached.message)
        self.assertIsNotNone(released)
        self.assertTrue(released.success, released.message)

    def test_gripper_action(self):
        """The GripperCommand action must be up (same interface as MoveIt / real bridge)."""
        self.assertTrue(
            self._wait_for_topic("/piper_gripper_controller/gripper_cmd/_action/status"),
            "/piper_gripper_controller/gripper_cmd action not found",
        )
