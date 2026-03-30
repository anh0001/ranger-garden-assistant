#!/usr/bin/env python3
"""
Headless smoke test for the Gazebo Fortress simulation.

Boots the sim, spawns the robot, and checks that core topics/actions are alive.
Run via: launch_test src/ranger_sim/test/test_sim_smoke.py
"""

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
        # Allow time for Gazebo + spawn + controllers
        TimerAction(period=15.0, actions=[launch_testing.actions.ReadyToTest()]),
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

    def _wait_for_topic(self, topic_name, timeout_sec=20.0):
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
