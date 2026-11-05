#!/usr/bin/env python3
"""
Headless smoke test for the Gazebo Fortress simulation.

Boots the sim, spawns the robot, and checks that core topics/actions are alive.
Needs a GPU (gpu_lidar / camera sensors), so it only runs with RANGER_SIM_GPU=1:
  RANGER_SIM_GPU=1 launch_test src/ranger_sim/test/test_sim_smoke.py
  RANGER_SIM_GPU=1 RANGER_SIM_SCENE=garden launch_test src/ranger_sim/test/test_sim_smoke.py
RANGER_SIM_SCENE picks the scene of config/pick_place_scenes.yaml (default
small_house once its assets are fetched with tools/fetch_sim_assets.py, else
garden). Run it in an isolated ROS domain (e.g. ROS_DOMAIN_ID=88
ROS_LOCALHOST_ONLY=1) so it can never talk to a real robot on the network.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing
import launch_testing.actions
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ranger_sim.srv import SetGrasp
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
import yaml

# Humble's launch_testing aborts if ReadyToTest is not emitted within 15 s
# (hard-coded), so signal readiness early and let the per-test waits absorb
# the rest of the startup (paused start, startup detach, controller
# activation after the unpause).
READY_DELAY_SEC = 10.0
_SHARE = get_package_share_directory("ranger_sim")
# The house needs the fetched third-party assets (tools/fetch_sim_assets.py);
# without them the garden is the only scene that boots.
_HOUSE_ASSETS = os.path.join(_SHARE, "models", "third_party", "aws_small_house", ".fetched")
SCENE = os.environ.get(
    "RANGER_SIM_SCENE", "small_house" if os.path.isfile(_HOUSE_ASSETS) else "garden"
)
with open(os.path.join(_SHARE, "config", "pick_place_scenes.yaml"), encoding="utf-8") as _scenes:
    GRASP_OBJECT = yaml.safe_load(_scenes)[SCENE]["object_name"]
# The house loads ~90 meshes: give its startup more time.
WAIT_SEC = 30.0 if SCENE == "garden" else 60.0

# Drive test: forward until odom reports DRIVE_DISTANCE_M (both scenes put
# the table edge 0.9 m ahead of the spawn pose, so stay well short of it).
STEERING_JOINTS = [
    "fl_steering_joint",
    "fr_steering_joint",
    "rl_steering_joint",
    "rr_steering_joint",
]
DRIVE_SPEED_MPS = 0.25  # pick_place_demo base_speed default
DRIVE_DISTANCE_M = 0.3
DRIVE_TIMEOUT_SEC = 20.0
MAX_Y_DRIFT_M = 0.01
MAX_STEERING_RAD = 0.05

# Wrist D405 colour image (config/gz_bridge.yaml, urdf/ranger_gazebo.urdf.xacro).
WRIST_IMAGE_TOPIC = "/piper/wrist_camera/color/image_raw"
WRIST_IMAGE_SIZE = (848, 480)


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
            "scene": SCENE,
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

    def test_wrist_camera_image_is_bridged(self):
        """The D405's colour image reaches ROS (the demo video's inset)."""
        self.assertTrue(
            self._wait_for_topic(WRIST_IMAGE_TOPIC), f"{WRIST_IMAGE_TOPIC} has no publisher"
        )
        images = []
        sub = self.node.create_subscription(Image, WRIST_IMAGE_TOPIC, images.append, 2)
        start = time.time()
        while time.time() - start < WAIT_SEC and not images:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        self.assertTrue(images, f"no image on {WRIST_IMAGE_TOPIC}")
        self.assertEqual((images[0].width, images[0].height), WRIST_IMAGE_SIZE)
        self.assertEqual(images[0].encoding, "rgb8")

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
        request.object = GRASP_OBJECT
        request.attach = attach
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        return future.result()

    def test_grasp_round_trip_on_the_scene_object(self):
        """Attach and release the scene's object through /grasp/set against the real plugin."""
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

    def test_gz_base_joint_states_topic(self):
        """Gazebo's JointStatePublisher must reach ROS for the base joint relay."""
        self.assertTrue(
            self._wait_for_topic("/gz/base_joint_states"),
            "/gz/base_joint_states topic not found",
        )

    def test_base_drives_straight_with_locked_steering(self):
        """Forward /cmd_vel must not swivel the steering joints or drift the base in y."""
        # Publishes /cmd_vel: only ever run in the isolated test domain against
        # the headless sim (RANGER_SIM_TEST_ENV), never with a real base bridged.
        # grasp_manager serves /grasp/set only after unpausing the world.
        self.assertTrue(self._wait_for_service("/grasp/set"), "sim never unpaused")
        self.assertTrue(self._wait_for_topic("/odom"), "/odom not found")
        self.assertTrue(self._wait_for_topic("/gz/base_joint_states"))
        odom, steering, worst = [], {}, {}

        def on_joints(msg):
            for name, position in zip(msg.name, msg.position):
                steering[name] = position
                worst[name] = max(worst.get(name, 0.0), abs(position))

        subs = [
            self.node.create_subscription(
                Odometry, "/odom", lambda msg: odom.append(msg.pose.pose), 10
            ),
            self.node.create_subscription(
                JointState, "/gz/base_joint_states", on_joints, 10
            ),
        ]
        pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        start = time.time()
        while time.time() - start < WAIT_SEC and not (odom and steering):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(odom and steering, "no /odom or base joint states")
        start_pose = odom[-1].position
        twist = Twist()
        twist.linear.x = DRIVE_SPEED_MPS
        start = time.time()
        while (
            time.time() - start < DRIVE_TIMEOUT_SEC
            and odom[-1].position.x - start_pose.x < DRIVE_DISTANCE_M
        ):
            pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        for _ in range(10):
            pub.publish(Twist())  # stop
            rclpy.spin_once(self.node, timeout_sec=0.1)
        for sub in subs:
            self.node.destroy_subscription(sub)
        self.node.destroy_publisher(pub)

        dx = odom[-1].position.x - start_pose.x
        dy = odom[-1].position.y - start_pose.y
        self.assertGreaterEqual(dx, DRIVE_DISTANCE_M, f"base barely moved ({dx:.2f} m)")
        self.assertLess(abs(dy), MAX_Y_DRIFT_M, f"base drifted {dy:.3f} m in y")
        for name in STEERING_JOINTS:
            self.assertIn(name, worst, f"{name} missing from /gz/base_joint_states")
            self.assertLess(
                worst[name], MAX_STEERING_RAD, f"{name} swivelled to {worst[name]:.3f} rad"
            )

    def test_gripper_action(self):
        """The GripperCommand action must be up (same interface as MoveIt / real bridge)."""
        self.assertTrue(
            self._wait_for_topic("/piper_gripper_controller/gripper_cmd/_action/status"),
            "/piper_gripper_controller/gripper_cmd action not found",
        )
