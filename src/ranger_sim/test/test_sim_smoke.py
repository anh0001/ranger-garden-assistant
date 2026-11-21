#!/usr/bin/env python3
"""
Headless smoke test for the Gazebo Fortress simulation.

Boots the sim, spawns the robot, and checks that core topics/actions are alive
and that the base steers its wheels, then drives (straight, sideways, spin).
Needs a GPU (gpu_lidar / camera sensors), so it only runs with RANGER_SIM_GPU=1:
  RANGER_SIM_GPU=1 launch_test src/ranger_sim/test/test_sim_smoke.py
  RANGER_SIM_GPU=1 RANGER_SIM_SCENE=garden launch_test src/ranger_sim/test/test_sim_smoke.py
RANGER_SIM_SCENE picks the scene of config/pick_place_scenes.yaml (default
small_house once its assets are fetched with tools/fetch_sim_assets.py, else
garden). Run it in an isolated ROS domain (e.g. ROS_DOMAIN_ID=88
ROS_LOCALHOST_ONLY=1) so it can never talk to a real robot on the network.
"""

import math
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
# The base steers before it drives (scripts/ranger_motion_adapter.py). A
# steering joint is on target within STEERING_TOLERANCE_RAD at the end of a
# drive, has arrived within STEERING_ARRIVED_RAD (the adapter releases the
# base at 0.05), and the base turns less than MAX_YAW_BEFORE_STEERED_RAD
# before the last one arrives. The steering axis is -z: joint = -heading.
STEERING_TOLERANCE_RAD = 0.1
STEERING_ARRIVED_RAD = 0.2
MAX_YAW_BEFORE_STEERED_RAD = 0.03
# Side slip: +y (PARALLEL mode) from the end of the straight drive; both
# scenes are clear for well over 0.10 m to the left there.
SIDE_SLIP_SPEED_MPS = 0.1
SIDE_SLIP_DISTANCE_M = 0.10
MAX_SIDE_SLIP_X_DRIFT_M = 0.01
MAX_SIDE_SLIP_YAW_RAD = 0.02
# Spin in place (SPINNING mode): every wheel tangent to its circle about
# base_footprint, i.e. at atan2(|x|, |y|) of its steering joint (x, y in
# base_link, ranger_complete.urdf.xacro); for a left spin the joint angle
# has the sign of x * y.
SPIN_RATE_RPS = 0.5
SPIN_ANGLE_RAD = 0.5
MAX_SPIN_DRIFT_M = 0.03
SPIN_STEERING_TARGETS = {
    name: math.copysign(math.atan2(abs(x), abs(y)), x * y)
    for name, (x, y) in zip(
        STEERING_JOINTS, [(0.25, 0.19), (0.25, -0.19), (-0.25, 0.19), (-0.23, -0.206)]
    )
}

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


def _yaw(pose):
    """Yaw (rad) of a geometry_msgs/Pose."""
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _yaw_change(start, pose):
    """Yaw of `pose` relative to `start` (rad), wrapped into [-pi, pi]."""
    return math.remainder(_yaw(pose) - _yaw(start), 2.0 * math.pi)


def _displacement(start, pose):
    """(dx, dy) from `start` to `pose`, in the body frame of `start`."""
    ex, ey = pose.position.x - start.position.x, pose.position.y - start.position.y
    c, s = math.cos(_yaw(start)), math.sin(_yaw(start))
    return c * ex + s * ey, -s * ex + c * ey


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

    def _drive(self, twist, done, timeout_sec=DRIVE_TIMEOUT_SEC):
        """
        Publish `twist` on /cmd_vel until `done(start_pose, pose)` or the timeout, then stop.

        Returns the /odom poses and, per base joint, the positions, each as
        (wall time, value) lists that start with the last sample before the
        drive. Publishes /cmd_vel: only ever run in the isolated test domain
        against the headless sim (RANGER_SIM_TEST_ENV), never with a real
        base bridged.
        """
        # grasp_manager serves /grasp/set only after unpausing the world.
        self.assertTrue(self._wait_for_service("/grasp/set"), "sim never unpaused")
        self.assertTrue(self._wait_for_topic("/odom"), "/odom not found")
        self.assertTrue(self._wait_for_topic("/gz/base_joint_states"))
        odom, joints = [], {}

        def on_joints(msg):
            now = time.time()
            for name, position in zip(msg.name, msg.position):
                joints.setdefault(name, []).append((now, position))

        subs = [
            self.node.create_subscription(
                Odometry, "/odom", lambda msg: odom.append((time.time(), msg.pose.pose)), 10
            ),
            self.node.create_subscription(
                JointState, "/gz/base_joint_states", on_joints, 10
            ),
        ]
        pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        start = time.time()
        while time.time() - start < WAIT_SEC and not (odom and joints):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(odom and joints, "no /odom or base joint states")
        del odom[:-1]
        for history in joints.values():
            del history[:-1]
        start_pose = odom[0][1]
        start = time.time()
        while time.time() - start < timeout_sec and not done(start_pose, odom[-1][1]):
            pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        for _ in range(10):
            pub.publish(Twist())  # stop
            rclpy.spin_once(self.node, timeout_sec=0.1)
        for sub in subs:
            self.node.destroy_subscription(sub)
        self.node.destroy_publisher(pub)
        return odom, joints

    def _assert_steering_near(self, joints, targets, tolerance, label):
        """Each steering joint's last position is within `tolerance` of `targets[name]`."""
        for name, target in targets.items():
            self.assertIn(name, joints, f"{name} missing from /gz/base_joint_states")
            position = joints[name][-1][1]
            self.assertLess(
                abs(position - target), tolerance,
                f"{name} at {position:.3f} rad, {label} is {target:.3f} rad",
            )

    def test_base_drives_straight_with_steering_at_zero(self):
        """Forward /cmd_vel must keep the steering at zero and not drift the base in y."""
        twist = Twist()
        twist.linear.x = DRIVE_SPEED_MPS

        odom, joints = self._drive(
            twist, lambda start, pose: _displacement(start, pose)[0] >= DRIVE_DISTANCE_M
        )

        dx, dy = _displacement(odom[0][1], odom[-1][1])
        self.assertGreaterEqual(dx, DRIVE_DISTANCE_M, f"base barely moved ({dx:.2f} m)")
        self.assertLess(abs(dy), MAX_Y_DRIFT_M, f"base drifted {dy:.3f} m in y")
        for name in STEERING_JOINTS:
            self.assertIn(name, joints, f"{name} missing from /gz/base_joint_states")
            worst = max(abs(position) for _, position in joints[name])
            self.assertLess(worst, MAX_STEERING_RAD, f"{name} swivelled to {worst:.3f} rad")

    def test_side_slip_steers_the_wheels_sideways(self):
        """Pure linear.y (PARALLEL mode) turns every wheel 90 deg, then slides the base."""
        twist = Twist()
        twist.linear.y = SIDE_SLIP_SPEED_MPS

        odom, joints = self._drive(
            twist, lambda start, pose: _displacement(start, pose)[1] >= SIDE_SLIP_DISTANCE_M
        )

        start, end = odom[0][1], odom[-1][1]
        dx, dy = _displacement(start, end)
        self.assertGreaterEqual(dy, SIDE_SLIP_DISTANCE_M, f"base barely moved ({dy:.3f} m)")
        self.assertLess(abs(dx), MAX_SIDE_SLIP_X_DRIFT_M, f"base drifted {dx:.3f} m in x")
        self.assertLess(abs(_yaw_change(start, end)), MAX_SIDE_SLIP_YAW_RAD)
        # A +y heading is a -pi/2 joint angle (steering axis -z).
        self._assert_steering_near(
            joints, {name: -math.pi / 2 for name in STEERING_JOINTS},
            STEERING_TOLERANCE_RAD, "the side-slip angle",
        )

    def test_spin_in_place_steers_the_wheels_first(self):
        """Pure angular.z (SPINNING mode) sets the wheels in an X before the base turns."""
        twist = Twist()
        twist.angular.z = SPIN_RATE_RPS

        odom, joints = self._drive(
            twist, lambda start, pose: abs(_yaw_change(start, pose)) >= SPIN_ANGLE_RAD
        )

        start, end = odom[0][1], odom[-1][1]
        self.assertGreaterEqual(abs(_yaw_change(start, end)), SPIN_ANGLE_RAD, "base barely turned")
        drift = math.hypot(*_displacement(start, end))
        self.assertLess(drift, MAX_SPIN_DRIFT_M, f"base drifted {drift:.3f} m while spinning")
        self._assert_steering_near(
            joints, SPIN_STEERING_TARGETS, STEERING_TOLERANCE_RAD, "the spin angle"
        )
        # Steer, then drive: the base has barely turned when the last wheel arrives.
        arrived = []
        for name, target in SPIN_STEERING_TARGETS.items():
            near = [
                t for t, position in joints[name]
                if abs(position - target) < STEERING_ARRIVED_RAD
            ]
            self.assertTrue(near, f"{name} never reached its spin angle")
            arrived.append(near[0])
        pose = [pose for t, pose in odom if t <= max(arrived)][-1]
        yaw = _yaw_change(start, pose)
        self.assertLess(
            abs(yaw), MAX_YAW_BEFORE_STEERED_RAD, f"base turned {yaw:.3f} rad before steering"
        )

    def test_gripper_action(self):
        """The GripperCommand action must be up (same interface as MoveIt / real bridge)."""
        self.assertTrue(
            self._wait_for_topic("/piper_gripper_controller/gripper_cmd/_action/status"),
            "/piper_gripper_controller/gripper_cmd action not found",
        )
