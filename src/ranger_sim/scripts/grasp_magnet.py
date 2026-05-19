#!/usr/bin/env python3
"""
Pose-follow "virtual attach" grasp helper for Gazebo Fortress.

A simple position-controlled gripper cannot hold an object by friction in
Fortress, and the DetachableJoint system plugin starts attached (dragging
the object off the table before any ROS node can detach) and its trigger /
state topics did not bridge reliably. This node implements a deterministic
kinematic grasp instead:

  - Subscribes std_msgs/Bool on /gripper/grasp  (True = hold, False = release)
  - While holding: at a fixed rate it reads the gripper link pose in the
    Gazebo world frame (from the bridged /gz_world_poses topic — the same
    source/frame as the cup, so no ROS-odom vs gz-world mismatch) and
    teleports the target model to follow it (with a grasp offset) via the
    Gazebo /world/<world>/set_pose service.
  - On release: stops following; the object stays where it was last placed.

Trade-off: a kinematic "magnet" grasp, not a physics joint — chosen because
reliably picking the object matters more than contact realism for this
scripted sim demo.
"""

import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


def _rotate_vec_by_quat(v, q):
    """Rotate vector v=(x,y,z) by quaternion q=(x,y,z,w)."""
    vx, vy, vz = v
    qx, qy, qz, qw = q
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    # v' = v + qw * t + cross(q.xyz, t)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


def _quat_mul(a, b):
    """Hamilton product a*b, quaternions as (x,y,z,w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _compose(parent, child):
    """Compose parent (world<-A) with child (A<-B); each is
    (trans=(x,y,z), quat=(x,y,z,w)). Returns world<-B as (trans, quat)."""
    pt, pq = parent
    ct, cq = child
    rx, ry, rz = _rotate_vec_by_quat(ct, pq)
    return (
        (pt[0] + rx, pt[1] + ry, pt[2] + rz),
        _quat_mul(pq, cq),
    )


class GraspMagnet(Node):
    def __init__(self):
        super().__init__("grasp_magnet")
        p = self.declare_parameter
        p("world_name", "garden_world")
        p("robot_model", "ranger")        # top-level model (world-absolute)
        p("gripper_link", "piper_link6")  # pose is relative to robot_model
        p("object_model", "cup")
        # Offset of the object relative to the gripper link, in link frame.
        p("grasp_offset", [0.0, 0.0, -0.06])
        p("rate_hz", 20.0)

        self.world = self.get_parameter("world_name").value
        self.robot = self.get_parameter("robot_model").value
        self.link = self.get_parameter("gripper_link").value
        self.obj = self.get_parameter("object_model").value
        self.offset = list(self.get_parameter("grasp_offset").value)
        self._service = f"/world/{self.world}/set_pose"
        self._gz = "gz" if self._have("gz") else "ign"

        self._held = False
        self._last = None  # last commanded (x,y,z) — skip tiny re-poses
        # dynamic_pose/info gives the robot model pose in world, but child
        # link poses RELATIVE to the model — compose them for world pose.
        self._link_rel = None   # ranger <- piper_link6
        self._robot_w = None    # world  <- ranger

        # set_pose shells out (blocking). Run it on a worker thread so the
        # ROS callbacks/timer never stall — the timer just posts the latest
        # target; the worker applies the most recent one (coalescing).
        self._pending = None
        self._wlock = threading.Lock()
        self._wevt = threading.Event()
        self._stop = False
        self._worker = threading.Thread(target=self._pose_worker, daemon=True)
        self._worker.start()

        # World-frame poses of all moving entities, same source/frame as
        # the cup we reposition (avoids any ROS-odom vs gz-world mismatch).
        self.create_subscription(
            TFMessage, "/gz_world_poses", self._poses_cb, 20)
        self.create_subscription(Bool, "/gripper/grasp", self._cmd_cb, 10)
        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"grasp_magnet ready: follows {self.link} -> model '{self.obj}' "
            f"via {self._service}"
        )

    @staticmethod
    def _have(tool):
        try:
            subprocess.run([tool, "--version"], check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except (FileNotFoundError, subprocess.SubprocessError):
            return False

    @staticmethod
    def _tf_to_pair(tf):
        t = tf.translation
        r = tf.rotation
        return ((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

    def _poses_cb(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id == self.link:
                self._link_rel = self._tf_to_pair(tr.transform)
            elif tr.child_frame_id == self.robot:
                self._robot_w = self._tf_to_pair(tr.transform)

    def _cmd_cb(self, msg):
        if msg.data != self._held:
            self.get_logger().info(
                f"grasp -> {'HOLD' if msg.data else 'RELEASE'}"
            )
            if not msg.data:
                self._last = None  # released — let it rest where it landed
                with self._wlock:
                    self._pending = None
        self._held = msg.data

    def _pose_worker(self):
        """Apply the latest requested cup pose (blocking subprocess) off
        the ROS executor thread, coalescing to the most recent target."""
        bad = 0
        while not self._stop:
            self._wevt.wait(timeout=0.5)
            self._wevt.clear()
            while True:
                with self._wlock:
                    target = self._pending
                    self._pending = None
                if target is None:
                    break
                x, y, z, q = target
                req = (
                    f'name: "{self.obj}" '
                    f'position {{ x: {x} y: {y} z: {z} }} '
                    f'orientation {{ x: {q[0]} y: {q[1]} z: {q[2]} w: {q[3]} }}'
                )
                try:
                    r = subprocess.run(
                        [self._gz, "service", "-s", self._service,
                         "--reqtype", "ignition.msgs.Pose",
                         "--reptype", "ignition.msgs.Boolean",
                         "--timeout", "300", "--req", req],
                        check=False, timeout=3,
                        capture_output=True, text=True,
                    )
                    if "true" not in (r.stdout or "").lower():
                        bad += 1
                        if bad in (1, 20):
                            self.get_logger().warn(
                                f"set_pose did not confirm "
                                f"(rc={r.returncode}, out={r.stdout!r} "
                                f"err={r.stderr!r})")
                    else:
                        bad = 0
                except subprocess.SubprocessError as exc:
                    self.get_logger().warn(f"set_pose call failed: {exc}",
                                           throttle_duration_sec=2.0)

    def _tick(self):
        if not self._held:
            return
        if self._link_rel is None or self._robot_w is None:
            self.get_logger().warn(
                f"world pose for '{self.link}'/'{self.robot}' not ready",
                throttle_duration_sec=2.0)
            return
        # world <- piper_link6  =  (world <- ranger) ∘ (ranger <- link)
        (wx, wy, wz), q = _compose(self._robot_w, self._link_rel)
        ox, oy, oz = _rotate_vec_by_quat(self.offset, q)
        x, y, z = wx + ox, wy + oy, wz + oz
        # Only re-pose when the gripper actually moved — avoids the cup
        # visibly jittering (and a set_pose call) every tick at rest.
        if self._last is not None:
            dx, dy, dz = x - self._last[0], y - self._last[1], z - self._last[2]
            if dx * dx + dy * dy + dz * dz < 1.0e-6:  # < 1 mm
                return
        self._last = (x, y, z)
        with self._wlock:
            self._pending = (x, y, z, q)
        self._wevt.set()


def main():
    rclpy.init()
    node = GraspMagnet()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop = True
        node._wevt.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
