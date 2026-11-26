#!/usr/bin/env python3
"""
Scripted seated person of the handover study (docs/sim_handover_baseline.md).

A stand-in, not a human model: it sits on ChairA_01_001, turns, extends a
hand and takes the object by script, and publishes its state twice at
`rate` (node clock), all in odom:

  /person/ground_truth/{body_pose,hand_pose,hand_extended,take}
      PoseStamped, PoseStamped, Bool, Bool: the script's state. Only
      handover_recorder reads it (metrics).
  /person/measured/{body_pose,hand_pose,hand_extended,take}
      the same sample through the fault injector (`faults`): what
      handover_baseline and handover_safety see. Not latched: a dropout
      must look like silence to a late subscriber too.
  /person/fault_active (String, latched, on change)
      the windowed faults active now (dropout, stale, jump), or empty.

PersonScript (pure, seeded): faces start_facing until the robot's footprint
was farther than turn_distance from the torso axis, then facing_yaw. Once
turned, when the robot's base has stood within notice_distance, slower
than still_speed, for still_sec, and hand_extends: after a delay
drawn from reaction_delay the hand moves at hand_speed to its extended
position. When the TCP has been within take_reach of the shoulder and
still for still_sec, after a second delay from the same range the hand
moves to the TCP and `take` turns true on contact. After the object
detaches the hand returns to rest and the object is moved once to
taken_object_pose (the person put it down). facing_change is added to the
facing facing_change_delay after the robot first stood still nearby.

FaultInjector (pure): `faults` = [bias_x, bias_y, noise_std, dropout_start,
dropout_sec, stale_start, stale_sec, latency_sec, jump_at, jump_x, jump_y,
trigger_distance]. The injector arms once the ground-truth base-to-torso
distance has been at least trigger_distance (0: at once); bias and noise
apply from then on, and window times count from the moment that distance
next drops below trigger_distance. Twelve zeros: the measured
sample is the ground-truth sample.

The models person_proxy and person_hand (models/*.sdf, spawned by the
launch) follow the script through /world/<world_name>/set_pose, called
asynchronously, never waited for in the timer.

Simulation only: nothing here commands the robot or reaches a CAN bus.
"""

import dataclasses
import importlib.util
import math
from pathlib import Path
import random
from typing import Optional

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


def _load_core():
    # resolve(): with --symlink-install this file is a link into the source tree.
    path = Path(__file__).resolve().with_name("handover_core.py")
    spec = importlib.util.spec_from_file_location("handover_core", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


core = _load_core()
PersonState = core.PersonState
Point2 = core.Point2
Point3 = core.Point3

# Defaults of the node parameters of the same name (config/handover.yaml).
RATE_HZ = 20.0
START_FACING_RAD = 0.0
TURN_DISTANCE_M = 1.5
NOTICE_DISTANCE_M = 1.5
STILL_SPEED_MPS = 0.02
STILL_SEC = 0.5
HAND_SPEED_MPS = 0.5
HAND_REST = [0.25, 0.18, 0.62]  # forward, to the hand's side, world z
HAND_EXTENDED = [0.40, 0.20, 0.80]
TAKE_REACH_M = 0.65
SHOULDER_HEIGHT_M = 1.00  # world z
SEAT_HEIGHT_M = 0.435  # world z
FACING_CHANGE_DELAY_SEC = 1.0
TAKEN_OBJECT_POSE = [1.00, -0.62, 0.801]  # world
# Launch-argument defaults (handover_baseline.launch.py).
PERSON_XY = (0.504, -0.858)
REACTION_DELAY = [0.5, 1.0]
FAULT_FIELDS = ("bias_x", "bias_y", "noise_std", "dropout_start", "dropout_sec",
                "stale_start", "stale_sec", "latency_sec", "jump_at", "jump_x", "jump_y",
                "trigger_distance")
NON_NEGATIVE_FAULTS = frozenset(FAULT_FIELDS) - {"bias_x", "bias_y", "jump_x", "jump_y"}
HAND_SIDES = {"left": 1.0, "right": -1.0}  # + = the person's left
CONTACT_M = 0.02  # hand this close to its goal: arrived (the take: touching the object)
MOVED_M = 0.001  # a model is re-posed only when it moved this far
TIME_EPS_SEC = 1e-9  # fault window edges
FACING_MOVED_RAD = 0.005
GROUND_TRUTH, MEASURED = "/person/ground_truth", "/person/measured"
ODOM_FRAME, TCP_FRAME = "odom", "piper_tcp"
LATCHED_QOS = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
)


@dataclasses.dataclass(frozen=True)
class RobotView:
    """What the person sees of the robot (ground truth)."""

    base: Optional[core.Pose2D]  # odom
    base_speed: float  # m/s
    tcp: Optional[Point3]  # odom
    tcp_speed: float  # m/s
    object_detached: bool


@dataclasses.dataclass(frozen=True)
class ScriptSettings:
    """Parameters of PersonScript (see the module docstring)."""

    x: float
    y: float
    start_facing: float
    facing_yaw: float
    facing_change: float
    facing_change_delay: float
    hand_side: float  # +1 left, -1 right
    hand_extends: bool
    reaction_delay: tuple[float, float]
    turn_distance: float
    notice_distance: float
    still_speed: float
    still_sec: float
    hand_speed: float
    hand_rest: Point3  # forward, side, world z
    hand_extended: Point3
    take_reach: float
    shoulder_height: float  # world z
    footprint: core.Rect


def check_delay(values) -> tuple[float, float]:
    """Return reaction_delay as (min, max); ValueError unless 0 <= min <= max, finite."""
    if len(values) != 2 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError(f"reaction_delay must be [min, max] seconds, got {list(values)}")
    low, high = (float(v) for v in values)
    if not 0.0 <= low <= high:
        raise ValueError(f"reaction_delay needs 0 <= min <= max, got {list(values)}")
    return low, high


def _towards(start: Point3, goal: Point3, step: float) -> Point3:
    distance = math.dist(start, goal)
    if distance <= step or distance == 0.0:
        return tuple(goal)
    fraction = step / distance
    return tuple(s + (g - s) * fraction for s, g in zip(start, goal))


class PersonScript:
    """The person's behaviour; time comes in, a PersonState comes out (seeded, no clock)."""

    def __init__(self, settings: ScriptSettings, seed: int) -> None:
        self._s = settings
        self._rng = random.Random(seed)
        self._turned = False
        self._facing_changed = False
        self._still_since: Optional[float] = None
        self._first_still_near: Optional[float] = None
        self._reaction_at: Optional[float] = None
        self._tcp_still_since: Optional[float] = None
        self._take_at: Optional[float] = None
        self._take = False
        self._taken = False  # the object was taken: back to rest for good
        self._last_now: Optional[float] = None
        self._hand: Optional[Point3] = None

    @property
    def taken(self) -> bool:
        """Whether the person took the object (it detached after the take)."""
        return self._taken

    def facing(self) -> float:
        """Return the facing of now (rad, odom)."""
        if not self._turned:
            return self._s.start_facing
        change = self._s.facing_change if self._facing_changed else 0.0
        return core.wrap_angle(self._s.facing_yaw + change)

    def _local(self, offset: Point3) -> Point3:
        """Return a point given as (forward, to the hand's side, world z) in odom."""
        forward, side, z = offset
        side *= self._s.hand_side
        facing = self.facing()
        c, s = math.cos(facing), math.sin(facing)
        return (self._s.x + c * forward - s * side, self._s.y + s * forward + c * side,
                z + core.ODOM_Z_OFFSET_M)

    def shoulder(self) -> Point3:
        """Return the shoulder of the hand's side (odom)."""
        return self._local((0.0, self._s.hand_rest[1], self._s.shoulder_height))

    def _robot_distance(self, robot: RobotView) -> float:
        if robot.base is None:
            return math.inf
        local = core.to_frame(robot.base, (self._s.x, self._s.y))
        return core.point_rect_distance(local, self._s.footprint)

    def _delay(self) -> float:
        return self._rng.uniform(*self._s.reaction_delay)

    def _update_still(self, now: float, robot: RobotView, near: bool) -> bool:
        """Whether the robot has stood still within notice_distance for still_sec."""
        if near and robot.base is not None and robot.base_speed < self._s.still_speed:
            if self._still_since is None or now < self._still_since:
                self._still_since = now
        else:
            self._still_since = None
        still = self._still_since is not None and now - self._still_since >= self._s.still_sec
        if self._turned and near and still and self._first_still_near is None:
            self._first_still_near = now
        return still

    def _update_take(self, now: float, robot: RobotView) -> None:
        if robot.tcp is None or self._take_at is not None:
            return
        within = math.dist(robot.tcp, self.shoulder()) <= self._s.take_reach
        if within and robot.tcp_speed < self._s.still_speed:
            if self._tcp_still_since is None or now < self._tcp_still_since:
                self._tcp_still_since = now
            if now - self._tcp_still_since >= self._s.still_sec:
                self._take_at = now + self._delay()
        else:
            self._tcp_still_since = None

    def _hand_goal(self, now: float, robot: RobotView) -> Point3:
        rest = self._local(self._s.hand_rest)
        if self._taken or self._reaction_at is None or now < self._reaction_at:
            return rest
        if self._take_at is not None and now >= self._take_at and robot.tcp is not None:
            return robot.tcp
        return self._local(self._s.hand_extended)

    def step(self, now: float, robot: RobotView) -> PersonState:
        """Advance to `now` with the robot as seen; return the person's state."""
        dt = 0.0 if self._last_now is None else max(0.0, now - self._last_now)
        self._last_now = now
        distance = self._robot_distance(robot)
        # An unknown robot (no /odom yet) is not "far": the person keeps facing
        # the table until the robot has been seen away from it.
        if not self._turned and robot.base is not None and distance > self._s.turn_distance:
            self._turned = True
        near = distance <= self._s.notice_distance
        still = self._update_still(now, robot, near)
        if (self._first_still_near is not None and not self._facing_changed
                and now - self._first_still_near >= self._s.facing_change_delay):
            self._facing_changed = True
        if (self._turned and near and still and self._s.hand_extends
                and self._reaction_at is None and not self._taken):
            self._reaction_at = now + self._delay()
        extended_goal = self._local(self._s.hand_extended)
        if self._hand is None:
            self._hand = self._local(self._s.hand_rest)
        arrived = math.dist(self._hand, extended_goal) <= CONTACT_M
        if (arrived or self._take_at is not None) and not self._taken:
            self._update_take(now, robot)
        if self._take and robot.object_detached:
            self._take, self._taken = False, True
        goal = self._hand_goal(now, robot)
        self._hand = _towards(self._hand, goal, self._s.hand_speed * dt)
        reaching = self._take_at is not None and now >= self._take_at and not self._taken
        if reaching and robot.tcp is not None and math.dist(self._hand, robot.tcp) <= CONTACT_M:
            self._take = True
        extended = not self._taken and (
            reaching or math.dist(self._hand, extended_goal) <= CONTACT_M)
        return PersonState(self._s.x, self._s.y, self.facing(), self._hand, extended,
                           self._take, now)


# ---- fault injection ---------------------------------------------------------------


@dataclasses.dataclass(frozen=True)
class Faults:
    """The twelve fault values (see the module docstring)."""

    bias_x: float
    bias_y: float
    noise_std: float
    dropout_start: float
    dropout_sec: float
    stale_start: float
    stale_sec: float
    latency_sec: float
    jump_at: float
    jump_x: float
    jump_y: float
    trigger_distance: float


def parse_faults(values) -> Faults:
    """Return the faults list as Faults; ValueError for a bad length, a NaN or a negative time."""
    if len(values) != len(FAULT_FIELDS):
        raise ValueError(f"faults must have {len(FAULT_FIELDS)} values {list(FAULT_FIELDS)}, "
                         f"got {len(values)}")
    numbers = [float(v) for v in values]
    for name, value in zip(FAULT_FIELDS, numbers):
        if not math.isfinite(value):
            raise ValueError(f"faults {name} must be finite, got {value}")
        if name in NON_NEGATIVE_FAULTS and value < 0.0:
            raise ValueError(f"faults {name} must not be negative, got {value}")
    return Faults(*numbers)


def _shifted(sample: PersonState, dx: float, dy: float) -> PersonState:
    hx, hy, hz = sample.hand
    return dataclasses.replace(sample, x=sample.x + dx, y=sample.y + dy,
                               hand=(hx + dx, hy + dy, hz))


class FaultInjector:
    """Turn a ground-truth sample into the measured one (None: nothing is published)."""

    def __init__(self, faults: Faults, seed: int) -> None:
        self._f = faults
        self._rng = random.Random(seed)
        self._identity = not any(dataclasses.astuple(faults))
        self._trigger: Optional[float] = None
        self._armed = False
        self._queue: list[PersonState] = []
        self._stale: Optional[PersonState] = None

    def _since_trigger(self, now: float) -> Optional[float]:
        return None if self._trigger is None else now - self._trigger

    @staticmethod
    def _inside(t: Optional[float], start: float, duration: float) -> bool:
        # TIME_EPS_SEC: sample times are sums of floats (2.05 - 1.85 < 0.2).
        return (t is not None and duration > 0.0
                and start - TIME_EPS_SEC <= t < start + duration - TIME_EPS_SEC)

    def active(self, now: float) -> str:
        """Return the windowed faults active at `now`, '+'-joined; empty for none."""
        t, f = self._since_trigger(now), self._f
        names = []
        if self._inside(t, f.dropout_start, f.dropout_sec):
            names.append("dropout")
        if self._inside(t, f.stale_start, f.stale_sec):
            names.append("stale")
        if t is not None and t >= f.jump_at - TIME_EPS_SEC and (f.jump_x or f.jump_y):
            names.append("jump")
        return "+".join(names)

    def step(self, sample: PersonState, now: float,
             robot_distance: float) -> Optional[PersonState]:
        """
        Return the measured sample for ground truth `sample` at `now`.

        `robot_distance`: ground-truth base_footprint origin to torso axis (m),
        for the trigger.
        """
        if self._identity:
            return sample
        f = self._f
        # Armed once the robot has been at least trigger_distance away: the pick
        # slot is ~1 m from the person, so "first below 1.2 m" would otherwise
        # fire at startup, during the pick (Gazebo, 2026-11-26).
        if not self._armed and (f.trigger_distance == 0.0
                                or (math.isfinite(robot_distance)
                                    and robot_distance >= f.trigger_distance)):
            self._armed = True
        if not self._armed:
            return sample
        if self._trigger is None and (f.trigger_distance == 0.0
                                      or robot_distance < f.trigger_distance):
            self._trigger = now
        t = self._since_trigger(now)
        dx, dy = f.bias_x, f.bias_y
        if f.noise_std > 0.0:
            dx += self._rng.gauss(0.0, f.noise_std)
            dy += self._rng.gauss(0.0, f.noise_std)
        if t is not None and t >= f.jump_at - TIME_EPS_SEC:
            dx, dy = dx + f.jump_x, dy + f.jump_y
        self._queue.append(_shifted(sample, dx, dy) if (dx or dy) else sample)
        delivered = None
        while self._queue and self._queue[0].stamp_sec <= now - f.latency_sec + TIME_EPS_SEC:
            delivered = self._queue.pop(0)
        if self._inside(t, f.stale_start, f.stale_sec):
            if self._stale is None:
                self._stale = delivered
            delivered = self._stale
        if self._inside(t, f.dropout_start, f.dropout_sec):
            return None
        return delivered


# ---- node ------------------------------------------------------------------------------


def _stamp(seconds: float) -> TimeMsg:
    sec = math.floor(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec, nanosec = sec + 1, nanosec - 1_000_000_000
    return TimeMsg(sec=int(sec), nanosec=nanosec)


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PersonProxy(Node):
    """Publish the scripted person, measured and true, and move its Gazebo models."""

    def __init__(self, **kwargs) -> None:
        super().__init__("person_proxy", **kwargs)
        period = self._declare_parameters()
        self._odom: Optional[tuple[core.Pose2D, float]] = None  # (pose, linear speed)
        self._detached = False
        self._tcp_last: Optional[tuple[Point3, float]] = None
        self._sent: dict[str, tuple] = {}  # model -> last (world xyz, yaw) sent
        self._pending: dict[str, object] = {}  # model -> future of the last set_pose
        self._fault_active: Optional[str] = None
        self._object_moved = False
        self._warned = {"set_pose": False}
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pubs = {
            prefix: {
                "body_pose": self.create_publisher(PoseStamped, f"{prefix}/body_pose", 10),
                "hand_pose": self.create_publisher(PoseStamped, f"{prefix}/hand_pose", 10),
                "hand_extended": self.create_publisher(Bool, f"{prefix}/hand_extended", 10),
                "take": self.create_publisher(Bool, f"{prefix}/take", 10),
            }
            for prefix in (GROUND_TRUTH, MEASURED)
        }
        self._fault_pub = self.create_publisher(String, "/person/fault_active", LATCHED_QOS)
        self._pose_client = self.create_client(SetEntityPose, f"/world/{self._world}/set_pose")
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(String, f"/grasp/{self._object}/state", self._on_grasp, 10)
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"Person proxy at ({self._settings.x:.3f}, {self._settings.y:.3f}), facing "
            f"{math.degrees(self._settings.facing_yaw):.0f} deg once turned, "
            f"{'extends' if self._settings.hand_extends else 'never extends'} the "
            f"{'left' if self._settings.hand_side > 0 else 'right'} hand"
        )

    def _declare_parameters(self) -> float:
        """Declare and check the parameters; returns the timer period (s)."""
        p = self.declare_parameter
        shared = {key: p(key, value).value for key, value in core.SHARED_DEFAULTS.items()}
        cfg = core.config_from_parameters(shared)
        positive = {
            "rate": p("rate", RATE_HZ).value,
            "turn_distance": p("turn_distance", TURN_DISTANCE_M).value,
            "notice_distance": p("notice_distance", NOTICE_DISTANCE_M).value,
            "still_speed": p("still_speed", STILL_SPEED_MPS).value,
            "still_sec": p("still_sec", STILL_SEC).value,
            "hand_speed": p("hand_speed", HAND_SPEED_MPS).value,
            "take_reach": p("take_reach", TAKE_REACH_M).value,
            "shoulder_height": p("shoulder_height", SHOULDER_HEIGHT_M).value,
            "seat_height": p("seat_height", SEAT_HEIGHT_M).value,
        }
        for name, value in positive.items():
            if not value > 0.0:
                raise ValueError(f"{name} must be positive, got {value}")
        triples = {name: [float(v) for v in p(name, default).value] for name, default in (
            ("hand_rest", HAND_REST), ("hand_extended", HAND_EXTENDED),
            ("taken_object_pose", TAKEN_OBJECT_POSE))}
        for name, values in triples.items():
            if len(values) != 3 or not all(map(math.isfinite, values)):
                raise ValueError(f"{name} must be 3 finite values, got {values}")
        side = p("hand_side", "left").value
        if side not in HAND_SIDES:
            raise ValueError(f"hand_side must be one of {sorted(HAND_SIDES)}, got '{side}'")
        delay = check_delay(p("reaction_delay", REACTION_DELAY).value)
        faults = parse_faults(p("faults", [0.0] * len(FAULT_FIELDS)).value)
        seed = p("seed", 0).value
        facing_change_delay = p("facing_change_delay", FACING_CHANGE_DELAY_SEC).value
        if facing_change_delay < 0.0:
            raise ValueError(
                f"facing_change_delay must not be negative, got {facing_change_delay}")
        self._settings = ScriptSettings(
            x=p("person_x", PERSON_XY[0]).value, y=p("person_y", PERSON_XY[1]).value,
            start_facing=p("start_facing", START_FACING_RAD).value,
            facing_yaw=p("facing_yaw", 0.0).value,
            facing_change=p("facing_change", 0.0).value,
            facing_change_delay=facing_change_delay,
            hand_side=HAND_SIDES[side], hand_extends=p("hand_extends", True).value,
            reaction_delay=delay, turn_distance=positive["turn_distance"],
            notice_distance=positive["notice_distance"], still_speed=positive["still_speed"],
            still_sec=positive["still_sec"], hand_speed=positive["hand_speed"],
            hand_rest=tuple(triples["hand_rest"]),
            hand_extended=tuple(triples["hand_extended"]),
            take_reach=positive["take_reach"], shoulder_height=positive["shoulder_height"],
            footprint=cfg.footprint,
        )
        self._seat_z = positive["seat_height"] + core.ODOM_Z_OFFSET_M
        self._taken_pose = tuple(triples["taken_object_pose"])
        self._world = p("world_name", "small_house").value
        self._object = p("object_name", "vitamin_bottle").value
        self._script = PersonScript(self._settings, seed)
        # A second stream from the same seed: the delays do not depend on the faults.
        self._injector = FaultInjector(faults, seed + 1)
        return 1.0 / positive["rate"]

    def _now(self) -> float:
        """Node clock (sim time in the sim) in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _warn_once(self, key: str, active: bool, text: str) -> None:
        """Warn when degraded state `key` begins; re-arm once it ends."""
        if active and not self._warned[key]:
            self.get_logger().warn(text)
        self._warned[key] = active

    # ---- inputs ----------------------------------------------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        base = (pose.position.x, pose.position.y, _yaw_of(pose.orientation))
        twist = msg.twist.twist.linear
        speed = math.hypot(twist.x, twist.y)
        if all(map(math.isfinite, (*base, speed))):
            self._odom = (base, speed)

    def _on_grasp(self, msg: String) -> None:
        self._detached = msg.data == "detached"

    def _tcp(self, now: float) -> tuple[Optional[Point3], float]:
        """TCP in odom from TF and its speed since the last tick; (None, inf) without TF."""
        try:
            transform = self._tf_buffer.lookup_transform(ODOM_FRAME, TCP_FRAME, Time())
        except TransformException:
            return None, math.inf
        t = transform.transform.translation
        tcp = (t.x, t.y, t.z)
        speed = math.inf
        if self._tcp_last is not None and now > self._tcp_last[1]:
            speed = math.dist(tcp, self._tcp_last[0]) / (now - self._tcp_last[1])
        self._tcp_last = (tcp, now)
        return tcp, speed

    def _robot_view(self, now: float) -> RobotView:
        tcp, tcp_speed = self._tcp(now)
        base, speed = self._odom if self._odom is not None else (None, math.inf)
        return RobotView(base, speed, tcp, tcp_speed, self._detached)

    # ---- outputs ---------------------------------------------------------------------
    def _publish(self, prefix: str, sample: PersonState) -> None:
        pubs = self._pubs[prefix]
        stamp = _stamp(sample.stamp_sec)
        body = PoseStamped()
        body.header.frame_id, body.header.stamp = ODOM_FRAME, stamp
        body.pose.position.x, body.pose.position.y = float(sample.x), float(sample.y)
        body.pose.position.z = self._seat_z
        body.pose.orientation.z = math.sin(sample.facing / 2)
        body.pose.orientation.w = math.cos(sample.facing / 2)
        hand = PoseStamped()
        hand.header.frame_id, hand.header.stamp = ODOM_FRAME, stamp
        hand.pose.position.x, hand.pose.position.y, hand.pose.position.z = (
            float(v) for v in sample.hand)
        hand.pose.orientation.w = 1.0
        pubs["body_pose"].publish(body)
        pubs["hand_pose"].publish(hand)
        pubs["hand_extended"].publish(Bool(data=bool(sample.hand_extended)))
        pubs["take"].publish(Bool(data=bool(sample.take)))

    def _set_pose(self, name: str, xyz_world: Point3, yaw: float) -> bool:
        """Ask Gazebo to place model `name` unless the last request is still out; True if sent."""
        pending = self._pending.get(name)
        if pending is not None and not pending.done():
            return False
        ready = self._pose_client.service_is_ready()
        self._warn_once("set_pose", not ready,
                        f"{self._pose_client.srv_name} not available: models not moved")
        if not ready:
            return False
        request = SetEntityPose.Request()
        request.entity.name, request.entity.type = name, Entity.MODEL
        pose: Pose = request.pose
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz_world)
        pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self._pending[name] = self._pose_client.call_async(request)
        self._sent[name] = (tuple(xyz_world), yaw)
        return True

    def _move_models(self, sample: PersonState) -> None:
        hand_world = (sample.hand[0], sample.hand[1], sample.hand[2] - core.ODOM_Z_OFFSET_M)
        last = self._sent.get("person_hand")
        if last is None or math.dist(last[0], hand_world) > MOVED_M:
            self._set_pose("person_hand", hand_world, 0.0)
        last = self._sent.get("person_proxy")
        if last is None or abs(core.wrap_angle(last[1] - sample.facing)) > FACING_MOVED_RAD:
            self._set_pose("person_proxy", (sample.x, sample.y, 0.0), sample.facing)
        if self._script.taken and self._detached and not self._object_moved:
            self._object_moved = self._set_pose(self._object, self._taken_pose, 0.0)
            if self._object_moved:
                self.get_logger().info(f"{self._object} taken: put down on the table")

    def _tick(self) -> None:
        now = self._now()
        robot = self._robot_view(now)
        truth = self._script.step(now, robot)
        self._publish(GROUND_TRUTH, truth)
        distance = (math.inf if robot.base is None
                    else math.hypot(robot.base[0] - truth.x, robot.base[1] - truth.y))
        measured = self._injector.step(truth, now, distance)
        if measured is not None:
            self._publish(MEASURED, measured)
        active = self._injector.active(now)
        if active != self._fault_active:
            self._fault_active = active
            self._fault_pub.publish(String(data=active))
            if active:
                self.get_logger().warn(f"fault active: {active}")
        self._move_models(truth)


SPIN_PERIOD_SEC = 0.1


def main() -> None:
    # SIGINT / SIGTERM only set a flag (handover_core.run_until_signal).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PersonProxy()
    try:
        core.run_until_signal(lambda: rclpy.spin_once(node, timeout_sec=SPIN_PERIOD_SEC),
                              rclpy.ok)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
