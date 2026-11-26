#!/usr/bin/env python3
"""
Ground-truth metrics of one handover episode, one CSV row (docs/sim_handover_baseline.md).

Reads only ground truth: /person/ground_truth/{body_pose,hand_pose,take},
/odom, TF odom -> piper_tcp, /grasp/<object_name>/state, plus the episode's
/handover_baseline/phase, /handover_safety/state, /person/fault_active and
the table place pose the baseline asked for
(/handover_safety/place_target_request). It never reads /person/measured/*.

The window opens at the first TRANSIT phase and closes at DONE: / FAILED:
(or at shutdown: outcome `aborted`; an episode that fails before TRANSIT
gives a row without metrics); every 1 / rate s of the node clock
(sim time) it integrates:

  viol_contact_sec        footprint -> person capsule <= 0
  viol_stop_distance_sec  that clearance < stop_distance while the base is
                          faster than moving_speed
  viol_height_cap_sec     TCP z > tcp_height_cap
  viol_release            1 when the object detached with neither a take nor
                          the TCP within place_tolerance of the place pose
  time_outside_35/70_sec  TRANSIT or APPROACH, base within view_radius of
                          the torso axis, |bearing| > view_comfort / view_limit
  max_closing_speed_mps   fastest decrease of the torso axis -> footprint
                          distance (the standoff distance) under closing_radius
  min_clearance_m, person_reach_m (hand rest -> hand at the take),
  waiting_sec (WAIT), duration_sec, rechoices (TRANSIT entries - 1),
  safety_stops (OK -> STOP edges), fault_stop_latency_sec (first fault
  edge -> the base slower than moving_speed for STILL_HOLD_SEC; counted only
  if the base moved at or after the edge, else 0), fault_motion_sec (time
  the base moved while a fault was active, after that first stop)

success: outcome handover or set_down and the four violation columns zero.
A FAILED: phase is outcome `stopped` while handover_safety reports STOP,
else `failed`. The row goes to results_file, default
~/.ros/handover_results/handover_<YYYYmmdd_HHMMSS>.csv (appended; the header
is written when the file is new). Simulation only.
"""

import csv
import dataclasses
from datetime import datetime
import importlib.util
import math
from pathlib import Path
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
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

# Defaults of the node parameters of the same name (config/handover.yaml).
RATE_HZ = 20.0
MOVING_SPEED_MPS = 0.005
VIEW_RADIUS_M = 1.5
CLOSING_RADIUS_M = 1.0
PLACE_TOLERANCE_M = 0.03
STILL_HOLD_SEC = 0.5  # "staying" under moving_speed, for the fault stop latency
MAX_STEP_SEC = 1.0  # a longer gap (paused sim, clock jump) is not integrated
DEFAULT_RESULTS_DIR = Path.home() / ".ros" / "handover_results"
GROUND_TRUTH = "/person/ground_truth"
PHASE_TOPIC = "/handover_baseline/phase"
SAFETY_STATE_TOPIC = "/handover_safety/state"
PLACE_TARGET_TOPIC = "/handover_safety/place_target_request"
ODOM_FRAME, TCP_FRAME = "odom", "piper_tcp"
TRANSIT, APPROACH, WAIT = "TRANSIT", "APPROACH", "WAIT"
DONE, FAILED = "DONE: ", "FAILED: "
SUCCESSES = ("handover", "set_down")
COLUMNS = (
    "episode_id", "seed", "facing_deg", "hand_side", "hand_extends", "reaction_delay_min",
    "reaction_delay_max", "facing_change_deg", "faults", "success", "outcome",
    "viol_contact_sec", "viol_stop_distance_sec", "viol_height_cap_sec", "viol_release",
    "time_outside_35_sec", "time_outside_70_sec", "max_closing_speed_mps", "min_clearance_m",
    "person_reach_m", "waiting_sec", "duration_sec", "rechoices", "safety_stops",
    "fault_stop_latency_sec", "fault_motion_sec",
)
VIOLATIONS = ("viol_contact_sec", "viol_stop_distance_sec", "viol_height_cap_sec",
              "viol_release")
LATCHED_QOS = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
)


@dataclasses.dataclass(frozen=True)
class EpisodeInfo:
    """The episode's settings, copied into the row."""

    episode_id: int = 0
    seed: int = 0
    facing_deg: float = 0.0
    hand_side: str = "left"
    hand_extends: bool = True
    reaction_delay: tuple[float, float] = (0.5, 1.0)
    facing_change_deg: float = 0.0
    faults: tuple[float, ...] = (0.0,) * 12


@dataclasses.dataclass(frozen=True)
class Limits:
    """The recorder's own thresholds."""

    moving_speed: float = MOVING_SPEED_MPS
    view_radius: float = VIEW_RADIUS_M
    closing_radius: float = CLOSING_RADIUS_M
    place_tolerance: float = PLACE_TOLERANCE_M


class EpisodeMetrics:
    """The metrics of the module docstring; time comes in as arguments."""

    def __init__(self, cfg, limits: Limits = Limits()) -> None:
        self._cfg, self._lim = cfg, limits
        self.phase: Optional[str] = None
        self.opened: Optional[float] = None
        self.closed: Optional[float] = None
        self.outcome: Optional[str] = None
        self._person: Optional[core.PersonState] = None
        self._base: Optional[core.Pose2D] = None
        self._speed = math.inf
        self._tcp: Optional[core.Point3] = None
        self._place: Optional[core.Point3] = None
        self._grasp: Optional[str] = None
        self._safety: Optional[str] = None
        self._last: Optional[float] = None
        self._distance: Optional[float] = None
        self._hand_rest: Optional[core.Point3] = None
        self._fault_at: Optional[float] = None
        self._fault_active = False
        self._moved_after_fault = False
        self._still_since: Optional[float] = None
        self.values = {
            "viol_contact_sec": 0.0, "viol_stop_distance_sec": 0.0, "viol_height_cap_sec": 0.0,
            "viol_release": 0, "time_outside_35_sec": 0.0, "time_outside_70_sec": 0.0,
            "max_closing_speed_mps": 0.0, "min_clearance_m": math.inf, "person_reach_m": None,
            "waiting_sec": 0.0, "rechoices": 0, "safety_stops": 0,
            "fault_stop_latency_sec": None, "fault_motion_sec": None,
        }

    @property
    def is_open(self) -> bool:
        """Whether the window is open (TRANSIT seen, not closed)."""
        return self.opened is not None and self.closed is None

    # ---- inputs ---------------------------------------------------------------------
    def on_phase(self, phase: str, now: float) -> None:
        """Take a phase of the baseline; TRANSIT opens the window, DONE: / FAILED: close it."""
        if self.closed is not None:
            return
        self.step(now)  # up to the edge with the old phase
        if phase == TRANSIT:
            if self.opened is None:
                self.opened = now
                self._last = now
                self._hand_rest = None if self._person is None else self._person.hand
            else:
                self.values["rechoices"] += 1
        self.phase = phase
        if phase.startswith((DONE, FAILED)):
            # Before TRANSIT too (a failed pick): the row then has no metrics.
            self.step(now)
            self.close(now, self._outcome_of(phase))

    def _outcome_of(self, phase: str) -> str:
        if phase.startswith(DONE):
            return phase[len(DONE):].strip()
        return "stopped" if (self._safety or "").startswith("STOP") else "failed"

    def on_person(self, sample: core.PersonState) -> None:
        """Take a ground-truth person sample; the first take in the window gives the reach."""
        if (sample.take and self.is_open and self.values["person_reach_m"] is None
                and self._hand_rest is not None):
            self.values["person_reach_m"] = math.dist(self._hand_rest, sample.hand)
        self._person = sample

    def on_base(self, pose: core.Pose2D, speed: float) -> None:
        """Take the base pose and linear speed from /odom."""
        self._base, self._speed = pose, speed

    def on_tcp(self, xyz: Optional[core.Point3]) -> None:
        """Take the TCP in odom (None: no transform)."""
        self._tcp = xyz

    def on_place_target(self, xyz: core.Point3) -> None:
        """Take the table place pose the baseline asked for."""
        self._place = xyz

    def on_grasp_state(self, state: str) -> None:
        """Take the object's attach state; a detach inside the window is the release."""
        if state == "detached" and self._grasp == "attached" and self.is_open:
            taken = self._person is not None and self._person.take
            placed = (self._place is not None and self._tcp is not None
                      and math.dist(self._place, self._tcp) <= self._lim.place_tolerance)
            if not (taken or placed):
                self.values["viol_release"] = 1
        self._grasp = state

    def on_fault(self, active: str, now: float) -> None:
        """/person/fault_active; the first non-empty value is the fault edge."""
        if active and self._fault_at is None:
            self._fault_at = now
        self._fault_active = bool(active)

    def on_safety_state(self, state: str) -> None:
        """/handover_safety/state; OK -> STOP edges inside the window count."""
        was_stop = (self._safety or "").startswith("STOP")
        if state.startswith("STOP") and not was_stop and self.is_open:
            self.values["safety_stops"] += 1
        self._safety = state

    # ---- integration ------------------------------------------------------------------
    def _fault_latency(self, now: float, dt: float) -> None:
        if self._fault_at is None:
            return
        moving = self._speed >= self._lim.moving_speed
        if self.values["fault_stop_latency_sec"] is not None:
            if moving and self._fault_active and 0.0 < dt <= MAX_STEP_SEC:
                self.values["fault_motion_sec"] += dt
            return
        if moving:
            self._moved_after_fault = True
            self._still_since = None
            return
        if self._still_since is None:
            self._still_since = now
        if now - self._still_since >= STILL_HOLD_SEC:
            # Still at the edge and since: nothing had to be stopped.
            latency = (max(0.0, self._still_since - self._fault_at)
                       if self._moved_after_fault else 0.0)
            self.values["fault_stop_latency_sec"] = latency
            self.values["fault_motion_sec"] = 0.0

    def step(self, now: float) -> None:
        """Integrate from the last step to `now` with the latest inputs."""
        dt = 0.0 if self._last is None else now - self._last
        self._fault_latency(now, dt)
        if not self.is_open or self._last is None:
            return
        self._last = now
        if not 0.0 < dt <= MAX_STEP_SEC:
            self._distance = None  # no closing speed across a gap
            return
        v, cfg, lim = self.values, self._cfg, self._lim
        if self._tcp is not None and self._tcp[2] > cfg.tcp_height_cap:
            v["viol_height_cap_sec"] += dt
        if self.phase == WAIT:
            v["waiting_sec"] += dt
        if self._person is None or self._base is None:
            return
        clearance = core.footprint_clearance(self._base, cfg.footprint, self._person, cfg)
        v["min_clearance_m"] = min(v["min_clearance_m"], clearance)
        if clearance <= 0.0:
            v["viol_contact_sec"] += dt
        if clearance < cfg.stop_distance and self._speed > lim.moving_speed:
            v["viol_stop_distance_sec"] += dt
        distance = math.hypot(self._base[0] - self._person.x, self._base[1] - self._person.y)
        if self.phase in (TRANSIT, APPROACH) and distance <= lim.view_radius:
            off = abs(core.bearing(self._person, self._base[:2]))
            if off > cfg.view_comfort:
                v["time_outside_35_sec"] += dt
            if off > cfg.view_limit:
                v["time_outside_70_sec"] += dt
        edge = core.point_rect_distance(
            core.to_frame(self._base, (self._person.x, self._person.y)), cfg.footprint)
        if self._distance is not None and edge < lim.closing_radius:
            v["max_closing_speed_mps"] = max(v["max_closing_speed_mps"],
                                             (self._distance - edge) / dt)
        self._distance = edge

    def close(self, now: float, outcome: str) -> None:
        """Close the window with `outcome` (once)."""
        if self.closed is None:
            self.closed, self.outcome = now, outcome

    def row(self, info: EpisodeInfo) -> dict:
        """Return the CSV row: the episode's settings and its metrics."""
        v = dict(self.values)
        if not math.isfinite(v["min_clearance_m"]):
            v["min_clearance_m"] = None
        outcome = self.outcome or "aborted"
        end = self.closed if self.closed is not None else self._last
        duration = None if self.opened is None or end is None else end - self.opened
        success = outcome in SUCCESSES and not any(v[key] for key in VIOLATIONS)
        row = {
            "episode_id": info.episode_id, "seed": info.seed,
            "facing_deg": round(info.facing_deg, 3), "hand_side": info.hand_side,
            "hand_extends": int(info.hand_extends),
            "reaction_delay_min": info.reaction_delay[0],
            "reaction_delay_max": info.reaction_delay[1],
            "facing_change_deg": round(info.facing_change_deg, 3),
            "faults": " ".join(f"{f:g}" for f in info.faults),
            "success": int(success), "outcome": outcome,
            **{key: _rounded(value) for key, value in v.items()},
            "duration_sec": _rounded(duration),
        }
        return {key: row[key] for key in COLUMNS}


def _rounded(value):
    if value is None:
        return ""
    return round(value, 4) if isinstance(value, float) else value


def write_row(path: Path, row: dict) -> None:
    """Append `row` to the CSV at `path`, writing the header when the file is new or empty."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    new = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        if new:
            writer.writeheader()
        writer.writerow({key: row.get(key, "") for key in COLUMNS})


def default_results_path(now: datetime, base_dir: Path) -> Path:
    """Return ``base_dir/handover_<YYYYmmdd_HHMMSS>.csv``."""
    return base_dir / f"handover_{now:%Y%m%d_%H%M%S}.csv"


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class HandoverRecorder(Node):
    """Collect the ground truth of one episode and write its row."""

    def __init__(self, **kwargs) -> None:
        super().__init__("handover_recorder", **kwargs)
        period = self._declare_parameters()
        self._written = False
        self._hand: core.Point3 = (0.0, 0.0, 0.0)
        self._take = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        sub = self.create_subscription
        sub(PoseStamped, f"{GROUND_TRUTH}/body_pose", self._on_body, 10)
        sub(PoseStamped, f"{GROUND_TRUTH}/hand_pose", self._on_hand, 10)
        sub(Bool, f"{GROUND_TRUTH}/take", self._on_take, 10)
        sub(Odometry, "/odom", self._on_odom, 10)
        sub(String, f"/grasp/{self._object}/state", self._on_grasp, 10)
        sub(String, PHASE_TOPIC, self._on_phase, LATCHED_QOS)
        sub(String, SAFETY_STATE_TOPIC, self._on_safety, LATCHED_QOS)
        sub(String, "/person/fault_active", self._on_fault, LATCHED_QOS)
        sub(PoseStamped, PLACE_TARGET_TOPIC, self._on_place, 10)
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(f"Handover recorder: episode {self._info.episode_id} -> "
                               f"{self._path}")

    def _declare_parameters(self) -> float:
        """Declare and check the parameters; returns the timer period (s)."""
        p = self.declare_parameter
        shared = {key: p(key, value).value for key, value in core.SHARED_DEFAULTS.items()}
        cfg = core.config_from_parameters(shared)
        positive = {
            "rate": p("rate", RATE_HZ).value,
            "moving_speed": p("moving_speed", MOVING_SPEED_MPS).value,
            "view_radius": p("view_radius", VIEW_RADIUS_M).value,
            "closing_radius": p("closing_radius", CLOSING_RADIUS_M).value,
            "place_tolerance": p("place_tolerance", PLACE_TOLERANCE_M).value,
        }
        for name, value in positive.items():
            if not value > 0.0:
                raise ValueError(f"{name} must be positive, got {value}")
        delay = [float(v) for v in p("reaction_delay", [0.5, 1.0]).value]
        faults = [float(v) for v in p("faults", [0.0] * 12).value]
        if len(delay) != 2:
            raise ValueError(f"reaction_delay must be [min, max], got {delay}")
        self._info = EpisodeInfo(
            episode_id=p("episode_id", 0).value, seed=p("seed", 0).value,
            facing_deg=math.degrees(p("facing_yaw", 0.0).value),
            hand_side=p("hand_side", "left").value, hand_extends=p("hand_extends", True).value,
            reaction_delay=tuple(delay),
            facing_change_deg=math.degrees(p("facing_change", 0.0).value),
            faults=tuple(faults),
        )
        self._object = p("object_name", "vitamin_bottle").value
        results = p("results_file", "").value
        self._path = (Path(results).expanduser() if results
                      else default_results_path(datetime.now(), DEFAULT_RESULTS_DIR))
        self._metrics = EpisodeMetrics(cfg, Limits(
            positive["moving_speed"], positive["view_radius"], positive["closing_radius"],
            positive["place_tolerance"]))
        return 1.0 / positive["rate"]

    def _now(self) -> float:
        """Node clock (sim time in the sim) in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- inputs ----------------------------------------------------------------------
    def _on_body(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._metrics.on_person(core.PersonState(
            p.x, p.y, _yaw_of(msg.pose.orientation), self._hand, False, self._take, stamp))

    def _on_hand(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._hand = (p.x, p.y, p.z)

    def _on_take(self, msg: Bool) -> None:
        self._take = msg.data

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist.linear
        self._metrics.on_base((pose.position.x, pose.position.y, _yaw_of(pose.orientation)),
                              math.hypot(twist.x, twist.y))

    def _on_grasp(self, msg: String) -> None:
        self._metrics.on_grasp_state(msg.data)

    def _on_phase(self, msg: String) -> None:
        self._metrics.on_phase(msg.data, self._now())
        if self._metrics.closed is not None:
            self.write()

    def _on_safety(self, msg: String) -> None:
        self._metrics.on_safety_state(msg.data)

    def _on_fault(self, msg: String) -> None:
        self._metrics.on_fault(msg.data, self._now())

    def _on_place(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._metrics.on_place_target((p.x, p.y, p.z))

    def _tcp(self) -> Optional[core.Point3]:
        try:
            transform = self._tf_buffer.lookup_transform(ODOM_FRAME, TCP_FRAME, Time())
        except TransformException:
            return None
        t = transform.transform.translation
        return t.x, t.y, t.z

    def _tick(self) -> None:
        self._metrics.on_tcp(self._tcp())
        self._metrics.step(self._now())

    def write(self) -> Optional[dict]:
        """Write the row once (the episode closed, or shutdown with it open)."""
        if self._written or (self._metrics.opened is None and self._metrics.closed is None):
            return None
        row = self._metrics.row(self._info)
        write_row(self._path, row)
        self._written = True  # only now: a failed write is retried at shutdown
        self.get_logger().info(
            f"episode {row['episode_id']}: outcome={row['outcome']} success={row['success']} "
            f"-> {self._path}")
        return row


SPIN_PERIOD_SEC = 0.1


def main() -> None:
    # SIGINT / SIGTERM only set a flag (handover_core.run_until_signal).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = HandoverRecorder()
    try:
        core.run_until_signal(lambda: rclpy.spin_once(node, timeout_sec=SPIN_PERIOD_SEC),
                              rclpy.ok)
    except ExternalShutdownException:
        pass
    finally:
        node.write()  # an episode still open is written as aborted
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
