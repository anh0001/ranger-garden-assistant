"""
Pure geometry and rules of the Stage 0 handover study (docs/sim_handover_baseline.md).

A seated person is a plan-view capsule: from the torso axis along the facing
for `person_leg_length`, radius `person_torso_radius`. The robot base is its
footprint rectangle (base_footprint frame). Everything is in odom, whose x, y
are the world's; odom z = world z + ODOM_Z_OFFSET_M.

  * candidates(): standoffs (bearing x distance from the config) that are
    statically free, with a reach proxy for the offer pose and the table
    place pose (offer_ok / place_ok)
  * offer_pose() / place_pose(): the TCP targets, computed by rule, never
    by the decision maker
  * speed_scale(): the clearance ramp of the rule baseline
  * route(): the legs of a base drive that keep route_margin to the person
    and static_margin to the base obstacles along the path the scripted
    pose controller drives (turn in place, then each body axis saturated on
    its own): direct, or via one waypoint on rings around the person
  * Decisions / RuleDecisions: the four decisions a learned policy will take
    over (standoff, requested speed scale, wait or go, re-choose)
  * config_from_parameters(): the node parameters as a checked HandoverConfig
  * run_until_signal(): the nodes' spin loop, stopped by SIGINT / SIGTERM
    through a flag, never by an exception raised inside the executor

The nodes person_proxy, handover_safety, handover_baseline and
handover_recorder load this file by path; it imports neither rclpy nor
casadi. Every threshold is a starting value (config/handover.yaml says where
each comes from), not a validated one. Simulation only.
"""

import dataclasses
import math
import signal
import threading
from typing import Callable, Optional, Protocol

Rect = tuple[float, float, float, float]  # x_min, y_min, x_max, y_max
Pose2D = tuple[float, float, float]  # x, y (m), yaw (rad) in odom
Point2 = tuple[float, float]
Point3 = tuple[float, float, float]

ODOM_Z_OFFSET_M = 0.010  # Gazebo rests base_footprint 10 mm below the ground
SHOULDER_XY_M = (0.15, 0.0)  # piper_link1/2 origin in base_footprint (URDF), z 0.53
PLACE_GRID_M = 0.02  # spacing of the table points place_pose() considers
WAIT, OFFER, PLACE = "wait", "offer", "place"
RECT_KEYS = ("base_obstacles", "base_keepouts", "arm_keepouts", "table_keepouts")
SINGLE_RECT_KEYS = ("footprint", "table_top")

# Parameter name -> default of every shared key of config/handover.yaml (`/**`).
# The nodes declare these in a loop; the YAML is the documented source.
SHARED_DEFAULTS: dict = {
    "offer_height": 0.80,
    "offer_distance": 0.45,
    "candidate_distances": [0.5, 0.6, 0.7, 0.8, 0.9],
    "candidate_bearings": [-1.5708, -1.0472, -0.6109, 0.0, 0.6109, 1.0472, 1.5708],
    "view_comfort": 0.6109,
    "view_limit": 1.2217,
    "stop_distance": 0.15,
    "tcp_height_cap": 1.00,
    "person_torso_radius": 0.22,
    "person_leg_length": 0.45,
    "ramp_near": 0.5,
    "ramp_far": 1.5,
    "scale_near": 0.3,
    "person_timeout": 0.3,
    "max_body_jump": 0.15,
    "jump_hold_sec": 1.0,
    "hand_wait_sec": 3.0,
    "take_timeout_sec": 10.0,
    "rechoose_facing": 0.6109,
    "max_rechoices": 2,
    "place_min": 0.35,
    "place_reach": 0.55,
    "place_edge_inset": 0.10,
    "place_free_radius": 0.06,
    "place_tcp_height": 0.871,
    "place_hover_height": 0.962,
    "nav_backoff": 0.30,
    "route_margin": 0.35,
    "box_half": 0.12,
    "static_margin": 0.05,
    "yaw_margin": 0.10,
    "offer_reach": 0.58,
    "robot_place_reach": 0.48,
    "footprint": [-0.34, -0.25, 0.394, 0.25],
    "base_obstacles": [0.271, -1.098, 0.737, -0.614, -1.273, -2.859, -0.228, -2.02,
                       -0.30, -3.40, 2.60, -3.05],
    "base_keepouts": [0.93, -1.20, 1.60, 0.60],
    "arm_keepouts": [0.271, -1.06, 0.34, -0.66],
    "table_top": [0.90, -1.20, 1.60, 0.60],
    "table_keepouts": [1.111, -1.069, 1.480, -0.738],
}
INTEGER_KEYS = frozenset({"max_rechoices"})
LIST_KEYS = frozenset({"candidate_distances", "candidate_bearings", *RECT_KEYS,
                       *SINGLE_RECT_KEYS})
# Scalars that may be zero or negative: the offer height is a world z.
SIGNED_KEYS = frozenset({"offer_height"})


@dataclasses.dataclass(frozen=True)
class Ramp:
    """Requested speed scale: scale_near at clearance <= near, 1.0 at >= far, linear between."""

    near: float  # m
    far: float  # m
    scale_near: float


@dataclasses.dataclass(frozen=True)
class HandoverConfig:
    """Every threshold of the study (config/handover.yaml); see config_from_parameters."""

    offer_height: float  # world z of the offered TCP
    offer_distance: float  # torso axis -> offer point, plan view
    candidate_distances: tuple[float, ...]  # torso axis -> footprint front edge
    candidate_bearings: tuple[float, ...]  # rad, relative to the facing, + = person's left
    view_comfort: float  # rad
    view_limit: float  # rad
    stop_distance: float  # footprint -> capsule, hard stop
    tcp_height_cap: float  # odom z, hard
    person_torso_radius: float
    person_leg_length: float
    ramp: Ramp
    person_timeout: float  # s
    max_body_jump: float  # m between two samples
    jump_hold_sec: float
    hand_wait_sec: float
    take_timeout_sec: float
    rechoose_facing: float  # rad
    max_rechoices: int
    place_min: float  # torso axis -> place point, plan view
    place_reach: float
    place_edge_inset: float
    place_free_radius: float
    place_tcp_height: float  # odom z
    place_hover_height: float  # odom z above the place point before lowering
    nav_backoff: float  # standoff -> navigation goal, away from the person
    route_margin: float  # footprint -> capsule along a base route
    box_half: float  # MPC base box half width around the standoff
    static_margin: float
    yaw_margin: float  # rad, the standoff is checked at yaw +- this
    offer_reach: float  # shoulder -> offer point, plan view, reach proxy
    robot_place_reach: float  # shoulder -> place point, plan view, reach proxy
    footprint: Rect  # base_footprint frame
    base_obstacles: tuple[Rect, ...]  # odom; the footprint keeps static_margin
    base_keepouts: tuple[Rect, ...]  # odom; the footprint may not overlap
    arm_keepouts: tuple[Rect, ...]  # odom; no shoulder/torso -> offer line crosses
    table_top: Rect  # odom
    table_keepouts: tuple[Rect, ...]  # odom; objects on the table


@dataclasses.dataclass(frozen=True)
class PersonState:
    """A seated person: torso axis, facing, hand, and the two signals of the handover."""

    x: float
    y: float
    facing: float  # rad in odom
    hand: Point3  # odom
    hand_extended: bool
    take: bool
    stamp_sec: float


@dataclasses.dataclass(frozen=True)
class Standoff:
    """A candidate base pose next to the person and what the reach proxy says about it."""

    bearing: float  # rad, relative to the facing
    distance: float  # torso axis -> footprint front edge
    yaw_offset: float  # rad, added to "facing the person"
    pose: Pose2D  # base_footprint in odom
    pre_pose: Pose2D  # the navigation goal, nav_backoff further away
    box: Rect  # whole-body MPC base box
    offer_ok: bool
    place_ok: bool
    clearance: float  # footprint -> capsule at `pose`


# ---- configuration --------------------------------------------------------------


def _number(key: str, value) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a number, got {value!r}")
    if not math.isfinite(value):
        raise ValueError(f"{key} must be finite, got {value}")
    return float(value)


def _numbers(key: str, value) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not hasattr(value, "__len__"):
        raise ValueError(f"{key} must be a list of numbers, got {value!r}")
    numbers = tuple(_number(key, v) for v in value)
    if not numbers:
        raise ValueError(f"{key} must not be empty")
    return numbers


def _rects(key: str, value) -> tuple[Rect, ...]:
    flat = _numbers(key, value)
    if len(flat) % 4:
        raise ValueError(f"{key}: {len(flat)} values is not a multiple of 4 "
                         "(x_min, y_min, x_max, y_max per rectangle)")
    rects = tuple(flat[i:i + 4] for i in range(0, len(flat), 4))
    for x_min, y_min, x_max, y_max in rects:
        if not (x_min < x_max and y_min < y_max):
            raise ValueError(f"{key} needs x_min < x_max and y_min < y_max, got {list(flat)}")
    return rects


def _parse(key: str, value):
    if key in INTEGER_KEYS:
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"{key} must be a non-negative integer, got {value!r}")
        return value
    if key in RECT_KEYS:
        return _rects(key, value)
    if key in SINGLE_RECT_KEYS:
        rects = _rects(key, value)
        if len(rects) != 1:
            raise ValueError(f"{key} must be one rectangle (4 values), got {list(value)}")
        return rects[0]
    if key in LIST_KEYS:
        return _numbers(key, value)
    number = _number(key, value)
    if key not in SIGNED_KEYS and number <= 0.0:
        raise ValueError(f"{key} must be positive, got {number}")
    return number


def _check_relations(v: dict) -> None:
    if min(v["candidate_distances"]) <= 0.0:
        raise ValueError(f"candidate_distances must be positive, got {v['candidate_distances']}")
    if max(abs(b) for b in v["candidate_bearings"]) > math.pi:
        raise ValueError(f"candidate_bearings must be within [-pi, pi], "
                         f"got {v['candidate_bearings']}")
    x_min, y_min, x_max, y_max = v["footprint"]
    if not (x_min < 0.0 < x_max and y_min < 0.0 < y_max):
        raise ValueError(f"footprint must contain base_footprint's origin, got {v['footprint']}")
    for low, high in (("ramp_near", "ramp_far"), ("place_min", "place_reach"),
                      ("view_comfort", "view_limit")):
        if not v[low] < v[high]:
            raise ValueError(f"{low} must be less than {high}, got {v[low]} and {v[high]}")
    if v["scale_near"] > 1.0:
        raise ValueError(f"scale_near must be at most 1, got {v['scale_near']}")


def config_from_parameters(values: dict) -> HandoverConfig:
    """Return the checked config of the shared parameters; ValueError naming the bad key."""
    missing = sorted(set(SHARED_DEFAULTS) - set(values))
    if missing:
        raise ValueError(f"{missing[0]} is missing")
    v = {key: _parse(key, values[key]) for key in SHARED_DEFAULTS}
    _check_relations(v)
    ramp = Ramp(v.pop("ramp_near"), v.pop("ramp_far"), v.pop("scale_near"))
    return HandoverConfig(ramp=ramp, **v)


def default_config() -> HandoverConfig:
    """Return the config of SHARED_DEFAULTS."""
    return config_from_parameters(SHARED_DEFAULTS)


# ---- plane geometry ------------------------------------------------------------


def wrap_angle(angle: float) -> float:
    """`angle` (rad) wrapped into (-pi, pi]."""
    wrapped = math.remainder(angle, 2.0 * math.pi)
    return math.pi if wrapped <= -math.pi else wrapped


def to_frame(pose: Pose2D, point: Point2) -> Point2:
    """`point` (odom) in the frame of `pose`."""
    x, y, yaw = pose
    dx, dy = point[0] - x, point[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def from_frame(pose: Pose2D, point: Point2) -> Point2:
    """`point` given in the frame of `pose`, in odom."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * point[0] - s * point[1], y + s * point[0] + c * point[1]


def point_rect_distance(point: Point2, rect: Rect) -> float:
    """Distance from `point` to the axis-aligned `rect`; 0 inside."""
    x_min, y_min, x_max, y_max = rect
    dx = max(x_min - point[0], 0.0, point[0] - x_max)
    dy = max(y_min - point[1], 0.0, point[1] - y_max)
    return math.hypot(dx, dy)


def point_segment_distance(point: Point2, a: Point2, b: Point2) -> float:
    """Distance from `point` to the segment a-b."""
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0.0 else ((point[0] - ax) * dx + (point[1] - ay) * dy) / length2
    t = min(1.0, max(0.0, t))
    return math.hypot(point[0] - (ax + t * dx), point[1] - (ay + t * dy))


def segment_hits_rect(a: Point2, b: Point2, rect: Rect) -> bool:
    """Whether the segment a-b touches the axis-aligned `rect` (Liang-Barsky clipping)."""
    x_min, y_min, x_max, y_max = rect
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, a[0] - x_min), (dx, x_max - a[0]), (-dy, a[1] - y_min),
                 (dy, y_max - a[1])):
        if p == 0.0:
            if q < 0.0:
                return False
            continue
        t = q / p
        if p < 0.0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return False
    return True


def _rect_corners(rect: Rect) -> tuple[Point2, ...]:
    x_min, y_min, x_max, y_max = rect
    return (x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)


def segment_rect_distance(a: Point2, b: Point2, rect: Rect) -> float:
    """Distance from the segment a-b to the axis-aligned `rect`; 0 when they touch."""
    if segment_hits_rect(a, b, rect):
        return 0.0
    # Disjoint convex shapes: the closest pair has a vertex of one of them.
    return min(point_rect_distance(a, rect), point_rect_distance(b, rect),
               *(point_segment_distance(c, a, b) for c in _rect_corners(rect)))


def _edges(points: tuple[Point2, ...]):
    return zip(points, points[1:] + points[:1])


def _overlap(points: tuple[Point2, ...], rect: Rect) -> bool:
    """Whether the convex polygon `points` and the axis-aligned `rect` overlap (SAT)."""
    corners = _rect_corners(rect)
    axes = [(1.0, 0.0), (0.0, 1.0)]
    axes += [(a[1] - b[1], b[0] - a[0]) for a, b in _edges(points)]
    for nx, ny in axes:
        own = [nx * p[0] + ny * p[1] for p in points]
        other = [nx * c[0] + ny * c[1] for c in corners]
        if max(own) < min(other) or max(other) < min(own):
            return False
    return True


def rect_gap(points: tuple[Point2, ...], rect: Rect) -> float:
    """Gap between the convex polygon `points` and the axis-aligned `rect`; 0 when they overlap."""
    if _overlap(points, rect):
        return 0.0
    return min(min(point_rect_distance(p, rect) for p in points),
               min(point_segment_distance(c, a, b)
                   for c in _rect_corners(rect) for a, b in _edges(points)))


def footprint_corners(pose: Pose2D, footprint: Rect) -> tuple[Point2, ...]:
    """Return the footprint rectangle at `pose` in odom, counter-clockwise."""
    return tuple(from_frame(pose, corner) for corner in _rect_corners(footprint))


# ---- person ---------------------------------------------------------------------


def bearing(person: PersonState, xy: Point2) -> float:
    """Angle of `xy` seen from the torso axis, relative to the facing; + = person's left."""
    return wrap_angle(math.atan2(xy[1] - person.y, xy[0] - person.x) - person.facing)


def capsule(person: PersonState, cfg: HandoverConfig) -> tuple[Point2, Point2, float]:
    """Plan-view body: segment torso axis -> knees along the facing, and its radius."""
    knees = (person.x + cfg.person_leg_length * math.cos(person.facing),
             person.y + cfg.person_leg_length * math.sin(person.facing))
    return (person.x, person.y), knees, cfg.person_torso_radius


def footprint_clearance(pose: Pose2D, footprint: Rect, person: PersonState,
                        cfg: HandoverConfig) -> float:
    """Distance from the footprint at `pose` to the person's capsule; negative inside it."""
    start, end, radius = capsule(person, cfg)
    local = segment_rect_distance(to_frame(pose, start), to_frame(pose, end), footprint)
    return local - radius


# ---- standoffs and targets ------------------------------------------------------------


def standoff_pose(person: PersonState, bearing_rad: float, distance: float, yaw_offset: float,
                  cfg: HandoverConfig) -> Pose2D:
    """Return the base pose facing the person, front edge `distance` away along `bearing_rad`."""
    direction = person.facing + bearing_rad
    reach = distance + cfg.footprint[2]
    return (person.x + reach * math.cos(direction), person.y + reach * math.sin(direction),
            wrap_angle(direction + math.pi + yaw_offset))


def pre_standoff_pose(person: PersonState, pose: Pose2D, cfg: HandoverConfig) -> Pose2D:
    """`pose` moved nav_backoff further from the torso axis: the navigation goal."""
    direction = math.atan2(pose[1] - person.y, pose[0] - person.x)
    return (pose[0] + cfg.nav_backoff * math.cos(direction),
            pose[1] + cfg.nav_backoff * math.sin(direction), pose[2])


def standoff_box(pose: Pose2D, cfg: HandoverConfig) -> Rect:
    """Return the whole-body MPC's base box: the standoff +- box_half."""
    x, y, _ = pose
    h = cfg.box_half
    return x - h, y - h, x + h, y + h


def offer_pose(person: PersonState, pose: Pose2D, cfg: HandoverConfig) -> tuple[Point3, float]:
    """Return the offer TCP (odom), offer_distance towards the base at `pose`, and its yaw."""
    direction = math.atan2(pose[1] - person.y, pose[0] - person.x)
    point = (person.x + cfg.offer_distance * math.cos(direction),
             person.y + cfg.offer_distance * math.sin(direction),
             cfg.offer_height + ODOM_Z_OFFSET_M)
    return point, pose[2]


def place_points(person: PersonState, cfg: HandoverConfig) -> list[Point2]:
    """Grid points of the table top that the person reaches and nothing stands on."""
    x_min, y_min, x_max, y_max = cfg.table_top
    inset = cfg.place_edge_inset
    columns = int(math.floor((x_max - x_min - 2 * inset) / PLACE_GRID_M + 1e-9)) + 1
    rows = int(math.floor((y_max - y_min - 2 * inset) / PLACE_GRID_M + 1e-9)) + 1
    r = cfg.place_free_radius
    grown = [(a - r, b - r, c + r, d + r) for a, b, c, d in cfg.table_keepouts]
    points = []
    for i in range(max(0, columns)):
        for j in range(max(0, rows)):
            point = (round(x_min + inset + i * PLACE_GRID_M, 6),
                     round(y_min + inset + j * PLACE_GRID_M, 6))
            reach = math.hypot(point[0] - person.x, point[1] - person.y)
            if not cfg.place_min <= reach <= cfg.place_reach:
                continue
            if any(point_rect_distance(point, rect) == 0.0 for rect in grown):
                continue
            points.append(point)
    return points


def place_pose(person: PersonState, pose: Pose2D,
               cfg: HandoverConfig) -> Optional[tuple[Point3, float]]:
    """Return the place point nearest to the base at `pose` and the heading to it, or None."""
    points = place_points(person, cfg)
    if not points:
        return None
    point = min(points, key=lambda p: (math.hypot(p[0] - pose[0], p[1] - pose[1]), p))
    yaw = math.atan2(point[1] - pose[1], point[0] - pose[0])
    return (point[0], point[1], cfg.place_tcp_height), yaw


def shoulder_distance(pose: Pose2D, box: Rect, point: Point2) -> float:
    """Plan-view distance of `point` to the shoulder, base anywhere in `box` at pose's yaw."""
    sx, sy = from_frame((0.0, 0.0, pose[2]), SHOULDER_XY_M)
    x_min, y_min, x_max, y_max = box
    return point_rect_distance(point, (x_min + sx, y_min + sy, x_max + sx, y_max + sy))


def _checked_poses(pose: Pose2D, box: Rect, cfg: HandoverConfig):
    for x, y in ((pose[0], pose[1]), *_rect_corners(box)):
        for dyaw in (0.0, -cfg.yaw_margin, cfg.yaw_margin):
            yield x, y, pose[2] + dyaw


def is_free(pose: Pose2D, person: PersonState, cfg: HandoverConfig) -> bool:
    """
    Whether a standoff is statically free.

    At the standoff and the four corners of its box, at its yaw and +-yaw_margin,
    the footprint keeps static_margin to the capsule and to every base
    obstacle and does not overlap a base keep-out; and neither the line
    shoulder -> offer point nor torso axis -> offer point crosses an arm keep-out.
    """
    box = standoff_box(pose, cfg)
    for checked in _checked_poses(pose, box, cfg):
        if footprint_clearance(checked, cfg.footprint, person, cfg) < cfg.static_margin:
            return False
        corners = footprint_corners(checked, cfg.footprint)
        if any(rect_gap(corners, rect) < cfg.static_margin for rect in cfg.base_obstacles):
            return False
        if any(rect_gap(corners, rect) <= 0.0 for rect in cfg.base_keepouts):
            return False
    offer, _ = offer_pose(person, pose, cfg)
    shoulder = from_frame(pose, SHOULDER_XY_M)
    for start in (shoulder, (person.x, person.y)):
        if any(segment_hits_rect(start, offer[:2], rect) for rect in cfg.arm_keepouts):
            return False
    return True


def candidates(person: PersonState, cfg: HandoverConfig,
               yaw_offset: float = 0.0) -> tuple[Standoff, ...]:
    """Return the statically free standoffs of the bearings x distances, with reach flags."""
    found = []
    for b in cfg.candidate_bearings:
        for distance in cfg.candidate_distances:
            pose = standoff_pose(person, b, distance, yaw_offset, cfg)
            if not is_free(pose, person, cfg):
                continue
            box = standoff_box(pose, cfg)
            offer, _ = offer_pose(person, pose, cfg)
            place = place_pose(person, pose, cfg)
            found.append(Standoff(
                bearing=b, distance=distance, yaw_offset=yaw_offset, pose=pose,
                pre_pose=pre_standoff_pose(person, pose, cfg), box=box,
                offer_ok=shoulder_distance(pose, box, offer[:2]) <= cfg.offer_reach,
                place_ok=(place is not None
                          and shoulder_distance(pose, box, place[0][:2])
                          <= cfg.robot_place_reach),
                clearance=footprint_clearance(pose, cfg.footprint, person, cfg),
            ))
    return tuple(found)


ROUTE_STEP_M = 0.05  # spacing of the footprint checks along a leg
# pick_place_demo's pose controller: per-axis ceiling (base_speed, BASE_SPEED_MPS),
# gain (BASE_GAIN, 1/s), done within BASE_TOLERANCE_M. The governed gate scales the
# whole twist, so the path does not depend on the speed scale.
CONTROLLER_AXIS_SPEED_MPS = 0.25
CONTROLLER_GAIN = 1.0
CONTROLLER_TOLERANCE_M = 0.005
CONTROLLER_DT_SEC = 0.02
ROUTE_TURN_STEP_RAD = 0.1  # spacing of the footprint checks along a turn in place
ROUTE_RADII_M = (1.2, 1.6, 2.0, 2.4)  # waypoint rings around the torso axis
ROUTE_ANGLES = 24  # waypoints per ring


def _pose_free(pose: Pose2D, person: PersonState, cfg: HandoverConfig, margin: float) -> bool:
    if footprint_clearance(pose, cfg.footprint, person, cfg) < margin:
        return False
    corners = footprint_corners(pose, cfg.footprint)
    if any(rect_gap(corners, rect) < cfg.static_margin for rect in cfg.base_obstacles):
        return False
    return not any(rect_gap(corners, rect) <= 0.0 for rect in cfg.base_keepouts)


def _turn_free(xy: Point2, start_yaw: float, end_yaw: float, person: PersonState,
               cfg: HandoverConfig, margin: float) -> bool:
    turn = wrap_angle(end_yaw - start_yaw)
    steps = max(1, int(math.ceil(abs(turn) / ROUTE_TURN_STEP_RAD)))
    return all(_pose_free((xy[0], xy[1], start_yaw + turn * i / steps), person, cfg, margin)
               for i in range(steps + 1))


def controller_path(start: Point2, end: Point2, yaw: float) -> list[Point2]:
    """
    Return points (ROUTE_STEP_M apart) of the pose controller's path at a fixed yaw.

    Each body axis moves at min(ceiling, gain x its error): with both errors
    large the base runs at 45 deg in its own frame, not straight at the goal.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = start
    points, travelled = [(x, y)], 0.0
    for _ in range(100000):
        dx, dy = end[0] - x, end[1] - y
        ex, ey = c * dx + s * dy, -s * dx + c * dy
        if math.hypot(ex, ey) < CONTROLLER_TOLERANCE_M:
            break
        vx = math.copysign(min(CONTROLLER_AXIS_SPEED_MPS, CONTROLLER_GAIN * abs(ex)), ex)
        vy = math.copysign(min(CONTROLLER_AXIS_SPEED_MPS, CONTROLLER_GAIN * abs(ey)), ey)
        step_x = (c * vx - s * vy) * CONTROLLER_DT_SEC
        step_y = (s * vx + c * vy) * CONTROLLER_DT_SEC
        x, y = x + step_x, y + step_y
        travelled += math.hypot(step_x, step_y)
        if travelled >= ROUTE_STEP_M:
            points.append((x, y))
            travelled = 0.0
    points.append(tuple(end))
    return points


def _leg_free(start: Point2, end: Point2, yaw: float, person: PersonState, cfg: HandoverConfig,
              margin: float) -> bool:
    return all(_pose_free((x, y, yaw), person, cfg, margin)
               for x, y in controller_path(start, end, yaw))


def _route_free(start: Pose2D, legs: tuple[Pose2D, ...], person: PersonState,
                cfg: HandoverConfig, margin: float) -> bool:
    """Whether turning to each leg's yaw where it begins, then translating, stays free."""
    here = start
    for leg in legs:
        if not _turn_free(here[:2], here[2], leg[2], person, cfg, margin):
            return False
        if not _leg_free(here[:2], leg[:2], leg[2], person, cfg, margin):
            return False
        here = leg
    return True


def _length(start: Pose2D, legs: tuple[Pose2D, ...]) -> float:
    points = [start[:2]] + [leg[:2] for leg in legs]
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def _shorter_free(start: Pose2D, legs: tuple[Pose2D, ...], best_length: float,
                  person: PersonState, cfg: HandoverConfig, margin: float) -> bool:
    return (_length(start, legs) < best_length
            and _pose_free(legs[0], person, cfg, margin)
            and _route_free(start, legs, person, cfg, margin))


def route(start: Pose2D, goal: Pose2D, person: PersonState,
          cfg: HandoverConfig) -> tuple[Pose2D, ...]:
    """
    Return the poses to drive through to `goal` (the last one is `goal`).

    Each leg turns in place to its pose's yaw, then translates to it along
    controller_path(), as pick_place_demo's pose controller drives (Nav2 is
    given the same poses as goals; its own path is not modelled). A route keeps route_margin
    (or less if an end is closer than that) between the footprint and the
    person's capsule, static_margin to every base obstacle, and off every base
    keep-out. Direct when free; else the shortest route via one waypoint on
    ROUTE_RADII_M around the torso axis, held at the start's or the goal's
    yaw; direct again when none is free (the safety layer still stops it).
    """
    margin = min(cfg.route_margin,
                 footprint_clearance(start, cfg.footprint, person, cfg),
                 footprint_clearance(goal, cfg.footprint, person, cfg))
    direct = (goal,)
    if _route_free(start, direct, person, cfg, margin):
        return direct
    best, best_length = direct, math.inf
    for radius in ROUTE_RADII_M:
        for k in range(ROUTE_ANGLES):
            angle = 2.0 * math.pi * k / ROUTE_ANGLES
            xy = (person.x + radius * math.cos(angle), person.y + radius * math.sin(angle))
            for yaw in (start[2], goal[2]):
                legs = ((xy[0], xy[1], yaw), goal)
                if _shorter_free(start, legs, best_length, person, cfg, margin):
                    best, best_length = legs, _length(start, legs)
    return best


def speed_scale(clearance: float, ramp: Ramp) -> float:
    """Return the ramp's scale at `clearance` (m); scale_near for a non-finite clearance."""
    if not math.isfinite(clearance) or clearance <= ramp.near:
        return ramp.scale_near
    if clearance >= ramp.far:
        return 1.0
    fraction = (clearance - ramp.near) / (ramp.far - ramp.near)
    return ramp.scale_near + fraction * (1.0 - ramp.scale_near)


# ---- decisions -------------------------------------------------------------------------


class Decisions(Protocol):
    """The four decisions of the handover: the rule baseline now, a learned policy later."""

    def choose_standoff(self, person: PersonState, options: tuple[Standoff, ...],
                        need_place: bool) -> Optional[Standoff]:
        """Return the standoff to drive to, or None for none."""

    def request_scale(self, clearance_m: float) -> float:
        """Return the requested speed scale in [0, 1] at footprint clearance `clearance_m`."""

    def wait_or_go(self, waited_sec: float, hand_extended: bool) -> str:
        """Return WAIT, OFFER or PLACE."""

    def should_rechoose(self, facing_at_choice: float, facing_now: float,
                        rechoices: int) -> bool:
        """Return True to back off and choose the standoff again."""


class RuleDecisions:
    """The rule baseline of the four decisions (docs/sim_handover_baseline.md)."""

    def __init__(self, cfg: HandoverConfig) -> None:
        self._cfg = cfg

    def choose_standoff(self, person: PersonState, options: tuple[Standoff, ...],
                        need_place: bool) -> Optional[Standoff]:
        """Offer-reachable (and place-reachable if needed): both flags, |bearing|, distance."""
        usable = [s for s in options if s.offer_ok and (s.place_ok or not need_place)]
        if not usable:
            return None
        return min(usable, key=lambda s: (not s.place_ok, abs(s.bearing), s.distance, s.bearing))

    def request_scale(self, clearance_m: float) -> float:
        """Request the clearance ramp's scale."""
        return speed_scale(clearance_m, self._cfg.ramp)

    def wait_or_go(self, waited_sec: float, hand_extended: bool) -> str:
        """Offer to an extended hand at once; place after hand_wait_sec without one."""
        if hand_extended:
            return OFFER
        return PLACE if waited_sec >= self._cfg.hand_wait_sec else WAIT

    def should_rechoose(self, facing_at_choice: float, facing_now: float,
                        rechoices: int) -> bool:
        """Re-choose when the facing moved more than rechoose_facing and re-choices are left."""
        turned = abs(wrap_angle(facing_now - facing_at_choice)) > self._cfg.rechoose_facing
        return turned and rechoices < self._cfg.max_rechoices


# ---- process ----------------------------------------------------------------------------


def run_until_signal(spin_once: Callable[[], None], ok: Callable[[], bool]) -> None:
    """
    Call spin_once() until ok() is False or SIGINT / SIGTERM arrives.

    The signals only set a flag. Python's own SIGINT handling raises
    KeyboardInterrupt wherever the main thread is, also inside rclpy's C
    calls, where it surfaces as "RuntimeError: Unable to convert call
    argument" and leaves the executor unusable for the exit path (seen in
    Gazebo, 2026-11-25). The previous handlers are restored on return.
    """
    stop = threading.Event()
    previous = {sig: signal.signal(sig, lambda signum, frame: stop.set())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        while ok() and not stop.is_set():
            spin_once()
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
