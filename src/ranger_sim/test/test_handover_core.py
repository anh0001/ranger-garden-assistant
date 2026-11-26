"""
Unit tests for scripts/handover_core.py and config/handover.yaml (no simulator, no casadi).

Pins the person model, the standoff candidates, the offer and place poses, the
speed ramp, the rule decisions and the config parsing, and checks the scene
facts the config relies on against worlds/small_house_world.sdf and, once the
house assets are fetched, the chair's collision mesh.
"""

import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG / "config" / "handover.yaml"
WORLD = PKG / "worlds" / "small_house_world.sdf"
CHAIR_MESH = (PKG / "models" / "third_party" / "aws_small_house" / "models"
              / "aws_robomaker_residential_ChairA_01" / "meshes" / "aws_ChairA_01_collision.DAE")
PERSON_XY = (0.504, -0.858)
BACKREST_MIN_Z_M = 0.7


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


core = _load("handover_core", PKG / "scripts" / "handover_core.py")
CFG = core.default_config()


def _person(facing_deg=0.0, x=PERSON_XY[0], y=PERSON_XY[1]):
    return core.PersonState(x, y, math.radians(facing_deg), (0.0, 0.0, 0.0), False, False, 0.0)


def _standoff(person, bearing_deg, distance, **kwargs):
    pose = core.standoff_pose(person, math.radians(bearing_deg), distance, 0.0, CFG)
    defaults = dict(bearing=math.radians(bearing_deg), distance=distance, yaw_offset=0.0,
                    pose=pose, pre_pose=pose, box=core.standoff_box(pose, CFG),
                    offer_ok=True, place_ok=False, clearance=1.0)
    return core.Standoff(**{**defaults, **kwargs})


def _key(standoff):
    return round(math.degrees(standoff.bearing)), standoff.distance


# ---- person ------------------------------------------------------------------------


@pytest.mark.parametrize("xy, expected_deg", [
    ((0.504, 0.142), 90.0), ((0.504, -1.858), -90.0), ((1.504, -0.858), 0.0),
    ((-0.496, -0.858), 180.0),
])
def test_bearing_is_positive_to_the_persons_left(xy, expected_deg):
    assert math.degrees(core.bearing(_person(), xy)) == pytest.approx(expected_deg)


def test_bearing_wraps_across_pi():
    person = _person(facing_deg=170.0)
    direction = math.radians(-170.0)  # 20 deg to the person's left, across the cut at pi
    xy = (PERSON_XY[0] + math.cos(direction), PERSON_XY[1] + math.sin(direction))

    assert math.degrees(core.bearing(person, xy)) == pytest.approx(20.0)


def test_wrap_angle_stays_in_minus_pi_to_pi():
    for angle in (-3 * math.pi, -math.pi, 0.0, math.pi, 7.0):
        wrapped = core.wrap_angle(angle)
        assert -math.pi < wrapped <= math.pi
        assert math.cos(wrapped) == pytest.approx(math.cos(angle))


def test_capsule_runs_from_the_torso_to_the_knees():
    start, end, radius = core.capsule(_person(90.0), CFG)

    assert start == PERSON_XY
    assert end == pytest.approx((PERSON_XY[0], PERSON_XY[1] + CFG.person_leg_length))
    assert radius == CFG.person_torso_radius


def test_footprint_beside_the_torso_is_clear_by_the_gap():
    person = _person()
    # Base facing -y, its front edge 0.5 m to the person's left of the torso axis.
    pose = (PERSON_XY[0], PERSON_XY[1] + 0.5 + 0.394, -math.pi / 2)

    clearance = core.footprint_clearance(pose, CFG.footprint, person, CFG)

    assert clearance == pytest.approx(0.5 - CFG.person_torso_radius, abs=1e-9)


def test_footprint_in_front_of_the_knees_is_measured_from_them():
    person = _person()
    knees_x = PERSON_XY[0] + CFG.person_leg_length
    pose = (knees_x + 0.6 + 0.394, PERSON_XY[1], math.pi)

    clearance = core.footprint_clearance(pose, CFG.footprint, person, CFG)

    assert clearance == pytest.approx(0.6 - CFG.person_torso_radius, abs=1e-9)


def test_footprint_over_the_capsule_has_a_negative_clearance():
    person = _person()

    assert core.footprint_clearance((PERSON_XY[0] + 0.3, PERSON_XY[1], 0.0), CFG.footprint,
                                    person, CFG) < 0.0


def test_rect_gap_is_zero_on_overlap_and_the_distance_otherwise():
    square = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))

    assert core.rect_gap(square, (0.5, 0.5, 2.0, 2.0)) == 0.0
    assert core.rect_gap(square, (1.3, 0.2, 2.0, 0.8)) == pytest.approx(0.3)
    assert core.rect_gap(square, (1.3, 1.4, 2.0, 2.0)) == pytest.approx(0.5)


def test_segment_hits_rect_only_when_it_crosses():
    rect = (0.0, 0.0, 1.0, 1.0)

    assert core.segment_hits_rect((-1.0, 0.5), (2.0, 0.5), rect)
    assert not core.segment_hits_rect((-1.0, 1.5), (2.0, 1.5), rect)
    assert not core.segment_hits_rect((-1.0, -1.0), (-0.1, 2.0), rect)


# ---- standoffs and targets ---------------------------------------------------------------


def test_standoff_to_the_left_faces_the_person():
    pose = core.standoff_pose(_person(), math.radians(90.0), 0.5, 0.0, CFG)

    assert pose == pytest.approx((0.504, 0.036, -math.pi / 2), abs=1e-9)


def test_pre_standoff_is_nav_backoff_further_away():
    person = _person()
    pose = core.standoff_pose(person, math.radians(90.0), 0.5, 0.0, CFG)

    pre = core.pre_standoff_pose(person, pose, CFG)

    assert pre == pytest.approx((0.504, 0.036 + CFG.nav_backoff, -math.pi / 2), abs=1e-9)


def test_box_is_the_standoff_plus_minus_box_half():
    box = core.standoff_box((1.0, 2.0, 0.3), CFG)

    assert box == pytest.approx((0.88, 1.88, 1.12, 2.12))


def test_offer_pose_is_offer_distance_from_the_torso_at_offer_height():
    person = _person()
    pose = core.standoff_pose(person, math.radians(90.0), 0.5, 0.0, CFG)

    position, yaw = core.offer_pose(person, pose, CFG)

    assert position == pytest.approx((0.504, -0.408, 0.81), abs=1e-9)
    assert yaw == pytest.approx(-math.pi / 2)


def test_place_pose_is_the_free_table_point_nearest_the_base():
    person = _person()
    pose = core.standoff_pose(person, math.radians(90.0), 0.5, 0.0, CFG)

    (x, y, z), yaw = core.place_pose(person, pose, CFG)

    assert (x, y) == pytest.approx((1.00, -0.64), abs=core.PLACE_GRID_M)
    assert z == CFG.place_tcp_height
    assert yaw == pytest.approx(math.atan2(y - pose[1], x - pose[0]))


def test_place_points_keep_off_the_tableware_and_the_edges():
    points = core.place_points(_person(), CFG)
    x_min, y_min, x_max, y_max = CFG.table_top
    ware = CFG.table_keepouts[0]

    assert points
    for x, y in points:
        assert x_min + CFG.place_edge_inset - 1e-9 <= x <= x_max - CFG.place_edge_inset + 1e-9
        assert y_min + CFG.place_edge_inset - 1e-9 <= y <= y_max - CFG.place_edge_inset + 1e-9
        assert core.point_rect_distance((x, y), ware) >= CFG.place_free_radius - 1e-9
        assert CFG.place_min <= math.dist((x, y), PERSON_XY) <= CFG.place_reach


def test_no_place_pose_for_a_person_far_from_the_table():
    person = _person(x=-1.5, y=-0.858)

    assert core.place_pose(person, (-1.5, 0.5, 0.0), CFG) is None


def test_a_candidate_behind_the_backrest_is_not_free():
    person = _person(facing_deg=90.0)
    pose = core.standoff_pose(person, math.radians(90.0), 0.5, 0.0, CFG)

    assert not core.is_free(pose, person, CFG)


def test_a_standoff_on_the_table_is_not_free():
    person = _person()
    pose = core.standoff_pose(person, 0.0, 0.5, 0.0, CFG)

    assert not core.is_free(pose, person, CFG)


def test_candidates_are_all_free():
    person = _person()

    for standoff in core.candidates(person, CFG):
        assert core.is_free(standoff.pose, person, CFG)
        assert standoff.clearance >= CFG.static_margin


@pytest.mark.parametrize("facing, expected", [
    (0, [(-60, 0.7)]), (30, [(-90, 0.7)]), (45, []), (60, []), (90, []),
])
def test_only_these_candidates_reach_both_the_offer_and_the_table(facing, expected):
    both = [_key(s) for s in core.candidates(_person(facing), CFG) if s.offer_ok and s.place_ok]

    assert both == expected


# ---- speed ramp ----------------------------------------------------------------------------


@pytest.mark.parametrize("clearance, expected", [
    (0.2, 0.3), (0.5, 0.3), (1.0, 0.65), (1.5, 1.0), (3.0, 1.0), (math.nan, 0.3),
])
def test_speed_ramp(clearance, expected):
    assert core.speed_scale(clearance, CFG.ramp) == pytest.approx(expected)


# ---- decisions --------------------------------------------------------------------------------


def test_choose_prefers_both_flags_over_a_smaller_bearing():
    person = _person()
    options = (_standoff(person, 0, 0.5), _standoff(person, -60, 0.7, place_ok=True))

    assert _key(core.RuleDecisions(CFG).choose_standoff(person, options, False)) == (-60, 0.7)


def test_choose_takes_the_smallest_bearing_then_the_smallest_distance():
    person = _person()
    options = (_standoff(person, 60, 0.5), _standoff(person, -35, 0.9),
               _standoff(person, 35, 0.6), _standoff(person, -35, 0.8))

    assert _key(core.RuleDecisions(CFG).choose_standoff(person, options, False)) == (35, 0.6)


def test_choose_breaks_an_exact_tie_the_same_way_every_time():
    person = _person()
    options = (_standoff(person, 60, 0.7), _standoff(person, -60, 0.7))

    for ordering in (options, options[::-1]):
        assert _key(core.RuleDecisions(CFG).choose_standoff(person, ordering, False)) == (-60, 0.7)


def test_choose_skips_unreachable_offers_and_needs_the_place_when_asked():
    person = _person()
    options = (_standoff(person, 0, 0.5, offer_ok=False), _standoff(person, 35, 0.6))
    decisions = core.RuleDecisions(CFG)

    assert _key(decisions.choose_standoff(person, options, False)) == (35, 0.6)
    assert decisions.choose_standoff(person, options, True) is None
    assert decisions.choose_standoff(person, (), False) is None


def test_choose_for_a_person_facing_away_from_the_table():
    person = _person(90.0)

    chosen = core.RuleDecisions(CFG).choose_standoff(person, core.candidates(person, CFG), False)

    assert _key(chosen) == (0, 0.9)


@pytest.mark.parametrize("waited, extended, expected", [
    (0.0, True, core.OFFER), (5.0, True, core.OFFER), (1.0, False, core.WAIT),
    (3.0, False, core.PLACE),
])
def test_wait_or_go(waited, extended, expected):
    assert core.RuleDecisions(CFG).wait_or_go(waited, extended) == expected


def test_rechoose_after_a_large_turn_until_the_limit():
    decisions = core.RuleDecisions(CFG)

    assert not decisions.should_rechoose(0.0, 0.5, 0)
    assert decisions.should_rechoose(0.0, 0.7, 0)
    assert not decisions.should_rechoose(3.0, -3.0, 0)  # 0.28 rad across pi
    assert decisions.should_rechoose(0.0, -0.7, CFG.max_rechoices - 1)
    assert not decisions.should_rechoose(0.0, 0.7, CFG.max_rechoices)


def test_rule_decisions_satisfy_the_protocol():
    for name in ("choose_standoff", "request_scale", "wait_or_go", "should_rechoose"):
        assert callable(getattr(core.RuleDecisions(CFG), name))
        assert hasattr(core.Decisions, name)


# ---- config ------------------------------------------------------------------------------------


def _values(**changes):
    return {**core.SHARED_DEFAULTS, **changes}


@pytest.mark.parametrize("key, value", [
    ("stop_distance", math.nan), ("stop_distance", 0.0), ("offer_reach", "far"),
    ("candidate_distances", []), ("candidate_distances", [0.5, -0.1]),
    ("candidate_bearings", [4.0]), ("footprint", [0.0, 0.0, 1.0]),
    ("footprint", [0.1, -0.2, 0.4, 0.2]), ("base_obstacles", [1.0, 0.0, 0.5, 1.0]),
    ("table_top", [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0]), ("max_rechoices", 1.5),
    ("max_rechoices", -1), ("ramp_near", 2.0), ("place_min", 0.6), ("scale_near", 1.5),
    ("view_comfort", 1.3),
])
def test_config_rejects_a_malformed_value_naming_the_key(key, value):
    with pytest.raises(ValueError, match=key):
        core.config_from_parameters(_values(**{key: value}))


def test_config_rejects_a_missing_key():
    values = _values()
    del values["tcp_height_cap"]

    with pytest.raises(ValueError, match="tcp_height_cap"):
        core.config_from_parameters(values)


def test_config_takes_integers_for_floats_and_flat_rectangle_lists():
    cfg = core.config_from_parameters(_values(
        stop_distance=1, base_obstacles=[0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0]))

    assert cfg.stop_distance == 1.0
    assert cfg.base_obstacles == ((0.0, 0.0, 1.0, 1.0), (2.0, 2.0, 3.0, 3.0))


def _yaml():
    return yaml.safe_load(CONFIG.read_text())


def test_the_yaml_shared_section_is_the_shared_defaults():
    shared = _yaml()["/**"]["ros__parameters"]

    assert shared == core.SHARED_DEFAULTS


def test_the_yaml_floats_are_floats():
    # A ROS double parameter rejects a YAML integer at startup.
    for node, section in _yaml().items():
        for key, value in section["ros__parameters"].items():
            if key in core.INTEGER_KEYS or isinstance(value, str):
                continue
            values = value if isinstance(value, list) else [value]
            assert all(isinstance(v, float) for v in values), (node, key)


def test_the_yaml_config_parses():
    assert core.config_from_parameters(_yaml()["/**"]["ros__parameters"]) == CFG


# ---- scene facts -----------------------------------------------------------------------------


def _world_pose(name):
    world = ET.parse(WORLD).getroot().find("world")
    for include in world.findall("include"):
        if include.findtext("name").strip() == name:
            return [float(v) for v in include.findtext("pose").split()]
    for model in world.findall("model"):
        if model.get("name") == name:
            return [float(v) for v in model.findtext("pose").split()]
    raise KeyError(name)


def test_the_table_top_is_the_world_tables():
    world = ET.parse(WORLD).getroot().find("world")
    table = next(m for m in world.findall("model") if m.get("name") == "table")
    x, y = _world_pose("table")[:2]
    size = [float(v) for v in table.find(".//collision[@name='top_collision']//size").text.split()]

    # Model yaw pi: the box is symmetric, its extent is the same.
    assert CFG.table_top == pytest.approx(
        (x - size[0] / 2, y - size[1] / 2, x + size[0] / 2, y + size[1] / 2), abs=1e-6)
    assert CFG.base_keepouts[0][1:] == pytest.approx(CFG.table_top[1:])


def test_the_person_sits_on_chair_a_01_001():
    x, y, _, _, _, yaw = _world_pose("ChairA_01_001")

    assert (x, y) == pytest.approx(PERSON_XY, abs=1e-3)
    assert abs(core.wrap_angle(yaw - math.pi)) < 0.01  # its own +x points away from the table


def _chair_vertices_in_the_world():
    root = ET.parse(CHAIR_MESH).getroot()
    ns = {"c": root.tag.split("}")[0][1:]}
    unit = float(root.find("c:asset/c:unit", ns).get("meter"))
    node = root.find(".//c:visual_scene//c:node", ns)
    matrix = np.array(node.findtext("c:matrix", namespaces=ns).split(), float).reshape(4, 4)
    arrays = []
    for mesh in root.findall(".//c:mesh", ns):
        source = mesh.find("c:vertices/c:input[@semantic='POSITION']", ns).get("source")[1:]
        array = mesh.find(f"c:source[@id='{source}']/c:float_array", ns)
        arrays.append(np.array(array.text.split(), float).reshape(-1, 3))
    # The vertices are in centimetres under a node scaled by 0.1: both apply.
    local = ((matrix[:3, :3] @ np.vstack(arrays).T).T + matrix[:3, 3]) * unit
    x, y, z, _, _, yaw = _world_pose("ChairA_01_001")
    c, s = math.cos(yaw), math.sin(yaw)
    return np.c_[c * local[:, 0] - s * local[:, 1] + x, s * local[:, 0] + c * local[:, 1] + y,
                 local[:, 2] + z]


@pytest.mark.skipif(not CHAIR_MESH.exists(), reason="house assets not fetched")
def test_the_chair_box_and_backrest_are_the_meshes():
    vertices = _chair_vertices_in_the_world()
    backrest = vertices[vertices[:, 2] > BACKREST_MIN_Z_M]

    assert CFG.base_obstacles[0] == pytest.approx(
        (*vertices[:, :2].min(axis=0), *vertices[:, :2].max(axis=0)), abs=0.002)
    # The backrest is on the -x side, away from the table: a seated person faces +x.
    assert backrest[:, 0].max() < PERSON_XY[0] - 0.15
    arm = CFG.arm_keepouts[0]
    assert arm[0] <= backrest[:, 0].min() + 0.002 and backrest[:, 0].max() <= arm[2] + 0.002
    assert arm[1] <= backrest[:, 1].min() + 0.002 and backrest[:, 1].max() <= arm[3] + 0.002


# ---- routes (the scripted pose controller's path) ---------------------------------------


def test_the_controller_path_is_straight_along_a_body_axis():
    path = core.controller_path((0.0, 0.0), (1.0, 0.0), 0.0)

    assert all(abs(y) < 1e-9 for _, y in path)
    assert path[-1] == (1.0, 0.0)


def test_the_controller_path_runs_at_45_degrees_in_the_body_frame_first():
    path = core.controller_path((0.0, 0.0), (2.0, 1.0), 0.0)

    early = path[5]
    assert early[0] == pytest.approx(early[1], abs=1e-6)  # both axes saturated
    assert path[-1] == (2.0, 1.0)
    # One integration step (<= 0.25 x sqrt(2) x 0.02 s) past the spacing at most.
    assert all(math.dist(a, b) <= core.ROUTE_STEP_M + 0.0075 for a, b in zip(path, path[1:-1]))


def test_the_dog_leg_of_the_failed_gazebo_leg_passes_the_person():
    # Gazebo, 2026-11-25: away -> pre-standoff (-60 deg, 0.7 m) stopped at (-0.20, -0.50).
    person = _person()
    path = core.controller_path((-2.0, 0.0), (1.201, -2.065), math.radians(120.0))

    closest = min(core.footprint_clearance((x, y, math.radians(120.0)), CFG.footprint, person,
                                           CFG) for x, y in path)
    assert closest < CFG.stop_distance


def _route_clearance(start, legs, person):
    here, worst = start, math.inf
    for leg in legs:
        for x, y in core.controller_path(here[:2], leg[:2], leg[2]):
            worst = min(worst, core.footprint_clearance((x, y, leg[2]), CFG.footprint, person,
                                                        CFG))
        here = leg
    return worst


def test_a_free_leg_is_driven_directly():
    legs = core.route((-2.0, 0.0, math.pi), (-1.0, 0.5, math.pi), _person(), CFG)

    assert legs == ((-1.0, 0.5, math.pi),)


@pytest.mark.parametrize("facing", [0, 30, 60, 90])
def test_the_routes_of_the_episode_keep_the_margin(facing):
    person = _person(facing)
    standoff = core.RuleDecisions(CFG).choose_standoff(person, core.candidates(person, CFG),
                                                       False)
    away = (-2.0, 0.0, math.pi)

    for start, goal in ((away, standoff.pre_pose), (standoff.pre_pose, away)):
        legs = core.route(start, goal, person, CFG)
        assert legs[-1] == goal
        assert _route_clearance(start, legs, person) >= CFG.route_margin - 1e-9
        for leg in legs:
            corners = core.footprint_corners(leg, CFG.footprint)
            assert all(core.rect_gap(corners, rect) >= CFG.static_margin
                       for rect in CFG.base_obstacles)


def test_the_pick_slot_backs_out_before_it_turns():
    legs = core.route((0.535, 0.0, 0.0), (-2.0, 0.0, math.pi), _person(), CFG)

    assert len(legs) == 2 and legs[0][2] == 0.0  # straight back at the pick's yaw first


def test_without_a_free_route_the_leg_is_direct():
    walled = core.config_from_parameters({**core.SHARED_DEFAULTS, "base_obstacles": [
        -10.0, -10.0, 10.0, -0.5]})  # everything south of y = -0.5 is blocked

    legs = core.route((-2.0, 0.0, math.pi), (1.2, -2.07, 2.09), _person(), walled)

    assert legs == ((1.2, -2.07, 2.09),)


# ---- process ------------------------------------------------------------------------------


def test_run_until_signal_stops_on_sigint_and_restores_the_handler():
    import os
    import signal

    before = signal.getsignal(signal.SIGINT)
    spins = []

    def spin_once():
        spins.append(1)
        if len(spins) == 3:
            os.kill(os.getpid(), signal.SIGINT)  # only sets the flag: no KeyboardInterrupt

    core.run_until_signal(spin_once, lambda: True)

    assert len(spins) == 3
    assert signal.getsignal(signal.SIGINT) is before


def test_run_until_signal_stops_when_not_ok():
    spins = []

    core.run_until_signal(lambda: spins.append(1), lambda: len(spins) < 2)

    assert len(spins) == 2
