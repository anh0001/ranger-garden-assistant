"""
Geometry checks for the pick-and-place demo (no simulator required).

For every scene of config/pick_place_scenes.yaml, the arm waypoints, the
base approach, the wrist camera's view at the standoff and the place spot of
the transfer are checked against the object and the surfaces of the scene's
world with forward kinematics built from the robot URDF; the sim-only
steering lock and base joint-state plugins of urdf/ranger_gazebo.urdf.xacro
are checked against config/gz_bridge.yaml and
scripts/base_joint_state_publisher.py.

Needs the install space sourced: the xacro files resolve $(find ...).
"""

from dataclasses import dataclass
import functools
import importlib.util
import itertools
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import rclpy
import xacro
import yaml
from launch.actions import DeclareLaunchArgument
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF

PKG = Path(__file__).resolve().parents[1]
SCENES = yaml.safe_load((PKG / "config" / "pick_place_scenes.yaml").read_text())
SIM_XACRO = PKG / "urdf" / "ranger_gazebo.urdf.xacro"
BRIDGE = PKG / "config" / "gz_bridge.yaml"
GRASPABLE = PKG / "config" / "graspable_objects.yaml"
DEMO_LAUNCHES = ("pick_place_demo.launch.py", "record_pick_place_video.launch.py")
# Optional scene keys: a scene without them places with the pick's hover/grasp.
PLACE_WAYPOINTS = ("place_hover_joints", "place_grasp_joints")

# Gripper geometry in the piper_link6 (= gripper_base) frame, measured from
# piper_description/meshes/{link7,link8,gripper_base}.STL: finger pads span
# PAD_Z along the approach (z) axis and +-PAD_HALF_HEIGHT along x; the palm
# ends at PALM_Z and spans PALM_X.
PAD_Z = (0.059, 0.136)
PAD_HALF_HEIGHT = 0.028
PALM_Z = 0.063
PALM_X = (-0.04, 0.0305)

TABLE_CLEARANCE_M = 0.02  # pads above the table top at GRASP
RIM_CLEARANCE_M = 0.01  # pads above the object's rim at HOVER
BODY_MARGIN_M = 0.005  # pads below the object's rim at GRASP
PALM_CLEARANCE_M = 0.01  # palm to the object's near face at GRASP
FINGER_CLEARANCE_M = 0.005  # open finger to the object's surface, per side
MAX_DESCENT_WANDER_M = 0.005  # link6 x drift along the hover -> grasp path
RESTING_TOLERANCE_M = 0.001  # object bottom vs. the table top
MIN_SUPPORT_M = 0.01  # object centre (its centre of mass) in from the table's front edge
CARRY_CLEARANCE_M = 0.05  # held object above the pick table top in the carry pose
PLACE_HOVER_MIN_M = 0.05  # place hover above the place grasp: the held object clears the surface
# Base collisions (wheels, lidar, camera, body) to the collision boxes of the
# surface it drives up to: the house counter is a cabinet down to the floor,
# and the lidar sticks out 0.394 m ahead of base_footprint.
BASE_CLEARANCE_M = 0.03
# AWS CookingBench_01 on the house counter: half its length along world x,
# measured from its visual mesh (x -3.572..-2.566 at the generated pose).
COOKTOP_HALF_LENGTH_M = 0.503
COOKTOP_CLEARANCE_M = 0.10  # house place spot to the cooktop: open fingers (6 cm) + margin
LEVEL_TOLERANCE = 0.02  # |approach z| of a level gripper (about 1 deg)
CAMERA_CENTRE_MAX_DEG = 15.0  # object off the wrist camera's optical axis at the standoff
D405_RANGE_M = (0.07, 0.50)  # RealSense D405 depth range
# Clear radius around the place spot: the open fingers (35 mm stroke + the
# finger) straddle the object, the pads end 3.1 cm past its centre.
PLACE_FREE_RADIUS_M = 0.06
PATH_SAMPLES = 21  # joint-space path samples (MoveIt plans these straight)

# Gazebo rests base_footprint 10 mm below the ground (measured with
# `ign model -m ranger -p`: steering locked, concentric wheel collisions);
# the wheel collision geometry predicts 20 mm (_wheel_footprint_height).
# The waypoints must work for both.
GAZEBO_FOOTPRINT_HEIGHT_M = -0.010

WHEEL_LINKS = ("fl_wheel_link", "fr_wheel_link", "rl_wheel_link", "rr_wheel_link")
STEERING_JOINTS = {
    "fl_steering_joint",
    "fr_steering_joint",
    "rl_steering_joint",
    "rr_steering_joint",
}
JOINT_POSITION_CONTROLLER = "ignition::gazebo::systems::JointPositionController"
JOINT_STATE_PUBLISHER = "ignition::gazebo::systems::JointStatePublisher"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


demo = _load("pick_place_demo", PKG / "scripts" / "pick_place_demo.py")
bjsp = _load("base_joint_state_publisher", PKG / "scripts" / "base_joint_state_publisher.py")
house_gen = _load("generate_small_house_world", PKG / "tools" / "generate_small_house_world.py")


def _launch_default(launch_name, arg_name):
    module = _load(launch_name.replace(".", "_"), PKG / "launch" / launch_name)
    entities = module.generate_launch_description().entities
    arg = next(
        e for e in entities if isinstance(e, DeclareLaunchArgument) and e.name == arg_name
    )
    return arg.default_value[0].text


# ---- forward kinematics ----------------------------------------------------


def _tf(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    """4x4 transform; URDF fixed-axis roll-pitch-yaw (R = Rz @ Ry @ Rx)."""
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    t = np.eye(4)
    t[:3, :3] = rz @ ry @ rx
    t[:3, 3] = xyz
    return t


def _axis_angle(axis, angle):
    """4x4 rotation by `angle` about the unit-normalised `axis` (Rodrigues)."""
    k = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    t = np.eye(4)
    t[:3, :3] = np.eye(3) + np.sin(angle) * kx + (1 - np.cos(angle)) * kx @ kx
    return t


def _chain(robot, tip, root="base_footprint"):
    by_child = {joint.child: joint for joint in robot.joints}
    chain = []
    link = tip
    while link != root:
        joint = by_child[link]
        chain.append(joint)
        link = joint.parent
    return chain[::-1]


def _fk(robot, tip, arm=(0.0,) * 6):
    """Pose of `tip` in base_footprint for arm joint values `arm` (rad)."""
    q = dict(zip(demo.ARM_JOINTS, arm))
    t = np.eye(4)
    for joint in _chain(robot, tip):
        origin = joint.origin
        if origin is not None:
            t = t @ _tf(origin.xyz, origin.rpy)
        if joint.type in ("revolute", "continuous"):
            t = t @ _axis_angle(joint.axis, q.get(joint.name, 0.0))
    return t


def _box_in_base(t, xs, zs):
    """(x_min, x_max, z_min, z_max) in base_footprint of a link6-frame x/z box."""
    corners = np.array([t @ (x, 0.0, z, 1.0) for x in xs for z in zs])
    return corners[:, 0].min(), corners[:, 0].max(), corners[:, 2].min(), corners[:, 2].max()


def _pads(t):
    return _box_in_base(t, (-PAD_HALF_HEIGHT, PAD_HALF_HEIGHT), PAD_Z)


def _palm_front_x(t):
    return _box_in_base(t, PALM_X, (PALM_Z,))[1]


# ---- scene -----------------------------------------------------------------


@dataclass(frozen=True)
class Scene:
    """Object (an upright cylinder) and table top in base_footprint, base at world (base_x, 0)."""

    obj_x: float
    obj_y: float
    obj_z: float  # centre
    obj_radius: float
    obj_height: float
    table_top: float

    @property
    def rim(self):
        return self.obj_z + self.obj_height / 2


@functools.lru_cache(maxsize=None)
def _world_root(world_file):
    return ET.parse(PKG / "worlds" / world_file).getroot()


def _world_model(root, name):
    return next(m for m in root.iter("model") if m.get("name") == name)


def _xyz(element):
    pose = element.findtext("pose")
    return [float(v) for v in pose.split()[:3]] if pose else [0.0, 0.0, 0.0]


def _object(root, name):
    """
    Return (x, y, centre_z, radius, height) in the world of the cylinder `name`.

    The object is an inline <model> or an <include> of a models/<X> wrapper
    (model://X); its collision cylinder may be offset in the link.
    """
    model = next((m for m in root.iter("model") if m.get("name") == name), None)
    if model is not None:
        origin = _xyz(model)
    else:
        include = next(i for i in root.iter("include") if i.findtext("name") == name)
        origin = _xyz(include)
        uri = include.findtext("uri").strip()
        assert uri.startswith("model://"), uri
        wrapper = PKG / "models" / uri[len("model://"):] / "model.sdf"
        model = ET.parse(wrapper).getroot().find("model")
    collision = model.find("link/collision")
    cylinder = collision.find("geometry/cylinder")
    x, y, z = np.add(origin, _xyz(collision))
    return x, y, z, float(cylinder.findtext("radius")), float(cylinder.findtext("length"))


def _table_top(root, name="table"):
    """World z of the table `name`'s top_collision and the world x of its front edge."""
    table = _world_model(root, name)
    top = next(c for c in table.iter("collision") if c.get("name") == "top_collision")
    size_x, size_y, thickness = (float(v) for v in top.findtext("geometry/box/size").split())
    yaw = float(table.findtext("pose").split()[5])
    half_x = (abs(math.cos(yaw)) * size_x + abs(math.sin(yaw)) * size_y) / 2
    table_x = _xyz(table)[0] + _xyz(top)[0]
    return _xyz(table)[2] + _xyz(top)[2] + thickness / 2, table_x - half_x


@dataclass(frozen=True)
class Surface:
    """A table top in the world: centre, yaw, size and top height."""

    x: float
    y: float
    yaw: float
    size_x: float
    size_y: float
    top_z: float

    def local(self, x, y):
        """World (x, y) in the top's own frame."""
        dx, dy = x - self.x, y - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def contains(self, x, y, margin=0.0):
        """Whether world (x, y) is over the top, at least `margin` in from its edges."""
        lx, ly = self.local(x, y)
        return abs(lx) <= self.size_x / 2 - margin and abs(ly) <= self.size_y / 2 - margin


def _surface(root, name):
    """Return the top_collision of the world model `name` (rotated tables included)."""
    model = _world_model(root, name)
    top = next(c for c in model.iter("collision") if c.get("name") == "top_collision")
    size_x, size_y, thickness = (float(v) for v in top.findtext("geometry/box/size").split())
    mx, my, mz = _xyz(model)
    yaw = float(model.findtext("pose").split()[5])
    cx, cy, cz = _xyz(top)
    c, s = math.cos(yaw), math.sin(yaw)
    return Surface(mx + c * cx - s * cy, my + s * cx + c * cy, yaw, size_x, size_y,
                   mz + cz + thickness / 2)


def _wheel_footprint_height(robot):
    """World z of base_footprint with the wheel collisions resting on z = 0."""
    bottoms = []
    for name in WHEEL_LINKS:
        collision = robot.link_map[name].collision
        centre = _fk(robot, name) @ _tf(collision.origin.xyz, collision.origin.rpy)
        bottoms.append(centre[2, 3] - collision.geometry.radius)
    return -min(bottoms)


def _scene(cfg, base_x, ground):
    """Scene `cfg` seen from a base at world x `base_x` whose footprint is at z `ground`."""
    root = _world_root(cfg.world_file)
    x, y, z, radius, height = _object(root, cfg.object_name)
    table_top, _ = _table_top(root)
    return Scene(
        obj_x=x - base_x,
        obj_y=y,
        obj_z=z - ground,
        obj_radius=radius,
        obj_height=height,
        table_top=table_top - ground,
    )


@pytest.fixture(scope="module", params=sorted(SCENES))
def scene_cfg(request):
    """One scene of config/pick_place_scenes.yaml."""
    return SimpleNamespace(name=request.param, **SCENES[request.param])


@pytest.fixture(scope="module")
def sim_urdf():
    return xacro.process_file(str(SIM_XACRO)).toxml()


@pytest.fixture(scope="module")
def robot(sim_urdf):
    return URDF.from_xml_string(sim_urdf)


@pytest.fixture(scope="module", params=["wheel_geometry", "gazebo_rest"])
def ground(request, robot):
    """World z of base_footprint: both the predicted and the measured value."""
    if request.param == "wheel_geometry":
        return _wheel_footprint_height(robot)
    return GAZEBO_FOOTPRINT_HEIGHT_M


# The base stops anywhere within BASE_TOLERANCE_M of the approach pose.
BASE_OFFSETS = pytest.mark.parametrize(
    "base_offset", [k * demo.BASE_TOLERANCE_M for k in (-1, 0, 1)]
)


def test_wheel_geometry_and_gazebo_agree_within_a_centimetre(robot):
    # If they drift apart, the ground band the waypoints are checked
    # against no longer describes the sim: re-measure and re-solve.
    assert abs(_wheel_footprint_height(robot) - GAZEBO_FOOTPRINT_HEIGHT_M) <= 0.011


def _waypoint(scene_cfg, name):
    """
    Arm joints of the waypoint `name`: "carry", or a scene key.

    A place_* key the scene lacks falls back to the pick's waypoint, as
    launch/pick_place_scenes.py does (OPTIONAL_KEYS).
    """
    if name == "carry":
        return demo.CARRY
    if name in PLACE_WAYPOINTS:
        return getattr(scene_cfg, name, getattr(scene_cfg, name[len("place_"):]))
    return getattr(scene_cfg, name)


# ---- arm waypoints vs. the world -------------------------------------------


def test_fk_reproduces_the_documented_tcps(robot):
    # Guards the FK helper itself: the demo docstring's TCPs.
    for joints, tcp in ((demo.HOME, (0.484, 0.0, 0.596)), (demo.CARRY, (0.340, 0.0, 0.950))):
        np.testing.assert_allclose(_fk(robot, "piper_tcp", joints)[:3, 3], tcp, atol=2e-3)


@pytest.mark.parametrize("waypoint", ["hover_joints", "grasp_joints", *PLACE_WAYPOINTS])
def test_grasp_approach_is_level_and_planar(robot, scene_cfg, waypoint):
    t = _fk(robot, "piper_link6", _waypoint(scene_cfg, waypoint))

    np.testing.assert_allclose(t[:3, 2], (1.0, 0.0, 0.0), atol=0.02)  # approach
    np.testing.assert_allclose(abs(t[1, 1]), 1.0, atol=0.02)  # fingers close along y
    assert abs(t[1, 3]) < 1e-3


@BASE_OFFSETS
def test_grasp_pads_straddle_the_cup_body(robot, scene_cfg, ground, base_offset):
    scene = _scene(scene_cfg, scene_cfg.base_approach_x + base_offset, ground)
    t = _fk(robot, "piper_link6", scene_cfg.grasp_joints)
    x_min, x_max, z_min, z_max = _pads(t)

    assert x_min <= scene.obj_x <= x_max, "pads do not reach across the object centre"
    assert z_min >= scene.table_top + TABLE_CLEARANCE_M, "pads too close to the table"
    assert z_max <= scene.rim - BODY_MARGIN_M, "pads above the object body"
    assert _palm_front_x(t) <= scene.obj_x - scene.obj_radius - PALM_CLEARANCE_M, (
        "palm hits the object"
    )


@BASE_OFFSETS
def test_hover_clears_the_cup_rim(robot, scene_cfg, ground, base_offset):
    scene = _scene(scene_cfg, scene_cfg.base_approach_x + base_offset, ground)
    hover = _fk(robot, "piper_link6", scene_cfg.hover_joints)
    grasp = _fk(robot, "piper_link6", scene_cfg.grasp_joints)

    assert _pads(hover)[2] >= scene.rim + RIM_CLEARANCE_M, "hover pads too close to the rim"
    assert abs(hover[0, 3] - grasp[0, 3]) <= MAX_DESCENT_WANDER_M, "hover is not above grasp"


@pytest.mark.parametrize("hover, grasp", [("hover_joints", "grasp_joints"), PLACE_WAYPOINTS])
def test_descent_from_hover_to_grasp_is_vertical(robot, scene_cfg, hover, grasp):
    # A straight joint-space path between the two waypoints keeps the
    # gripper over the object (it is not in the MoveIt planning scene).
    path = _path(_waypoint(scene_cfg, hover), _waypoint(scene_cfg, grasp))
    poses = [_fk(robot, "piper_link6", arm) for arm in path]
    xs = [t[0, 3] for t in poses]

    assert max(xs) - min(xs) <= MAX_DESCENT_WANDER_M
    assert all(abs(t[2, 2]) < 0.02 for t in poses), "approach tilts during the descent"
    assert poses[0][2, 3] - poses[-1][2, 3] >= PLACE_HOVER_MIN_M, "hover not above the grasp"


def test_open_gripper_clears_the_cup_and_closes_on_it(scene_cfg):
    scene = _scene(scene_cfg, scene_cfg.base_approach_x, GAZEBO_FOOTPRINT_HEIGHT_M)

    assert demo.GRIPPER_OPEN - scene.obj_radius >= FINGER_CLEARANCE_M
    assert abs(scene_cfg.gripper_closed - scene.obj_radius) <= 0.002


def test_object_stands_on_the_table(scene_cfg):
    # The garden cup's near face overhangs the edge by 1 cm (centre 1.5 cm
    # in); what keeps an upright cylinder standing is its centre of mass.
    root = _world_root(scene_cfg.world_file)
    x, _, centre_z, _, height = _object(root, scene_cfg.object_name)
    table_top, front_edge = _table_top(root)

    assert abs((centre_z - height / 2) - table_top) <= RESTING_TOLERANCE_M, (
        "the object does not rest on the table top"
    )
    assert x - front_edge >= MIN_SUPPORT_M, "the object's centre is too close to the edge"


def test_garden_scene_matches_the_demo_defaults():
    # The node's own defaults (used without a launch file) are the garden's.
    garden = SCENES["garden"]

    assert garden["hover_joints"] == demo.HOVER
    assert garden["grasp_joints"] == demo.GRASP
    assert garden["gripper_closed"] == demo.GRIPPER_CLOSED
    assert garden["base_approach_x"] == demo.BASE_APPROACH_X
    assert garden["standoff_m"] == demo.STANDOFF_M
    assert garden["place_base_pose"] == demo.PLACE_BASE_POSE
    assert garden["place_route"] == demo.PLACE_ROUTE


# ---- carry pose and the paths in and out of it -------------------------------


def _path(start, end, samples=PATH_SAMPLES):
    """Straight joint-space path from `start` to `end`."""
    start, end = np.asarray(start, float), np.asarray(end, float)
    return [start + k * (end - start) for k in np.linspace(0.0, 1.0, samples)]


def _held_centre(robot, scene, grasp_joints, arm):
    """Centre (base_footprint) of the object grasped at `grasp_joints`, the arm at `arm`."""
    grasp = _fk(robot, "piper_link6", grasp_joints)
    centre = np.array([scene.obj_x, scene.obj_y, scene.obj_z, 1.0])
    return (_fk(robot, "piper_link6", arm) @ np.linalg.solve(grasp, centre))[:3]


def test_carry_pose_is_level_retracted_and_clears_the_pick_table(robot, scene_cfg, ground):
    # The place surface may be higher (the house counter): at the place the
    # carry pose only has to stay behind its edge, see
    # test_place_standoff_path_stays_behind_the_surface_edge.
    scene = _scene(scene_cfg, scene_cfg.base_approach_x, ground)
    _, front_edge = _table_top(_world_root(scene_cfg.world_file))
    carry = _fk(robot, "piper_link6", demo.CARRY)
    centre = _held_centre(robot, scene, scene_cfg.grasp_joints, demo.CARRY)

    np.testing.assert_allclose(carry[:3, 2], (1.0, 0.0, 0.0), atol=LEVEL_TOLERANCE)
    assert (
        _fk(robot, "piper_tcp", demo.CARRY)[0, 3]
        <= _fk(robot, "piper_tcp", scene_cfg.grasp_joints)[0, 3]
    ), "the carry pose is not retracted"
    assert centre[2] - scene.obj_height / 2 >= scene.table_top + CARRY_CLEARANCE_M
    # Behind the table edge: the base can turn and back away with it.
    assert centre[0] + scene.obj_radius < front_edge - scene_cfg.base_approach_x


@pytest.mark.parametrize("pair", [
    ("hover_joints", "grasp_joints"), ("hover_joints", "carry"), ("grasp_joints", "carry"),
    ("place_hover_joints", "carry"),
])
def test_level_waypoints_stay_level_in_between(robot, scene_cfg, pair):
    # Joints 2, 3 and 5 pitch about parallel axes and the three waypoints
    # share their sum: the gripper, and a held object, stay level all along.
    start, end = (_waypoint(scene_cfg, name) for name in pair)
    for arm in _path(start, end):
        assert abs(_fk(robot, "piper_link6", arm)[2, 2]) < LEVEL_TOLERANCE


def test_held_object_clears_the_table_into_the_carry_pose(robot, scene_cfg, ground):
    # Lift (straight up to HOVER), then retract to CARRY over the pick table.
    # At the place the arm rises from CARRY at the standoff, away from the
    # surface (test_place_standoff_path_stays_behind_the_surface_edge).
    scene = _scene(scene_cfg, scene_cfg.base_approach_x, ground)
    for arm in _path(scene_cfg.hover_joints, demo.CARRY):
        centre = _held_centre(robot, scene, scene_cfg.grasp_joints, arm)
        assert centre[2] - scene.obj_height / 2 >= scene.table_top + TABLE_CLEARANCE_M


@pytest.mark.parametrize("waypoint", ["grasp_joints", "hover_joints"])
def test_standoff_paths_clear_the_table(robot, scene_cfg, ground, waypoint):
    # At the standoff the arm drops from CARRY to grasp ("look at the
    # object") or hover height before the base slides in.
    standoff_x = scene_cfg.base_approach_x - scene_cfg.standoff_m
    scene = _scene(scene_cfg, standoff_x, ground)
    _, front_edge = _table_top(_world_root(scene_cfg.world_file))
    for arm in _path(demo.CARRY, getattr(scene_cfg, waypoint)):
        t = _fk(robot, "piper_link6", arm)
        _, pads_x_max, pads_z_min, _ = _pads(t)

        assert max(pads_x_max, _palm_front_x(t)) < front_edge - standoff_x
        assert pads_z_min >= scene.table_top + TABLE_CLEARANCE_M


def _edge_distance(surface, pose):
    """Distance along the heading of base pose `pose` to where its centre line meets `surface`."""
    x, y, yaw = pose
    origin = surface.local(x, y)
    heading = (math.cos(yaw - surface.yaw), math.sin(yaw - surface.yaw))
    entry, leave = -math.inf, math.inf
    for p, d, half in zip(origin, heading, (surface.size_x / 2, surface.size_y / 2)):
        if abs(d) < 1e-12:
            if abs(p) > half:
                return math.inf
            continue
        near, far = sorted(((-half - p) / d, (half - p) / d))
        entry, leave = max(entry, near), min(leave, far)
    return entry if entry <= leave and leave >= 0.0 else math.inf


def test_place_standoff_path_stays_behind_the_surface_edge(robot, scene_cfg):
    # At the place standoff the arm rises from CARRY to the place hover with
    # the object held. The house counter is 10 cm above the pick table, so in
    # CARRY the object is below its top: gripper and object must stay short
    # of its edge until the base slides in.
    scene = _scene(scene_cfg, scene_cfg.base_approach_x, GAZEBO_FOOTPRINT_HEIGHT_M)
    surface = _surface(_world_root(scene_cfg.world_file), scene_cfg.place_surface)
    standoff = demo.standoff_pose(tuple(scene_cfg.place_base_pose), scene_cfg.standoff_m)
    edge = _edge_distance(surface, standoff)
    for arm in _path(demo.CARRY, _waypoint(scene_cfg, "place_hover_joints")):
        t = _fk(robot, "piper_link6", arm)
        centre = _held_centre(robot, scene, scene_cfg.grasp_joints, arm)

        assert max(_pads(t)[1], _palm_front_x(t), centre[0] + scene.obj_radius) < edge


def test_held_object_clears_the_place_surface_at_the_place_hover(robot, scene_cfg, ground):
    # The base slides in with the arm at the place hover: the held object
    # and the pads pass over the place surface's edge.
    top = _surface(_world_root(scene_cfg.world_file), scene_cfg.place_surface).top_z - ground
    scene = _scene(scene_cfg, scene_cfg.base_approach_x, ground)
    arm = _waypoint(scene_cfg, "place_hover_joints")
    centre = _held_centre(robot, scene, scene_cfg.grasp_joints, arm)

    assert centre[2] - scene.obj_height / 2 >= top + TABLE_CLEARANCE_M
    assert _pads(_fk(robot, "piper_link6", arm))[2] >= top + TABLE_CLEARANCE_M


# ---- the wrist camera's view -----------------------------------------------


def _wrist_camera_fov(sim_urdf):
    """(horizontal, vertical) half field of view (rad) of the wrist RGBD camera."""
    sensors = ET.fromstring(sim_urdf).iter("sensor")
    sensor = next(s for s in sensors if s.get("name") == "wrist_rgbd")
    camera = sensor.find("camera")
    half_h = float(camera.findtext("horizontal_fov")) / 2
    width, height = (int(camera.findtext(f"image/{k}")) for k in ("width", "height"))
    return half_h, math.atan(math.tan(half_h) * height / width)


def _view_of(robot, scene, arm):
    """(angle off the wrist camera's optical axis in deg, range in m) of the object centre."""
    optical = _fk(robot, "piper_camera_optical_frame", arm)
    ray = np.array([scene.obj_x, scene.obj_y, scene.obj_z]) - optical[:3, 3]
    distance = np.linalg.norm(ray)
    return math.degrees(math.acos(ray @ optical[:3, 2] / distance)), distance


def test_wrist_camera_rides_on_top_and_looks_down(robot, scene_cfg):
    optical = _fk(robot, "piper_camera_optical_frame", scene_cfg.grasp_joints)

    assert optical[2, 3] > _fk(robot, "piper_link6", scene_cfg.grasp_joints)[2, 3]
    assert optical[2, 2] < -0.4  # about 30 deg down along the approach


@BASE_OFFSETS
def test_object_is_centred_in_the_wrist_camera_at_the_standoff(
    robot, scene_cfg, ground, base_offset
):
    base_x = scene_cfg.base_approach_x - scene_cfg.standoff_m + base_offset
    angle, distance = _view_of(robot, _scene(scene_cfg, base_x, ground), scene_cfg.grasp_joints)

    assert angle <= CAMERA_CENTRE_MAX_DEG, f"object {angle:.1f} deg off the optical axis"
    assert D405_RANGE_M[0] < distance < D405_RANGE_M[1]


def test_object_stays_in_view_halfway_through_the_slide_in(robot, sim_urdf, scene_cfg, ground):
    _, half_vfov = _wrist_camera_fov(sim_urdf)
    base_x = scene_cfg.base_approach_x - scene_cfg.standoff_m / 2
    angle, _ = _view_of(robot, _scene(scene_cfg, base_x, ground), scene_cfg.grasp_joints)

    assert angle <= math.degrees(half_vfov)


# ---- the place spot of the transfer ----------------------------------------


def _place_reach(robot, scene_cfg):
    """How much further ahead of the base the place grasp holds the object than the pick grasp."""
    place = _fk(robot, "piper_tcp", _waypoint(scene_cfg, "place_grasp_joints"))
    return place[0, 3] - _fk(robot, "piper_tcp", scene_cfg.grasp_joints)[0, 3]


def _place_spot(robot, scene_cfg):
    """
    Return the world (x, y) the place sets the object down at.

    The object keeps its pose in the level gripper: it lands as far ahead of
    the base as it was picked, plus the place grasp's extra reach.
    """
    x, y, *_ = _object(_world_root(scene_cfg.world_file), scene_cfg.object_name)
    ahead = x - scene_cfg.base_approach_x + _place_reach(robot, scene_cfg)
    side = y  # in the base frame at the pick
    px, py, yaw = scene_cfg.place_base_pose
    c, s = math.cos(yaw), math.sin(yaw)
    return px + c * ahead - s * side, py + s * ahead + c * side


def _base_box(robot):
    """(front x, half width) of the base_link collision box in base_footprint."""
    collision = robot.link_map["base_link"].collision
    centre = _fk(robot, "base_link") @ _tf(collision.origin.xyz, collision.origin.rpy)
    size_x, size_y, _ = collision.geometry.size
    return centre[0, 3] + size_x / 2, size_y / 2


def test_place_spot_rests_on_the_surface(robot, scene_cfg):
    root = _world_root(scene_cfg.world_file)
    surface = _surface(root, scene_cfg.place_surface)
    pick_top, _ = _table_top(root)
    lift = (_fk(robot, "piper_tcp", _waypoint(scene_cfg, "place_grasp_joints"))[2, 3]
            - _fk(robot, "piper_tcp", scene_cfg.grasp_joints)[2, 3])

    assert surface.contains(*_place_spot(robot, scene_cfg), margin=MIN_SUPPORT_M)
    # The held object keeps its pose in the gripper: the place grasp must be
    # as much higher than the pick grasp as the place surface is above the
    # pick table, or the object is pushed into it or dropped onto it.
    assert abs(lift - (surface.top_z - pick_top)) <= RESTING_TOLERANCE_M


def test_place_spot_is_free(robot, scene_cfg):
    root = _world_root(scene_cfg.world_file)
    present = {m.get("name") for m in root.iter("model")}
    present |= {i.findtext("name") for i in root.iter("include")}
    graspable = {o["name"] for o in yaml.safe_load(GRASPABLE.read_text())["objects"]}
    spot_x, spot_y = _place_spot(robot, scene_cfg)

    for name in sorted((graspable & present) - {scene_cfg.object_name}):
        x, y, _, radius, _ = _object(root, name)
        assert math.hypot(x - spot_x, y - spot_y) >= PLACE_FREE_RADIUS_M + radius, name


def test_place_approach_keeps_the_base_off_the_surface(robot, scene_cfg):
    surface = _surface(_world_root(scene_cfg.world_file), scene_cfg.place_surface)
    front, half_width = _base_box(robot)
    approach = scene_cfg.place_base_pose
    standoff = demo.standoff_pose(tuple(approach), scene_cfg.standoff_m)

    for x, y, yaw in (approach, standoff):
        c, s = math.cos(yaw), math.sin(yaw)
        for side in (-half_width, half_width):
            assert not surface.contains(x + c * front - s * side, y + s * front + c * side)


def _pose_tf(element):
    """4x4 transform of an SDF element's <pose> (identity without one)."""
    text = element.findtext("pose")
    values = [float(v) for v in text.split()] if text else [0.0] * 6
    return _tf(values[:3], values[3:])


def _corners(half):
    """Return the 8 corners of the box from -`half` to `half`."""
    return [half * np.array(signs) for signs in itertools.product((-1.0, 1.0), repeat=3)]


def _shape_points(geometry):
    """Points (own frame) whose bounding box in any frame contains the collision `geometry`."""
    kind = type(geometry).__name__
    if kind == "Box":
        return _corners(np.asarray(geometry.size) / 2)
    if kind == "Cylinder":
        # 72 rim points per cap, pushed out to the circumscribed polygon.
        r = geometry.radius / math.cos(math.pi / 72)
        return [(r * math.cos(a), r * math.sin(a), h * geometry.length / 2)
                for a in np.linspace(0, 2 * math.pi, 72, endpoint=False) for h in (-1, 1)]
    if kind == "Sphere":
        return _corners(np.full(3, geometry.radius))
    raise AssertionError(f"no sampling for a {kind} base collision")


def _base_parts(robot):
    """{name: points in base_footprint} of every base collision (the arm is checked on its own)."""
    parts = {}
    for link in robot.links:
        if link.name.startswith("piper"):
            continue
        for k, collision in enumerate(link.collisions):
            t = _fk(robot, link.name)
            if collision.origin is not None:
                t = t @ _tf(collision.origin.xyz, collision.origin.rpy)
            points = np.array([(*p, 1.0) for p in _shape_points(collision.geometry)])
            parts[f"{link.name}[{k}]"] = (t @ points.T).T[:, :3]
    return parts


def _model_boxes(root, name):
    """[(name, world -> box transform, half size)] of the box collisions of world model `name`."""
    model = _world_model(root, name)
    boxes = []
    for link in model.findall("link"):
        for collision in link.findall("collision"):
            size = collision.findtext("geometry/box/size")
            assert size, f"{name}/{collision.get('name')}: only box collisions are checked"
            t = _pose_tf(model) @ _pose_tf(link) @ _pose_tf(collision)
            half = np.array([float(v) for v in size.split()]) / 2
            boxes.append((collision.get("name"), np.linalg.inv(t), half))
    return boxes


def _approach(scene_cfg, which):
    """(surface model, base pose) of the "pick" or the "place" approach."""
    if which == "pick":
        return "table", (scene_cfg.base_approach_x, 0.0, 0.0)
    return scene_cfg.place_surface, tuple(scene_cfg.place_base_pose)


@pytest.mark.parametrize("which", ["pick", "place"])
@BASE_OFFSETS
def test_base_clears_the_surface_it_drives_up_to(robot, scene_cfg, ground, which, base_offset):
    # The tables are open below their tops, but the house counter is a
    # cabinet down to the floor: every base collision (the lidar sticks out
    # 0.394 m) must stop short of every collision box of the surface.
    root = _world_root(scene_cfg.world_file)
    surface, (x, y, yaw) = _approach(scene_cfg, which)
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = x + c * base_offset, y + s * base_offset  # overshoot along the heading
    base = _tf((x, y, ground), (0.0, 0.0, yaw))
    for part, points in _base_parts(robot).items():
        world = base @ np.c_[points, np.ones(len(points))].T
        for box, to_box, half in _model_boxes(root, surface):
            local = (to_box @ world)[:3]
            # Per box axis, how far the part's bounding box is from the box (> 0: apart).
            gaps = np.maximum(local.min(axis=1) - half, -half - local.max(axis=1))
            assert gaps.max() >= BASE_CLEARANCE_M, (
                f"{part} {gaps.max():.3f} m from {surface}/{box}"
            )


def test_house_place_spot_is_clear_of_the_cooktop(robot):
    cfg = SimpleNamespace(name="small_house", **SCENES["small_house"])
    root = _world_root(cfg.world_file)
    bench = next(i for i in root.iter("include") if i.findtext("name") == "CookingBench_01_001")
    spot_x, _ = _place_spot(robot, cfg)

    # The cooktop stands east (+x) of the spot, on the same counter.
    assert _xyz(bench)[0] - COOKTOP_HALF_LENGTH_M - spot_x >= COOKTOP_CLEARANCE_M


def test_house_place_surface_top_is_the_generator_constant():
    house = SCENES["small_house"]
    surface = _surface(_world_root(house["world_file"]), house["place_surface"])

    assert surface.top_z == pytest.approx(house_gen.COUNTER_TOP_M, abs=1e-6)


# ---- tools/solve_level_waypoint.py -----------------------------------------


@pytest.mark.parametrize("waypoint", ["hover_joints", "grasp_joints", "carry", *PLACE_WAYPOINTS])
def test_level_waypoint_solver_reproduces_the_waypoints(robot, scene_cfg, waypoint):
    pytest.importorskip("scipy")
    solver = _load("solve_level_waypoint", PKG / "tools" / "solve_level_waypoint.py")
    joints = _waypoint(scene_cfg, waypoint)
    link6 = _fk(robot, "piper_link6", joints)

    solved, cost = solver.solve(sys.modules[__name__], robot, link6[0, 3], link6[2, 3])

    assert cost <= solver.MAX_COST
    np.testing.assert_allclose(solved, joints, atol=2e-3)


@pytest.mark.parametrize("launch_name", DEMO_LAUNCHES)
def test_demo_launches_default_to_the_small_house_scene(launch_name):
    assert _launch_default(launch_name, "scene") == "small_house"


# ---- sim-only base plugins -------------------------------------------------


def _plugins(sim_urdf, name):
    return [p for p in ET.fromstring(sim_urdf).iter("plugin") if p.get("name") == name]


def test_sim_xacro_locks_every_steering_joint(sim_urdf):
    controllers = _plugins(sim_urdf, JOINT_POSITION_CONTROLLER)

    assert sorted(p.findtext("joint_name") for p in controllers) == sorted(STEERING_JOINTS)
    for plugin in controllers:
        assert plugin.findtext("use_velocity_commands") == "true"
        assert float(plugin.findtext("initial_position")) == 0.0


def test_sim_wheel_collisions_are_concentric(robot):
    # An off-axle collision rotates with the wheel like a cam: the free-
    # rolling (steering-locked) base then creeps into the low spots.
    for name in WHEEL_LINKS:
        assert robot.link_map[name].collision.origin.xyz == [0.0, 0.0, 0.0], name


def test_sim_xacro_publishes_the_base_joints_only(sim_urdf):
    (plugin,) = _plugins(sim_urdf, JOINT_STATE_PUBLISHER)
    bridge = [
        e for e in yaml.safe_load(BRIDGE.read_text())
        if e.get("ros_topic_name") == "/gz/base_joint_states"
    ]

    assert sorted(j.text for j in plugin.iter("joint_name")) == sorted(bjsp.BASE_JOINTS)
    assert len(bridge) == 1
    assert bridge[0]["gz_topic_name"] == plugin.findtext("topic")
    assert bridge[0]["direction"] == "GZ_TO_ROS"
    assert bridge[0]["ros_type_name"] == "sensor_msgs/msg/JointState"
    assert bridge[0]["gz_type_name"] == "ignition.msgs.Model"


# ---- base joint-state relay ------------------------------------------------


@pytest.fixture
def relay(monkeypatch):
    rclpy.init()
    node = bjsp.BaseJointStatePublisher()
    sent = []
    monkeypatch.setattr(node, "_pub", SimpleNamespace(publish=sent.append))
    yield node, sent
    node.destroy_node()
    rclpy.shutdown()


def _gz_state(names, position, velocity=()):
    msg = JointState()
    msg.name = list(names)
    msg.position = list(position)
    msg.velocity = list(velocity)
    return msg


def _published(sent):
    msg = sent[-1]
    return dict(zip(msg.name, msg.position)), dict(zip(msg.name, msg.velocity))


def test_base_joint_relay_publishes_zeros_then_live_values(relay):
    node, sent = relay

    node._tick()
    assert _published(sent)[0] == {name: 0.0 for name in bjsp.BASE_JOINTS}

    node._on_gz(_gz_state(["fl_steering_joint", "piper_joint1"], [0.3, 1.0], [0.5, 2.0]))
    node._tick()
    positions, velocities = _published(sent)

    assert sent[-1].name == bjsp.BASE_JOINTS
    assert positions["fl_steering_joint"] == pytest.approx(0.3)
    assert velocities["fl_steering_joint"] == pytest.approx(0.5)
    assert positions["fr_wheel"] == 0.0
    assert "piper_joint1" not in positions


def test_base_joint_relay_tolerates_missing_velocities(relay):
    node, sent = relay

    node._on_gz(_gz_state(["fr_wheel"], [1.5]))
    node._tick()
    positions, velocities = _published(sent)

    assert positions["fr_wheel"] == pytest.approx(1.5)
    assert velocities["fr_wheel"] == 0.0
