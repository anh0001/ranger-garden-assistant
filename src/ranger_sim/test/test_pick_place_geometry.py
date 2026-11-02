"""
Geometry checks for the pick-and-place demo (no simulator required).

The arm waypoints of scripts/pick_place_demo.py are checked against the cup
and the table of worlds/garden_world.sdf with forward kinematics built from
the robot URDF; the sim-only steering lock and base joint-state plugins of
urdf/ranger_gazebo.urdf.xacro are checked against config/gz_bridge.yaml and
scripts/base_joint_state_publisher.py.

Needs the install space sourced: the xacro files resolve $(find ...).
"""

from dataclasses import dataclass
import importlib.util
from pathlib import Path
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
WORLD = PKG / "worlds" / "garden_world.sdf"
SIM_XACRO = PKG / "urdf" / "ranger_gazebo.urdf.xacro"
BRIDGE = PKG / "config" / "gz_bridge.yaml"
DEMO_LAUNCHES = ("pick_place_demo.launch.py", "record_pick_place_video.launch.py")

# Gripper geometry in the piper_link6 (= gripper_base) frame, measured from
# piper_description/meshes/{link7,link8,gripper_base}.STL: finger pads span
# PAD_Z along the approach (z) axis and +-PAD_HALF_HEIGHT along x; the palm
# ends at PALM_Z and spans PALM_X.
PAD_Z = (0.059, 0.136)
PAD_HALF_HEIGHT = 0.028
PALM_Z = 0.063
PALM_X = (-0.04, 0.0305)

TABLE_CLEARANCE_M = 0.02  # pads above the table top at GRASP
RIM_CLEARANCE_M = 0.01  # pads above the cup rim at HOVER
BODY_MARGIN_M = 0.005  # pads below the cup rim at GRASP
PALM_CLEARANCE_M = 0.01  # palm to the cup's near face at GRASP
FINGER_CLEARANCE_M = 0.005  # open finger to the cup surface, per side
MAX_DESCENT_WANDER_M = 0.005  # link6 x drift along the hover -> grasp path

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


def _launch_default(launch_name, arg_name):
    module = _load(launch_name.replace(".", "_"), PKG / "launch" / launch_name)
    entities = module.generate_launch_description().entities
    arg = next(
        e for e in entities if isinstance(e, DeclareLaunchArgument) and e.name == arg_name
    )
    return arg.default_value[0].text


BASE_APPROACH_X = float(_launch_default("pick_place_demo.launch.py", "base_approach_x"))


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
    """Cup and table top in base_footprint, base at world (base_x, 0)."""

    cup_x: float
    cup_y: float
    cup_z: float
    cup_radius: float
    cup_height: float
    table_top: float

    @property
    def rim(self):
        return self.cup_z + self.cup_height / 2


def _world_model(name):
    return next(m for m in ET.parse(WORLD).getroot().iter("model") if m.get("name") == name)


def _xyz(element):
    return [float(v) for v in element.findtext("pose").split()[:3]]


def _wheel_footprint_height(robot):
    """World z of base_footprint with the wheel collisions resting on z = 0."""
    bottoms = []
    for name in WHEEL_LINKS:
        collision = robot.link_map[name].collision
        centre = _fk(robot, name) @ _tf(collision.origin.xyz, collision.origin.rpy)
        bottoms.append(centre[2, 3] - collision.geometry.radius)
    return -min(bottoms)


def _scene(robot, base_x, ground):
    """Scene seen from a base at world x `base_x` whose footprint is at z `ground`."""
    cup = _world_model("cup")
    cup_x, cup_y, cup_z = _xyz(cup)
    cylinder = cup.find("link/collision/geometry/cylinder")
    table = _world_model("table")
    top = next(c for c in table.iter("collision") if c.get("name") == "top_collision")
    top_thickness = float(top.findtext("geometry/box/size").split()[2])
    table_top = _xyz(table)[2] + _xyz(top)[2] + top_thickness / 2
    return Scene(
        cup_x=cup_x - base_x,
        cup_y=cup_y,
        cup_z=cup_z - ground,
        cup_radius=float(cylinder.findtext("radius")),
        cup_height=float(cylinder.findtext("length")),
        table_top=table_top - ground,
    )


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
BASE_POSES = pytest.mark.parametrize(
    "base_x",
    [BASE_APPROACH_X + k * demo.BASE_TOLERANCE_M for k in (-1, 0, 1)],
)


def test_wheel_geometry_and_gazebo_agree_within_a_centimetre(robot):
    # If they drift apart, the ground band the waypoints are checked
    # against no longer describes the sim: re-measure and re-solve.
    assert abs(_wheel_footprint_height(robot) - GAZEBO_FOOTPRINT_HEIGHT_M) <= 0.011


# ---- arm waypoints vs. the world -------------------------------------------


def test_fk_reproduces_the_documented_tcps(robot):
    # Guards the FK helper itself: the demo docstring's TCPs.
    for joints, tcp in ((demo.HOME, (0.484, 0.0, 0.596)), (demo.LIFT, (0.347, 0.0, 1.111))):
        np.testing.assert_allclose(_fk(robot, "piper_tcp", joints)[:3, 3], tcp, atol=2e-3)


@pytest.mark.parametrize("name", ["HOVER", "GRASP"])
def test_grasp_approach_is_level_and_planar(robot, name):
    t = _fk(robot, "piper_link6", getattr(demo, name))

    np.testing.assert_allclose(t[:3, 2], (1.0, 0.0, 0.0), atol=0.02)  # approach
    np.testing.assert_allclose(abs(t[1, 1]), 1.0, atol=0.02)  # fingers close along y
    assert abs(t[1, 3]) < 1e-3


@BASE_POSES
def test_grasp_pads_straddle_the_cup_body(robot, ground, base_x):
    scene = _scene(robot, base_x, ground)
    t = _fk(robot, "piper_link6", demo.GRASP)
    x_min, x_max, z_min, z_max = _pads(t)

    assert x_min <= scene.cup_x <= x_max, "pads do not reach across the cup centre"
    assert z_min >= scene.table_top + TABLE_CLEARANCE_M, "pads too close to the table"
    assert z_max <= scene.rim - BODY_MARGIN_M, "pads above the cup body"
    assert _palm_front_x(t) <= scene.cup_x - scene.cup_radius - PALM_CLEARANCE_M, (
        "palm hits the cup"
    )


@BASE_POSES
def test_hover_clears_the_cup_rim(robot, ground, base_x):
    scene = _scene(robot, base_x, ground)
    hover = _fk(robot, "piper_link6", demo.HOVER)
    grasp = _fk(robot, "piper_link6", demo.GRASP)

    assert _pads(hover)[2] >= scene.rim + RIM_CLEARANCE_M, "hover pads too close to the rim"
    assert abs(hover[0, 3] - grasp[0, 3]) <= MAX_DESCENT_WANDER_M, "hover is not above grasp"


def test_descent_from_hover_to_grasp_is_vertical(robot):
    # A straight joint-space path between the two waypoints keeps the
    # gripper over the cup (the cup is not in the MoveIt planning scene).
    hover, grasp = np.array(demo.HOVER), np.array(demo.GRASP)
    poses = [_fk(robot, "piper_link6", hover + s * (grasp - hover)) for s in np.linspace(0, 1, 21)]
    xs = [t[0, 3] for t in poses]

    assert max(xs) - min(xs) <= MAX_DESCENT_WANDER_M
    assert all(abs(t[2, 2]) < 0.02 for t in poses), "approach tilts during the descent"


def test_open_gripper_clears_the_cup_and_closes_on_it(robot):
    scene = _scene(robot, BASE_APPROACH_X, GAZEBO_FOOTPRINT_HEIGHT_M)

    assert demo.GRIPPER_OPEN - scene.cup_radius >= FINGER_CLEARANCE_M
    assert abs(demo.GRIPPER_CLOSED - scene.cup_radius) <= 0.002


@pytest.mark.parametrize("launch_name", DEMO_LAUNCHES)
def test_launch_files_close_the_gripper_like_the_demo(launch_name):
    assert float(_launch_default(launch_name, "gripper_closed")) == demo.GRIPPER_CLOSED


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
