"""
Unit tests for scripts/pick_place_demo.py (no simulator, no MoveIt).

Covers the base pose controller and the step sequence of the transfer. The
drive tests run the controller against a kinematic base: a fake
rclpy.spin_once integrates the last published /cmd_vel into /odom, like
Gazebo's VelocityControl + OdometryPublisher (no noise) do in the sim.
"""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import Pose, Twist
import pytest
import rclpy
from rclpy.parameter import Parameter
import yaml

PKG = Path(__file__).resolve().parents[1]
SCENES = yaml.safe_load((PKG / "config" / "pick_place_scenes.yaml").read_text())
START = (0.0, 0.0, 0.0)
SIM_STEP_SEC = 0.02  # one /odom message at the sim's 50 Hz
VIDEO_CAPTION_MAX_CHARS = 60  # scripts/record_demo_video.py CAPTION_MAX_CHARS


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


demo = _load("pick_place_demo", PKG / "scripts" / "pick_place_demo.py")
scenes = _load("pick_place_scenes", PKG / "launch" / "pick_place_scenes.py")


def _scene_parameters(name):
    """Return the demo parameters the launch files pass for scene `name` (optional keys set)."""
    keys = ("object_name", "base_approach_x", "gripper_closed", "hover_joints",
            "grasp_joints", "place_hover_joints", "place_grasp_joints", "standoff_m",
            "place_base_pose", "place_route", "grasp_tcp", "hover_tcp")
    return {key: scenes.scene_value(SCENES[name], key) for key in keys}


@pytest.fixture
def make_node():
    """Return a PickPlaceDemo factory that takes parameter overrides as keywords."""
    rclpy.init()
    nodes = []

    def make(**parameters):
        overrides = [Parameter(name, value=value) for name, value in parameters.items()]
        node = demo.PickPlaceDemo(parameter_overrides=overrides)
        nodes.append(node)
        return node

    yield make
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


# ---- pure helpers ----------------------------------------------------------


def test_pose_error_is_in_the_body_frame():
    dx, dy, dyaw = demo.pose_error((1.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2))

    assert (dx, dy, dyaw) == pytest.approx((0.0, -1.0, -math.pi / 2))


def test_pose_error_wraps_yaw():
    *_, dyaw = demo.pose_error((0.0, 0.0, 3.1), (0.0, 0.0, -3.1))

    assert dyaw == pytest.approx(6.2 - 2 * math.pi)  # -0.083, not a 6.2 rad spin


@pytest.mark.parametrize("angle, wrapped", [
    (0.5, 0.5), (-math.pi, math.pi), (math.pi, math.pi), (2.5 * math.pi, math.pi / 2),
    (-3 * math.pi / 2, math.pi / 2),
])
def test_wrap_angle_is_in_minus_pi_exclusive_to_pi(angle, wrapped):
    assert demo.wrap_angle(angle) == pytest.approx(wrapped)


def test_yaw_of_reads_a_quaternion():
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.35), w=math.cos(0.35))

    assert demo.yaw_of(q) == pytest.approx(0.7)


def test_saturate_applies_floor_ceiling_and_sign():
    assert demo.saturate(-0.001, 1.0, 0.02, 0.25) == pytest.approx(-0.02)
    assert demo.saturate(2.0, 1.0, 0.02, 0.25) == pytest.approx(0.25)
    assert demo.saturate(0.1, 1.0, 0.02, 0.25) == pytest.approx(0.1)


def test_route_triples_splits_and_rejects_ragged():
    assert demo.route_triples([0.0, 0.0, 0.0, 1.0, 2.0, 3.0]) == [(0.0, 0.0, 0.0), (1.0, 2.0, 3.0)]
    assert demo.route_triples([]) == []
    with pytest.raises(ValueError, match="multiple of 3"):
        demo.route_triples([0.0, 0.0, 0.0, 1.0, 2.0])


def test_standoff_pose_backs_off_along_the_heading():
    assert demo.standoff_pose((0.0, 1.07, math.pi / 2), 0.15) == pytest.approx(
        (0.0, 0.92, math.pi / 2), abs=1e-9
    )
    assert demo.standoff_pose((0.52, 0.0, 0.0), 0.15) == pytest.approx((0.37, 0.0, 0.0))


def test_scene_standoffs_are_short_of_the_approaches():
    garden = SCENES["garden"]
    pick = (garden["base_approach_x"], 0.0, 0.0)

    assert demo.standoff_pose(pick, garden["standoff_m"]) == pytest.approx((0.37, 0.0, 0.0))
    # The scene's yaw 1.5708 is pi/2 to 4e-6 rad: x is 0 to under a micrometre.
    assert demo.standoff_pose(tuple(garden["place_base_pose"]), garden["standoff_m"]) == (
        pytest.approx((0.0, 0.92, 1.5708), abs=1e-6)
    )


def test_format_pose_is_short_ascii():
    text = demo.format_pose((0.37, -0.0004, math.pi / 2))

    assert text == "(0.370, -0.000, 90 deg)"


# ---- base command law ------------------------------------------------------


def test_base_turns_in_place_first():
    vx, vy, wz = demo.base_command((1.0, 1.0, math.pi / 2), START, 0.25, turning=True)

    assert (vx, vy) == (0.0, 0.0)
    assert wz == pytest.approx(demo.BASE_MAX_YAW_RATE)


def test_base_turn_keeps_a_rate_floor():
    *_, wz = demo.base_command((0.0, 0.0, -0.01), START, 0.25, turning=True)

    assert wz == pytest.approx(-demo.BASE_MIN_YAW_RATE)


def test_base_drive_holds_the_yaw_and_ignores_a_tiny_lateral_error():
    # |dy| below half the position tolerance: left alone, no dithering at the floor.
    vx, vy, wz = demo.base_command((1.0, 0.002, 0.002), START, 0.25, turning=False)

    assert vx == pytest.approx(0.25)
    assert vy == 0.0
    assert wz == pytest.approx(demo.BASE_YAW_GAIN * 0.002)  # proportional, no floor


def test_base_drive_corrects_a_lateral_error_and_caps_the_speed():
    vx, vy, _ = demo.base_command((0.0, -0.5, 0.0), START, 0.08, turning=False)

    assert vx == 0.0  # dx = 0: no push at the speed floor
    assert vy == pytest.approx(-0.08)


def test_a_small_lateral_error_is_corrected_at_the_floor():
    _, vy, _ = demo.base_command((0.0, 0.003, 0.0), START, 0.25, turning=False)

    assert vy == pytest.approx(demo.BASE_MIN_LATERAL_SPEED_MPS)


# Yaw errors between the tolerance and the realign threshold.
BETWEEN = 2 * demo.BASE_YAW_TOLERANCE_RAD


def test_a_turn_runs_until_the_yaw_is_within_tolerance():
    tol = demo.BASE_YAW_TOLERANCE_RAD
    assert demo.base_turning((1.0, 0.0, 1.2 * tol), START, turning=True)
    assert not demo.base_turning((1.0, 0.0, 0.8 * tol), START, turning=True)


def test_driving_only_turns_back_beyond_the_realign_threshold():
    # A late /cmd_vel lets a turn overshoot the tolerance band: the base
    # drives on with the small error instead of stopping to turn back.
    assert BETWEEN < demo.BASE_YAW_REALIGN_RAD
    assert not demo.base_turning((1.0, 0.0, BETWEEN), START, turning=False)
    assert demo.base_turning((1.0, 0.0, 2 * demo.BASE_YAW_REALIGN_RAD), START, turning=False)


def test_a_last_turn_fixes_the_yaw_once_the_position_is_reached():
    assert demo.base_turning((0.0, 0.0, BETWEEN), START, turning=False)


def test_at_pose_needs_position_and_yaw():
    tol = demo.BASE_TOLERANCE_M
    assert demo.at_pose((0.0, 0.0, 0.0), (0.6 * tol, 0.6 * tol, 0.0))
    assert not demo.at_pose((0.0, 0.0, 0.0), (tol, 0.0, 0.0))
    assert not demo.at_pose((0.0, 0.0, 0.0), (0.0, 0.0, 2 * demo.BASE_YAW_TOLERANCE_RAD))


# ---- drive_to_pose on a kinematic base -------------------------------------

# The fake base follows /cmd_vel exactly, as the sim base does: with the
# wheels steered and rolling (scripts/ranger_motion_adapter.py) it follows
# 0.05 rad/s and 0.01 m/s sideways (measured in garden_world on 2026-11-21).
# The steering-locked base it replaced skidded, with dead zones of 0.13 rad/s
# and 0.035 m/s that this file used to model.
# /cmd_vel takes effect this many /odom updates late (the sim: about 1).
LAGS = (0, 1, 2)


def _odom(x, y, yaw):
    pose = Pose()
    pose.position.x, pose.position.y = x, y
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return pose


@pytest.fixture(params=LAGS, ids=[f"lag{lag}" for lag in LAGS])
def kinematic_base(request, make_node, monkeypatch):
    """
    Return an executing node on a fake base, its published twists and a clock.

    The base applies /cmd_vel exactly, `lag` updates late. clock.path
    collects every base pose /odom reports, clock.stamps the time of every
    published twist. Set clock.moving = False
    for a stuck base, clock.odom_until for an /odom that stops, and
    clock.interrupt_at for a Ctrl-C at that time.
    """
    lag = request.param
    node = make_node(execute=True)
    sent = []
    clock = SimpleNamespace(now=1000.0, moving=True, path=[], stamps=[],
                            odom_until=math.inf, interrupt_at=math.inf)

    def publish(twist):
        sent.append(twist)
        clock.stamps.append(clock.now)

    monkeypatch.setattr(node, "_cmd_vel", SimpleNamespace(publish=publish))
    monkeypatch.setattr(demo.time, "time", lambda: clock.now)

    def spin_once(_node, timeout_sec=None):
        clock.now += SIM_STEP_SEC
        if clock.now >= clock.interrupt_at:
            raise KeyboardInterrupt
        if clock.now > clock.odom_until:
            return  # /odom stopped: the node hears nothing more
        node._odom_time = clock.now
        if len(sent) <= lag or not clock.moving:
            return
        x, y, yaw = node._odom_pose()
        tw = sent[-1 - lag]
        vx, vy = tw.linear.x, tw.linear.y
        c, s = math.cos(yaw), math.sin(yaw)
        node._odom = _odom(
            x + (c * vx - s * vy) * SIM_STEP_SEC,
            y + (s * vx + c * vy) * SIM_STEP_SEC,
            yaw + tw.angular.z * SIM_STEP_SEC,
        )
        clock.path.append(node._odom_pose())

    monkeypatch.setattr(demo.rclpy, "spin_once", spin_once)
    return node, sent, clock


def _is_stop(twist):
    return twist == Twist()


def test_drive_to_pose_turns_then_drives_to_the_target(kinematic_base):
    node, sent, clock = kinematic_base
    node._odom = _odom(0.0, 0.0, 0.0)
    target = (0.0, 0.92, math.pi / 2)

    assert node.drive_to_pose(target, "drive to the place")

    assert demo.at_pose(target, node._odom_pose())
    assert _is_stop(sent[-1])
    turning = [p for p in clock.path if abs(p[2] - target[2]) >= demo.BASE_YAW_REALIGN_RAD]
    assert turning, "never turned"
    assert all(math.hypot(x, y) < 1e-9 for x, y, _ in turning), "moved before facing the target"


def test_drive_to_pose_finishes_a_turn_and_removes_its_drift(kinematic_base):
    # Where the first garden run stalled: 85 deg into the turn at the
    # origin, 12 mm off it (the old skidding base dragged itself along).
    node, _, _ = kinematic_base
    node._odom = _odom(0.0115, 0.0052, math.radians(85.45))
    target = (0.0, 0.0, math.pi / 2)

    assert node.drive_to_pose(target, "drive to the place")

    assert demo.at_pose(target, node._odom_pose())


def test_drive_to_pose_is_not_slowed_by_a_late_cmd_vel(kinematic_base):
    # A 20 mrad heading error before a 0.92 m leg: the turn into the 5 mrad
    # band overshoots by a late command; the base must not stop and turn
    # back every update (it took 18 s with one update of lag, 30 with two).
    node, _, clock = kinematic_base
    node._odom = _odom(0.004, 0.003, math.pi / 2 - 0.02)
    start = clock.now

    assert node.drive_to_pose((0.0, 0.92, math.pi / 2), "drive to the place")

    assert clock.now - start < 10.0


def test_drive_stops_the_base_when_interrupted(kinematic_base):
    node, sent, clock = kinematic_base
    node._odom = _odom(0.0, 0.0, 0.0)
    clock.interrupt_at = clock.now + 1.0  # Ctrl-C mid-leg

    with pytest.raises(KeyboardInterrupt):
        node.drive_to_pose((1.0, 0.0, 0.0), "drive to the table")

    assert any(not _is_stop(tw) for tw in sent), "never moved"
    assert _is_stop(sent[-1])


def test_stale_odom_stops_the_base(kinematic_base):
    node, sent, clock = kinematic_base
    node._odom = _odom(0.0, 0.0, 0.0)
    clock.odom_until = clock.now + 1.0  # the bridge dies mid-leg

    assert not node.drive_to_pose((1.0, 0.0, 0.0), "drive to the table")

    late = [tw for t, tw in zip(clock.stamps, sent)
            if t > clock.odom_until + demo.ODOM_STALE_SEC]
    assert late and all(_is_stop(tw) for tw in late), "drove on a stale pose"


def test_drive_to_pose_backs_up_and_slides_slowly(kinematic_base):
    node, sent, _ = kinematic_base
    node._odom = _odom(0.52, 0.0, 0.0)

    assert node.drive_to_pose((0.37, 0.0, 0.0), "back off", speed=demo.BASE_APPROACH_SPEED_MPS)

    assert node._odom_pose()[0] == pytest.approx(0.37, abs=demo.BASE_TOLERANCE_M)
    assert max(abs(tw.linear.x) for tw in sent) <= demo.BASE_APPROACH_SPEED_MPS
    assert all(tw.linear.x <= 0.0 for tw in sent[:-1]), "overshot the target"


def test_zero_length_leg_returns_at_once(kinematic_base):
    node, sent, _ = kinematic_base
    node._odom = _odom(0.0, 0.0, 0.0)

    assert node.drive_to_pose(START, "return to start")

    assert len(sent) == 1 and _is_stop(sent[0])


def test_drive_to_pose_times_out_and_stops(kinematic_base):
    node, sent, clock = kinematic_base
    node._odom = _odom(0.0, 0.0, 0.0)
    clock.moving = False  # the base is stuck

    assert not node.drive_to_pose((1.0, 0.0, 0.0), "drive to the table")

    assert clock.now - 1000.0 >= demo.BASE_DRIVE_TIMEOUT_SEC
    assert _is_stop(sent[-1])


def test_plan_only_drive_does_not_publish(make_node, monkeypatch):
    node = make_node(execute=False)
    sent = []
    monkeypatch.setattr(node, "_cmd_vel", SimpleNamespace(publish=sent.append))

    assert node.drive_to_pose((1.0, 0.0, 0.0), "drive to the table")

    assert sent == []


class _FakeDemo:
    """Stands in for PickPlaceDemo in main(): run() raises KeyboardInterrupt."""

    execute = True
    stops = []

    def run(self):
        raise KeyboardInterrupt

    def stop_base(self):
        self.stops.append(self.execute)

    def get_logger(self):
        return SimpleNamespace(info=lambda *_: None, warn=lambda *_: None,
                               error=lambda *_: None)

    def destroy_node(self):
        pass


@pytest.mark.parametrize("execute, stops", [(True, [True]), (False, [])])
def test_main_stops_the_base_on_ctrl_c(monkeypatch, execute, stops):
    fake = type("Fake", (_FakeDemo,), {"execute": execute, "stops": []})
    monkeypatch.setattr(demo, "PickPlaceDemo", fake)

    try:
        demo.main()
    except KeyboardInterrupt:  # would also end the pytest session
        pytest.fail("main() let Ctrl-C escape instead of stopping the base")

    assert fake.stops == stops


# ---- the step sequence -----------------------------------------------------

GARDEN_CAPTIONS = [
    "release object", "open gripper", "'home'", "carry pose",
    # pick at table A
    "drive to the table", "look at the object", "approach the object",
    "attach object", "close gripper", "lift", "carry pose",
    # transfer: place_route (2 poses), then standoff B
    "drive to the place", "drive to the place", "drive to the place",
    # place at table B
    "hover above the place", "approach the place", "place", "open gripper",
    "release object", "clear (hover)", "back off", "carry pose",
    # home: place_route reversed, then the start pose
    "return", "return", "return to start", "retract 'home'",
]
# Base legs the arm must start in the level carry pose: away from the tables.
CARRY_LEGS = {"drive to the table", "drive to the place", "return", "return to start"}
# Base legs that slide the open fingers around an object, or back off from it.
SLIDE_LEGS = ("approach the object", "approach the place", "back off")


@pytest.fixture
def recorded(make_node, monkeypatch):
    """Return a factory of (node, calls): nodes whose actions only record (kind, label, value)."""

    def make(scene="garden", fail_at=None, **parameters):
        node = make_node(**{**_scene_parameters(scene), **parameters})
        calls = []

        def record(kind, label, value=None):
            calls.append((kind, label, value))
            return len(calls) != fail_at

        monkeypatch.setattr(node, "_wait_servers", lambda: None)
        monkeypatch.setattr(node, "_wait_for_odom_pose", lambda: START)
        monkeypatch.setattr(node, "move_to_joint_state", lambda label, q: record("arm", label, q))
        monkeypatch.setattr(
            node, "drive_to_pose",
            lambda target, label, speed=None: record("drive", label, (target, speed)),
        )
        monkeypatch.setattr(node, "set_gripper", lambda position, label: record("gripper", label))
        monkeypatch.setattr(node, "grasp", lambda attach: record("grasp", attach))
        return node, calls

    return make


def test_sequence_order(recorded):
    node, _ = recorded()

    captions = [caption for caption, _ in node.sequence(node.transfer_plan(), START)]

    assert captions == GARDEN_CAPTIONS


def test_house_sequence_drives_its_longer_route(recorded):
    node, calls = recorded("small_house")
    route = demo.route_triples(SCENES["small_house"]["place_route"])

    node.run()

    drives = [(label, value[0]) for kind, label, value in calls if kind == "drive"]
    assert [pose for label, pose in drives if label == "drive to the place"][:-1] == route
    assert [pose for label, pose in drives if label == "return"] == route[::-1]


def test_captions_fit_the_video_status_bar(recorded):
    for scene in sorted(SCENES):
        node, _ = recorded(scene)
        for caption, _ in node.sequence(node.transfer_plan(), START):
            assert caption.isascii() and len(caption) <= VIDEO_CAPTION_MAX_CHARS, caption


@pytest.mark.parametrize("scene", sorted(SCENES))
def test_arm_is_in_the_carry_pose_on_every_base_leg_away_from_a_table(recorded, scene):
    node, calls = recorded(scene)
    node.run()

    arm = None
    legs = 0
    for kind, label, value in calls:
        if kind == "arm":
            arm = list(value)
        elif kind == "drive":
            legs += 1
            if label in CARRY_LEGS:
                assert arm == demo.CARRY, f"'{label}' with the arm at {arm}"
    # 2 pick legs, 3 place legs, return to start, the route out and back.
    assert legs == 6 + 2 * len(demo.route_triples(SCENES[scene]["place_route"]))


@pytest.mark.parametrize("scene", sorted(SCENES))
def test_slide_ins_are_slow_at_grasp_or_hover_height(recorded, scene):
    node, calls = recorded(scene)
    node.run()

    arm, slides = None, {}
    for kind, label, value in calls:
        if kind == "arm":
            arm = list(value)
        elif kind == "drive" and label in SLIDE_LEGS:
            slides[label] = (arm, value[1])
    cfg = _scene_parameters(scene)
    slow = demo.BASE_APPROACH_SPEED_MPS
    assert slides == {
        "approach the object": (cfg["grasp_joints"], slow),
        "approach the place": (cfg["place_hover_joints"], slow),
        "back off": (cfg["place_hover_joints"], slow),
    }


def test_house_places_with_the_counter_waypoints(recorded):
    node, calls = recorded("small_house")
    node.run()

    arm = {label: list(value) for kind, label, value in calls if kind == "arm"}
    house = SCENES["small_house"]
    assert arm["look at the object"] == house["grasp_joints"]
    assert arm["lift"] == house["hover_joints"]
    assert arm["hover above the place"] == house["place_hover_joints"]
    assert arm["place"] == house["place_grasp_joints"]
    assert arm["clear (hover)"] == house["place_hover_joints"]


def test_place_waypoints_default_to_the_garden_pick_waypoints(make_node):
    # The node's own defaults (no launch file) are the garden's, which
    # places with its pick waypoints.
    plan = make_node().transfer_plan()

    assert plan.place_hover == tuple(demo.HOVER)
    assert plan.place_grasp == tuple(demo.GRASP)


def test_the_object_is_attached_before_the_gripper_closes(recorded):
    node, calls = recorded()
    node.run()

    events = [(kind, label) for kind, label, _ in calls if kind in ("grasp", "gripper")]
    assert events == [
        ("grasp", False), ("gripper", "open"),
        ("grasp", True), ("gripper", "close"),
        ("gripper", "open"), ("grasp", False),
    ]


def test_run_aborts_on_first_failure(recorded, monkeypatch):
    node, calls = recorded(fail_at=3)  # 'home'
    errors = []
    logger = node.get_logger()
    monkeypatch.setattr(node, "get_logger", lambda: SimpleNamespace(
        info=logger.info, warn=logger.warn, error=errors.append))

    node.run()

    assert len(calls) == 3
    assert errors == ["step ''home'' failed — aborting"]


def test_place_base_pose_must_have_three_values(recorded):
    node, _ = recorded(place_base_pose=[0.0, 1.0])

    with pytest.raises(ValueError, match="place_base_pose"):
        node.transfer_plan()


def test_ragged_route_fails_at_startup(recorded):
    node, _ = recorded(place_route=[0.0, 0.0, 0.0, 1.0, 2.0])

    with pytest.raises(ValueError, match="place_route"):
        node.run()


def test_execute_without_odom_is_an_error(recorded, monkeypatch):
    node, calls = recorded(execute=True)
    monkeypatch.setattr(node, "_wait_for_odom_pose", lambda: None)

    with pytest.raises(RuntimeError, match="/odom"):
        node.run()
    assert calls == []


def test_plan_only_run_commands_nothing(make_node, monkeypatch):
    # The real step methods in plan-only mode: no /cmd_vel, no /grasp/set,
    # no gripper goal. Only MoveIt is faked (planning needs move_group).
    node = make_node(execute=False)
    sent, planned = [], []

    def forbidden(*_args, **_kwargs):
        raise AssertionError("plan-only mode must not command hardware interfaces")

    monkeypatch.setattr(node, "_wait_servers", lambda: None)
    monkeypatch.setattr(node, "_wait_for_odom_pose", lambda: None)  # no sim at all
    monkeypatch.setattr(node, "_cmd_vel", SimpleNamespace(publish=sent.append))
    monkeypatch.setattr(node._grasp, "call_async", forbidden)
    monkeypatch.setattr(node._gripper, "send_goal_async", forbidden)
    monkeypatch.setattr(
        node, "move_to_joint_state", lambda label, q: planned.append(label) or True
    )

    node.run()

    assert sent == []
    assert planned[-1] == "retract 'home'"


# ---- approach error, approach_only and the approach result line ------------

APPROACH_ONLY_CAPTIONS = [
    "release object", "open gripper", "'home'", "carry pose",
    "drive to the table", "look at the object", "approach the object",
    "attach object", "close gripper", "lift",
    # set the object back where it was, then end as the transfer does
    "set down", "open gripper", "release object", "back off", "carry pose",
    "return to start", "retract 'home'",
]


def _captions(node):
    return [caption for caption, _ in node.sequence(node.transfer_plan(), START)]


def test_displaced_pose_rotates_the_error_into_the_heading():
    pose = demo.displaced_pose((1.0, 2.0, math.pi / 2), (0.1, 0.0, 0.0))

    assert pose == pytest.approx((1.0, 2.1, math.pi / 2))


def test_displaced_pose_adds_the_yaw_error_and_wraps_it():
    x, y, yaw = demo.displaced_pose((0.385, 0.0, 0.0), (-0.08, 0.06, 0.07))

    assert (x, y, yaw) == pytest.approx((0.305, 0.06, 0.07))
    assert demo.displaced_pose((0.0, 0.0, 3.1), (0.0, 0.0, 0.1))[2] == pytest.approx(
        3.2 - 2 * math.pi)


def test_a_zero_error_leaves_the_pose_exactly():
    pose = (0.385, 0.0, 0.0)

    assert demo.displaced_pose(pose, (0.0, 0.0, 0.0)) == pose


def test_tcp_target_puts_the_base_frame_tcp_in_odom():
    assert demo.tcp_target((0.535, 0.0, 0.0), (0.430, 0.871)) == pytest.approx(
        (0.965, 0.0, 0.871))
    assert demo.tcp_target((0.0, 1.0, math.pi / 2), (0.4, 0.9)) == pytest.approx(
        (0.0, 1.4, 0.9))


def test_tcp_offset_is_in_millimetres_along_the_pick_heading():
    offset = demo.tcp_offset_mm((0.9631, 0.0002, 0.8707), (0.965, 0.0, 0.871))

    assert offset == pytest.approx((-1.9, 0.2, -0.3))
    # Heading +y: along = +y, across = -x.
    assert demo.tcp_offset_mm((0.001, 1.002, 0.0), (0.0, 1.0, 0.0), math.pi / 2) == (
        pytest.approx((2.0, -1.0, 0.0)))


@pytest.mark.parametrize("offset, aligned", [
    ((0.0, 0.0, 0.0), True),
    ((-10.0, 0.0, 0.0), True), ((-10.1, 0.0, 0.0), False),  # TCP short: pads still across
    ((5.0, 0.0, 0.0), True), ((5.1, 0.0, 0.0), False),  # TCP past: palm nears the object
    ((0.0, 6.0, 0.0), True), ((0.0, -6.1, 0.0), False),  # finger clearance per side
    ((0.0, 0.0, -10.0), True), ((0.0, 0.0, 10.1), False),  # pads vs rim and table
])
def test_alignment_holds_inside_the_finger_palm_and_pad_clearances(offset, aligned):
    assert demo.is_aligned(offset) is aligned


def test_approach_result_line_format():
    line = demo.format_approach_result("mpc", 7.43, (-1.9, 0.2, -0.3), math.radians(4.8))

    assert line == ("approach result: mode=mpc time=7.4 s tcp offset=(-1.9, +0.2, -0.3) mm "
                    "yaw=+4.8 deg aligned=yes")


def test_approach_result_line_without_a_tcp():
    line = demo.format_approach_result("sequential", 12.0, None, 0.0)

    assert line == ("approach result: mode=sequential time=12.0 s tcp offset=n/a "
                    "yaw=+0.0 deg aligned=n/a")


def test_a_zero_error_keeps_todays_steps(recorded):
    node, _ = recorded(approach_error=[0.0, 0.0, 0.0])

    assert _captions(node) == GARDEN_CAPTIONS


def test_an_error_inserts_the_corrective_leg(recorded):
    node, calls = recorded(approach_error=[-0.10, 0.10, 0.05])
    node.run()

    plan = node.transfer_plan()
    drives = [(label, value[0]) for kind, label, value in calls if kind == "drive"]
    assert drives[:3] == [
        ("drive to the table", plan.pick_start),
        ("correct the standoff", plan.pick_standoff),
        ("approach the object", plan.pick),
    ]
    assert plan.pick_start == pytest.approx(
        demo.displaced_pose(plan.pick_standoff, (-0.10, 0.10, 0.05)))
    captions = _captions(node)
    assert captions[captions.index("drive to the table") + 1] == "correct the standoff"


@pytest.mark.parametrize("error", [
    [0.1, 0.0],  # not [dx, dy, dyaw]
    [-0.20, 0.20, 0.0],  # 0.28 m: beyond Nav2's xy goal tolerance it stands in for
    [0.0, 0.0, 0.2],  # beyond Nav2's yaw goal tolerance
    [0.15, 0.0, 0.0],  # at or past the pick approach: the base would drive into the table
])
def test_bad_approach_error_fails_before_any_wait(recorded, monkeypatch, error):
    node, calls = recorded(approach_error=error)
    monkeypatch.setattr(node, "_wait_servers", lambda: pytest.fail("waited"))

    with pytest.raises(ValueError, match="approach_error"):
        node.run()
    assert calls == []


@pytest.mark.parametrize("key", ["grasp_tcp", "hover_tcp"])
def test_tcp_parameters_must_be_x_and_z(recorded, key):
    node, _ = recorded(**{key: [0.43, 0.0, 0.87]})

    with pytest.raises(ValueError, match=key):
        node.transfer_plan()


@pytest.mark.parametrize("scene", sorted(SCENES))
def test_plan_carries_the_grasp_target_and_the_hover_height(recorded, scene):
    node, _ = recorded(scene)
    cfg = SCENES[scene]

    plan = node.transfer_plan()

    assert plan.grasp_target == pytest.approx(
        (cfg["base_approach_x"] + cfg["grasp_tcp"][0], 0.0, cfg["grasp_tcp"][1]))
    assert plan.hover_tcp_z == cfg["hover_tcp"][1]
    assert plan.pick_start == plan.pick_standoff  # no error by default


def test_approach_only_sets_the_object_back_and_ends_away_from_the_table(recorded):
    node, calls = recorded(approach_only=True)
    node.run()

    assert _captions(node) == APPROACH_ONLY_CAPTIONS
    plan = node.transfer_plan()
    arm = {label: value for kind, label, value in calls if kind == "arm"}
    assert arm["set down"] == plan.grasp
    (back_off,) = [value for kind, label, value in calls if label == "back off"]
    assert back_off == (plan.pick_standoff, demo.BASE_APPROACH_SPEED_MPS)
    events = [(kind, label) for kind, label, _ in calls if kind in ("grasp", "gripper")]
    assert events[-3:] == [("gripper", "close"), ("gripper", "open"), ("grasp", False)]


def _executing_with_log(recorded, monkeypatch, tcp, clock=(100.0, 107.43), **parameters):
    """Return an executing recorded node whose TCP lookup gives `tcp`; logs join its calls."""
    node, calls = recorded(execute=True, **parameters)
    times = iter(clock)
    monkeypatch.setattr(node, "_clock_sec", lambda: next(times))
    monkeypatch.setattr(node, "_tcp_in_odom", lambda: tcp)
    monkeypatch.setattr(node, "_odom_pose", lambda: (0.5349, 0.0, math.radians(0.2)))
    logger = node.get_logger()
    monkeypatch.setattr(node, "get_logger", lambda: SimpleNamespace(
        info=lambda msg: calls.append(("log", msg, None)),
        warn=lambda msg: calls.append(("warn", msg, None)), error=logger.error))
    return node, calls


def test_the_approach_result_is_logged_between_attach_and_close(recorded, monkeypatch):
    node, calls = _executing_with_log(recorded, monkeypatch, (0.9431, 0.0002, 0.8418))
    node.run()

    events = [(kind, label) for kind, label, _ in calls if kind in ("grasp", "gripper", "log")]
    results = [label for kind, label in events if label_is_result(kind, label)]
    assert results == [
        "approach result: mode=sequential time=7.4 s tcp offset=(-6.9, +0.2, -0.2) mm "
        "yaw=+0.2 deg aligned=yes"
    ]
    at = events.index(("log", results[0]))
    assert events[at - 1] == ("grasp", True) and events[at + 1] == ("gripper", "close")


def label_is_result(kind, label):
    return kind == "log" and label.startswith("approach result:")


def test_a_failed_tcp_lookup_warns_and_does_not_fail_the_pick(recorded, monkeypatch):
    node, calls = _executing_with_log(recorded, monkeypatch, None)

    node.run()

    assert any(label_is_result(kind, label) and "tcp offset=n/a" in label
               for kind, label, _ in calls)
    assert ("gripper", "close", None) in calls  # the pick went on


def test_tcp_lookup_without_tf_returns_none_and_warns(make_node, monkeypatch):
    node = make_node(execute=True)
    monkeypatch.setattr(demo, "TF_TIMEOUT_SEC", 0.05)
    warnings = []
    logger = node.get_logger()
    monkeypatch.setattr(node, "get_logger", lambda: SimpleNamespace(
        info=logger.info, warn=warnings.append, error=logger.error))

    assert node._tcp_in_odom() is None
    assert len(warnings) == 1 and "piper_tcp" in warnings[0]


@pytest.mark.parametrize("approach_only", [False, True])
def test_plan_only_with_an_error_commands_nothing(make_node, monkeypatch, approach_only):
    node = make_node(execute=False, approach_error=[-0.08, 0.06, 0.07],
                     approach_only=approach_only)
    sent = []

    def forbidden(*_args, **_kwargs):
        raise AssertionError("plan-only mode must not command or measure anything")

    monkeypatch.setattr(node, "_wait_servers", lambda: None)
    monkeypatch.setattr(node, "_wait_for_odom_pose", lambda: None)
    monkeypatch.setattr(node, "_cmd_vel", SimpleNamespace(publish=sent.append))
    monkeypatch.setattr(node._grasp, "call_async", forbidden)
    monkeypatch.setattr(node._gripper, "send_goal_async", forbidden)
    monkeypatch.setattr(node, "_tcp_in_odom", forbidden)
    monkeypatch.setattr(node, "move_to_joint_state", lambda label, q: True)

    node.run()

    assert sent == []


@pytest.mark.parametrize("error", [[math.nan, 0.0, 0.0], [0.0, math.inf, 0.0]])
def test_non_finite_approach_error_is_rejected(error):
    with pytest.raises(ValueError, match="approach_error"):
        demo.check_approach_error(error, 0.15)


def test_non_finite_tcp_is_rejected():
    with pytest.raises(ValueError, match="grasp_tcp"):
        demo.tcp_xz("grasp_tcp", [0.43, math.nan])


def test_a_stale_tcp_transform_warns(make_node, monkeypatch):
    from geometry_msgs.msg import TransformStamped

    node = make_node(execute=True)
    old = TransformStamped()
    old.header.stamp.sec = 100  # node clock (wall time here) is decades later
    old.transform.translation.x = 0.9
    monkeypatch.setattr(node, "_tf_buffer", SimpleNamespace(
        lookup_transform=lambda *_args: old))
    warnings = []
    logger = node.get_logger()
    monkeypatch.setattr(node, "get_logger", lambda: SimpleNamespace(
        info=logger.info, warn=warnings.append, error=logger.error))

    assert node._tcp_in_odom() == (0.9, 0.0, 0.0)
    assert len(warnings) == 1 and "old" in warnings[0]
