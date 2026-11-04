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


def _scene_parameters(name):
    cfg = SCENES[name]
    keys = ("object_name", "base_approach_x", "gripper_closed", "hover_joints",
            "grasp_joints", "standoff_m", "place_base_pose", "place_route")
    return {key: cfg[key] for key in keys}


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


def test_lateral_floor_is_above_the_sideways_dead_zone():
    _, vy, _ = demo.base_command((0.0, 0.003, 0.0), START, 0.25, turning=False)

    assert vy == pytest.approx(demo.BASE_MIN_LATERAL_SPEED_MPS)
    assert demo.BASE_MIN_LATERAL_SPEED_MPS > 2 * SIM_LATERAL_DEAD_ZONE_MPS
    assert demo.BASE_MIN_YAW_RATE > 2 * SIM_YAW_DEAD_ZONE_RAD_S


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

# The sim base skids: its steering is locked straight, so turning in place
# and moving sideways drag the wheels across the ground. Measured in
# garden_world on 2026-11-04 (constant commands, 2 s each): no motion below
# these rates, about (command - dead zone) above them. Forward is exact.
SIM_YAW_DEAD_ZONE_RAD_S = 0.13
SIM_LATERAL_DEAD_ZONE_MPS = 0.035
BASES = {"ideal": (0.0, 0.0), "skidding": (SIM_YAW_DEAD_ZONE_RAD_S, SIM_LATERAL_DEAD_ZONE_MPS)}
# /cmd_vel takes effect this many /odom updates late (the sim: about 1).
LAGS = (0, 1, 2)
FAKE_BASES = [(base, lag) for base in sorted(BASES) for lag in LAGS]


def _odom(x, y, yaw):
    pose = Pose()
    pose.position.x, pose.position.y = x, y
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return pose


def _dead_zone(command, zone):
    return math.copysign(max(0.0, abs(command) - zone), command)


@pytest.fixture(params=FAKE_BASES, ids=[f"{base}-lag{lag}" for base, lag in FAKE_BASES])
def kinematic_base(request, make_node, monkeypatch):
    """
    Return an executing node on a fake base, its published twists and a clock.

    The base is ideal or skids like the sim's (BASES) and applies /cmd_vel
    `lag` updates late. clock.path collects every base pose /odom reports,
    clock.stamps the time of every published twist. Set clock.moving = False
    for a stuck base, clock.odom_until for an /odom that stops, and
    clock.interrupt_at for a Ctrl-C at that time.
    """
    (yaw_zone, lateral_zone), lag = BASES[request.param[0]], request.param[1]
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
        vx, vy = tw.linear.x, _dead_zone(tw.linear.y, lateral_zone)
        c, s = math.cos(yaw), math.sin(yaw)
        node._odom = _odom(
            x + (c * vx - s * vy) * SIM_STEP_SEC,
            y + (s * vx + c * vy) * SIM_STEP_SEC,
            yaw + _dead_zone(tw.angular.z, yaw_zone) * SIM_STEP_SEC,
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
    # origin, 12 mm off it (the skidding turn drags the base along).
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


def test_arm_is_in_the_carry_pose_on_every_base_leg_away_from_a_table(recorded):
    node, calls = recorded()
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
    assert legs == 10


def test_slide_ins_are_slow_at_grasp_or_hover_height(recorded):
    node, calls = recorded()
    node.run()

    arm, slides = None, {}
    for kind, label, value in calls:
        if kind == "arm":
            arm = list(value)
        elif kind == "drive" and label in SLIDE_LEGS:
            slides[label] = (arm, value[1])
    cfg = SCENES["garden"]
    slow = demo.BASE_APPROACH_SPEED_MPS
    assert slides == {
        "approach the object": (cfg["grasp_joints"], slow),
        "approach the place": (cfg["hover_joints"], slow),
        "back off": (cfg["hover_joints"], slow),
    }


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
