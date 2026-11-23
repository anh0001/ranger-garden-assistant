"""
Unit tests for scripts/wbmpc_reach_demo.py (no simulator, no MoveIt, no Nav2).

Covers the step order, plan-only mode, the failure and restore paths and
the target message. The step actions are replaced by recorders where the
order is under test; plan-only runs the real methods with every command
interface made to fail if touched.
"""

import importlib.util
import math
from pathlib import Path
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

PKG = Path(__file__).resolve().parents[1]
VIDEO_CAPTION_MAX_CHARS = 60  # scripts/record_demo_video.py CAPTION_MAX_CHARS


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


demo = _load("wbmpc_reach_demo", PKG / "scripts" / "wbmpc_reach_demo.py")

PARKED = [
    "open gripper", "carry pose", "base to the whole-body MPC", "arm to streamed setpoints",
    "reach: base + arm (MPC)", "hold at the target", "arm back to MoveIt", "base back to Nav2",
    "carry pose",
]
WITH_NAV = [
    "open gripper", "carry pose", "Nav2 MPPI: drive away", "Nav2 MPPI: to the handover pose",
    "base to the whole-body MPC", "arm to streamed setpoints", "reach: base + arm (MPC)",
    "hold at the target", "arm back to MoveIt", "base back to Nav2", "carry pose",
    "Nav2 MPPI: drive away",
]


@pytest.fixture
def make_node():
    rclpy.init()
    nodes = []

    def make(**parameters):
        overrides = [Parameter(name, value=value) for name, value in parameters.items()]
        nodes.append(demo.WbmpcReachDemo(parameter_overrides=overrides))
        return nodes[-1]

    yield make
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def recorded(make_node, monkeypatch):
    """Return a factory of (node, calls): nodes whose actions only record (kind, value)."""

    def make(fail_at=None, raise_at=None, **parameters):
        node = make_node(**parameters)
        calls = []

        def record(kind, value=None):
            calls.append((kind, value))
            if len(calls) == raise_at:
                raise KeyboardInterrupt
            return len(calls) != fail_at

        monkeypatch.setattr(node, "_wait_servers", lambda: None)
        monkeypatch.setattr(node, "_wait_reach_servers", lambda: None)
        monkeypatch.setattr(node, "move_to_joint_state", lambda label, q: record("arm", label))
        monkeypatch.setattr(node, "set_gripper", lambda position, label: record("gripper", label))
        monkeypatch.setattr(node, "set_base_owner", lambda manipulate: record("base", manipulate))
        monkeypatch.setattr(node, "switch_arm",
                            lambda stream, strict=True: record("stream", stream))
        monkeypatch.setattr(node, "navigate", lambda pose, label: record("nav", pose))
        monkeypatch.setattr(node, "reach", lambda label: record("reach", label))
        monkeypatch.setattr(node, "hold", lambda sec: record("hold", sec))
        monkeypatch.setattr(node, "stop_base", lambda: record("stop"))
        return node, calls

    return make


def _captions(node):
    return [caption for caption, _ in node.sequence(node.reach_plan())]


# ---- step order -----------------------------------------------------------------


def test_parked_sequence_hands_over_reaches_and_hands_back(recorded):
    node, _ = recorded()

    assert _captions(node) == PARKED


def test_nav_sequence_drives_away_and_back_before_the_handover(recorded):
    node, _ = recorded(use_nav=True)

    assert _captions(node) == WITH_NAV


@pytest.mark.parametrize("use_nav", [False, True])
def test_captions_fit_the_video(recorded, use_nav):
    node, _ = recorded(use_nav=use_nav)

    assert all(len(caption) <= VIDEO_CAPTION_MAX_CHARS for caption in _captions(node))


def test_handover_order_is_gate_then_arm_then_target(recorded):
    node, calls = recorded(execute=True, use_nav=True)

    node.run()

    kinds = [kind for kind, _ in calls]
    assert kinds.index("base") < kinds.index("stream") < kinds.index("reach")
    assert calls[kinds.index("base")] == ("base", True)
    navs = [value for kind, value in calls if kind == "nav"]
    assert navs == [(-2.0, 0.0, 3.1416), (0.0, 0.0, 0.0), (-2.0, 0.0, 3.1416)]


def test_hand_back_is_arm_then_gate_then_moveit(recorded):
    node, calls = recorded(execute=True)

    node.run()

    after_reach = calls[[kind for kind, _ in calls].index("hold") + 1:]
    assert after_reach[:3] == [("stream", False), ("base", False), ("arm", "carry pose")]


# ---- failure and restore -------------------------------------------------------


def test_a_failing_step_aborts_the_rest_and_restores(recorded):
    node, calls = recorded(execute=True, fail_at=5)  # the reach

    node.run()

    assert ("reach", "reach: base + arm (MPC)") in calls
    assert ("hold", 2.0) not in calls
    assert calls[-3:] == [("base", False), ("stream", False), ("stop", None)]


def test_ctrl_c_still_restores(recorded):
    node, calls = recorded(execute=True, raise_at=5)

    with pytest.raises(KeyboardInterrupt):
        node.run()

    assert calls[-3:] == [("base", False), ("stream", False), ("stop", None)]


def test_plan_only_restore_does_nothing(recorded):
    node, calls = recorded(execute=False)

    node.restore()

    assert calls == []


@pytest.mark.parametrize("parameter, value", [
    ("target_position", [0.9, 0.1]),
    ("nav_away_pose", [-2.0, 0.0]),
    ("nav_handover_pose", [0.0, 0.0, 0.0, 1.0]),
])
def test_bad_values_fail_before_any_wait(recorded, monkeypatch, parameter, value):
    node, _ = recorded(**{parameter: value})
    monkeypatch.setattr(node, "_wait_servers", lambda: pytest.fail("waited"))

    with pytest.raises(ValueError, match=parameter):
        node.run()


def test_plan_only_run_commands_nothing(make_node, monkeypatch):
    node = make_node(execute=False, use_nav=True)
    planned = []

    def forbidden(*_args, **_kwargs):
        raise AssertionError("plan-only mode must not command anything")

    monkeypatch.setattr(node, "_wait_servers", lambda: None)
    monkeypatch.setattr(node, "_wait_reach_servers", forbidden)
    monkeypatch.setattr(node, "_cmd_vel", SimpleNamespace(publish=forbidden))
    monkeypatch.setattr(node, "_target_pub", SimpleNamespace(publish=forbidden))
    monkeypatch.setattr(node._gate_client, "call_async", forbidden)
    monkeypatch.setattr(node._switch_client, "call_async", forbidden)
    monkeypatch.setattr(node._nav, "send_goal_async", forbidden)
    monkeypatch.setattr(node._gripper, "send_goal_async", forbidden)
    monkeypatch.setattr(
        node, "move_to_joint_state", lambda label, q: planned.append(label) or True
    )

    node.run()

    assert planned == ["carry pose", "carry pose"]


# ---- target and states ---------------------------------------------------------


def test_target_message_is_a_level_gripper_in_odom(make_node):
    node = make_node(target_position=[0.95, 0.15, 1.0], target_yaw=0.3)

    msg = node.target_message(node.reach_plan())

    assert msg.header.frame_id == "odom"
    p, o = msg.pose.position, msg.pose.orientation
    assert (p.x, p.y, p.z) == (0.95, 0.15, 1.0)
    assert (o.x, o.y, o.z, o.w) == pytest.approx(demo.level_quaternion(0.3))


@pytest.mark.parametrize("yaw", [-2.0, 0.0, 0.3, math.pi])
def test_level_quaternion_matches_the_mpc_core(yaw):
    pytest.importorskip("casadi")
    core = _load("whole_body_mpc_core", PKG / "scripts" / "whole_body_mpc_core.py")

    assert demo.level_quaternion(yaw) == pytest.approx(core.level_quaternion(yaw))


def test_wait_for_state_gives_up_at_once_on_a_fault(make_node):
    node = make_node()
    node._on_mpc_state(String(data="FAULT: odom older than 0.3 s"))
    start = time.monotonic()

    reached = node.wait_for_state("REACHED", timeout_sec=30.0)

    assert not reached
    assert time.monotonic() - start < 1.0


def test_wait_for_state_sees_the_wanted_state(make_node):
    node = make_node()
    node._on_mpc_state(String(data="REACHED"))

    assert node.wait_for_state("REACHED", timeout_sec=1.0)


def test_switch_arm_skips_a_switch_to_the_active_controller(make_node, monkeypatch):
    node = make_node(execute=True)
    monkeypatch.setattr(node._switch_client, "call_async",
                        lambda *_: pytest.fail("no switch needed"))

    assert node.switch_arm(stream=False)


# ---- main ---------------------------------------------------------------------------


class _FakeDemo:
    """Stands in for WbmpcReachDemo in main(): run() raises KeyboardInterrupt."""

    destroyed = []

    def run(self):
        raise KeyboardInterrupt

    def get_logger(self):
        return SimpleNamespace(info=lambda *_: None, warn=lambda *_: None,
                               error=lambda *_: None)

    def destroy_node(self):
        self.destroyed.append(True)


def test_main_ends_cleanly_on_ctrl_c(monkeypatch):
    fake = type("Fake", (_FakeDemo,), {"destroyed": []})
    monkeypatch.setattr(demo, "WbmpcReachDemo", fake)

    try:
        demo.main()
    except KeyboardInterrupt:
        pytest.fail("main() let Ctrl-C escape")

    assert fake.destroyed == [True]


# ---- executing paths with fake servers -------------------------------------------


def _done(result):
    from rclpy.task import Future

    future = Future()
    future.set_result(result)
    return future


@pytest.fixture
def executing(make_node, monkeypatch):
    """Return an executing demo with short timeouts and the base pose faked."""
    node = make_node(execute=True, handover_timeout=0.2, reach_timeout=0.3, nav_timeout=0.3)
    monkeypatch.setattr(node, "_odom_pose", lambda: (0.3, 0.1, 0.0))
    return node


@pytest.mark.parametrize("manipulate, mode", [(True, "MANIPULATE"), (False, "NAVIGATE")])
def test_set_base_owner_waits_for_the_gate_mode(executing, monkeypatch, manipulate, mode):
    from std_srvs.srv import SetBool

    def answer(request):
        executing._on_gate_mode(String(data=mode))
        return _done(SetBool.Response(success=True))

    monkeypatch.setattr(executing._gate_client, "call_async", answer)

    assert executing.set_base_owner(manipulate)


def test_set_base_owner_fails_when_the_gate_does_not_settle(executing, monkeypatch):
    from std_srvs.srv import SetBool

    monkeypatch.setattr(executing._gate_client, "call_async",
                        lambda request: _done(SetBool.Response(success=True)))
    executing._on_gate_mode(String(data="SETTLING"))

    assert not executing.set_base_owner(True)


def test_set_base_owner_fails_without_an_answer(executing, monkeypatch):
    from rclpy.task import Future

    monkeypatch.setattr(executing._gate_client, "call_async", lambda request: Future())

    assert not executing.set_base_owner(True)


@pytest.mark.parametrize("ok", [True, False])
def test_switch_arm_follows_the_controller_manager(executing, monkeypatch, ok):
    from controller_manager_msgs.srv import SwitchController

    requests = []

    def answer(request):
        requests.append(request)
        return _done(SwitchController.Response(ok=ok))

    monkeypatch.setattr(executing._switch_client, "call_async", answer)

    assert executing.switch_arm(stream=True) is ok
    assert requests[0].activate_controllers == ["piper_arm_stream_controller"]
    assert requests[0].deactivate_controllers == ["piper_arm_controller"]
    assert requests[0].strictness == SwitchController.Request.STRICT
    assert executing._streaming is ok


@pytest.mark.parametrize("state, reached", [("REACHED", True), ("FAULT: odom", False)])
def test_reach_publishes_the_target_and_waits_for_the_mpc(executing, monkeypatch, state,
                                                          reached):
    sent = []

    def publish(msg):
        sent.append(msg)
        executing._on_mpc_state(String(data=state))

    monkeypatch.setattr(executing, "_target_pub", SimpleNamespace(publish=publish))

    assert executing.reach("reach") is reached
    assert sent[0].header.frame_id == "odom"


def test_reach_ignores_a_reached_state_from_before_the_target(executing, monkeypatch):
    executing._on_mpc_state(String(data="REACHED"))
    monkeypatch.setattr(executing, "_target_pub", SimpleNamespace(publish=lambda msg: None))

    assert not executing.reach("reach")


@pytest.mark.parametrize("state, ok", [("REACHED", True), ("FAULT: lag", False)])
def test_hold_fails_on_a_fault(executing, state, ok):
    executing._on_mpc_state(String(data=state))

    assert executing.hold(0.1) is ok


def _goal_handle(accepted=True, status=None):
    from action_msgs.msg import GoalStatus
    from rclpy.task import Future

    result = Future() if status is None else _done(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED if status == "ok" else GoalStatus.STATUS_ABORTED))
    cancelled = []

    def cancel():
        cancelled.append(True)
        return _done(SimpleNamespace(return_code=0))

    return SimpleNamespace(accepted=accepted, get_result_async=lambda: result,
                           cancel_goal_async=cancel, cancelled=cancelled)


@pytest.mark.parametrize("handle, ok", [
    (dict(status="ok"), True),
    (dict(status="aborted"), False),
    (dict(accepted=False), False),
])
def test_navigate_reports_the_nav2_result(executing, monkeypatch, handle, ok):
    goals = []

    def send(goal):
        goals.append(goal)
        return _done(_goal_handle(**handle))

    monkeypatch.setattr(executing._nav, "send_goal_async", send)

    assert executing.navigate((-2.0, 0.0, math.pi), "away") is ok
    assert goals[0].pose.header.frame_id == "map"
    assert goals[0].pose.pose.orientation.z == pytest.approx(1.0)


def test_navigate_cancels_a_goal_that_does_not_finish(executing, monkeypatch):
    handle = _goal_handle()  # result never arrives
    monkeypatch.setattr(executing._nav, "send_goal_async", lambda goal: _done(handle))

    assert not executing.navigate((0.0, 0.0, 0.0), "to the handover pose")
    assert handle.cancelled == [True]


def test_navigate_resends_an_unanswered_goal_once(executing, monkeypatch):
    from rclpy.task import Future

    monkeypatch.setattr(executing._nav, "send_goal_async", lambda goal: Future())
    monkeypatch.setattr(demo, "NAV_ACCEPT_TIMEOUT_SEC", 0.05)

    assert not executing.navigate((0.0, 0.0, 0.0), "to the handover pose")


def test_reach_servers_must_all_answer(executing, monkeypatch):
    monkeypatch.setattr(executing._gate_client, "wait_for_service", lambda timeout: False)

    with pytest.raises(RuntimeError, match="set_manipulate"):
        executing._wait_reach_servers()


def test_reach_servers_include_nav2_only_with_use_nav(make_node, monkeypatch):
    node = make_node(execute=True, use_nav=True)
    for client in (node._gate_client, node._switch_client):
        monkeypatch.setattr(client, "wait_for_service", lambda timeout: True)
    node._on_mpc_state(String(data="IDLE"))
    monkeypatch.setattr(node._target_pub, "get_subscription_count", lambda: 1)
    monkeypatch.setattr(node._nav, "wait_for_server", lambda timeout: False)

    with pytest.raises(RuntimeError, match="navigate_to_pose"):
        node._wait_reach_servers()


def test_main_reports_a_failed_demo_and_exits_non_zero(monkeypatch):
    errors = []

    class Failing(_FakeDemo):
        destroyed = []

        def run(self):
            raise RuntimeError("no /move_action")

        def get_logger(self):
            return SimpleNamespace(info=lambda *_: None, warn=lambda *_: None,
                                   error=errors.append)

    monkeypatch.setattr(demo, "WbmpcReachDemo", Failing)

    with pytest.raises(SystemExit) as stopped:
        demo.main()

    assert stopped.value.code == 1
    assert errors == ["demo failed: no /move_action"]


# ---- restore after review (cancel Nav2, isolated steps, best-effort switch) --------


def test_ctrl_c_during_a_nav2_leg_cancels_the_goal_in_restore(executing, monkeypatch):
    handle = _goal_handle()  # result never arrives
    monkeypatch.setattr(executing._nav, "send_goal_async", lambda goal: _done(handle))
    waits = []
    real_wait = demo.rclpy.spin_until_future_complete

    def wait(node, future, timeout_sec=None):
        waits.append(future)
        if len(waits) == 2:  # the wait for the Nav2 result
            raise KeyboardInterrupt
        return real_wait(node, future, timeout_sec=timeout_sec)

    monkeypatch.setattr(demo.rclpy, "spin_until_future_complete", wait)
    calls = []
    monkeypatch.setattr(executing, "set_base_owner", lambda m: calls.append(("base", m)))
    monkeypatch.setattr(executing, "switch_arm",
                        lambda stream, strict=True: calls.append(("stream", strict)))
    monkeypatch.setattr(executing, "stop_base", lambda: calls.append(("stop", None)))

    with pytest.raises(KeyboardInterrupt):
        executing.navigate((-2.0, 0.0, math.pi), "away")
    executing.restore()

    assert handle.cancelled == [True]
    assert calls == [("base", False), ("stream", False), ("stop", None)]


def test_restore_runs_every_step_even_when_one_fails(executing, monkeypatch):
    calls = []

    def broken(manipulate):
        raise RuntimeError("context shut down")

    monkeypatch.setattr(executing, "set_base_owner", broken)
    monkeypatch.setattr(executing, "switch_arm",
                        lambda stream, strict=True: calls.append("stream"))
    monkeypatch.setattr(executing, "stop_base", lambda: calls.append("stop"))

    executing.restore()

    assert calls == ["stream", "stop"]


def test_restore_switches_back_best_effort_whatever_it_believes(executing, monkeypatch):
    from controller_manager_msgs.srv import SwitchController

    requests = []

    def answer(request):
        requests.append(request)
        return _done(SwitchController.Response(ok=True))

    monkeypatch.setattr(executing._switch_client, "call_async", answer)
    monkeypatch.setattr(executing, "set_base_owner", lambda manipulate: True)
    monkeypatch.setattr(executing, "stop_base", lambda: None)

    executing.restore()  # _streaming is False: a crashed run may have left it on

    assert len(requests) == 1
    assert requests[0].activate_controllers == ["piper_arm_controller"]
    assert requests[0].strictness == SwitchController.Request.BEST_EFFORT


def test_an_unanswered_switch_leaves_the_arm_state_unknown(executing, monkeypatch):
    from rclpy.task import Future

    monkeypatch.setattr(executing._switch_client, "call_async", lambda request: Future())

    assert not executing.switch_arm(stream=True)
    assert executing._streaming is None


def test_run_reports_whether_the_sequence_completed(recorded):
    done, _ = recorded(execute=True)
    failed, _ = recorded(execute=True, fail_at=3)

    assert done.run() is True
    assert failed.run() is False


# ---- reach_pose and slide (the MPC pick's moving target) -------------------------

START_POINT = (0.800, 0.0, 0.871)
END_POINT = (0.815, 0.0, 0.871)


@pytest.mark.parametrize("elapsed, expected", [
    (0.0, START_POINT),
    (0.075, (0.8075, 0.0, 0.871)),  # halfway at 0.1 m/s
    (0.15, END_POINT),
    (5.0, END_POINT),
    (-1.0, START_POINT),  # a sim-time reset never moves the target backwards past the start
])
def test_slide_point_moves_along_the_line_and_stops_at_the_end(elapsed, expected):
    assert demo.slide_point(START_POINT, END_POINT, 0.1, elapsed) == pytest.approx(expected)


def test_slide_point_ends_exactly_at_the_end():
    assert demo.slide_point(START_POINT, END_POINT, 0.1, 1.0) == END_POINT


def test_a_zero_length_slide_is_at_its_end_at_once():
    assert demo.slide_point(END_POINT, END_POINT, 0.03, 0.0) == END_POINT


def test_the_node_name_can_be_given(make_node):
    rclpy_node = demo.WbmpcReachDemo(node_name="pick_place_demo")
    try:
        assert rclpy_node.get_name() == "pick_place_demo"
    finally:
        rclpy_node.destroy_node()


def _streaming(executing, monkeypatch, on_publish=None):
    """Record the published targets; on_publish(points) may change the MPC state."""
    points = []

    def publish(msg):
        p = msg.pose.position
        points.append((p.x, p.y, p.z))
        if on_publish is not None:
            on_publish(points)

    monkeypatch.setattr(executing, "_target_pub", SimpleNamespace(publish=publish))
    monkeypatch.setattr(demo, "SLIDE_PERIOD_SEC", 0.02)
    return points


def test_reach_pose_publishes_the_pose_and_waits_for_reached(executing, monkeypatch):
    sent = []

    def publish(msg):
        sent.append(msg)
        executing._on_mpc_state(String(data="REACHED"))

    monkeypatch.setattr(executing, "_target_pub", SimpleNamespace(publish=publish))

    assert executing.reach_pose(END_POINT, 0.2, "MPC: pre-grasp")
    p, o = sent[0].pose.position, sent[0].pose.orientation
    assert (p.x, p.y, p.z) == END_POINT
    assert (o.x, o.y, o.z, o.w) == pytest.approx(demo.level_quaternion(0.2))


def test_slide_streams_advancing_targets_ending_exactly_at_the_end(executing, monkeypatch):
    def reached_at_the_end(points):
        state = "REACHED" if points[-1] == END_POINT else "TRACKING"
        executing._on_mpc_state(String(data=state))

    points = _streaming(executing, monkeypatch, reached_at_the_end)

    assert executing.slide(START_POINT, END_POINT, 0.0, 0.1, "MPC: slide in")
    xs = [x for x, _, _ in points]
    assert len(points) >= 3, points
    # The clock runs on between the start and the first target: microns.
    assert xs == sorted(xs) and points[0] == pytest.approx(START_POINT, abs=1e-4)
    assert points[-1] == END_POINT and points.count(END_POINT) == 1


def test_slide_accepts_an_mpc_that_stays_reached_throughout(executing, monkeypatch):
    # The MPC publishes its state on change only: if it is REACHED for every
    # target of the slide it never says so again, and the slide must not wait.
    executing._on_mpc_state(String(data="REACHED"))
    _streaming(executing, monkeypatch)

    assert executing.slide(START_POINT, END_POINT, 0.0, 0.1, "MPC: lift")


def test_a_fault_during_the_slide_ends_it_at_once(executing, monkeypatch):
    def fault_on_the_second(points):
        if len(points) == 2:
            executing._on_mpc_state(String(data="FAULT: command lag 0.2 rad"))

    points = _streaming(executing, monkeypatch, fault_on_the_second)
    begin = time.monotonic()

    assert not executing.slide(START_POINT, (1.0, 0.0, 0.871), 0.0, 0.1, "MPC: slide in")
    assert len(points) == 2
    assert time.monotonic() - begin < 1.0


def test_a_zero_length_slide_sends_one_target_then_waits(executing, monkeypatch):
    points = _streaming(
        executing, monkeypatch, lambda points: executing._on_mpc_state(String(data="REACHED")))

    assert executing.slide(END_POINT, END_POINT, 0.0, 0.03, "MPC: slide in")
    assert points == [END_POINT]


def test_slide_gives_up_when_the_clock_stands_still(executing, monkeypatch):
    # A paused sim: the node clock never reaches the next point.
    _streaming(executing, monkeypatch)
    monkeypatch.setattr(executing, "_clock_sec", lambda: 42.0)

    assert not executing.slide(START_POINT, END_POINT, 0.0, 0.1, "MPC: slide in")


def test_slide_speed_must_be_positive(executing):
    with pytest.raises(ValueError, match="speed"):
        executing.slide(START_POINT, END_POINT, 0.0, 0.0, "MPC: slide in")


def test_plan_only_slide_and_reach_pose_publish_nothing(make_node, monkeypatch):
    node = make_node(execute=False)
    monkeypatch.setattr(node, "_target_pub",
                        SimpleNamespace(publish=lambda msg: pytest.fail("published")))

    assert node.slide(START_POINT, END_POINT, 0.0, 0.03, "MPC: slide in")
    assert node.reach_pose(END_POINT, 0.0, "MPC: pre-grasp")


def test_slide_settles_on_the_node_clock_not_the_wall_clock(make_node, monkeypatch):
    # A slow sim (real-time factor 0.2): the MPC answers the end target only
    # after two of its 50 ms ticks of sim time. Its REACHED from before the end
    # target is stale (a REACHED is latched up to 3x the tolerance) and must
    # not end the slide.
    node = make_node(execute=True, reach_timeout=3.0)
    monkeypatch.setattr(node, "_odom_pose", lambda: (0.3, 0.1, 0.0))
    begin, end_sent = time.monotonic(), []

    def sim_clock():
        now = (time.monotonic() - begin) * 0.2
        if end_sent:
            since = now - end_sent[0]
            node._mpc_state = ("REACHED" if since >= 0.2 else
                               "TRACKING" if since >= 0.1 else node._mpc_state)
        return now

    def publish(msg):
        p = msg.pose.position
        if (p.x, p.y, p.z) == END_POINT and not end_sent:
            end_sent.append(sim_clock())

    monkeypatch.setattr(node, "_clock_sec", sim_clock)
    monkeypatch.setattr(node, "_target_pub", SimpleNamespace(publish=publish))
    monkeypatch.setattr(demo, "SLIDE_PERIOD_SEC", 0.02)
    node._on_mpc_state(String(data="REACHED"))

    assert node.slide(START_POINT, END_POINT, 0.0, 0.1, "MPC: slide in")
    assert sim_clock() - end_sent[0] >= 0.2, "took the stale REACHED"


def test_destroying_the_node_twice_is_harmless(make_node):
    node = make_node()

    node.destroy_node()
    node.destroy_node()  # the fixture destroys it once more
