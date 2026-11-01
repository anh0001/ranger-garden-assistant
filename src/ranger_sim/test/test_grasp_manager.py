"""Unit tests for scripts/grasp_manager.py (no simulator required)."""

import importlib.util
import time
from pathlib import Path

import pytest
import rclpy
import yaml
from rclpy.parameter import Parameter

PKG_DIR = Path(__file__).resolve().parents[1]


def _load_script(name: str):
    spec = importlib.util.spec_from_file_location(
        name, PKG_DIR / "scripts" / f"{name}.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gm = _load_script("grasp_manager")

VALID_CONFIG = {
    "gripper_link": "piper_link6",
    "objects": [
        {"name": "cup", "link": "link"},
        {"name": "drawer_cab", "link": "drawer"},
    ],
}


# ---- config parsing ------------------------------------------------------
def test_parse_grasp_config_keeps_file_order():
    config = gm.parse_grasp_config(VALID_CONFIG)

    assert config.gripper_link == "piper_link6"
    assert config.names == ("cup", "drawer_cab")
    assert config.objects[1] == gm.GraspObject(name="drawer_cab", link="drawer")


@pytest.mark.parametrize(
    "data, fragment",
    [
        ([], "mapping"),
        ({"objects": VALID_CONFIG["objects"]}, "gripper_link"),
        ({"gripper_link": "l6", "objects": []}, "non-empty list"),
        ({"gripper_link": "l6", "objects": ["cup"]}, "objects[0] must be a mapping"),
        ({"gripper_link": "l6", "objects": [{"link": "link"}]}, "'name'"),
        ({"gripper_link": "l6", "objects": [{"name": "cup"}]}, "'link'"),
    ],
)
def test_parse_grasp_config_rejects_malformed_input(data, fragment):
    with pytest.raises(ValueError) as excinfo:
        gm.parse_grasp_config(data)

    assert fragment in str(excinfo.value)


def test_parse_grasp_config_rejects_duplicate_names():
    data = {"gripper_link": "l6", "objects": [{"name": "cup", "link": "a"}] * 2}

    with pytest.raises(ValueError) as excinfo:
        gm.parse_grasp_config(data)

    assert "duplicate object names: ['cup']" in str(excinfo.value)


def test_shipped_config_is_valid():
    config = gm.load_grasp_config(str(PKG_DIR / "config" / "graspable_objects.yaml"))

    assert config.gripper_link == "piper_link6"
    assert "cup" in config.names


def test_bridge_has_entries_for_every_graspable_object():
    # gz_bridge.yaml is hand-synced with graspable_objects.yaml.
    config = gm.load_grasp_config(str(PKG_DIR / "config" / "graspable_objects.yaml"))
    with open(PKG_DIR / "config" / "gz_bridge.yaml", encoding="utf-8") as stream:
        entries = {e.get("topic_name"): e for e in yaml.safe_load(stream)}

    for name in config.names:
        for suffix, direction, gz_type in (
            ("attach", "ROS_TO_GZ", "ignition.msgs.Empty"),
            ("detach", "ROS_TO_GZ", "ignition.msgs.Empty"),
            ("state", "GZ_TO_ROS", "ignition.msgs.StringMsg"),
        ):
            entry = entries.get(f"/grasp/{name}/{suffix}")
            assert entry is not None, f"missing bridge entry /grasp/{name}/{suffix}"
            assert entry["direction"] == direction
            assert entry["gz_type_name"] == gz_type


# ---- CLI helpers -----------------------------------------------------------
def test_parse_model_list_extracts_model_names():
    output = (
        "Requesting state for world [garden_world]...\n\n"
        "Available models:\n    - ground_plane\n    - cup\n    - ranger\n"
    )

    assert gm.parse_model_list(output) == frozenset({"ground_plane", "cup", "ranger"})


def test_parse_model_list_returns_empty_for_no_world():
    assert gm.parse_model_list("No world found.\n") == frozenset()


def test_select_gz_cli_prefers_ign(monkeypatch):
    monkeypatch.setattr(gm.shutil, "which", lambda tool: f"/usr/bin/{tool}")

    assert gm.select_gz_cli() == "ign"


def test_select_gz_cli_falls_back_to_gz(monkeypatch):
    monkeypatch.setattr(
        gm.shutil, "which", lambda tool: "/usr/bin/gz" if tool == "gz" else None
    )

    assert gm.select_gz_cli() == "gz"


def test_select_gz_cli_returns_none_without_tools(monkeypatch):
    monkeypatch.setattr(gm.shutil, "which", lambda tool: None)

    assert gm.select_gz_cli() is None


# ---- request handling (ROS node, no simulator) ---------------------------
@pytest.fixture
def manager():
    rclpy.init()
    node = gm.GraspManager()
    node.set_parameters([Parameter("state_timeout_sec", Parameter.Type.DOUBLE, 0.2)])
    node._present = frozenset({"cup"})
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _record_requests(node, monkeypatch, result=(True, "ok")):
    calls = []

    def fake_request_state(name, target):
        calls.append((name, target))
        return result

    monkeypatch.setattr(node, "_request_state", fake_request_state)
    return calls


def test_unknown_object_is_rejected_with_known_list(manager):
    ok, message = manager._handle_request("teapot", True)

    assert not ok
    assert "unknown object 'teapot'" in message and "'cup'" in message


def test_object_absent_from_world_is_rejected(manager):
    ok, message = manager._handle_request("ycb_mug", True)

    assert not ok
    assert "not in world" in message


def test_detach_of_detached_object_succeeds_without_publishing(manager, monkeypatch):
    calls = _record_requests(manager, monkeypatch)
    manager._states["cup"] = gm.STATE_DETACHED

    ok, message = manager._handle_request("cup", False)

    assert ok and message == "already detached"
    assert calls == []


def test_reattach_sends_detach_first(manager, monkeypatch):
    calls = _record_requests(manager, monkeypatch)
    manager._states["cup"] = gm.STATE_ATTACHED

    ok, _ = manager._handle_request("cup", True)

    assert ok
    assert calls == [("cup", gm.STATE_DETACHED), ("cup", gm.STATE_ATTACHED)]


def test_attach_of_detached_object_only_attaches(manager, monkeypatch):
    calls = _record_requests(manager, monkeypatch)
    manager._states["cup"] = gm.STATE_DETACHED

    manager._handle_request("cup", True)

    assert calls == [("cup", gm.STATE_ATTACHED)]


def test_reattach_reports_failed_detach(manager, monkeypatch):
    _record_requests(manager, monkeypatch, result=(False, "no confirmation"))
    manager._states["cup"] = gm.STATE_ATTACHED

    ok, message = manager._handle_request("cup", True)

    assert not ok
    assert message.startswith("attach: detach first failed:")


def test_attach_with_unknown_state_detaches_first(manager, monkeypatch):
    # Unknown state could be "attached": detach first so the attach is
    # guaranteed to produce a transition instead of being ignored.
    calls = _record_requests(manager, monkeypatch)

    manager._handle_request("cup", True)

    assert calls == [("cup", gm.STATE_DETACHED), ("cup", gm.STATE_ATTACHED)]


def test_request_state_succeeds_when_plugin_reports_target(manager, monkeypatch):
    def plugin(_msg):
        manager._on_state("cup", gm.String(data=gm.STATE_ATTACHED))

    monkeypatch.setattr(manager._attach_pubs["cup"], "publish", plugin)

    assert manager._request_state("cup", gm.STATE_ATTACHED) == (True, "attached")


def _advancing_sim_clock(manager, monkeypatch, step_ns=250_000_000):
    """Fake sim time that advances by step_ns on every read."""
    ticks = iter(range(0, 10**12, step_ns))
    monkeypatch.setattr(manager, "_sim_time_ns", lambda: next(ticks))


def _bridge_connected(manager, monkeypatch, name="cup"):
    for pub in (manager._attach_pubs[name], manager._detach_pubs[name]):
        monkeypatch.setattr(pub, "get_subscription_count", lambda: 1)


def test_silent_detach_while_sim_runs_means_already_detached(manager, monkeypatch):
    # The plugin ignores a redundant detach without publishing anything,
    # e.g. after this node restarted against a running simulation.
    _advancing_sim_clock(manager, monkeypatch)
    _bridge_connected(manager, monkeypatch)

    ok, message = manager._request_state("cup", gm.STATE_DETACHED)

    assert ok and "already detached" in message
    assert manager._state("cup") == gm.STATE_DETACHED


def test_silent_detach_with_frozen_sim_still_fails(manager, monkeypatch):
    monkeypatch.setattr(manager, "_sim_time_ns", lambda: 5_000_000_000)
    _bridge_connected(manager, monkeypatch)

    ok, _ = manager._request_state("cup", gm.STATE_DETACHED)

    assert not ok


def test_silent_detach_without_bridge_still_fails(manager, monkeypatch):
    _advancing_sim_clock(manager, monkeypatch)

    ok, _ = manager._request_state("cup", gm.STATE_DETACHED)

    assert not ok


def test_silent_attach_is_never_inferred(manager, monkeypatch):
    _advancing_sim_clock(manager, monkeypatch)
    _bridge_connected(manager, monkeypatch)

    ok, message = manager._request_state("cup", gm.STATE_ATTACHED)

    assert not ok
    assert "no 'attached' confirmation" in message


def test_request_without_plugin_times_out_quickly(manager):
    # No simulator: the state never changes, so the request must give up
    # after state_timeout_sec instead of hanging.
    start = time.monotonic()

    ok, message = manager._request_state("cup", gm.STATE_ATTACHED)

    assert not ok
    assert "no 'attached' confirmation" in message
    assert time.monotonic() - start < 1.0


def test_state_message_updates_cached_state(manager):
    manager._on_state("cup", gm.String(data=gm.STATE_ATTACHED))

    assert manager._state("cup") == gm.STATE_ATTACHED


def test_sim_time_warning_only_when_objects_detached_late(manager, monkeypatch):
    records = []

    class _Logger:
        def warn(self, message):
            records.append(("warn", message))

        def info(self, message):
            records.append(("info", message))

    monkeypatch.setattr(manager, "get_logger", _Logger)
    sim_times = iter([None, 0, 2_000_000_000, 2_000_000_000])
    monkeypatch.setattr(manager, "_sim_time_ns", lambda: next(sim_times))

    manager._warn_if_sim_was_running(frozenset({"cup"}))  # not on sim time
    manager._warn_if_sim_was_running(frozenset({"cup"}))  # paused at t = 0
    manager._warn_if_sim_was_running(frozenset({"cup"}))  # cup was dragged
    manager._warn_if_sim_was_running(frozenset())  # restart: nothing to detach

    assert [level for level, _ in records] == ["warn", "info"]
    assert "paused:=false" in records[0][1]
    assert "restarted" in records[1][1]


def test_sim_time_is_none_without_use_sim_time(manager):
    assert manager._sim_time_ns() is None


# ---- startup sequence (mocked simulator) ---------------------------------
class _RecordingLogger:
    def __init__(self):
        self.records = []

    def info(self, message):
        self.records.append(("info", message))

    def warn(self, message):
        self.records.append(("warn", message))

    def error(self, message):
        self.records.append(("error", message))

    def levels(self, level):
        return [message for lvl, message in self.records if lvl == level]


@pytest.fixture
def log(manager, monkeypatch):
    recorder = _RecordingLogger()
    monkeypatch.setattr(manager, "get_logger", lambda: recorder)
    return recorder


def _completed(stdout, returncode=0):
    return gm.subprocess.CompletedProcess(
        args=[], returncode=returncode, stdout=stdout, stderr=""
    )


def test_on_set_grasp_fills_response_and_logs(manager, log):
    request = gm.SetGrasp.Request(object="teapot", attach=True)

    response = manager._on_set_grasp(request, gm.SetGrasp.Response())

    assert not response.success
    assert "unknown object" in response.message
    assert log.levels("warn")


def test_on_set_grasp_logs_success_at_info(manager, log):
    manager._states["cup"] = gm.STATE_DETACHED
    request = gm.SetGrasp.Request(object="cup", attach=False)

    response = manager._on_set_grasp(request, gm.SetGrasp.Response())

    assert response.success
    assert log.levels("info") and not log.levels("warn")


def test_wait_for_bridge_logs_error_when_bridge_is_missing(manager, log):
    manager.set_parameters([Parameter("bridge_wait_sec", Parameter.Type.DOUBLE, 0.2)])

    manager._wait_for_bridge()

    assert any("bridge entries not up" in m for m in log.levels("error"))


def test_list_models_parses_cli_output(manager, monkeypatch):
    manager._cli = "ign"
    monkeypatch.setattr(
        gm.subprocess, "run", lambda *a, **k: _completed("    - ranger\n")
    )

    assert manager._list_models() == frozenset({"ranger"})


def test_list_models_returns_none_when_cli_fails(manager, monkeypatch, log):
    manager._cli = "ign"

    def boom(*args, **kwargs):
        raise OSError("no such file")

    monkeypatch.setattr(gm.subprocess, "run", boom)

    assert manager._list_models() is None
    assert log.levels("warn")


def test_discover_present_objects_waits_for_robot(manager, monkeypatch, log):
    manager._cli = "ign"
    answers = iter([frozenset({"cup"}), frozenset({"cup", "ranger", "sofa"})])
    monkeypatch.setattr(manager, "_list_models", lambda: next(answers))
    monkeypatch.setattr(gm, "MODEL_LIST_PERIOD_SEC", 0.0)

    present = manager._discover_present_objects()

    assert present == frozenset({"cup"})
    assert manager._present == frozenset({"cup"})


def test_discover_present_objects_gives_up_after_spawn_wait(manager, monkeypatch, log):
    manager._cli = "ign"
    manager.set_parameters([Parameter("spawn_wait_sec", Parameter.Type.DOUBLE, 0.1)])
    monkeypatch.setattr(manager, "_list_models", lambda: frozenset({"cup"}))
    monkeypatch.setattr(gm, "MODEL_LIST_PERIOD_SEC", 0.01)

    assert manager._discover_present_objects() is None
    assert any("not in world" in m for m in log.levels("error"))


def test_discover_present_objects_without_cli_is_unknown(manager, log):
    manager._cli = None

    assert manager._discover_present_objects() is None
    assert log.levels("warn")


def test_detach_all_confirms_objects_that_report_detached(manager, monkeypatch, log):
    # Simulate the plugin: a detach request makes it report "detached".
    def plugin_detach(_msg):
        manager._on_state("cup", gm.String(data=gm.STATE_DETACHED))

    monkeypatch.setattr(manager._detach_pubs["cup"], "publish", plugin_detach)

    manager._detach_all(frozenset({"cup"}))

    assert any("detached at startup: ['cup']" in m for m in log.levels("info"))
    assert not log.levels("error")


def test_detach_all_reports_present_object_without_confirmation(manager, log):
    manager.set_parameters(
        [Parameter("detach_timeout_sec", Parameter.Type.DOUBLE, 0.2)]
    )

    manager._detach_all(frozenset({"cup"}))

    assert any("may stay welded" in m for m in log.levels("error"))


def test_detach_all_with_unknown_presence_only_logs_info(manager, log):
    manager.set_parameters(
        [Parameter("detach_timeout_sec", Parameter.Type.DOUBLE, 0.2)]
    )

    manager._detach_all(None)

    assert not log.levels("error")
    assert any("not in this world?" in m for m in log.levels("info"))


def test_unpause_falls_back_to_cli_when_service_missing(manager, monkeypatch, log):
    monkeypatch.setattr(manager._control, "wait_for_service", lambda timeout_sec: False)
    calls = []
    monkeypatch.setattr(manager, "_unpause_cli", lambda: calls.append("cli"))

    manager._unpause()

    assert calls == ["cli"]
    assert log.levels("warn")


def test_unpause_cli_accepts_true_reply(manager, monkeypatch, log):
    manager._cli = "ign"
    commands = []

    def fake_run(command, **kwargs):
        commands.append(command)
        return _completed("data: true\n")

    monkeypatch.setattr(gm.subprocess, "run", fake_run)

    manager._unpause_cli()

    assert commands[0][:3] == ["ign", "service", "-s"]
    assert "pause: false" in commands[0]
    assert not log.levels("error")


def test_unpause_cli_reports_unconfirmed_reply(manager, monkeypatch, log):
    manager._cli = "ign"
    monkeypatch.setattr(
        gm.subprocess, "run", lambda *a, **k: _completed("", returncode=1)
    )

    manager._unpause_cli()

    assert any("not confirmed" in m for m in log.levels("error"))


def test_unpause_cli_without_tool_logs_error(manager, log):
    manager._cli = None

    manager._unpause_cli()

    assert any("cannot unpause" in m for m in log.levels("error"))


def test_run_startup_unpauses_and_serves_even_if_a_step_fails(
    manager, monkeypatch, log
):
    def broken():
        raise RuntimeError("bridge exploded")

    unpaused = []
    monkeypatch.setattr(manager, "_wait_for_bridge", broken)
    monkeypatch.setattr(manager, "_unpause", lambda: unpaused.append(True))

    manager.run_startup()

    assert unpaused == [True]
    assert manager._service is not None
    assert any("bridge exploded" in m for m in log.levels("error"))
    assert any("grasp manager ready" in m for m in log.levels("info"))


def test_run_startup_detaches_before_unpausing(manager, monkeypatch, log):
    order = []
    monkeypatch.setattr(manager, "_wait_for_bridge", lambda: order.append("bridge"))
    monkeypatch.setattr(
        manager, "_discover_present_objects", lambda: frozenset({"cup"})
    )
    monkeypatch.setattr(
        manager, "_detach_all", lambda present: order.append("detach") or frozenset()
    )
    monkeypatch.setattr(manager, "_unpause", lambda: order.append("unpause"))

    manager.run_startup()

    assert order == ["bridge", "detach", "unpause"]
    assert manager._service is not None


def test_run_startup_serves_even_if_unpause_raises(manager, monkeypatch, log):
    def broken_unpause():
        raise RuntimeError("no world")

    monkeypatch.setattr(manager, "_wait_for_bridge", lambda: None)
    monkeypatch.setattr(manager, "_warn_if_sim_was_running", lambda detached: None)
    monkeypatch.setattr(manager, "_discover_present_objects", lambda: None)
    monkeypatch.setattr(manager, "_detach_all", lambda present: frozenset())
    monkeypatch.setattr(manager, "_unpause", broken_unpause)

    manager.run_startup()

    assert manager._service is not None
    assert any("unpause failed" in m for m in log.levels("error"))


def test_run_startup_after_stop_request_does_not_serve(manager, monkeypatch, log):
    unpaused = []
    monkeypatch.setattr(manager, "_unpause", lambda: unpaused.append(True))
    manager.set_parameters([Parameter("bridge_wait_sec", Parameter.Type.DOUBLE, 0.2)])
    manager._cli = None
    manager.request_stop()

    manager.run_startup()

    assert unpaused == []
    assert manager._service is None


def test_unpause_uses_control_world_service_when_available(manager, monkeypatch, log):
    done = gm.Future()
    done.set_result(gm.ControlWorld.Response(success=True))
    requests = []

    def fake_call(request):
        requests.append(request)
        return done

    monkeypatch.setattr(manager._control, "wait_for_service", lambda timeout_sec: True)
    monkeypatch.setattr(manager._control, "call_async", fake_call)
    monkeypatch.setattr(
        manager, "_unpause_cli", lambda: pytest.fail("CLI fallback used")
    )

    manager._unpause()

    assert requests[0].world_control.pause is False


def test_detach_all_infers_detached_when_sim_already_running(manager, monkeypatch, log):
    _advancing_sim_clock(manager, monkeypatch)
    _bridge_connected(manager, monkeypatch)

    manager._detach_all(frozenset({"cup"}))

    assert manager._state("cup") == gm.STATE_DETACHED
    assert any("already detached" in m for m in log.levels("info"))
    assert not log.levels("error")


def test_parse_world_name_reads_cli_header():
    output = "\nRequesting state for world [apartment]...\n\nAvailable models:\n"

    assert gm.parse_world_name(output) == "apartment"
    assert gm.parse_world_name("No world found.") is None


def test_world_name_mismatch_redirects_cli_unpause(manager, monkeypatch, log):
    manager._cli = "ign"
    commands = []

    def fake_run(command, **kwargs):
        commands.append(command)
        return _completed("data: true\n")

    monkeypatch.setattr(gm.subprocess, "run", fake_run)

    manager._check_world_name("apartment")
    manager._unpause_cli()

    assert "/world/apartment/control" in commands[0]
    assert any("pass world_name:=apartment" in m for m in log.levels("error"))
