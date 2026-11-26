"""
Unit tests for scripts/handover_recorder.py (no simulator).

EpisodeMetrics is pure: a synthetic episode (a straight approach along a
known line at a known speed) gives the expected bearing times, closing
speed, clearance and duration. The node is checked for its window, its
single row and the CSV file.
"""

import csv
from datetime import datetime
import importlib.util
import math
from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

PKG = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("handover_recorder",
                                               PKG / "scripts" / "handover_recorder.py")
recorder = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(recorder)
core = recorder.core
CFG = core.default_config()

PERSON_XY = (0.504, -0.858)
DT = 0.05
LEFT_YAW = -math.pi / 2  # a base to the person's left, facing it


def _person(take=False, hand=(0.75, -0.68, 0.63), facing=0.0):
    return core.PersonState(*PERSON_XY, facing, hand, False, take, 0.0)


def _metrics():
    return recorder.EpisodeMetrics(CFG)


def _approach(metrics, start_y, end_y, speed, t, phase="TRANSIT"):
    """Drive the base along x = person x from start_y to end_y at `speed`; return the time."""
    y = start_y
    metrics.on_phase(phase, t)
    while y > end_y + 1e-9:
        t = round(t + DT, 6)
        y = max(end_y, y - speed * DT)
        metrics.on_base((PERSON_XY[0], y, LEFT_YAW), speed)
        metrics.step(t)
    return t


def test_nothing_counts_before_transit():
    metrics = _metrics()
    metrics.on_person(_person())
    metrics.on_base((PERSON_XY[0], -0.2, LEFT_YAW), 0.1)  # inside the stop distance, moving

    for i in range(10):
        metrics.step(i * DT)

    row = metrics.row(recorder.EpisodeInfo())
    assert row["viol_stop_distance_sec"] == 0.0 and row["outcome"] == "aborted"
    assert row["duration_sec"] == ""


def test_a_straight_approach_from_the_left_gives_the_expected_metrics():
    metrics = _metrics()
    metrics.on_person(_person())
    metrics.on_base((PERSON_XY[0], 1.0, LEFT_YAW), 0.0)
    t = _approach(metrics, 1.0, 0.336, 0.2, 10.0)
    metrics.on_phase("DONE: handover", t)

    row = metrics.row(recorder.EpisodeInfo())
    # Bearing 90 deg: outside 35 and 70 deg once within 1.5 m of the torso axis (y <= 0.642).
    inside = (0.642 - 0.336) / 0.2
    assert row["time_outside_35_sec"] == pytest.approx(inside, abs=2 * DT)
    assert row["time_outside_70_sec"] == pytest.approx(inside, abs=2 * DT)
    assert row["max_closing_speed_mps"] == pytest.approx(0.2, abs=1e-6)
    # Front edge at 0.336 - 0.394 = -0.058: 0.8 m from the torso axis, minus the radius.
    assert row["min_clearance_m"] == pytest.approx(0.8 - CFG.person_torso_radius, abs=1e-6)
    assert row["duration_sec"] == pytest.approx(t - 10.0, abs=1e-6)
    assert row["success"] == 1 and row["outcome"] == "handover"


def test_an_approach_in_front_is_inside_the_view():
    metrics = _metrics()
    metrics.on_person(_person(facing=math.pi / 2))  # facing +y: the base comes from the front
    t = _approach(metrics, 1.0, 0.436, 0.2, 0.0)
    metrics.on_phase("DONE: handover", t)

    row = metrics.row(recorder.EpisodeInfo())
    assert row["time_outside_35_sec"] == 0.0 and row["time_outside_70_sec"] == 0.0


def test_closing_speed_counts_only_within_the_closing_radius():
    metrics = _metrics()
    metrics.on_person(_person())
    t = _approach(metrics, 3.0, 0.6, 0.5, 0.0)  # fast far away ...
    metrics.on_phase("DONE: handover", t)

    # ... the front edge (y - 0.394) comes within 1 m of the torso axis from y 0.536: never.
    assert metrics.row(recorder.EpisodeInfo())["max_closing_speed_mps"] == 0.0


def test_the_stop_distance_counts_while_moving_and_not_while_still():
    metrics = _metrics()
    metrics.on_person(_person())
    metrics.on_phase("TRANSIT", 0.0)
    close = (PERSON_XY[0], -0.858 + 0.22 + 0.10 + 0.394, LEFT_YAW)  # clearance 0.10

    for i in range(1, 11):
        metrics.on_base(close, 0.05)
        metrics.step(i * DT)
    for i in range(11, 21):
        metrics.on_base(close, 0.0)
        metrics.step(i * DT)

    row = metrics.row(recorder.EpisodeInfo())
    assert row["viol_stop_distance_sec"] == pytest.approx(0.5)
    assert row["viol_contact_sec"] == 0.0


def test_contact_and_height_cap_are_violations():
    metrics = _metrics()
    metrics.on_person(_person())
    metrics.on_phase("TRANSIT", 0.0)
    metrics.on_base((PERSON_XY[0] + 0.3, PERSON_XY[1], 0.0), 0.0)
    metrics.on_tcp((0.5, -0.4, 1.05))

    metrics.step(DT)
    metrics.on_phase("DONE: handover", 2 * DT)

    row = metrics.row(recorder.EpisodeInfo())
    assert row["viol_contact_sec"] == pytest.approx(2 * DT)
    assert row["viol_height_cap_sec"] == pytest.approx(2 * DT)
    assert row["success"] == 0


def _released(take=False, tcp=(1.0, -0.64, 0.871), place=None):
    metrics = _metrics()
    metrics.on_person(_person())
    metrics.on_grasp_state("attached")
    metrics.on_phase("TRANSIT", 0.0)
    metrics.on_tcp(tcp)
    if place is not None:
        metrics.on_place_target(place)
    metrics.on_person(_person(take=take))
    metrics.on_grasp_state("detached")
    metrics.on_phase("DONE: handover", 1.0)
    return metrics.row(recorder.EpisodeInfo())


def test_a_release_without_take_or_place_is_a_violation():
    assert _released()["viol_release"] == 1
    assert _released()["success"] == 0


def test_a_release_on_a_take_or_at_the_place_pose_is_legal():
    assert _released(take=True)["viol_release"] == 0
    assert _released(place=(1.0, -0.64, 0.871))["viol_release"] == 0
    assert _released(place=(1.0, -0.60, 0.871))["viol_release"] == 1  # 4 cm off


def test_the_take_gives_the_persons_reach():
    metrics = _metrics()
    metrics.on_person(_person(hand=(0.75, -0.68, 0.63)))
    metrics.on_phase("TRANSIT", 0.0)

    metrics.on_person(_person(take=True, hand=(0.75, -0.38, 0.83)))
    metrics.on_person(_person(take=True, hand=(0.9, -0.38, 0.83)))  # later samples do not count

    assert metrics.row(recorder.EpisodeInfo())["person_reach_m"] == pytest.approx(
        math.hypot(0.30, 0.20), abs=1e-4)


def test_rechoices_are_transit_entries_minus_one_and_waiting_is_wait_time():
    metrics = _metrics()
    metrics.on_person(_person())
    metrics.on_base((-2.0, 0.0, 0.0), 0.0)
    for phase, t in (("PICK", 0.0), ("TRANSIT", 1.0), ("TRANSIT", 2.0), ("TRANSIT", 3.0),
                     ("APPROACH", 4.0), ("WAIT", 5.0)):
        metrics.on_phase(phase, t)
    for i in range(1, 21):
        metrics.step(5.0 + i * DT)
    metrics.on_phase("DONE: timeout", 6.0)

    row = metrics.row(recorder.EpisodeInfo())
    assert row["rechoices"] == 2
    assert row["waiting_sec"] == pytest.approx(1.0)
    assert row["outcome"] == "timeout" and row["success"] == 0


def test_the_fault_latency_runs_from_the_edge_to_a_lasting_stop():
    metrics = _metrics()
    metrics.on_person(_person())
    metrics.on_phase("TRANSIT", 0.0)
    metrics.on_fault("", 0.0)
    metrics.on_fault("dropout", 1.0)
    for i in range(1, 60):
        t = 1.0 + i * DT
        metrics.on_base((-1.0, 0.0, 0.0), 0.1 if t < 1.35 else 0.0)
        metrics.step(t)

    assert metrics.row(recorder.EpisodeInfo())["fault_stop_latency_sec"] == pytest.approx(
        0.35, abs=DT + 1e-6)


def test_no_fault_leaves_the_latency_empty():
    metrics = _metrics()
    metrics.on_phase("TRANSIT", 0.0)
    metrics.on_base((-1.0, 0.0, 0.0), 0.0)
    for i in range(1, 30):
        metrics.step(i * DT)

    assert metrics.row(recorder.EpisodeInfo())["fault_stop_latency_sec"] == ""


def test_safety_stops_count_edges_inside_the_window_and_a_failure_under_stop_is_stopped():
    metrics = _metrics()
    metrics.on_safety_state("STOP: person data stale")  # before the window: not counted
    metrics.on_safety_state("OK")
    metrics.on_phase("TRANSIT", 0.0)
    for state in ("STOP: stop distance", "STOP: person data stale", "OK",
                  "STOP: person data jumped"):
        metrics.on_safety_state(state)
    metrics.on_phase("FAILED: drive to the pre-standoff", 1.0)

    row = metrics.row(recorder.EpisodeInfo())
    assert row["safety_stops"] == 2
    assert row["outcome"] == "stopped"


def test_a_failure_before_transit_still_gives_a_row_without_metrics():
    metrics = _metrics()
    metrics.on_phase("PICK", 0.0)
    metrics.on_phase("FAILED: drive away", 30.0)

    row = metrics.row(recorder.EpisodeInfo())
    assert row["outcome"] == "failed" and row["success"] == 0
    assert row["duration_sec"] == "" and row["min_clearance_m"] == ""


def test_a_failure_without_a_stop_is_failed():
    metrics = _metrics()
    metrics.on_phase("TRANSIT", 0.0)
    metrics.on_safety_state("OK")
    metrics.on_phase("FAILED: MPC: offer", 1.0)

    assert metrics.row(recorder.EpisodeInfo())["outcome"] == "failed"


def test_the_row_carries_the_episode_settings():
    info = recorder.EpisodeInfo(episode_id=7, seed=3, facing_deg=30.0, hand_side="right",
                                hand_extends=False, reaction_delay=(2.0, 4.0),
                                facing_change_deg=45.8, faults=(0.1,) + (0.0,) * 11)

    row = _metrics().row(info)

    assert list(row) == list(recorder.COLUMNS)
    assert (row["episode_id"], row["seed"], row["hand_side"], row["hand_extends"]) == (
        7, 3, "right", 0)
    assert row["faults"] == "0.1 0 0 0 0 0 0 0 0 0 0 0"


# ---- CSV ---------------------------------------------------------------------------------


def test_the_header_is_written_once_and_rows_append(tmp_path):
    path = tmp_path / "deep" / "episodes.csv"
    row = _metrics().row(recorder.EpisodeInfo())

    recorder.write_row(path, row)
    recorder.write_row(path, {**row, "episode_id": 1})

    rows = list(csv.DictReader(path.open()))
    assert [r["episode_id"] for r in rows] == ["0", "1"]
    assert path.read_text().count("episode_id") == 1


def test_the_default_path_is_timestamped():
    path = recorder.default_results_path(datetime(2026, 11, 25, 21, 5, 9), Path("/tmp/x"))

    assert path == Path("/tmp/x/handover_20261125_210509.csv")


# ---- node -----------------------------------------------------------------------------------


@pytest.fixture
def make_node():
    rclpy.init()
    nodes = []

    def make(**params):
        overrides = [Parameter(name, value=value) for name, value in params.items()]
        nodes.append(recorder.HandoverRecorder(parameter_overrides=overrides))
        return nodes[-1]

    try:
        yield make
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def test_the_node_writes_one_row_when_the_episode_ends(make_node, tmp_path):
    path = tmp_path / "episodes.csv"
    node = make_node(results_file=str(path), episode_id=4, facing_yaw=math.radians(30.0))
    clock = [5.0]
    node._now = lambda: clock[0]

    node._on_phase(String(data="TRANSIT"))
    clock[0] = 9.0
    node._on_phase(String(data="DONE: set_down"))
    node.write()

    rows = list(csv.DictReader(path.open()))
    assert len(rows) == 1
    assert rows[0]["episode_id"] == "4" and rows[0]["outcome"] == "set_down"
    assert float(rows[0]["facing_deg"]) == pytest.approx(30.0)
    assert float(rows[0]["duration_sec"]) == pytest.approx(4.0)


def test_shutdown_with_an_open_episode_writes_it_as_aborted(make_node, tmp_path):
    path = tmp_path / "episodes.csv"
    node = make_node(results_file=str(path))
    node._now = lambda: 1.0
    node._on_phase(String(data="TRANSIT"))

    node.write()

    assert list(csv.DictReader(path.open()))[0]["outcome"] == "aborted"


def test_shutdown_before_the_episode_writes_nothing(make_node, tmp_path):
    path = tmp_path / "episodes.csv"
    node = make_node(results_file=str(path))

    node.write()

    assert not path.exists()


def test_the_node_never_reads_the_measured_person(make_node):
    node = make_node()

    topics = {name for name, _ in node.get_subscriber_names_and_types_by_node(
        node.get_name(), node.get_namespace())}

    assert not any(topic.startswith("/person/measured") for topic in topics)
    assert "/person/ground_truth/body_pose" in topics


@pytest.mark.parametrize("name", ["rate", "moving_speed", "place_tolerance"])
def test_non_positive_parameters_fail_at_start(make_node, name):
    with pytest.raises(ValueError, match=name):
        make_node(**{name: 0.0})


# ---- review fixes (2026-11-25) ----------------------------------------------------------


def _fault_episode(speeds, fault_at=1.0, fault_end=None):
    """Feed base speeds per DT from t = 1.0; a fault from fault_at (to fault_end)."""
    metrics = _metrics()
    metrics.on_person(_person())
    metrics.on_phase("TRANSIT", 0.0)
    metrics.on_fault("", 0.0)
    metrics.on_fault("dropout", fault_at)
    for i, speed in enumerate(speeds, start=1):
        t = round(1.0 + i * DT, 6)
        if fault_end is not None and abs(t - fault_end) < 1e-9:
            metrics.on_fault("", t)
        metrics.on_base((-1.0, 0.0, 0.0), speed)
        metrics.step(t)
    return metrics.row(recorder.EpisodeInfo())


def test_a_base_still_at_the_fault_edge_has_no_latency_to_hide_behind():
    row = _fault_episode([0.0] * 20 + [0.1] * 10 + [0.0] * 10)

    assert row["fault_stop_latency_sec"] == 0.0
    assert row["fault_motion_sec"] == pytest.approx(10 * DT)  # moved under the fault


def test_motion_after_the_fault_ends_is_not_counted():
    row = _fault_episode([0.0] * 20 + [0.1] * 10, fault_end=round(1.0 + 15 * DT, 6))

    assert row["fault_motion_sec"] == 0.0


def test_no_closing_speed_across_a_clock_gap():
    metrics = _metrics()
    metrics.on_person(_person())
    metrics.on_phase("TRANSIT", 0.0)
    metrics.on_base((PERSON_XY[0], 0.40, LEFT_YAW), 0.0)
    metrics.step(DT)
    metrics.on_base((PERSON_XY[0], 0.10, LEFT_YAW), 0.0)  # moved 0.3 m during a 3 s stall
    metrics.step(DT + 3.0)
    metrics.step(2 * DT + 3.0)

    assert metrics.row(recorder.EpisodeInfo())["max_closing_speed_mps"] == 0.0


def test_a_failed_write_is_retried(make_node, tmp_path, monkeypatch):
    path = tmp_path / "episodes.csv"
    node = make_node(results_file=str(path))
    node._now = lambda: 1.0
    node._on_phase(String(data="TRANSIT"))
    real = recorder.write_row

    def full_disk(target, row):
        raise OSError("No space left on device")

    monkeypatch.setattr(recorder, "write_row", full_disk)
    with pytest.raises(OSError):
        node.write()
    monkeypatch.setattr(recorder, "write_row", real)
    node.write()

    assert len(list(csv.DictReader(path.open()))) == 1
