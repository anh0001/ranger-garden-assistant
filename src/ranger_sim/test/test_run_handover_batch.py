"""Tests for tools/run_handover_batch.py (offline: nothing is launched)."""

import csv
import importlib.util
import math
from pathlib import Path
import subprocess
import sys

import pytest

PKG = Path(__file__).resolve().parents[1]
TOOL = PKG / "tools" / "run_handover_batch.py"
_spec = importlib.util.spec_from_file_location("run_handover_batch", TOOL)
batch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(batch)

LISTING = """\
\x1b[92mpiper_arm_controller[joint_trajectory_controller/JointTrajectoryController] active\x1b[0m
piper_gripper_controller[position_controllers/GripperActionController] active
piper_arm_stream_controller[forward_command_controller/ForwardCommandController] inactive
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
"""


def test_the_baseline_set_has_34_distinct_seeded_episodes():
    episodes = batch.baseline_episodes(0)

    assert len(episodes) == 34
    assert [e["episode_id"] for e in episodes] == list(range(34))
    assert len({e["seed"] for e in episodes}) == 34
    combos = {(round(math.degrees(e["facing_yaw"])), e["hand_side"], e["hand_extends"],
               tuple(e["reaction_delay"])) for e in episodes[:32]}
    assert len(combos) == 32
    assert [e["facing_change"] for e in episodes[32:]] == [0.8, 0.8]
    assert all(e["faults"] == [0.0] * 12 for e in episodes)


def test_the_batch_seed_changes_every_episode_seed():
    assert {e["seed"] for e in batch.baseline_episodes(0)}.isdisjoint(
        {e["seed"] for e in batch.baseline_episodes(1)})


def test_the_fault_set_triggers_at_1_2_m_and_keeps_the_ground_truth():
    episodes = batch.fault_episodes(0)
    labels = [e["label"] for e in episodes]

    assert len(episodes) == 27
    assert all(e["faults"][11] == 1.2 for e in episodes)
    assert all(math.degrees(e["facing_yaw"]) == 0.0 and e["hand_side"] == "left"
               and e["hand_extends"] for e in episodes)
    assert sum(label.startswith("dropout") for label in labels) == 3
    assert sum(label.startswith("stale") for label in labels) == 2
    assert sum(label.startswith("jump") for label in labels) == 4
    assert sum(label.startswith("bias") for label in labels) == 18


def test_a_bias_points_away_from_the_robot_and_a_jump_both_ways():
    episodes = {e["label"]: e["faults"] for e in batch.fault_episodes(0)}
    toward = (math.cos(math.radians(-60.0)), math.sin(math.radians(-60.0)))

    bias = episodes["bias 0.2 m away #0"]
    assert (bias[0], bias[1]) == pytest.approx((-0.2 * toward[0], -0.2 * toward[1]))
    jump = episodes["jump 1 m toward"]
    assert (jump[9], jump[10]) == pytest.approx(toward)
    assert jump[8] == batch.FAULT_START_SEC


def test_launch_arguments_are_literal_ros_values():
    episode = batch.baseline_episodes(0)[3]  # facing 0, left, no hand, delay [2, 4]

    args = batch.launch_arguments(episode, Path("/tmp/r/episodes.csv"), use_nav=False)

    assert args[:2] == ["execute:=true", "use_nav:=false"]
    assert f"seed:={episode['seed']}" in args
    assert "hand_extends:=false" in args
    assert "reaction_delay:=[2, 4]" in args
    assert "faults:=[" + ", ".join(["0"] * 12) + "]" in args
    assert args[-1] == "results_file:=/tmp/r/episodes.csv"


def test_the_bringup_has_the_gate_and_nav_as_asked():
    assert batch.bringup_command(True)[-3:] == [
        "headless:=true", "handover_gate:=true", "launch_nav:=true"]


# As printed by `ros2 control list_controllers` in the sim (Humble, colours on).
COLUMNS_LISTING = (
    "\x1b[92mjoint_state_broadcaster      \x1b[0m joint_state_broadcaster/JointStateBroadcaster"
    "          \x1b[92mactive  \x1b[0m\n"
    "\x1b[92mpiper_arm_controller         \x1b[0m joint_trajectory_controller/"
    "JointTrajectoryController  \x1b[92mactive  \x1b[0m\n"
    "\x1b[96mpiper_arm_stream_controller  \x1b[0m forward_command_controller/"
    "ForwardCommandController    \x1b[96minactive\x1b[0m\n"
    "\x1b[92mpiper_gripper_controller     \x1b[0m position_controllers/GripperActionController"
    "           \x1b[92mactive  \x1b[0m\n"
)


def test_controllers_ready_reads_the_humble_columns():
    assert batch.controllers_ready(COLUMNS_LISTING)
    assert not batch.controllers_ready(COLUMNS_LISTING.replace("inactive", "active  "))


def test_strip_ansi_and_controllers_ready():
    assert "\x1b" not in batch.strip_ansi(LISTING)
    assert batch.controllers_ready(LISTING)
    assert not batch.controllers_ready(LISTING.replace("] inactive", "] active"))
    assert not batch.controllers_ready(LISTING.replace(
        "GripperActionController] active", "GripperActionController] inactive"))
    assert not batch.controllers_ready("")


def test_each_episode_has_its_own_domain_and_partition():
    env = batch.episode_environment(7, base={"PATH": "/bin"})

    assert env["ROS_DOMAIN_ID"] == "57" and env["ROS_LOCALHOST_ONLY"] == "1"
    assert env["IGN_PARTITION"] == env["GZ_PARTITION"] == "handover_batch_7"
    assert env["PATH"] == "/bin"
    with pytest.raises(ValueError):
        batch.episode_environment(60, base={})


def test_a_bringup_failure_is_recorded_as_a_row(tmp_path):
    episode = batch.fault_episodes(0)[0]
    path = tmp_path / "episodes.csv"

    batch.failed_row(episode, path, "bringup_failed")

    (row,) = list(csv.DictReader(path.open()))
    assert row["outcome"] == "bringup_failed" and row["success"] == "0"
    assert row["faults"].split()[3:5] == ["0.5", "0.5"]


def test_the_dry_run_prints_34_command_pairs():
    result = subprocess.run([sys.executable, str(TOOL), "--set", "baseline", "--dry-run"],
                            capture_output=True, text=True, check=True)

    lines = result.stdout.splitlines()
    assert sum(line.startswith("ros2 launch ranger_sim sim_bringup") for line in lines) == 34
    assert sum(line.startswith("ros2 launch ranger_sim handover_baseline")
               for line in lines) == 34


def test_the_dry_run_can_select_episodes(capsys):
    assert batch.main(["--set", "faults", "--dry-run", "--only", "2", "5"]) == 0

    out = capsys.readouterr().out
    assert out.count("handover_baseline.launch.py") == 2
    assert "# 2: dropout to the end" in out


def test_the_runner_refuses_to_start_beside_a_running_gazebo(monkeypatch, tmp_path):
    monkeypatch.setattr(batch, "gazebo_running", lambda: True)

    with pytest.raises(RuntimeError, match="ign gazebo"):
        batch.run_episode(0, batch.baseline_episodes(0)[0], tmp_path, use_nav=False)


def test_stop_group_escalates_by_group_id(monkeypatch):
    sent = []

    class Process:
        pid = 4242

        def poll(self):
            return None

        def wait(self, timeout):
            if len(sent) < 3:
                raise subprocess.TimeoutExpired("x", timeout)

    monkeypatch.setattr(batch.os, "getpgid", lambda pid: 99)
    monkeypatch.setattr(batch.os, "killpg", lambda group, sig: sent.append((group, sig)))
    monkeypatch.setattr(batch, "session_pids", lambda session: [])

    batch.stop_group(Process())

    assert sent == [(99, batch.signal.SIGINT), (99, batch.signal.SIGTERM),
                    (99, batch.signal.SIGKILL)]


def test_stop_group_also_stops_what_is_left_in_the_session(monkeypatch):
    killed, left = [], [[7001, 7002], [7002], []]

    class Process:
        pid = 4242

        def poll(self):
            return 0  # the launch itself is gone

    monkeypatch.setattr(batch, "session_pids", lambda session: left.pop(0))
    monkeypatch.setattr(batch.os, "kill", lambda pid, sig: killed.append((pid, sig)))
    monkeypatch.setattr(batch.time, "sleep", lambda sec: None)

    batch.stop_group(Process())

    assert killed == [(7001, batch.signal.SIGINT), (7002, batch.signal.SIGINT),
                      (7002, batch.signal.SIGKILL)]


def test_session_pids_reads_ps(monkeypatch):
    listing = "  1 1\n 4242 4242\n 4300 4242\n 4301 4242\n 5000 17\n"
    monkeypatch.setattr(batch.subprocess, "run", lambda *a, **k: type(
        "R", (), {"stdout": listing})())

    assert batch.session_pids(4242) == [4300, 4301]


def test_an_episode_runs_in_its_own_session_and_is_stopped_on_a_timeout(monkeypatch, tmp_path):
    started, stopped = [], []

    class Bringup:
        pid = 1

    class Episode:
        pid = 2

        def wait(self, timeout):
            raise subprocess.TimeoutExpired("ros2 launch", timeout)

    def popen(cmd, **kwargs):
        started.append(kwargs.get("start_new_session"))
        return Bringup() if "sim_bringup.launch.py" in cmd else Episode()

    monkeypatch.setattr(batch, "gazebo_running", lambda: False)
    monkeypatch.setattr(batch.subprocess, "Popen", popen)
    monkeypatch.setattr(batch, "_wait_controllers", lambda env, process, timeout: True)
    monkeypatch.setattr(batch, "_wait_grasp_manager", lambda log, process, timeout: None)
    monkeypatch.setattr(batch, "stop_group", lambda process: stopped.append(process.pid))

    result = batch.run_episode(0, batch.baseline_episodes(0)[0], tmp_path, use_nav=False)

    assert result == "episode_failed"
    assert started == [True, True]
    assert stopped == [2, 1]  # the episode's nodes first, then the sim


def test_the_runner_exits_non_zero_when_an_episode_failed(monkeypatch, tmp_path):
    monkeypatch.setattr(batch, "run_episode", lambda index, e, out, use_nav: "episode_failed")

    assert batch.main(["--set", "faults", "--only", "0", "--out", str(tmp_path)]) == 1


@pytest.mark.parametrize("log, problem", [
    ("[grasp_manager]: grasp manager ready: 3 objects", None),
    ("[grasp_manager]: no 'detached' confirmation within 20.0 s for ['supplement_bottle']: "
     "they may stay welded to the gripper\n[grasp_manager]: grasp manager ready", "detach"),
    ("[move_group]: starting", ""),
])
def test_a_bringup_with_an_object_left_on_the_gripper_is_unusable(log, problem):
    result = batch.bringup_problem(log)

    if problem is None or problem == "":
        assert result == problem
    else:
        assert problem in result


def test_a_welded_bringup_is_retried_then_recorded(monkeypatch, tmp_path):
    starts = []

    class Bringup:
        pid = 1

    def popen(cmd, **kwargs):
        starts.append(cmd)
        return Bringup()

    monkeypatch.setattr(batch, "gazebo_running", lambda: False)
    monkeypatch.setattr(batch.subprocess, "Popen", popen)
    monkeypatch.setattr(batch, "_wait_controllers", lambda env, process, timeout: True)
    monkeypatch.setattr(batch, "_wait_grasp_manager",
                        lambda log, process, timeout: "could not detach")
    monkeypatch.setattr(batch, "stop_group", lambda process: None)

    result = batch.run_episode(0, batch.baseline_episodes(0)[0], tmp_path, use_nav=False)

    assert result == "bringup_failed"
    assert len(starts) == 2  # retried once, the episode itself never started
    assert (tmp_path / "sim_0.attempt1.log").exists()
