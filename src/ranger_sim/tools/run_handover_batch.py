#!/usr/bin/env python3
"""
Run seeded sets of handover baseline episodes, one fresh sim per episode.

See docs/sim_handover_baseline.md.

  python3 src/ranger_sim/tools/run_handover_batch.py --set baseline --dry-run
  python3 src/ranger_sim/tools/run_handover_batch.py --set baseline \
      --out ~/.ros/handover_results/baseline --seed 0
  python3 src/ranger_sim/tools/run_handover_batch.py --set faults --use-nav false \
      --out ~/.ros/handover_results/faults --seed 0

Sets:
  baseline  facing {0, 30, 60, 90} deg x hand {left, right} x extends
            {true, false} x reaction delay {[0.5, 1.0], [2.0, 4.0]} = 32,
            plus two episodes with a facing change of 0.8 rad: 34
  faults    facing 0, left hand, extends, delay [0.5, 1.0], faults counted
            from 1.2 m: dropout 0.5 s, 2 s, to the end; stale 2 s, to the
            end; jump 0.3 and 1.0 m toward and away from the robot; bias
            0.05 to 0.40 m away from the robot, three seeds each: 27

Per episode, in its own ROS domain (50 + index), with ROS_LOCALHOST_ONLY=1
and its own gz partition: refuse to start while an `ign gazebo` server runs;
start sim_bringup (headless, handover_gate, launch_nav = --use-nav) in its
own session, log to <out>/sim_<i>.log; wait for the controllers and for
grasp_manager to have detached every object (a bringup where it could not is
unusable: an object rides on the gripper); run
handover_baseline.launch.py execute:=true with results_file
<out>/episodes.csv; then SIGINT the bringup's group, SIGTERM after 10 s,
SIGKILL after 10 more, and the same to whatever is left in its session. A
bringup that never gets its controllers is retried once, then recorded as a
row with outcome bringup_failed. Every episode
writes a row; failed episodes are reported, not re-run.

The runner only starts `ros2 launch` and `ros2 control list_controllers`:
it publishes nothing and calls no service. Simulation only: never run it
on a machine with a powered, bridged robot.
"""

import argparse
import importlib.util
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from typing import Optional

PKG = Path(__file__).resolve().parents[1]
FACINGS_DEG = (0, 30, 60, 90)
HAND_SIDES = ("left", "right")
EXTENDS = (True, False)
DELAYS = ((0.5, 1.0), (2.0, 4.0))
FACING_CHANGE_RAD = 0.8
FACING_CHANGE_EPISODES = ((0, "left"), (30, "right"))  # (facing deg, hand side)
FAULT_TRIGGER_M = 1.2
FAULT_START_SEC = 0.5  # after the trigger
TO_THE_END_SEC = 1.0e6
DROPOUTS_SEC = (0.5, 2.0, TO_THE_END_SEC)
STALES_SEC = (2.0, TO_THE_END_SEC)
JUMPS_M = (0.3, 1.0)
BIASES_M = (0.05, 0.10, 0.15, 0.20, 0.30, 0.40)
BIAS_SEEDS = 3
# The robot comes from bearing -60 deg of a person facing 0 (the rule's standoff).
ROBOT_DIRECTION_RAD = math.radians(-60.0)
FAULT_NAMES = ("bias_x", "bias_y", "noise_std", "dropout_start", "dropout_sec", "stale_start",
               "stale_sec", "latency_sec", "jump_at", "jump_x", "jump_y", "trigger_distance")
DOMAIN_BASE = 50
MAX_DOMAIN = 101
CONTROLLERS_ACTIVE = ("piper_arm_controller", "piper_gripper_controller")
CONTROLLERS_INACTIVE = ("piper_arm_stream_controller",)
BRINGUP_TIMEOUT_SEC = 240.0
CONTROLLER_POLL_SEC = 5.0
EPISODE_TIMEOUT_SEC = 1200.0
STOP_GRACE_SEC = 10.0
BRINGUP_ATTEMPTS = 2
GRASP_READY_TIMEOUT_SEC = 60.0
# grasp_manager's startup lines (scripts/grasp_manager.py).
GRASP_READY = "grasp manager ready"
GRASP_WELDED = "may stay welded"
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def episode_seed(batch_seed: int, index: int) -> int:
    """Return the episode's own seed, from the batch seed and its index."""
    return batch_seed * 1000 + index


def _episode(index: int, seed: int, facing_deg: float = 0.0, hand_side: str = "left",
             extends: bool = True, delay=(0.5, 1.0), facing_change: float = 0.0,
             faults=None, label: str = "") -> dict:
    return {
        "episode_id": index, "seed": episode_seed(seed, index),
        "facing_yaw": round(math.radians(facing_deg), 6), "hand_side": hand_side,
        "hand_extends": extends, "reaction_delay": list(delay),
        "facing_change": facing_change, "faults": list(faults or [0.0] * len(FAULT_NAMES)),
        "label": label,
    }


def baseline_episodes(seed: int) -> list[dict]:
    """Return the 34 episodes of the baseline table."""
    specs = []
    for facing in FACINGS_DEG:
        for side in HAND_SIDES:
            for extends in EXTENDS:
                for delay in DELAYS:
                    hand = "hand" if extends else "no-hand"
                    specs.append(dict(facing_deg=facing, hand_side=side, extends=extends,
                                      delay=delay,
                                      label=f"f{facing} {side} {hand} d{delay[0]:g}-{delay[1]:g}"))
    for facing, side in FACING_CHANGE_EPISODES:
        specs.append(dict(facing_deg=facing, hand_side=side, facing_change=FACING_CHANGE_RAD,
                          label=f"f{facing} {side} turns {FACING_CHANGE_RAD:g} rad"))
    return [_episode(index, seed, **spec) for index, spec in enumerate(specs)]


def _faults(**values) -> list[float]:
    faults = {name: 0.0 for name in FAULT_NAMES}
    faults["trigger_distance"] = FAULT_TRIGGER_M
    faults.update(values)
    return [float(faults[name]) for name in FAULT_NAMES]


def _duration_label(kind: str, duration: float) -> str:
    return f"{kind} {'to the end' if duration == TO_THE_END_SEC else f'{duration:g} s'}"


def fault_episodes(seed: int) -> list[dict]:
    """Return the faulty-person-data matrix (27 episodes); ground truth stays unchanged."""
    toward = (math.cos(ROBOT_DIRECTION_RAD), math.sin(ROBOT_DIRECTION_RAD))
    specs = []
    for duration in DROPOUTS_SEC:
        specs.append((_duration_label("dropout", duration),
                      _faults(dropout_start=FAULT_START_SEC, dropout_sec=duration)))
    for duration in STALES_SEC:
        specs.append((_duration_label("stale", duration),
                      _faults(stale_start=FAULT_START_SEC, stale_sec=duration)))
    for size in JUMPS_M:
        for sign, word in ((1.0, "toward"), (-1.0, "away")):
            specs.append((f"jump {size:g} m {word}", _faults(
                jump_at=FAULT_START_SEC, jump_x=round(sign * size * toward[0], 6),
                jump_y=round(sign * size * toward[1], 6))))
    for size in BIASES_M:
        for repeat in range(BIAS_SEEDS):
            # Away from the robot: the measured person seems farther than it is.
            specs.append((f"bias {size:g} m away #{repeat}", _faults(
                bias_x=round(-size * toward[0], 6), bias_y=round(-size * toward[1], 6))))
    return [_episode(index, seed, faults=faults, label=label)
            for index, (label, faults) in enumerate(specs)]


def _literal(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, list):
        return "[" + ", ".join(f"{float(v):g}" for v in value) + "]"
    return str(value)


def launch_arguments(episode: dict, results_file: Path, use_nav: bool) -> list[str]:
    """Return the handover_baseline.launch.py arguments of `episode`."""
    keys = ("episode_id", "seed", "facing_yaw", "hand_side", "hand_extends", "reaction_delay",
            "facing_change", "faults")
    return ["execute:=true", f"use_nav:={_literal(use_nav)}",
            *(f"{key}:={_literal(episode[key])}" for key in keys),
            f"results_file:={results_file}"]


def bringup_command(use_nav: bool) -> list[str]:
    """Return the sim_bringup command of one episode."""
    return ["ros2", "launch", "ranger_sim", "sim_bringup.launch.py", "headless:=true",
            "handover_gate:=true", f"launch_nav:={_literal(use_nav)}"]


def episode_command(episode: dict, results_file: Path, use_nav: bool) -> list[str]:
    """Return the handover_baseline.launch.py command of one episode."""
    return ["ros2", "launch", "ranger_sim", "handover_baseline.launch.py",
            *launch_arguments(episode, results_file, use_nav)]


def strip_ansi(text: str) -> str:
    """`text` without ANSI colour codes."""
    return ANSI.sub("", text)


def controllers_ready(text: str) -> bool:
    """Whether `ros2 control list_controllers` shows the arm ready for an episode."""
    states = {}
    for line in strip_ansi(text).splitlines():
        # Humble prints "name  type  state"; older releases "name[type] state".
        tokens = line.replace("[", " ").replace("]", " ").split()
        if len(tokens) >= 3:
            states[tokens[0]] = tokens[-1]
    return (all(states.get(name) == "active" for name in CONTROLLERS_ACTIVE)
            and all(states.get(name) == "inactive" for name in CONTROLLERS_INACTIVE))


def episode_environment(index: int, base: Optional[dict] = None) -> dict:
    """Own ROS domain, local-only DDS and own gz partition for episode `index`."""
    domain = DOMAIN_BASE + index
    if domain > MAX_DOMAIN:
        raise ValueError(f"episode {index}: ROS domain {domain} above {MAX_DOMAIN}")
    env = dict(os.environ if base is None else base)
    partition = f"handover_batch_{index}"
    env.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1", IGN_PARTITION=partition,
               GZ_PARTITION=partition)
    return env


# ---- running -----------------------------------------------------------------------------


def gazebo_running() -> bool:
    """Whether an `ign gazebo` process exists (its command line starts with it)."""
    return subprocess.run(["pgrep", "-f", "^ign gazebo"], stdout=subprocess.DEVNULL,
                          check=False).returncode == 0


def session_pids(session: int) -> list[int]:
    """Return the processes of session `session` (the bringup's own, start_new_session)."""
    listing = subprocess.run(["ps", "-eo", "pid=,sid="], capture_output=True, text=True,
                             check=False).stdout
    pids = []
    for line in listing.splitlines():
        fields = line.split()
        if len(fields) == 2 and int(fields[1]) == session and int(fields[0]) != session:
            pids.append(int(fields[0]))
    return pids


def stop_group(process: subprocess.Popen) -> None:
    """
    Stop the bringup: SIGINT its group, then SIGTERM, then SIGKILL; then its session.

    By group and session id, never by a name pattern. A process that left the
    group (Gazebo can run in its own) is still in the bringup's session.
    """
    session = process.pid  # start_new_session: the bringup leads its session
    if process.poll() is None:
        try:
            group = os.getpgid(process.pid)
        except ProcessLookupError:  # it just exited
            group = None
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL) if group else ():
            try:
                os.killpg(group, sig)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=STOP_GRACE_SEC)
                break
            except subprocess.TimeoutExpired:
                continue
    for sig in (signal.SIGINT, signal.SIGKILL):
        survivors = session_pids(session)
        for pid in survivors:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        if survivors:
            time.sleep(STOP_GRACE_SEC)


def bringup_problem(log_text: str) -> Optional[str]:
    """Return why a bringup is unusable from its log, '' while not yet known, None if fine."""
    if GRASP_WELDED in log_text:
        # A graspable object the startup could not detach rides on the gripper:
        # the scripted pick then strikes the bottle (Gazebo, 2026-11-26).
        return "grasp_manager could not detach every object at startup"
    return None if GRASP_READY in log_text else ""


def _wait_grasp_manager(log: Path, process: subprocess.Popen, timeout: float) -> Optional[str]:
    """Wait for grasp_manager's startup verdict in the bringup log; the problem, or None."""
    deadline = time.time() + timeout
    while time.time() < deadline and process.poll() is None:
        problem = bringup_problem(log.read_text(errors="replace"))
        if problem != "":
            return problem
        time.sleep(CONTROLLER_POLL_SEC)
    return "grasp_manager not ready"


def _wait_controllers(env: dict, process: subprocess.Popen, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline and process.poll() is None:
        try:
            listing = subprocess.run(["ros2", "control", "list_controllers"], env=env,
                                     capture_output=True, text=True, timeout=30, check=False)
            if controllers_ready(listing.stdout):
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(CONTROLLER_POLL_SEC)
    return False


def failed_row(episode: dict, results_file: Path, outcome: str) -> None:
    """Append a row with `outcome` and no metrics (the recorder never ran)."""
    spec = importlib.util.spec_from_file_location(
        "handover_recorder", PKG / "scripts" / "handover_recorder.py")
    recorder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(recorder)
    info = recorder.EpisodeInfo(
        episode_id=episode["episode_id"], seed=episode["seed"],
        facing_deg=math.degrees(episode["facing_yaw"]), hand_side=episode["hand_side"],
        hand_extends=episode["hand_extends"], reaction_delay=tuple(episode["reaction_delay"]),
        facing_change_deg=math.degrees(episode["facing_change"]),
        faults=tuple(episode["faults"]))
    row = recorder.EpisodeMetrics(recorder.core.default_config()).row(info)
    row["outcome"] = outcome
    recorder.write_row(results_file, row)


def run_episode(index: int, episode: dict, out: Path, use_nav: bool) -> str:
    """Run one episode in a fresh sim; return 'ok', 'bringup_failed' or 'episode_failed'."""
    env = episode_environment(index)
    results = out / "episodes.csv"
    for attempt in range(1, BRINGUP_ATTEMPTS + 1):
        if gazebo_running():
            raise RuntimeError("an `ign gazebo` server is running: stop it first "
                               "(kill -INT <pid>, then kill -KILL <pid>)")
        log_path = out / f"sim_{index}.log"
        if log_path.exists():
            log_path.rename(out / f"sim_{index}.attempt{attempt - 1}.log")
        with log_path.open("w") as log:
            bringup = subprocess.Popen(bringup_command(use_nav), env=env, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            try:
                if not _wait_controllers(env, bringup, BRINGUP_TIMEOUT_SEC):
                    print(f"[{index}] bringup attempt {attempt}: controllers not ready",
                          flush=True)
                    continue
                problem = _wait_grasp_manager(log_path, bringup, GRASP_READY_TIMEOUT_SEC)
                if problem:
                    print(f"[{index}] bringup attempt {attempt}: {problem}", flush=True)
                    continue
                with (out / f"episode_{index}.log").open("w") as episode_log:
                    # Its own session: on a timeout or Ctrl-C every node of the
                    # episode gets SIGINT, so the recorder can still write its row.
                    run = subprocess.Popen(episode_command(episode, results, use_nav),
                                           env=env, stdout=episode_log,
                                           stderr=subprocess.STDOUT, start_new_session=True)
                    try:
                        code = run.wait(timeout=EPISODE_TIMEOUT_SEC)
                    except subprocess.TimeoutExpired:
                        code = None
                    finally:
                        stop_group(run)
                return "ok" if code == 0 else "episode_failed"
            finally:
                stop_group(bringup)
    failed_row(episode, results, "bringup_failed")
    return "bringup_failed"


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Run seeded handover baseline episodes.")
    parser.add_argument("--set", choices=("baseline", "faults"), default="baseline")
    parser.add_argument("--out", type=Path, default=Path.home() / ".ros" / "handover_results")
    # Scripted legs by default: Nav2 did not reach a reversing goal (see the doc).
    parser.add_argument("--use-nav", choices=("true", "false"), default="false")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--only", type=int, nargs="*", default=None,
                        help="episode indices to run (default: all)")
    parser.add_argument("--dry-run", action="store_true", help="print the commands only")
    args = parser.parse_args(argv)
    use_nav = args.use_nav == "true"
    episodes = (baseline_episodes if args.set == "baseline" else fault_episodes)(args.seed)
    selected = [e for e in episodes if args.only is None or e["episode_id"] in args.only]
    out = args.out.expanduser()
    if args.dry_run:
        for episode in selected:
            print(f"# {episode['episode_id']}: {episode['label']}")
            print(" ".join(bringup_command(use_nav)))
            print(" ".join(episode_command(episode, out / "episodes.csv", use_nav)))
        return 0
    out.mkdir(parents=True, exist_ok=True)
    failures = []
    for episode in selected:
        index = episode["episode_id"]
        print(f"[{index}] {episode['label']} ...", flush=True)
        started = time.time()
        result = run_episode(index, episode, out, use_nav)
        print(f"[{index}] {result} after {time.time() - started:.0f} s", flush=True)
        if result != "ok":
            failures.append((index, result))
    print(f"{len(selected) - len(failures)} of {len(selected)} episodes ran to the end; "
          f"rows in {out / 'episodes.csv'}")
    for index, result in failures:
        print(f"  episode {index}: {result}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
