#!/usr/bin/env python3
"""
Serve /grasp/set (ranger_sim/srv/SetGrasp) on top of Gazebo DetachableJoint.

Why this node exists
--------------------
ranger_gazebo.urdf.xacro emits one DetachableJoint plugin per object in
config/graspable_objects.yaml. Fortress 6.16 has no ``initial_attach``, so
each plugin creates its fixed joint as soon as the robot spawns: every
graspable object starts welded to the gripper. Gazebo is therefore launched
paused, and at startup this node

  1. waits for the bridge (ROS side of /grasp/<obj>/{attach,detach,state}),
  2. waits until the robot model exists in the world (``ign model --list``),
  3. re-sends ``detach`` to every graspable object present in the world
     until each reports "detached" -- the plugin ignores redundant requests,
     and its initial "attached" message is usually published before
     gz-transport discovery completes, so the state is confirmed, never
     assumed,
  4. unpauses the world (ControlWorld service, ``ign service`` fallback),
  5. advertises /grasp/set.

The world is unpaused even when a step times out: a frozen simulation is
worse than a dragged object, and every failure is logged.

The plugin only reports transitions and stays silent on a redundant
request. A *detach* that gets no answer although simulation time advanced
(the plugin had iterations to process it) therefore means "already
detached" -- this lets the node recover after a restart against a running
sim or a lost state message. An *attach* is never inferred: when the state
is not known to be "detached", the node detaches first, so a real attach
always produces a transition.

Task code only ever calls /grasp/set, so another simulator backend can serve
the same service later without touching the callers.
"""

import os
import re
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from functools import partial

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from ranger_sim.srv import SetGrasp
from ros_gz_interfaces.srv import ControlWorld
from std_msgs.msg import Empty, String

STATE_ATTACHED = "attached"
STATE_DETACHED = "detached"
POLL_PERIOD_SEC = 0.1
MODEL_LIST_PERIOD_SEC = 1.0
CLI_TIMEOUT_SEC = 10.0
CONTROL_SERVICE_WAIT_SEC = 5.0
# Simulated time (~50 physics iterations at 4 ms) after which a detach that
# produced no transition is taken as "already detached".
IGNORED_REQUEST_SIM_NS = 200_000_000
STARTUP_JOIN_SEC = 2.0
_WORLD_LINE = re.compile(r"world \[([^\]]+)\]")


@dataclass(frozen=True)
class GraspObject:
    """A graspable Gazebo model and the link the gripper attaches to."""

    name: str
    link: str


@dataclass(frozen=True)
class GraspConfig:
    """Parsed config/graspable_objects.yaml."""

    gripper_link: str
    objects: tuple[GraspObject, ...]

    @property
    def names(self) -> tuple[str, ...]:
        """Object names in file order."""
        return tuple(obj.name for obj in self.objects)


def parse_grasp_config(data: object) -> GraspConfig:
    """Validate the YAML structure; raise ValueError naming the problem."""
    if not isinstance(data, dict):
        raise ValueError("graspable objects file must be a mapping")
    gripper_link = data.get("gripper_link")
    if not isinstance(gripper_link, str) or not gripper_link:
        raise ValueError("'gripper_link' must be a non-empty string")
    raw_objects = data.get("objects")
    if not isinstance(raw_objects, list) or not raw_objects:
        raise ValueError("'objects' must be a non-empty list")
    objects = []
    for index, entry in enumerate(raw_objects):
        if not isinstance(entry, dict):
            raise ValueError(f"objects[{index}] must be a mapping")
        name, link = entry.get("name"), entry.get("link")
        if not isinstance(name, str) or not name:
            raise ValueError(f"objects[{index}]: 'name' must be a non-empty string")
        if not isinstance(link, str) or not link:
            raise ValueError(
                f"objects[{index}] '{name}': 'link' must be a non-empty string"
            )
        objects.append(GraspObject(name=name, link=link))
    names = [obj.name for obj in objects]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise ValueError(f"duplicate object names: {duplicates}")
    return GraspConfig(gripper_link=gripper_link, objects=tuple(objects))


def load_grasp_config(path: str) -> GraspConfig:
    """Load and validate a graspable-objects YAML file."""
    with open(path, encoding="utf-8") as stream:
        return parse_grasp_config(yaml.safe_load(stream))


def parse_model_list(output: str) -> frozenset[str]:
    """Extract model names from ``ign model --list`` output ("    - <name>")."""
    names = set()
    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith("- "):
            names.add(stripped[2:].strip())
    return frozenset(names)


def parse_world_name(output: str) -> str | None:
    """Extract the world from ``Requesting state for world [<name>]...``."""
    match = _WORLD_LINE.search(output)
    return match.group(1) if match else None


def select_gz_cli() -> str | None:
    """Return the Fortress CLI: ``ign`` (``gz`` may be Gazebo Classic 11)."""
    for tool in ("ign", "gz"):
        if shutil.which(tool):
            return tool
    return None


def default_objects_file() -> str:
    """Return the installed config/graspable_objects.yaml path."""
    share = get_package_share_directory("ranger_sim")
    return os.path.join(share, "config", "graspable_objects.yaml")


class GraspManager(Node):
    """Own the DetachableJoint topics; expose attach/detach as /grasp/set."""

    def __init__(self) -> None:
        super().__init__("grasp_manager")
        p = self.declare_parameter
        p("world_name", "garden_world")
        p("robot_model", "ranger")  # model name given to ros_gz_sim create
        p("objects_file", default_objects_file())
        p("auto_unpause", True)
        p("bridge_wait_sec", 10.0)  # ROS side of the bridge entries
        p("spawn_wait_sec", 60.0)  # robot model to appear in the world
        # Startup detach confirmation. The first sim iterations after the
        # robot spawns are blocked by sensor/rendering initialisation, so the
        # plugin only processes the detach once that finishes (~7 s headless
        # on an RTX 6000; slower GPUs / Jetson need more).
        p("detach_timeout_sec", 20.0)
        p("state_timeout_sec", 1.0)  # per /grasp/set request

        self._world = self.get_parameter("world_name").value
        self._cli_world = self._world  # corrected from `ign model --list`
        self._robot = self.get_parameter("robot_model").value
        self._config = load_grasp_config(self.get_parameter("objects_file").value)
        self._cli = select_gz_cli()

        # State messages in one mutually exclusive group: delivered in order,
        # so an "attached" -> "detached" burst cannot be applied reversed.
        # /grasp/set in its own group: concurrent callers queue in the
        # executor instead of occupying the threads the state callbacks need.
        self._state_group = MutuallyExclusiveCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._client_group = MutuallyExclusiveCallbackGroup()
        self._state_cv = threading.Condition()
        self._states: dict[str, str | None] = {
            name: None for name in self._config.names
        }
        self._present: frozenset[str] | None = None  # None = unknown
        self._stop = threading.Event()
        self._service = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._attach_pubs = {
            name: self.create_publisher(Empty, f"/grasp/{name}/attach", qos)
            for name in self._config.names
        }
        self._detach_pubs = {
            name: self.create_publisher(Empty, f"/grasp/{name}/detach", qos)
            for name in self._config.names
        }
        self._state_subs = [
            self.create_subscription(
                String,
                f"/grasp/{name}/state",
                partial(self._on_state, name),
                10,
                callback_group=self._state_group,
            )
            for name in self._config.names
        ]
        self._control = self.create_client(
            ControlWorld,
            f"/world/{self._world}/control",
            callback_group=self._client_group,
        )

    def request_stop(self) -> None:
        """Make the startup thread return at its next wait."""
        self._stop.set()

    # ---- callbacks -----------------------------------------------------
    def _on_state(self, name: str, msg: String) -> None:
        with self._state_cv:
            previous = self._states[name]
            self._states[name] = msg.data
            self._state_cv.notify_all()
        if msg.data != previous:
            self.get_logger().info(f"{name}: {msg.data}")

    def _on_set_grasp(
        self, request: SetGrasp.Request, response: SetGrasp.Response
    ) -> SetGrasp.Response:
        response.success, response.message = self._handle_request(
            request.object, request.attach
        )
        summary = (
            f"/grasp/set {request.object} attach={request.attach}: {response.message}"
        )
        # rclpy pins one severity per call site, so keep two log calls.
        if response.success:
            self.get_logger().info(summary)
        else:
            self.get_logger().warn(summary)
        return response

    # ---- request handling ------------------------------------------------
    def _handle_request(self, name: str, attach: bool) -> tuple[bool, str]:
        if name not in self._states:
            return False, f"unknown object '{name}'; known: {list(self._config.names)}"
        if self._present is not None and name not in self._present:
            return False, f"object '{name}' is not in world '{self._world}'"
        state = self._state(name)
        if not attach:
            if state == STATE_DETACHED:
                return True, "already detached"
            return self._request_state(name, STATE_DETACHED)
        if state != STATE_DETACHED:
            # Attached: the plugin ignores attach, so re-create the joint at
            # the current relative pose. Unknown: detach first so the attach
            # below is guaranteed to produce a transition.
            ok, message = self._request_state(name, STATE_DETACHED)
            if not ok:
                return False, f"attach: detach first failed: {message}"
        return self._request_state(name, STATE_ATTACHED)

    def _state(self, name: str) -> str | None:
        with self._state_cv:
            return self._states[name]

    def _sim_time_ns(self) -> int | None:
        """Return simulation time, or None when the node is not on sim time."""
        if not self.get_parameter("use_sim_time").value:
            return None
        return self.get_clock().now().nanoseconds

    def _plugin_had_time(self, pub: Publisher, start_sim_ns: int | None) -> bool:
        """Return True if the bridge is connected and the sim advanced enough."""
        now = self._sim_time_ns()
        if start_sim_ns is None or now is None or pub.get_subscription_count() == 0:
            return False
        return now - start_sim_ns >= IGNORED_REQUEST_SIM_NS

    def _request_state(self, name: str, target: str) -> tuple[bool, str]:
        """Publish attach/detach until the plugin reports ``target``."""
        timeout = float(self.get_parameter("state_timeout_sec").value)
        detaching = target == STATE_DETACHED
        pub = self._detach_pubs[name] if detaching else self._attach_pubs[name]
        start_sim = self._sim_time_ns()
        deadline = time.monotonic() + timeout
        with self._state_cv:
            while self._states[name] != target:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                # Redundant requests are ignored by the plugin, so re-sending
                # covers a lost first message without side effects.
                pub.publish(Empty())
                self._state_cv.wait(timeout=min(POLL_PERIOD_SEC, remaining))
            if self._states[name] == target:
                return True, target
        if detaching and self._plugin_had_time(pub, start_sim):
            with self._state_cv:
                self._states[name] = STATE_DETACHED
            return True, "detached (no transition reported: was already detached)"
        return False, f"no '{target}' confirmation within {timeout:.1f} s"

    # ---- startup ---------------------------------------------------------
    def run_startup(self) -> None:
        """Detach every present object, unpause, then advertise /grasp/set."""
        try:
            self._wait_for_bridge()
            present = self._discover_present_objects()
            detached = self._detach_all(present)
            self._warn_if_sim_was_running(detached)
        except Exception as exc:  # noqa: BLE001 - never leave the sim frozen
            self.get_logger().error(f"grasp startup failed: {exc}")
        finally:
            self._finish_startup()

    def _finish_startup(self) -> None:
        if self._stop.is_set():
            return
        auto_unpause = self.get_parameter("auto_unpause").value
        if auto_unpause:
            try:
                self._unpause()
            except Exception as exc:  # noqa: BLE001 - still serve /grasp/set
                self.get_logger().error(f"unpause failed: {exc}")
        self._service = self.create_service(
            SetGrasp,
            "/grasp/set",
            self._on_set_grasp,
            callback_group=self._service_group,
        )
        count = (
            len(self._present) if self._present is not None else len(self._config.names)
        )
        unpaused = "sim unpaused" if auto_unpause else "sim left paused"
        self.get_logger().info(
            f"grasp manager ready: {count} objects, world '{self._world}', {unpaused}"
        )

    def _wait_for_bridge(self) -> None:
        timeout = float(self.get_parameter("bridge_wait_sec").value)
        deadline = time.monotonic() + timeout
        pubs = [*self._attach_pubs.values(), *self._detach_pubs.values()]
        state_topics = [f"/grasp/{name}/state" for name in self._config.names]
        while time.monotonic() < deadline and not self._stop.is_set():
            pubs_ready = all(pub.get_subscription_count() > 0 for pub in pubs)
            subs_ready = all(self.count_publishers(topic) > 0 for topic in state_topics)
            if pubs_ready and subs_ready:
                return
            self._stop.wait(POLL_PERIOD_SEC)
        if not self._stop.is_set():
            self.get_logger().error(
                f"grasp bridge entries not up after {timeout:.0f} s "
                "(check the /grasp/* section of config/gz_bridge.yaml)"
            )

    def _list_models(self) -> frozenset[str] | None:
        try:
            result = subprocess.run(
                [self._cli, "model", "--list"],
                check=False,
                timeout=CLI_TIMEOUT_SEC,
                capture_output=True,
                text=True,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.get_logger().warn(f"'{self._cli} model --list' failed: {exc}")
            return None
        self._check_world_name(parse_world_name(result.stdout))
        return parse_model_list(result.stdout)

    def _check_world_name(self, reported: str | None) -> None:
        """Warn once and use the running world's name for the CLI fallback."""
        if reported is None or reported == self._cli_world:
            return
        self._cli_world = reported
        self.get_logger().error(
            f"running world is '{reported}' but world_name is '{self._world}': "
            f"/world/{self._world}/control will fail; pass world_name:={reported}"
        )

    def _discover_present_objects(self) -> frozenset[str] | None:
        """Wait for the robot; return the graspable objects in the world."""
        if self._cli is None:
            self.get_logger().warn("no 'ign'/'gz' CLI found: object presence unknown")
            return None
        timeout = float(self.get_parameter("spawn_wait_sec").value)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self._stop.is_set():
            models = self._list_models()
            if models is not None and self._robot in models:
                present = frozenset(self._config.names) & models
                absent = sorted(set(self._config.names) - present)
                self._present = present
                self.get_logger().info(
                    f"robot '{self._robot}' spawned; graspable objects in world: "
                    f"{sorted(present)} (not in this world: {absent})"
                )
                return present
            self._stop.wait(MODEL_LIST_PERIOD_SEC)
        if not self._stop.is_set():
            self.get_logger().error(
                f"robot model '{self._robot}' not in world after {timeout:.0f} s; "
                "detaching every configured object blindly"
            )
        return None

    def _detach_all(self, present: frozenset[str] | None) -> frozenset[str]:
        """Re-send detach until each object confirms; return those that did."""
        candidates = (
            sorted(present) if present is not None else list(self._config.names)
        )
        timeout = float(self.get_parameter("detach_timeout_sec").value)
        deadline = time.monotonic() + timeout
        start_sim = self._sim_time_ns()
        pending = set(candidates)
        inferred: set[str] = set()
        while pending and time.monotonic() < deadline and not self._stop.is_set():
            for name in pending:
                self._detach_pubs[name].publish(Empty())
            with self._state_cv:
                self._state_cv.wait(timeout=POLL_PERIOD_SEC)
                pending = {n for n in pending if self._states[n] != STATE_DETACHED}
                # Sim already running (e.g. this node restarted): silence
                # after enough simulated time means "already detached".
                quiet = {
                    n
                    for n in pending
                    if self._plugin_had_time(self._detach_pubs[n], start_sim)
                }
                for name in quiet:
                    self._states[name] = STATE_DETACHED
                inferred |= quiet
                pending -= quiet
        self._report_detach(candidates, pending, inferred, present, timeout)
        return frozenset(candidates) - pending - inferred

    def _report_detach(
        self,
        candidates: list[str],
        pending: set[str],
        inferred: set[str],
        present: frozenset[str] | None,
        timeout: float,
    ) -> None:
        confirmed = sorted(set(candidates) - pending - inferred)
        if confirmed:
            self.get_logger().info(f"detached at startup: {confirmed}")
        if inferred:
            self.get_logger().info(
                f"no transition from {sorted(inferred)} while the sim was running: "
                "already detached"
            )
        if pending and present is not None:
            self.get_logger().error(
                f"no 'detached' confirmation within {timeout:.1f} s for "
                f"{sorted(pending)}: they may stay welded to the gripper"
            )
        elif pending:
            self.get_logger().info(
                f"no state from {sorted(pending)} (not in this world?)"
            )

    def _warn_if_sim_was_running(self, detached: frozenset[str]) -> None:
        """Flag objects that were still welded while the sim already ran."""
        sim_time = self._sim_time_ns()
        if sim_time is None or sim_time <= 0:
            return
        if detached:
            self.get_logger().warn(
                f"simulation time was already {sim_time / 1e9:.2f} s when "
                f"{sorted(detached)} got detached (launched with paused:=false?): "
                "they may have been dragged by the gripper"
            )
        else:
            self.get_logger().info(
                f"simulation already running ({sim_time / 1e9:.2f} s; grasp_manager "
                "restarted?): no object needed detaching"
            )

    def _unpause(self) -> None:
        if self._control.wait_for_service(timeout_sec=CONTROL_SERVICE_WAIT_SEC):
            request = ControlWorld.Request()
            request.world_control.pause = False
            future = self._control.call_async(request)
            if _wait_future(future, CONTROL_SERVICE_WAIT_SEC):
                result = future.result()
                if result is not None and result.success:
                    return
            self.get_logger().warn("ControlWorld unpause not confirmed; trying CLI")
        else:
            self.get_logger().warn(
                f"/world/{self._world}/control not available after "
                f"{CONTROL_SERVICE_WAIT_SEC:.0f} s; trying CLI"
            )
        self._unpause_cli()

    def _unpause_cli(self) -> None:
        if self._cli is None:
            self.get_logger().error("cannot unpause: no 'ign'/'gz' CLI found")
            return
        command = [
            self._cli, "service", "-s", f"/world/{self._cli_world}/control",
            "--reqtype", "ignition.msgs.WorldControl",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "1000", "--req", "pause: false",
        ]  # fmt: skip
        try:
            result = subprocess.run(
                command,
                check=False,
                timeout=CLI_TIMEOUT_SEC,
                capture_output=True,
                text=True,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.get_logger().error(f"CLI unpause failed: {exc}")
            return
        if "true" not in (result.stdout or "").lower():
            self.get_logger().error(
                f"CLI unpause not confirmed (rc={result.returncode}, "
                f"out={result.stdout!r} err={result.stderr!r})"
            )


def _wait_future(future: Future, timeout_sec: float) -> bool:
    """Block (wall clock) until ``future`` completes; executor spins elsewhere."""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    return done.wait(timeout_sec)


def main() -> None:
    rclpy.init()
    node = GraspManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    # Startup waits on wall time (the sim clock is frozen while paused) and
    # needs the executor spinning to receive the plugin state messages.
    startup = threading.Thread(target=node.run_startup, daemon=True)
    startup.start()
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.request_stop()
        startup.join(timeout=STARTUP_JOIN_SEC)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
