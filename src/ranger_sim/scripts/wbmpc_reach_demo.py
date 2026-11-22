#!/usr/bin/env python3
"""
Nav2 MPPI -> whole-body MPC handover demo for the Ranger Mini 3.0 + PiPER in Gazebo Fortress.

Sequence (use_nav adds the Nav2 legs):
  1. Open the gripper, arm to the level carry pose (MoveIt).
  2. [Nav2] drive away (nav_away_pose), then to the handover pose
     (nav_handover_pose): Nav2 stops within its goal tolerance (0.25 m,
     0.10 rad in config/nav2_sim.yaml), wherever that is.
  3. Hand the base to the MPC: base_handover_gate settles (zeros until the
     base is still), then forwards /cmd_vel_manip, translation only.
  4. Arm to streamed setpoints: piper_arm_controller (MoveIt's trajectory
     controller) out, piper_arm_stream_controller in (STRICT switch).
  5. Publish the target on /whole_body_mpc/target; whole_body_mpc slides the
     base and moves the arm together until it reports REACHED.
  6. Hold, then hand back in reverse: arm to the trajectory controller, base
     to navigation, arm to the carry pose (MoveIt), [Nav2] drive away again.

The handover order matters: the base must be still before anything else,
and the MPC starts commanding the moment it has MANIPULATE and a target, so
the target goes out only after the arm switch succeeded. Whatever ends the
run (a failed step, an error, Ctrl-C or SIGTERM), restore() cancels a Nav2
goal still running, hands the base back to navigation, the arm back to the
trajectory controller (best effort, whatever the demo believes is active)
and stops the base; each of these runs even if another fails. The process
exits 1 when the sequence did not complete.

Extends PickPlaceDemo (scripts/pick_place_demo.py, loaded by path) for its
MoveIt, gripper, odom and wait helpers.

Safety
------
`execute` defaults to **False** (plan-only): MoveIt plans the arm moves, and
nothing else is called, published or sent; Nav2 is not even waited for.
Set `execute:=true` to run it (Gazebo only, sim_bringup with
handover_gate:=true). Simulation only: nothing here reaches a CAN bus or a
real-robot topic.
"""

from dataclasses import dataclass
import importlib.util
import math
from pathlib import Path
import signal
import sys
import time
from typing import Callable, Optional

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool


def _load_sibling(name: str):
    # resolve(): with --symlink-install this file is a link into the source tree.
    path = Path(__file__).resolve().with_name(f"{name}.py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


pick = _load_sibling("pick_place_demo")

Pose2D = pick.Pose2D
Step = pick.Step

TARGET_POSITION = [0.95, 0.15, 1.00]  # odom; 0.20 m beyond the arm-only reach
NAV_AWAY_POSE = [-2.0, 0.0, 3.1416]
NAV_HANDOVER_POSE = [0.0, 0.0, 0.0]
REACH_TIMEOUT_SEC = 40.0
HANDOVER_TIMEOUT_SEC = 10.0
NAV_TIMEOUT_SEC = 120.0
HOLD_SEC = 2.0
SERVER_WAIT_SEC = pick.SERVER_WAIT_SEC  # gate service, controller manager
NAV_SERVER_WAIT_SEC = 60.0  # Nav2 activates ~10 s after sim_bringup
MPC_WAIT_SEC = 30.0  # whole_body_mpc builds its model from /robot_description
NAV_GOAL_ATTEMPTS = 2  # Humble: the first goal response can be lost (see pick_place_demo)
NAV_ACCEPT_TIMEOUT_SEC = 5.0
NAV_CANCEL_TIMEOUT_SEC = 5.0
STATE_POLL_SEC = 0.05

GATE_SERVICE = "/base_handover_gate/set_manipulate"
GATE_MODE_TOPIC = "/base_handover_gate/mode"
MPC_STATE_TOPIC = "/whole_body_mpc/state"
MPC_TARGET_TOPIC = "/whole_body_mpc/target"
SWITCH_SERVICE = "/controller_manager/switch_controller"
NAV_ACTION = "/navigate_to_pose"
TRAJECTORY_CONTROLLER = "piper_arm_controller"
STREAM_CONTROLLER = "piper_arm_stream_controller"
TARGET_FRAME = "odom"
NAV_FRAME = "map"  # = odom in the sim (static identity map -> odom)
FAULT = "FAULT: "
LATCHED_QOS = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
)
_SIN_45 = math.sin(math.pi / 4)


def level_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """
    Return the level gripper orientation approaching along `yaw` as (x, y, z, w).

    Rz(yaw) * Ry(+90 deg): the same as whole_body_mpc_core.level_quaternion,
    written out so the demo does not need casadi (the tests compare the two).
    """
    s, c = math.sin(yaw / 2), math.cos(yaw / 2)
    return (-s * _SIN_45, c * _SIN_45, s * _SIN_45, c * _SIN_45)


def _triple(name: str, values) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"{name} must have 3 values, got {list(values)}")
    return tuple(float(v) for v in values)


@dataclass(frozen=True)
class ReachPlan:
    """Arm pose, Nav2 poses and the reach target of one run."""

    carry: tuple[float, ...]
    target_position: tuple[float, float, float]  # odom
    target_yaw: float
    away: Pose2D  # map
    handover: Pose2D  # map
    use_nav: bool
    hold_sec: float


class WbmpcReachDemo(pick.PickPlaceDemo):
    """Carry pose, [Nav2 legs], hand-over, whole-body MPC reach, hand back."""

    def __init__(self, **node_kwargs) -> None:
        super().__init__("wbmpc_reach_demo", **node_kwargs)
        p = self.declare_parameter
        p("use_nav", False)
        p("target_position", TARGET_POSITION)
        p("target_yaw", 0.0)
        p("nav_away_pose", NAV_AWAY_POSE)
        p("nav_handover_pose", NAV_HANDOVER_POSE)
        p("reach_timeout", REACH_TIMEOUT_SEC)
        p("handover_timeout", HANDOVER_TIMEOUT_SEC)
        p("nav_timeout", NAV_TIMEOUT_SEC)
        p("hold_sec", HOLD_SEC)
        self._gate_mode: Optional[str] = None
        self._mpc_state: Optional[str] = None
        # The stream controller is active (True), not (False), or unknown (None:
        # a switch went unanswered). The trajectory controller is active at bringup.
        self._streaming: Optional[bool] = False
        self._nav_handle = None  # accepted Nav2 goal that has not finished
        self._gate_client = self.create_client(SetBool, GATE_SERVICE)
        self._switch_client = self.create_client(SwitchController, SWITCH_SERVICE)
        self._nav = ActionClient(self, NavigateToPose, NAV_ACTION)
        self._target_pub = self.create_publisher(PoseStamped, MPC_TARGET_TOPIC, 10)
        self.create_subscription(String, GATE_MODE_TOPIC, self._on_gate_mode, LATCHED_QOS)
        self.create_subscription(String, MPC_STATE_TOPIC, self._on_mpc_state, LATCHED_QOS)

    def _value(self, name: str):
        return self.get_parameter(name).value

    def _on_gate_mode(self, msg: String) -> None:
        self._gate_mode = msg.data

    def _on_mpc_state(self, msg: String) -> None:
        self._mpc_state = msg.data

    def _faulted(self) -> bool:
        return (self._mpc_state or "").startswith(FAULT)

    def _spin_until(self, done: Callable[[], bool], timeout_sec: float) -> bool:
        """Spin until `done()` (True) or the wall-clock timeout (False)."""
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            if done():
                return True
            rclpy.spin_once(self, timeout_sec=STATE_POLL_SEC)
        return done()

    def _call(self, client, request, label: str):
        """Call a service and wait up to handover_timeout; None (logged) without an answer."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._value("handover_timeout"))
        if future.result() is None:
            self.get_logger().error(f"{label}: no answer from {client.srv_name}")
        return future.result()

    # ---- plan --------------------------------------------------------------------
    def reach_plan(self) -> ReachPlan:
        """Return the run's poses and target from the parameters; ValueError on a bad one."""
        return ReachPlan(
            carry=tuple(self._value("carry_joints")),
            target_position=_triple("target_position", self._value("target_position")),
            target_yaw=float(self._value("target_yaw")),
            away=_triple("nav_away_pose", self._value("nav_away_pose")),
            handover=_triple("nav_handover_pose", self._value("nav_handover_pose")),
            use_nav=bool(self._value("use_nav")),
            hold_sec=float(self._value("hold_sec")),
        )

    def target_message(self, plan: ReachPlan) -> PoseStamped:
        """Return the reach target: level gripper at the target position, in odom."""
        msg = PoseStamped()
        msg.header.frame_id = TARGET_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        position = msg.pose.position
        position.x, position.y, position.z = plan.target_position
        o = msg.pose.orientation
        o.x, o.y, o.z, o.w = level_quaternion(plan.target_yaw)
        return msg

    # ---- hand-over -------------------------------------------------------------------
    def set_base_owner(self, manipulate: bool) -> bool:
        """Ask the gate for MANIPULATE (True) or NAVIGATE and wait until it is there."""
        wanted = "MANIPULATE" if manipulate else "NAVIGATE"
        if not self.execute:
            self.get_logger().info(f"[plan-only] would hand the base to {wanted} (skipped)")
            return True
        response = self._call(self._gate_client, SetBool.Request(data=manipulate), "gate")
        if response is None or not response.success:
            return False
        if not self._spin_until(lambda: self._gate_mode == wanted,
                                self._value("handover_timeout")):
            self.get_logger().error(
                f"base not handed to {wanted} in time (gate mode {self._gate_mode})"
            )
            return False
        self.get_logger().info(f"base owner -> {wanted}")
        return True

    def switch_arm(self, stream: bool, strict: bool = True) -> bool:
        """
        Activate the stream (True) or the trajectory controller, deactivating the other.

        Strict: skipped when the demo knows it is so, refused by the controller
        manager otherwise. Not strict (restore): always sent, best effort, so
        it also works from an unknown state.
        """
        on, off = ((STREAM_CONTROLLER, TRAJECTORY_CONTROLLER) if stream
                   else (TRAJECTORY_CONTROLLER, STREAM_CONTROLLER))
        if not self.execute:
            self.get_logger().info(f"[plan-only] would switch the arm to {on} (skipped)")
            return True
        if strict and self._streaming is stream:
            return True
        request = SwitchController.Request()
        request.activate_controllers, request.deactivate_controllers = [on], [off]
        request.strictness = (SwitchController.Request.STRICT if strict
                              else SwitchController.Request.BEST_EFFORT)
        request.timeout = Duration(sec=int(self._value("handover_timeout")))
        response = self._call(self._switch_client, request, "controller switch")
        if response is None:
            self._streaming = None  # the switch may or may not have happened
            return False
        if not response.ok:
            self.get_logger().error(f"controller switch to {on} refused")
            return False
        self._streaming = stream
        self.get_logger().info(f"arm -> {on}")
        return True

    def wait_for_state(self, wanted: str, timeout_sec: float) -> bool:
        """Wait for the MPC state `wanted`; False at once on a FAULT, or at the timeout."""
        self._spin_until(lambda: self._mpc_state == wanted or self._faulted(), timeout_sec)
        if self._mpc_state == wanted:
            return True
        self.get_logger().error(f"whole_body_mpc did not reach {wanted}: {self._mpc_state}")
        return False

    def reach(self, label: str) -> bool:
        """Send the target and wait until whole_body_mpc reports REACHED."""
        plan = self.reach_plan()
        if not self.execute:
            self.get_logger().info(f"[plan-only] {label}: would send the target (skipped)")
            return True
        self._mpc_state = None  # only a state reported after this target counts
        self._target_pub.publish(self.target_message(plan))
        x, y, z = plan.target_position
        self.get_logger().info(f"{label} -> TCP ({x:.3f}, {y:.3f}, {z:.3f}) in odom")
        if not self.wait_for_state("REACHED", self._value("reach_timeout")):
            return False
        self.get_logger().info(f"{label}: REACHED, base at {pick.format_pose(self._odom_pose())}")
        return True

    def hold(self, seconds: float) -> bool:
        """Stay at the target for `seconds`; False if the MPC faults meanwhile."""
        if not self.execute:
            return True
        self._spin_until(self._faulted, seconds)
        if self._faulted():
            self.get_logger().error(f"whole_body_mpc faulted while holding: {self._mpc_state}")
            return False
        return True

    # ---- Nav2 ---------------------------------------------------------------------------
    def _nav_goal(self, pose: Pose2D) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = NAV_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose[0], pose[1]
        goal.pose.pose.orientation.z = math.sin(pose[2] / 2)
        goal.pose.pose.orientation.w = math.cos(pose[2] / 2)
        return goal

    def navigate(self, pose: Pose2D, label: str) -> bool:
        """Send a NavigateToPose goal (frame map) and wait for its result."""
        if not self.execute:
            self.get_logger().info(
                f"[plan-only] {label}: would send a Nav2 goal {pick.format_pose(pose)} (skipped)"
            )
            return True
        self.get_logger().info(f"{label} -> {pick.format_pose(pose)}")
        handle = None
        for attempt in range(1, NAV_GOAL_ATTEMPTS + 1):
            future = self._nav.send_goal_async(self._nav_goal(pose))
            rclpy.spin_until_future_complete(self, future, timeout_sec=NAV_ACCEPT_TIMEOUT_SEC)
            handle = future.result()
            if handle is not None:
                break
            self.get_logger().warn(f"{label}: no goal response (attempt {attempt}), re-sending")
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{label}: Nav2 goal rejected or unanswered")
            return False
        self._nav_handle = handle  # restore() cancels it if the wait is interrupted
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=self._value("nav_timeout"))
        if result.result() is None:
            self.get_logger().error(f"{label}: Nav2 did not finish in time")
            self._cancel_nav_goal()
            return False
        self._nav_handle = None
        status = result.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"{label}: Nav2 goal ended with status {status}")
            return False
        self.get_logger().info(f"{label}: base at {pick.format_pose(self._odom_pose())}")
        return True

    def _cancel_nav_goal(self) -> None:
        """Cancel a Nav2 goal that has not finished and wait (bounded) for the answer."""
        handle, self._nav_handle = self._nav_handle, None
        if handle is None:
            return
        future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=NAV_CANCEL_TIMEOUT_SEC)
        if future.result() is None:
            self.get_logger().error("Nav2 goal cancel not confirmed: Nav2 may still drive")
        else:
            self.get_logger().info("Nav2 goal cancelled")

    # ---- orchestration ------------------------------------------------------------------
    def sequence(self, plan: ReachPlan) -> list[Step]:
        """Every step of the run in order."""
        away = "Nav2 MPPI: drive away"
        handover = "Nav2 MPPI: to the handover pose"
        drive_away = [(away, lambda: self.navigate(plan.away, away))]
        to_handover = [(handover, lambda: self.navigate(plan.handover, handover))]
        reach = "reach: base + arm (MPC)"
        return [
            ("open gripper", lambda: self.set_gripper(pick.GRIPPER_OPEN, "open")),
            self._arm_step("carry pose", plan.carry),
            *(drive_away + to_handover if plan.use_nav else []),
            # Hand-over: base still first, then the arm, then the target.
            ("base to the whole-body MPC", lambda: self.set_base_owner(True)),
            ("arm to streamed setpoints", lambda: self.switch_arm(stream=True)),
            (reach, lambda: self.reach(reach)),
            ("hold at the target", lambda: self.hold(plan.hold_sec)),
            # Hand back in reverse; MoveIt only once its controller is active.
            ("arm back to MoveIt", lambda: self.switch_arm(stream=False)),
            ("base back to Nav2", lambda: self.set_base_owner(False)),
            self._arm_step("carry pose", plan.carry),
            *(drive_away if plan.use_nav else []),
        ]

    def restore(self) -> None:
        """Hand the base to navigation and the arm to MoveIt, stop the base (executing only)."""
        if not self.execute:
            return
        steps = (
            # Nav2 first: in NAVIGATE the gate passes its twist straight through.
            ("cancel the Nav2 goal", self._cancel_nav_goal),
            # Then the gate: SETTLING stops the MPC's base commands at once.
            ("base back to navigation", lambda: self.set_base_owner(False)),
            ("arm back to MoveIt", lambda: self.switch_arm(stream=False, strict=False)),
            ("stop the base", self.stop_base),
        )
        for name, step in steps:
            try:
                step()
            except (Exception, KeyboardInterrupt) as exc:  # noqa: BLE001 - restore every step
                self.get_logger().error(f"restore: {name} failed: {exc!r}")

    def _wait_reach_servers(self) -> None:
        """Wait for the gate, the controller manager, whole_body_mpc and (use_nav) Nav2."""
        waits = [
            (GATE_SERVICE, lambda: self._gate_client.wait_for_service(SERVER_WAIT_SEC)),
            (SWITCH_SERVICE, lambda: self._switch_client.wait_for_service(SERVER_WAIT_SEC)),
            ("whole_body_mpc", lambda: self._spin_until(
                lambda: self._mpc_state is not None
                and self._target_pub.get_subscription_count() > 0, MPC_WAIT_SEC)),
        ]
        if self._value("use_nav"):
            waits.append((NAV_ACTION, lambda: self._nav.wait_for_server(NAV_SERVER_WAIT_SEC)))
        for name, wait in waits:
            self.get_logger().info(f"Waiting for {name} ...")
            if not wait():
                raise RuntimeError(f"{name} not available (sim_bringup handover_gate:=true?)")

    def run(self) -> bool:
        """Run the sequence, restoring on any exit; True when every step succeeded."""
        plan = self.reach_plan()  # malformed values fail before any wait
        self._wait_servers()
        if self.execute:
            self._wait_reach_servers()
        try:
            for name, action in self.sequence(plan):
                if not action():
                    self.get_logger().error(f"step '{name}' failed — aborting")
                    return False
            self.get_logger().info(
                "Whole-body MPC reach complete "
                f"({'executed' if self.execute else 'plan-only'})."
            )
            return True
        finally:
            self.restore()


def _interrupt(signum, frame) -> None:
    raise KeyboardInterrupt


def main() -> None:
    # Ctrl-C as a plain KeyboardInterrupt: rclpy's own SIGINT handler would
    # shut the context down first, and restore() could not hand back. launch
    # escalates SIGINT to SIGTERM: that one ends in restore() too.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    previous = signal.signal(signal.SIGTERM, _interrupt)
    node = WbmpcReachDemo()
    completed = True  # Ctrl-C is not a failure
    try:
        completed = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted")
    except Exception as exc:  # noqa: BLE001 - demo: surface any failure clearly
        node.get_logger().error(f"demo failed: {exc}")
        completed = False
    finally:
        signal.signal(signal.SIGTERM, previous)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not completed:
        sys.exit(1)


if __name__ == "__main__":
    main()
