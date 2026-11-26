#!/usr/bin/env python3
"""
Rule baseline of the handover study: deliver the bottle to a seated person.

See docs/sim_handover_baseline.md.

One episode, phases on ~/phase (String, latched, on change):

  PICK      the scripted pick of pick_place_demo (release, open, home, carry,
            drive to the table, look, approach, attach, close, lift, carry)
  AWAY      drive to nav_away_pose (the person turns to its episode facing)
  TRANSIT   choose a standoff from the MEASURED person, drive to its
            pre-standoff (Nav2 with use_nav, else the scripted pose
            controller), wait there for the hand; re-choose if the decisions
            say so (the robot goes on to the next pre-standoff)
  APPROACH  publish the MPC base box (standoff +- box_half), hand the base
            to the MPC (gate), the arm to streamed setpoints
  OFFER     MPC to the offer pose (hand extended) ...
  PLACE     ... or, without a hand within hand_wait_sec, above the table place
            pose (place_hover_height), straight down to it, and the place pose
            once more on the place topic (the release permit binds to it); a
            standoff without place_ok first re-chooses with need_place
  WAIT      offer: the measured take, up to take_timeout_sec (then the
            outcome is `timeout` and the object stays in the gripper)
  RELEASE   the release permit of handover_safety, open, /grasp/set release
  RETREAT   back off retreat_distance along the approach (MPC), hand back,
            carry pose, back to the pre-standoff, away
  DONE: <outcome> | FAILED: <step>

TRANSIT is published on every entry, a re-choice included (the recorder
counts re-choices from it); every other phase on change only.

Outcomes: handover, set_down, timeout, no_standoff, stopped (a step failed
while handover_safety reported STOP), failed. The last log line is
`handover result: outcome=<o> rechoices=<n> wait=<s> s`.

The four decisions (standoff, requested speed scale, wait or go,
re-choose) come only from the injected Decisions object (RuleDecisions of
scripts/handover_core.py by default); a learned policy replaces it. The
node has no route to the hard limits: it REQUESTS a speed scale
(/handover_safety/scale_request, 1 / scale_rate s, the decision's scale at
the measured clearance), sends MPC targets to /handover_safety/target_request
(offer, back-off) and /handover_safety/place_target_request (table place
pose), and opens the gripper only after /handover_safety/release granted
it. It publishes nothing on /whole_body_mpc/target or a speed scale topic.

Extends WbmpcReachDemo (scripts/wbmpc_reach_demo.py, loaded by path) for its
hand-over, MPC target, Nav2 and restore helpers: whatever ends the run, the
Nav2 goal is cancelled, the base handed back to navigation, the arm to the
trajectory controller, and the base stopped.

Safety
------
`execute` defaults to **False** (plan-only): MoveIt plans the arm moves,
nothing else is published, called or sent. `execute:=true` needs a running
sim (sim_bringup handover_gate:=true) and handover_safety. Simulation only:
nothing here reaches a CAN bus or a real-robot topic.
"""

import dataclasses
import importlib.util
import math
from pathlib import Path
import time
from typing import Callable, Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from std_srvs.srv import Trigger


def _load_sibling(name: str):
    # resolve(): with --symlink-install this file is a link into the source tree.
    path = Path(__file__).resolve().with_name(f"{name}.py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


reach = _load_sibling("wbmpc_reach_demo")
pick = reach.pick
core = _load_sibling("handover_core")

Decisions = core.Decisions
RuleDecisions = core.RuleDecisions
Pose2D = pick.Pose2D

# Defaults of the node parameters of the same name (config/handover.yaml).
SCALE_RATE_HZ = 10.0
RETREAT_DISTANCE_M = 0.15
RETREAT_SPEED_MPS = 0.03  # wbmpc_pick_place_demo SLIDE_SPEED_MPS
PERSON_WAIT_SEC = 10.0
# Wall-clock limit of a scripted base leg (use_nav false). pick_place_demo's 30 s
# is too short here: the requested speed scale is 0.3 next to the person (the
# 2.4 m leg away from the pick slot took 30 s in Gazebo and stopped 3 cm short).
DRIVE_TIMEOUT_SEC = 90.0
# Wall-clock guards of the waits that count sim time: only for a paused sim, so
# generous (a real-time factor of 0.1 still fits).
HAND_WAIT_WALL_SEC = 180.0
TAKE_WAIT_WALL_MARGIN_SEC = 120.0
SCALE_REQUEST_TOPIC = "/handover_safety/scale_request"
TARGET_REQUEST_TOPIC = "/handover_safety/target_request"
PLACE_TARGET_REQUEST_TOPIC = "/handover_safety/place_target_request"
RELEASE_SERVICE = "/handover_safety/release"
SAFETY_STATE_TOPIC = "/handover_safety/state"
BASE_BOX_TOPIC = "/whole_body_mpc/base_box"
TRACKING_ERROR_TOPIC = "/whole_body_mpc/tracking_error"
MEASURED = "/person/measured"
PICK, AWAY, TRANSIT, APPROACH = "PICK", "AWAY", "TRANSIT", "APPROACH"
OFFER_PHASE, PLACE_PHASE, WAIT_PHASE = "OFFER", "PLACE", "WAIT"
RELEASE, RETREAT = "RELEASE", "RETREAT"
DONE, FAILED = "DONE: ", "FAILED: "
RECHOOSE = "rechoose"
HANDOVER, SET_DOWN, TIMEOUT = "handover", "set_down", "timeout"
NO_STANDOFF, STOPPED, FAILED_OUTCOME = "no_standoff", "stopped", "failed"
SAFETY_STOP = "STOP: "
SAFETY_UNGOVERNED = "STOP: gate not governed"  # handover_safety before the gate answered


class StepFailed(Exception):
    """A step of the episode returned False."""

    def __init__(self, caption: str) -> None:
        super().__init__(caption)
        self.caption = caption


@dataclasses.dataclass
class Episode:
    """What the episode decided so far (mutable, one per run)."""

    rechoices: int = 0
    need_place: bool = False
    standoff: Optional[core.Standoff] = None
    facing_at_choice: float = 0.0
    waited_sec: float = 0.0
    target: Optional[tuple[core.Point3, float]] = None  # the delivery target and its yaw


class HandoverBaseline(reach.WbmpcReachDemo):
    """Pick, then deliver to the measured person through handover_safety."""

    def __init__(self, decisions: Optional[Decisions] = None, **node_kwargs) -> None:
        super().__init__("handover_baseline", **node_kwargs)
        p = self.declare_parameter
        shared = {key: p(key, value).value for key, value in core.SHARED_DEFAULTS.items()}
        self.cfg = core.config_from_parameters(shared)
        positive = {
            "scale_rate": p("scale_rate", SCALE_RATE_HZ).value,
            "retreat_distance": p("retreat_distance", RETREAT_DISTANCE_M).value,
            "retreat_speed": p("retreat_speed", RETREAT_SPEED_MPS).value,
            "person_wait": p("person_wait", PERSON_WAIT_SEC).value,
            "drive_timeout": p("drive_timeout", DRIVE_TIMEOUT_SEC).value,
        }
        for name, value in positive.items():
            if not value > 0.0:
                raise ValueError(f"{name} must be positive, got {value}")
        self._settings = positive
        self.drive_timeout_sec = positive["drive_timeout"]
        self.decisions: Decisions = (decisions if decisions is not None
                                     else RuleDecisions(self.cfg))
        self.captions: list[str] = []
        self._episode = Episode()
        self._phase: Optional[str] = None
        self._safety_state: Optional[str] = None
        self._body: Optional[tuple[float, float, float, float]] = None  # x, y, facing, stamp
        self._hand: core.Point3 = (0.0, 0.0, 0.0)
        self._hand_extended = False
        self._take = False
        self._tracking_error: Optional[tuple[float, float]] = None  # m, rad
        # No route to /whole_body_mpc/target: targets go to the safety node.
        self.destroy_publisher(self._target_pub)
        self._offer_pub = self.create_publisher(PoseStamped, TARGET_REQUEST_TOPIC, 10)
        self._place_pub = self.create_publisher(PoseStamped, PLACE_TARGET_REQUEST_TOPIC, 10)
        self._target_pub = self._offer_pub  # reach_pose() and slide() publish on it
        self._scale_pub = self.create_publisher(Float64, SCALE_REQUEST_TOPIC, 10)
        self._box_pub = self.create_publisher(Float64MultiArray, BASE_BOX_TOPIC, 10)
        self._phase_pub = self.create_publisher(String, "~/phase", reach.LATCHED_QOS)
        self._release_client = self.create_client(Trigger, RELEASE_SERVICE)
        sub = self.create_subscription
        sub(PoseStamped, f"{MEASURED}/body_pose", self._on_body, 10)
        sub(PoseStamped, f"{MEASURED}/hand_pose", self._on_hand, 10)
        sub(Bool, f"{MEASURED}/hand_extended", self._on_extended, 10)
        sub(Bool, f"{MEASURED}/take", self._on_take, 10)
        sub(String, SAFETY_STATE_TOPIC, self._on_safety_state, reach.LATCHED_QOS)
        sub(Float64MultiArray, TRACKING_ERROR_TOPIC, self._on_tracking_error, 10)
        self._scale_timer = self.create_timer(1.0 / positive["scale_rate"], self._request_scale)

    # ---- measured person ---------------------------------------------------------
    def _on_body(self, msg: PoseStamped) -> None:
        facing = pick.yaw_of(msg.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        values = (msg.pose.position.x, msg.pose.position.y, facing)
        if all(map(math.isfinite, values)):
            self._body = (*values, stamp)

    def _on_hand(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._hand = (p.x, p.y, p.z)

    def _on_extended(self, msg: Bool) -> None:
        self._hand_extended = msg.data

    def _on_take(self, msg: Bool) -> None:
        self._take = msg.data

    def _on_safety_state(self, msg: String) -> None:
        self._safety_state = msg.data

    def _on_tracking_error(self, msg: Float64MultiArray) -> None:
        if len(msg.data) == 2:
            self._tracking_error = (msg.data[0], msg.data[1])

    def grasp(self, attach: bool) -> bool:
        """
        /grasp/set as PickPlaceDemo does, retrying a failed attach once.

        grasp_manager waits 1 s of wall time for the plugin's "attached"; in
        Gazebo that missed 3 of 10 picks (2026-11-25), a sim flake unrelated
        to the handover. A retry is safe: grasp_manager detaches first when the
        state is unknown, and confirms at once an attach that landed late.
        """
        if super().grasp(attach):
            return True
        if not (attach and self.execute):
            return False
        self.get_logger().warn("attach not confirmed: retrying once")
        return super().grasp(attach)

    def wait_for_state(self, wanted: str, timeout_sec: float) -> bool:
        """Wait as WbmpcReachDemo does; on a failure also log the MPC's last tracking error."""
        reached = super().wait_for_state(wanted, timeout_sec)
        if not reached and self._tracking_error is not None:
            distance, angle = self._tracking_error
            self.get_logger().error(f"last tracking error {distance * 1e3:.1f} mm, "
                                    f"{math.degrees(angle):.1f} deg")
        return reached

    def person(self) -> Optional[core.PersonState]:
        """Return the latest measured person, or None before the first body sample."""
        if self._body is None:
            return None
        x, y, facing, stamp = self._body
        return core.PersonState(x, y, facing, self._hand, self._hand_extended, self._take, stamp)

    def _request_scale(self) -> None:
        """Publish the decision's speed scale at the measured clearance (executing only)."""
        if not self.execute:
            return
        person, pose = self.person(), self._odom_pose()
        if person is None or pose is None:
            scale = 0.0
        else:
            clearance = core.footprint_clearance(pose, self.cfg.footprint, person, self.cfg)
            scale = float(self.decisions.request_scale(clearance))
        self._scale_pub.publish(Float64(data=scale))

    # ---- steps -----------------------------------------------------------------------
    def set_phase(self, name: str, every_entry: bool = False) -> None:
        """Publish the phase on change, or on every call (the recorder windows its metrics)."""
        if name == self._phase and not every_entry:
            return
        self._phase = name
        self.get_logger().info(f"phase {name}")
        if self.execute:
            self._phase_pub.publish(String(data=name))

    def _do(self, caption: str, action: Callable[[], bool]) -> None:
        """Run one step; StepFailed when it returns False."""
        self.captions.append(caption)
        if not action():
            raise StepFailed(caption)

    def _leg(self, pose: Pose2D, label: str) -> bool:
        if self._value("use_nav"):
            return self.navigate(pose, label)
        return self.drive_to_pose(pose, label)

    def drive(self, pose: Pose2D, label: str) -> bool:
        """
        Drive the base to `pose` along handover_core.route() around the measured person.

        Each pose of the route is one leg: a Nav2 goal with use_nav, else the
        scripted pose controller, whose path route() checks. Without a pose
        or a person the leg is direct.
        """
        start, person = self._odom_pose(), self.person()
        legs = ((pose,) if start is None or person is None
                else core.route(start, pose, person, self.cfg))
        for via in legs[:-1]:
            self.get_logger().info(f"{label}: via {pick.format_pose(via)}")
            if not self._leg(via, f"{label} (via)"):
                return False
        return self._leg(legs[-1], label)

    def choose(self, episode: Episode) -> Optional[core.Standoff]:
        """Ask the decisions for a standoff for the measured person."""
        person = self.person()
        if person is None:
            self.get_logger().warn("no measured person: no standoff")
            return None
        options = core.candidates(person, self.cfg)
        chosen = self.decisions.choose_standoff(person, options, episode.need_place)
        if chosen is not None:
            episode.standoff, episode.facing_at_choice = chosen, person.facing
            self.get_logger().info(
                f"standoff: bearing {math.degrees(chosen.bearing):.0f} deg, "
                f"{chosen.distance:.2f} m, base {pick.format_pose(chosen.pose)} "
                f"(offer {'ok' if chosen.offer_ok else 'no'}, "
                f"place {'ok' if chosen.place_ok else 'no'}; {len(options)} free)")
        return chosen

    def await_hand(self, episode: Episode) -> str:
        """Wait until the decisions say offer, place or re-choose; '' when out of time."""
        if not self.execute:
            self.get_logger().info("[plan-only] would wait for the hand (offer assumed)")
            return core.OFFER
        start, deadline = self._clock_sec(), time.time() + HAND_WAIT_WALL_SEC
        while rclpy.ok() and time.time() < deadline:
            person = self.person()
            episode.waited_sec = self._clock_sec() - start
            if person is not None and self.decisions.should_rechoose(
                    episode.facing_at_choice, person.facing, episode.rechoices):
                return RECHOOSE
            answer = self.decisions.wait_or_go(
                episode.waited_sec, person is not None and person.hand_extended)
            if answer != core.WAIT:
                return answer
            rclpy.spin_once(self, timeout_sec=reach.STATE_POLL_SEC)
        self.get_logger().error("no decision while waiting for the hand: out of time")
        return ""

    def publish_box(self, standoff: core.Standoff) -> bool:
        """Give whole_body_mpc the base box around the standoff (before the hand-over)."""
        if not self.execute:
            self.get_logger().info("[plan-only] would send the MPC base box (skipped)")
            return True
        self._box_pub.publish(Float64MultiArray(data=[float(v) for v in standoff.box]))
        return True

    def deliver(self, position: core.Point3, yaw: float, place: bool, label: str) -> bool:
        """MPC to the offer (or the table place) pose, through handover_safety."""
        self._target_pub = self._place_pub if place else self._offer_pub
        try:
            return self.reach_pose(position, yaw, label)
        finally:
            self._target_pub = self._offer_pub

    def set_down(self, target: tuple[core.Point3, float]) -> bool:
        """
        Lower the held object onto the table place pose from above it.

        Reaching for the place pose directly lowers the object while it crosses
        the table edge; so: MPC above the point, a straight slide down (both on
        ~/target_request), then the place pose itself on ~/place_target_request.
        """
        (x, y, z), yaw = target
        hover, place = (x, y, self.cfg.place_hover_height), (x, y, z)
        speed = self._settings["retreat_speed"]
        if not self.reach_pose(hover, yaw, "MPC: above the place"):
            return False
        if not self.slide(hover, place, yaw, speed, "MPC: lower to the table"):
            return False
        self._target_pub = self._place_pub
        try:
            return self.slide(place, place, yaw, speed, "MPC: set down")
        finally:
            self._target_pub = self._offer_pub

    def await_take(self) -> bool:
        """Wait for the measured take; False after take_timeout_sec."""
        if not self.execute:
            self.get_logger().info("[plan-only] would wait for the take (assumed)")
            return True
        timeout = self.cfg.take_timeout_sec
        start = self._clock_sec()
        deadline = time.time() + timeout + TAKE_WAIT_WALL_MARGIN_SEC
        while rclpy.ok() and time.time() < deadline:
            if self._take:
                return True
            if self._clock_sec() - start >= timeout:
                break
            rclpy.spin_once(self, timeout_sec=reach.STATE_POLL_SEC)
        self.get_logger().warn(f"no take within {timeout:.0f} s: keeping the object")
        return False

    def release(self) -> bool:
        """Ask handover_safety for the permit, then open the gripper and release the object."""
        if not self.execute:
            self.get_logger().info("[plan-only] would ask for the release permit (skipped)")
            return True
        response = self._call(self._release_client, Trigger.Request(), "release permit")
        if response is None:
            return False
        if not response.success:
            self.get_logger().error(f"release refused: {response.message}")
            return False
        self.get_logger().info(f"release permitted: {response.message}")
        return self.set_gripper(pick.GRIPPER_OPEN, "open") and self.grasp(False)

    def back_off(self, target: tuple[core.Point3, float]) -> bool:
        """Slide the TCP back retreat_distance along the approach (after the delivery)."""
        (x, y, z), yaw = target
        distance = self._settings["retreat_distance"]
        end = (x - distance * math.cos(yaw), y - distance * math.sin(yaw), z)
        return self.slide((x, y, z), end, yaw, self._settings["retreat_speed"], "MPC: back off")

    # ---- episode -----------------------------------------------------------------------
    def prefix_steps(self, plan: pick.TransferPlan) -> list[pick.Step]:
        """Return the scripted pick of pick_place_demo: known start state, pick, carry pose."""
        return [
            ("release object", lambda: self.grasp(False)),
            ("open gripper", lambda: self.set_gripper(pick.GRIPPER_OPEN, "open")),
            self._arm_step("'home'", plan.home),
            self._arm_step("carry pose", plan.carry),
            *self._pick_steps(plan),
        ]

    def _transit(self, episode: Episode) -> Optional[str]:
        """Choose, drive to the pre-standoff, wait; returns OFFER, PLACE or None (no standoff)."""
        while True:
            self.set_phase(TRANSIT, every_entry=True)
            standoff = self.choose(episode)
            if standoff is None:
                return None
            self._do("drive to the pre-standoff",
                     lambda: self.drive(standoff.pre_pose, "drive to the pre-standoff"))
            answer = self.await_hand(episode)
            if not answer:
                raise StepFailed("wait for the hand")
            if answer == RECHOOSE or (answer == core.PLACE and not standoff.place_ok):
                episode.rechoices += 1
                episode.need_place = episode.need_place or answer == core.PLACE
                why = "the facing changed" if answer == RECHOOSE else "the table is needed"
                self.get_logger().info(f"re-choosing the standoff ({episode.rechoices}): {why}")
                continue
            return answer

    def _deliver_and_release(self, episode: Episode, answer: str) -> str:
        person, standoff = self.person(), episode.standoff
        place = answer == core.PLACE
        target = (core.place_pose(person, standoff.pose, self.cfg) if place
                  else core.offer_pose(person, standoff.pose, self.cfg))
        if target is None:
            raise StepFailed("no table place pose")
        episode.target = target
        self.set_phase(PLACE_PHASE if place else OFFER_PHASE)
        if place:
            self._do("MPC: set down", lambda: self.set_down(target))
        else:
            offer = "MPC: offer"
            self._do(offer, lambda: self.deliver(target[0], target[1], False, offer))
        self.set_phase(WAIT_PHASE)
        if not place and not self.await_take():
            return TIMEOUT
        self.set_phase(RELEASE)
        self._do("release", self.release)
        return SET_DOWN if place else HANDOVER

    def _retreat(self, episode: Episode, plan: pick.TransferPlan) -> None:
        self.set_phase(RETREAT)
        self._do("MPC: back off", lambda: self.back_off(episode.target))
        self._do("arm back to MoveIt", lambda: self.switch_arm(stream=False))
        self._do("base back to navigation", lambda: self.set_base_owner(False))
        self._do(*self._arm_step("carry pose", plan.carry))
        pre = episode.standoff.pre_pose
        self._do("back to the pre-standoff", lambda: self.drive(pre, "back to the pre-standoff"))
        self._do("drive away", lambda: self.drive(self._away(), "drive away"))

    def _away(self) -> Pose2D:
        return reach._triple("nav_away_pose", self._value("nav_away_pose"))

    def episode(self, plan: pick.TransferPlan) -> str:
        """Run the episode; returns its outcome, raises StepFailed for a failed step."""
        episode = self._episode
        self.set_phase(PICK)
        for caption, action in self.prefix_steps(plan):
            self._do(caption, action)
        self.set_phase(AWAY)
        self._do("drive away", lambda: self.drive(self._away(), "drive away"))
        answer = self._transit(episode)
        if answer is None:
            return NO_STANDOFF
        self.set_phase(APPROACH)
        standoff = episode.standoff
        self._do("MPC base box", lambda: self.publish_box(standoff))
        self._do("base to the whole-body MPC", lambda: self.set_base_owner(True))
        self._do("arm to streamed setpoints", lambda: self.switch_arm(stream=True))
        outcome = self._deliver_and_release(episode, answer)
        self._retreat(episode, plan)
        return outcome

    def _wait_reach_servers(self) -> None:
        """Wait for the gate, controllers, whole_body_mpc, handover_safety, a person, [Nav2]."""
        waits = [
            (reach.GATE_SERVICE, lambda: self._gate_client.wait_for_service(
                reach.SERVER_WAIT_SEC)),
            (reach.SWITCH_SERVICE, lambda: self._switch_client.wait_for_service(
                reach.SERVER_WAIT_SEC)),
            ("whole_body_mpc", lambda: self._spin_until(
                lambda: self._mpc_state is not None, reach.MPC_WAIT_SEC)),
            (RELEASE_SERVICE, lambda: self._release_client.wait_for_service(
                reach.SERVER_WAIT_SEC)),
            ("handover_safety", lambda: self._spin_until(
                lambda: self._offer_pub.get_subscription_count() > 0
                and self._place_pub.get_subscription_count() > 0, reach.SERVER_WAIT_SEC)),
            # Nothing may move before the gate obeys the safety node's scale.
            ("handover_safety governing the gate", lambda: self._spin_until(
                lambda: self._safety_state is not None
                and self._safety_state != SAFETY_UNGOVERNED, reach.SERVER_WAIT_SEC)),
            ("a measured person sample", lambda: self._spin_until(
                lambda: self._body is not None, self._settings["person_wait"])),
        ]
        if self._value("use_nav"):
            waits.append((reach.NAV_ACTION, lambda: self._nav.wait_for_server(
                reach.NAV_SERVER_WAIT_SEC)))
        for name, wait in waits:
            self.get_logger().info(f"Waiting for {name} ...")
            if not wait():
                raise RuntimeError(f"{name} not available (sim_bringup handover_gate:=true, "
                                   "handover_baseline.launch.py?)")

    def _failure_outcome(self, failure: StepFailed) -> str:
        stopped = (self._safety_state or "").startswith(SAFETY_STOP)
        detail = f" (handover_safety {self._safety_state})" if stopped else ""
        self.get_logger().error(f"step '{failure.caption}' failed — aborting{detail}")
        return STOPPED if stopped else FAILED_OUTCOME

    def run(self) -> bool:
        """Run one episode, restoring on any exit; True when it ended with DONE."""
        try:
            plan = self.transfer_plan()  # malformed scene values fail before any wait
            self._wait_servers()
            if self.execute:
                self._wait_reach_servers()
        except (Exception, KeyboardInterrupt):
            self.set_phase(FAILED + "start")  # the recorder still writes a row
            raise
        self._episode = Episode()
        outcome, final = FAILED_OUTCOME, None
        try:
            outcome = self.episode(plan)
            final = DONE + outcome
            return True
        except StepFailed as failure:
            outcome = self._failure_outcome(failure)
            final = FAILED + failure.caption
            return False
        finally:
            if final is None:  # an exception or Ctrl-C
                final = FAILED + (self.captions[-1] if self.captions else "start")
            self.set_phase(final)
            self.get_logger().info(
                f"handover result: outcome={outcome} rechoices={self._episode.rechoices} "
                f"wait={self._episode.waited_sec:.1f} s")
            self.restore()


def main() -> None:
    reach.main(HandoverBaseline)


if __name__ == "__main__":
    main()
