#!/usr/bin/env python3
"""
Pick-and-place demo with the pick done by the whole-body MPC (Ranger Mini 3.0 + PiPER, Gazebo).

The transfer of scripts/pick_place_demo.py with another pick
(docs/sim_wbmpc_pick.md):
  1. drive to the table: the scripted base leg to the pick standoff, moved
     by approach_error (the stand-in for a coarse navigation stop), arm in
     the carry pose. The approach is timed from here.
  2. Hand the base and then the arm to the MPC (as wbmpc_reach_demo.py).
  3. MPC: pre-grasp: level gripper at grasp height, standoff_m short of the
     grasp along the pick heading; base and arm move together.
  4. MPC: slide in: the target moves along the approach at SLIDE_SPEED_MPS
     to the grasp. The MPC minimises the pose error, it does not promise a
     straight TCP path: a slowly moving target keeps the TCP on the line.
  5. attach object, close gripper (the MPC holds in REACHED). The approach
     result line (mode=mpc) is logged after the attach.
  6. MPC: lift: the target moves straight up to the hover height. After an
     MPC pick the base is not where hover_joints assume, so a joint-space
     lift would swing the held object sideways.
  7. Hand back: arm to MoveIt, base to navigation, carry pose (MoveIt).
     Route, place and return are pick_place_demo's, unchanged.
With approach_only the MPC sets the object back down and slides back out
(lift and slide in reversed), then hands back; carry pose, return, 'home'.

Targets in odom, gripper level, approach along the pick heading: grasp =
pick base pose + grasp_tcp (the FK of grasp_joints), pre-grasp standoff_m
back from it, lift at hover_tcp's height. whole_body_mpc gets q_nominal =
grasp_joints, base_box = the scene's mpc_base_box and a 3 mm position
tolerance from the launch (launch/pick_place_nodes.py). Whatever ends the
run, restore() hands the base back to navigation and the arm to the
trajectory controller and stops the base (wbmpc_reach_demo.py).

Safety
------
`execute` defaults to **False** (plan-only): MoveIt plans, nothing else is
called, published or sent. `execute:=true` needs sim_bringup with
handover_gate:=true. Simulation only: nothing here reaches a CAN bus or a
real-robot topic.
"""

from dataclasses import dataclass
import importlib.util
from pathlib import Path
from typing import Optional


def _load_sibling(name: str):
    # resolve(): with --symlink-install this file is a link into the source tree.
    path = Path(__file__).resolve().with_name(f"{name}.py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


reach = _load_sibling("wbmpc_reach_demo")
pick = reach.pick  # the pick_place_demo module wbmpc_reach_demo builds on

Point = tuple[float, float, float]
Step = pick.Step
TransferPlan = pick.TransferPlan

# Target speed along the approach: in the planning spike (the MPC core on a
# kinematic plant with 0.1 s arm lag) the TCP left the line by at most
# 1.7 mm sideways and 1.9 mm vertically at this speed; the open fingers clear
# the object by 8-10 mm per side.
SLIDE_SPEED_MPS = 0.03


@dataclass(frozen=True)
class PickTargets:
    """TCP targets (odom) of the MPC pick."""

    pre_grasp: Point  # grasp height, standoff_m short of the grasp
    grasp: Point
    lift: Point  # straight above the grasp at the hover height


def pick_targets(plan: TransferPlan) -> PickTargets:
    """Return the MPC targets of `plan`: the pre-grasp is back from the grasp as the standoff."""
    gx, gy, gz = plan.grasp_target
    (px, py, _), (sx, sy, _) = plan.pick, plan.pick_standoff
    return PickTargets(
        pre_grasp=(gx - (px - sx), gy - (py - sy), gz),
        grasp=(gx, gy, gz),
        lift=(gx, gy, plan.hover_tcp_z),
    )


class WbmpcPickPlaceDemo(reach.WbmpcReachDemo):
    """pick_place_demo's transfer with the pick done by whole_body_mpc."""

    APPROACH_MODE = "mpc"

    def __init__(self, **node_kwargs) -> None:
        # The video recorder captions the logs of pick_place_demo.
        super().__init__("pick_place_demo", **node_kwargs)

    # ---- steps -----------------------------------------------------------------------
    def _slide_step(self, caption: str, start: Point, end: Point, plan: TransferPlan) -> Step:
        return caption, lambda: self.slide(start, end, plan.pick[2], SLIDE_SPEED_MPS, caption)

    def _hand_over(self) -> list[Step]:
        # Base still first, then the arm; the targets go out only after both.
        return [
            ("base to the whole-body MPC", lambda: self.set_base_owner(True)),
            ("arm to streamed setpoints", lambda: self.switch_arm(stream=True)),
        ]

    def _hand_back(self) -> list[Step]:
        # MoveIt only once its controller is active again.
        return [
            ("arm back to MoveIt", lambda: self.switch_arm(stream=False)),
            ("base back to navigation", lambda: self.set_base_owner(False)),
        ]

    def _grasp_steps(self, plan: TransferPlan) -> list[Step]:
        """First leg, hand-over, pre-grasp, slide in, attach, close, lift (no corrective leg)."""
        targets = pick_targets(plan)
        heading = plan.pick[2]
        return [
            self._first_leg(plan),
            *self._hand_over(),
            ("MPC: pre-grasp",
             lambda: self.reach_pose(targets.pre_grasp, heading, "MPC: pre-grasp")),
            self._slide_step("MPC: slide in", targets.pre_grasp, targets.grasp, plan),
            # Attach before closing, as the scripted pick does.
            self._attach_step(plan),
            ("close gripper", lambda: self.set_gripper(plan.gripper_closed, "close")),
            self._slide_step("MPC: lift", targets.grasp, targets.lift, plan),
        ]

    def _pick_steps(self, plan: TransferPlan) -> list[Step]:
        return [*self._grasp_steps(plan), *self._hand_back(),
                self._arm_step("carry pose", plan.carry)]

    def _put_back_steps(self, plan: TransferPlan) -> list[Step]:
        """approach_only: the lift and the slide in reversed, then hand back."""
        targets = pick_targets(plan)
        return [
            self._slide_step("MPC: set down", targets.lift, targets.grasp, plan),
            ("open gripper", lambda: self.set_gripper(pick.GRIPPER_OPEN, "open")),
            ("release object", lambda: self.grasp(False)),
            self._slide_step("MPC: back off", targets.grasp, targets.pre_grasp, plan),
            *self._hand_back(),
            self._arm_step("carry pose", plan.carry),
        ]

    def sequence(self, plan: TransferPlan, start: Optional[pick.Pose2D]) -> list[Step]:
        """Return pick_place_demo's steps, not the reach's: only the pick steps differ."""
        return pick.PickPlaceDemo.sequence(self, plan, start)

    # ---- run -----------------------------------------------------------------------
    def run(self) -> bool:
        """Run the transfer, restoring on any exit; True when every step succeeded."""
        plan = self.transfer_plan()  # malformed scene values fail before any wait
        self._wait_servers()
        if self.execute:
            self._wait_reach_servers()
        start = self._wait_for_odom_pose()
        if self.execute and start is None:
            raise RuntimeError("no /odom received: cannot drive the base")
        try:
            for name, action in self.sequence(plan, start):
                if not action():
                    self.get_logger().error(f"step '{name}' failed — aborting")
                    return False
            self.get_logger().info(
                "Pick-and-place sequence complete "
                f"({'executed' if self.execute else 'plan-only'})."
            )
            return True
        finally:
            self.restore()


def main() -> None:
    reach.main(WbmpcPickPlaceDemo)


if __name__ == "__main__":
    main()
