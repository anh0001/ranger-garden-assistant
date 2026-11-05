#!/usr/bin/env python3
"""
Solve a level PiPER waypoint for the pick-and-place demo (planar IK).

  python3 src/ranger_sim/tools/solve_level_waypoint.py X Z
  python3 src/ranger_sim/tools/solve_level_waypoint.py X Z --seed 0.9 -1.4 0.6

Finds the arm joints that put piper_link6 at (X, 0, Z) in base_footprint with
the gripper level: approach axis (link6 z) along +x, fingers closing along y.
Joints 1, 4 and 6 stay at 0; joints 2, 3 and 5 are solved with
scipy.optimize.least_squares within their URDF limits, on the forward
kinematics of test/test_pick_place_geometry.py over urdf/ranger_gazebo.urdf.xacro
(source the install space: the xacro resolves $(find ...)). Prints the joint
6-vector rounded to 3 decimals, the TCP and the wrist camera's optical axis;
exits 1 if no level pose reaches the target.

This is how the demo's level waypoints were derived. Reference solutions:

  link6 (0.200, 0.950) -> [0.0, 0.719, -1.496, 0.0, 0.864, 0.0]  TCP (0.340, 0, 0.950)  CARRY
  link6 (0.190, 0.842) -> [0.0, 0.415, -0.944, 0.0, 0.616, 0.0]  arm-only standoff (garden)
  link6 (0.290, 0.842) -> [0.0, 0.732, -0.943, 0.0, 0.298, 0.0]  GRASP (garden)
  link6 (0.370, 0.974) -> [0.0, 1.221, -1.766, 0.0, 0.632, 0.0]  place GRASP, counter (house)
  link6 (0.370, 1.034) -> [0.0, 1.455, -2.339, 0.0, 0.971, 0.0]  place HOVER (house)

Re-run it if the arm mount or the D405 calibration changes.
"""

import argparse
import importlib.util
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from urdf_parser_py.urdf import URDF
import xacro

PKG = Path(__file__).resolve().parents[1]
SIM_XACRO = PKG / "urdf" / "ranger_gazebo.urdf.xacro"
GEOMETRY_TEST = PKG / "test" / "test_pick_place_geometry.py"
SOLVED_JOINTS = (1, 2, 4)  # indices of piper_joint2, 3, 5 in the 6-vector
DEFAULT_SEED = (0.732, -0.943, 0.298)  # the garden GRASP: level, elbow up
MAX_COST = 1e-8  # least_squares cost (half the sum of squared residuals)
DECIMALS = 3


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_robot() -> URDF:
    """Return the sim robot model (urdf/ranger_gazebo.urdf.xacro)."""
    return URDF.from_xml_string(xacro.process_file(str(SIM_XACRO)).toxml())


def joint_bounds(robot: URDF, names: list[str]) -> tuple[list[float], list[float]]:
    """URDF (lower, upper) limits of the solved joints."""
    limits = [robot.joint_map[names[i]].limit for i in SOLVED_JOINTS]
    return [lim.lower for lim in limits], [lim.upper for lim in limits]


def solve(
    geometry, robot: URDF, x: float, z: float, seed=DEFAULT_SEED
) -> tuple[list[float], float]:
    """Level arm joints with link6 at (x, 0, z), rounded, and the solver cost."""

    def joints(q) -> list[float]:
        full = [0.0] * 6
        for index, value in zip(SOLVED_JOINTS, q):
            full[index] = float(value)
        return full

    def residuals(q) -> list[float]:
        t = geometry._fk(robot, "piper_link6", joints(q))
        return [t[0, 3] - x, t[2, 3] - z, t[0, 2] - 1.0, t[2, 2]]

    result = least_squares(residuals, seed, bounds=joint_bounds(robot, geometry.demo.ARM_JOINTS))
    # + 0.0 turns a rounded -0.0 into 0.0
    return [round(v, DECIMALS) + 0.0 for v in joints(result.x)], float(result.cost)


def describe(geometry, robot: URDF, arm: list[float]) -> str:
    """TCP and wrist camera optical axis (base_footprint) of the arm joints `arm`."""
    tcp = geometry._fk(robot, "piper_tcp", arm)[:3, 3]
    axis = geometry._fk(robot, "piper_camera_optical_frame", arm)[:3, 2]
    return f"TCP {np.round(tcp, 3).tolist()}  camera optical axis {np.round(axis, 3).tolist()}"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Solve a level PiPER waypoint (planar IK).")
    parser.add_argument("x", type=float, help="link6 x in base_footprint (m)")
    parser.add_argument("z", type=float, help="link6 z in base_footprint (m)")
    parser.add_argument("--seed", type=float, nargs=3, default=DEFAULT_SEED,
                        metavar=("J2", "J3", "J5"), help="initial guess (rad)")
    args = parser.parse_args(argv)

    geometry = _load("test_pick_place_geometry", GEOMETRY_TEST)
    robot = load_robot()
    arm, cost = solve(geometry, robot, args.x, args.z, args.seed)
    print(arm)
    print(describe(geometry, robot, arm))
    if cost > MAX_COST:
        print(f"no level pose at link6 ({args.x}, {args.z}): cost {cost:.2e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
