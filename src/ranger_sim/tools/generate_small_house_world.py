#!/usr/bin/env python3
"""
Generate worlds/small_house_world.sdf from the AWS Small House and our template.

  python3 src/ranger_sim/tools/generate_small_house_world.py           # write the world
  python3 src/ranger_sim/tools/generate_small_house_world.py --check   # exit 1 if stale

The house (tools/aws_small_house/small_house.world, MIT-0, a verbatim copy of
TeamSOBITS' Fortress port) is moved by one rigid transform that puts its kitchen
table where the template's inline `table` stands; the template's inline
`kitchen_counter` replaces the kitchen cabinet. The robot is not moved: it
spawns at the world origin with yaw 0, because pick_place_demo drives along
/odom x and sim_bringup publishes an identity map -> odom. Every house model
becomes a flat static <include>; its lights are moved with it. The output is
deterministic (source order, fixed float format), so --check is stable.
"""

import argparse
import copy
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

TOOLS = Path(__file__).resolve().parent
PKG = TOOLS.parent
SOURCE_WORLD = TOOLS / "aws_small_house" / "small_house.world"
TEMPLATE = TOOLS / "small_house_world.template.sdf"
OUTPUT = PKG / "worlds" / "small_house_world.sdf"
MARKER = "<!-- GENERATED: HOUSE -->"
INDENT = "    "

ANCHOR_MODEL = "KitchenTable_01_001"
ANCHOR_WORLD_POSE = (1.25, -0.30, 0.0, 0.0, 0.0, -math.pi)  # = the template's `table`
DROP_MODELS = {
    "KitchenTable_01_001": "replaced by the inline `table` (collision at the visual top)",
    "ChairA_01_002": "stands in the robot's approach slot",
    "KitchenCabinet_01_001": (
        "replaced by the inline `kitchen_counter` (collision at the visual top)"
    ),
}
# House models standing on the replaced table: the AWS world rests them on its
# collision-mesh top (0.8185 m), which would leave them floating 1.7 cm above
# the visible top. They are re-seated on the inline table's top.
TABLE_TOP_M = 0.801
ON_TABLE_MODELS = ("Tableware_01_001",)
# Top of the inline `kitchen_counter`, the place surface of the transfer: its
# visible top is 0.9044 m, rounded down as the table's (0.8014 -> 0.801).
COUNTER_TOP_M = 0.904


def pose_matrix(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    """4x4 transform of an SDF pose (fixed-axis roll-pitch-yaw: R = Rz @ Ry @ Rx)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    t = np.eye(4)
    t[:3, :3] = rz @ ry @ rx
    t[:3, 3] = (x, y, z)
    return t


def matrix_pose(t: np.ndarray) -> tuple[float, ...]:
    """Inverse of pose_matrix: (x, y, z, roll, pitch, yaw)."""
    r = t[:3, :3]
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    roll = math.atan2(r[2, 1], r[2, 2])
    yaw = math.atan2(r[1, 0], r[0, 0])
    return (float(t[0, 3]), float(t[1, 3]), float(t[2, 3]), roll, pitch, yaw)


def anchor_transform(anchor_house_pose, anchor_world_pose) -> np.ndarray:
    """Transform that moves the house so its anchor lands on `anchor_world_pose`."""
    return pose_matrix(*anchor_world_pose) @ np.linalg.inv(pose_matrix(*anchor_house_pose))


def _pose_of(element: ET.Element) -> list[float]:
    return [float(v) for v in element.findtext("pose").split()]


def _fmt(value: float) -> str:
    text = f"{value:.6f}"
    return "0.000000" if text == "-0.000000" else text


def _pose_text(pose) -> str:
    return " ".join(_fmt(v) for v in pose)


def house_includes(source_root: ET.Element, t: np.ndarray, drop) -> list[str]:
    """One flat static <include> per house model not in `drop`, moved by `t`."""
    lines = []
    for model in source_root.find("world").findall("model"):
        name = model.get("name")
        if name in drop:
            continue
        pose = list(matrix_pose(t @ pose_matrix(*_pose_of(model))))
        if name in ON_TABLE_MODELS:
            pose[2] = TABLE_TOP_M
        uri = model.findtext("include/uri").strip()
        lines.append(
            f"<include><name>{name}</name><uri>{uri}</uri><static>true</static>"
            f"<pose>{_pose_text(pose)}</pose></include>"
        )
    return lines


def house_lights(source_root: ET.Element, t: np.ndarray) -> list[str]:
    """
    Return the house lights, moved by `t`.

    Only the pose changes: <direction> is expressed in the light's own frame,
    so rotating the pose turns it too.
    """
    blocks = []
    for light in source_root.find("world").findall("light"):
        moved = copy.deepcopy(light)
        pose = moved.find("pose")
        pose.attrib.pop("relative_to", None)
        pose.text = _pose_text(matrix_pose(t @ pose_matrix(*_pose_of(light))))
        moved.tail = None
        ET.indent(moved, space="  ")
        blocks.append(ET.tostring(moved, encoding="unicode"))
    return blocks


def _check_drops(source_root: ET.Element) -> None:
    names = {m.get("name") for m in source_root.find("world").findall("model")}
    unknown = sorted(set(DROP_MODELS) - names)
    if unknown:
        raise ValueError(f"DROP_MODELS not in the source world: {', '.join(unknown)}")
    missing = sorted(set(ON_TABLE_MODELS) - names)
    if missing:
        raise ValueError(f"ON_TABLE_MODELS not in the source world: {', '.join(missing)}")


def build_world(source_text: str, template_text: str) -> str:
    """Return the template with the moved house models and lights at MARKER."""
    if template_text.count(MARKER) != 1:
        raise ValueError(f"the template must contain {MARKER!r} exactly once")
    source_root = ET.fromstring(source_text)
    _check_drops(source_root)
    anchor = next(
        m for m in source_root.find("world").findall("model") if m.get("name") == ANCHOR_MODEL
    )
    t = anchor_transform(_pose_of(anchor), ANCHOR_WORLD_POSE)
    dropped = "; ".join(f"{name} ({why})" for name, why in DROP_MODELS.items())
    header = (
        "<!-- GENERATED by tools/generate_small_house_world.py from "
        "tools/aws_small_house/small_house.world (MIT-0): do not edit. "
        f"Dropped: {dropped}. Re-seated on the table top: {', '.join(ON_TABLE_MODELS)}. -->"
    )
    lines = [header, *house_includes(source_root, t, DROP_MODELS)]
    for block in house_lights(source_root, t):
        lines.extend(block.splitlines())
    return template_text.replace(MARKER, ("\n" + INDENT).join(lines))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip())
    parser.add_argument(
        "--check", action="store_true", help="exit 1 if the committed world is stale"
    )
    args = parser.parse_args(argv)
    try:
        world = build_world(SOURCE_WORLD.read_text(), TEMPLATE.read_text())
    except (OSError, ValueError, ET.ParseError) as exc:
        print(f"generate_small_house_world: {exc}", file=sys.stderr)
        return 1
    relative = OUTPUT.relative_to(PKG)
    if args.check:
        if not OUTPUT.exists() or OUTPUT.read_text() != world:
            print(
                f"{relative} is stale; run tools/generate_small_house_world.py",
                file=sys.stderr,
            )
            return 1
        print(f"{relative} is up to date")
        return 0
    OUTPUT.write_text(world)
    print(f"wrote {relative}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
