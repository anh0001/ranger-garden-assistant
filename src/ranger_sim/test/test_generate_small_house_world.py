"""
Tests for tools/generate_small_house_world.py (offline).

They use the committed copy tools/aws_small_house/small_house.world, so they
need neither the network nor models/third_party.
"""

import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1]

# Where the robot drives and works: from behind the spawn pose to the table
# edge (world x 0.90), one base width either side of its line (y = 0).
CORRIDOR_X = (-0.45, 0.85)
CORRIDOR_Y = (-0.45, 0.45)
GRASPABLES = ("vitamin_bottle", "supplement_bottle", "sprinkles_jar")  # the template's objects


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gen = _load("generate_small_house_world", PKG / "tools" / "generate_small_house_world.py")


def _poses(world, tag):
    """{name: [x, y, z, roll, pitch, yaw]} of the world's `tag` children."""
    poses = {}
    for element in world.findall(tag):
        name = element.findtext("name") if tag == "include" else element.get("name")
        poses[name.strip()] = [float(v) for v in element.findtext("pose").split()]
    return poses


def _wrapped(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _assert_same_pose(actual, expected, atol=1e-6):
    np.testing.assert_allclose(actual[:3], expected[:3], atol=atol)
    for a, e in zip(actual[3:], expected[3:]):
        assert abs(_wrapped(a - e)) <= atol, (actual, expected)


@pytest.fixture(scope="module")
def source():
    return ET.parse(gen.SOURCE_WORLD).getroot().find("world")


@pytest.fixture(scope="module")
def generated_text():
    return gen.build_world(gen.SOURCE_WORLD.read_text(), gen.TEMPLATE.read_text())


@pytest.fixture(scope="module")
def generated(generated_text):
    return ET.fromstring(generated_text).find("world")


@pytest.fixture(scope="module")
def transform(source):
    return gen.anchor_transform(_poses(source, "model")[gen.ANCHOR_MODEL], gen.ANCHOR_WORLD_POSE)


def test_pose_matrix_round_trips():
    pose = (1.0, -2.0, 0.5, 0.1, -0.2, 2.5)
    _assert_same_pose(gen.matrix_pose(gen.pose_matrix(*pose)), pose, atol=1e-12)


def test_anchor_lands_on_the_target_pose(source, transform):
    anchor = gen.pose_matrix(*_poses(source, "model")[gen.ANCHOR_MODEL])

    _assert_same_pose(gen.matrix_pose(transform @ anchor), gen.ANCHOR_WORLD_POSE)


def test_anchor_target_is_the_inline_table(generated):
    table = next(m for m in generated.findall("model") if m.get("name") == "table")
    table_pose = [float(v) for v in table.findtext("pose").split()]

    _assert_same_pose(table_pose, gen.ANCHOR_WORLD_POSE)


def test_transform_preserves_distances_between_house_models(source, generated):
    before, after = _poses(source, "model"), _poses(generated, "include")
    for a, b in (("Refrigerator_01_001", "Bed_01_001"), ("SofaC_01_001", "ChairA_01_001")):
        d_before = np.linalg.norm(np.subtract(before[a][:3], before[b][:3]))
        d_after = np.linalg.norm(np.subtract(after[a][:3], after[b][:3]))
        assert d_after == pytest.approx(d_before, abs=1e-5), (a, b)


def test_dropped_models_are_absent_and_the_rest_present(source, generated):
    house = set(_poses(source, "model"))
    included = set(_poses(generated, "include"))

    assert not included & set(gen.DROP_MODELS)
    assert house - set(gen.DROP_MODELS) <= included


def test_an_unknown_drop_is_an_error(monkeypatch):
    monkeypatch.setattr(gen, "DROP_MODELS", {"NoSuchModel_01_001": "typo"})

    with pytest.raises(ValueError, match="NoSuchModel_01_001"):
        gen.build_world(gen.SOURCE_WORLD.read_text(), gen.TEMPLATE.read_text())


def test_no_house_model_origin_in_the_robot_corridor(generated):
    poses = _poses(generated, "include")
    for name in set(poses) - set(GRASPABLES):
        x, y = poses[name][:2]
        inside = CORRIDOR_X[0] <= x <= CORRIDOR_X[1] and CORRIDOR_Y[0] <= y <= CORRIDOR_Y[1]
        assert not inside, f"{name} stands in the robot's corridor at ({x:.2f}, {y:.2f})"


def _table_top(world):
    """World z of the inline `table`'s top_collision surface."""
    table = next(m for m in world.findall("model") if m.get("name") == "table")
    top = next(c for c in table.iter("collision") if c.get("name") == "top_collision")
    thickness = float(top.findtext("geometry/box/size").split()[2])
    return (float(table.findtext("pose").split()[2]) + float(top.findtext("pose").split()[2])
            + thickness / 2)


@pytest.mark.parametrize("name", [*gen.ON_TABLE_MODELS, *GRASPABLES])
def test_everything_on_the_table_rests_on_its_top(generated, name):
    # Origins at the bottom: the GSO wrappers and the re-seated AWS tableware.
    assert _poses(generated, "include")[name][2] == pytest.approx(_table_top(generated), abs=1e-6)
    assert gen.TABLE_TOP_M == pytest.approx(_table_top(generated), abs=1e-6)


def test_house_lights_are_carried_over_with_the_house(source, generated, transform):
    before, after = _poses(source, "light"), _poses(generated, "light")
    for name, pose in before.items():
        expected = gen.matrix_pose(transform @ gen.pose_matrix(*pose))
        _assert_same_pose(after[name], expected)
    assert {"sun", "fill_light"} <= set(after), "the template's own lights are kept"


def test_generated_poses_have_no_empty_relative_to(generated):
    assert all(p.get("relative_to") is None for p in generated.iter("pose"))


def test_committed_world_is_up_to_date(generated_text):
    assert gen.OUTPUT.read_text() == generated_text, (
        "worlds/small_house_world.sdf is stale; run tools/generate_small_house_world.py"
    )
