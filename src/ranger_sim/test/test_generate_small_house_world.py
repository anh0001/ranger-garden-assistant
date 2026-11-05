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

# Where the robot drives and works, ((x_min, x_max), (y_min, y_max)) in the
# world, one base width either side of its line: from the kitchen turn at
# x = -4.0 along y = 0 to the table edge (world x 0.90), and from that turn
# south along x = -4.0 to the kitchen counter's front face (y -2.416).
CORRIDORS = (
    ((-4.45, 0.85), (-0.45, 0.45)),
    ((-4.45, -3.55), (-2.40, 0.45)),
)
GRASPABLES = ("vitamin_bottle", "supplement_bottle", "sprinkles_jar")  # the template's objects
# Static house models standing on the counter, origin at their bottom. The
# AWS world sets them up to 1.6 mm into its visible top (0.9044 m).
ON_COUNTER_MODELS = ("SeasoningBox_01_001", "CookingBench_01_001")
COUNTER_RESTING_TOLERANCE_M = 0.002
COUNTER_VISUAL = "aws_KitchenCabinet_01_visual.DAE"


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


@pytest.mark.parametrize("corridor", CORRIDORS, ids=["west", "kitchen"])
def test_no_house_model_origin_in_the_robot_corridor(generated, corridor):
    (x_min, x_max), (y_min, y_max) = corridor
    poses = _poses(generated, "include")
    for name in set(poses) - set(GRASPABLES):
        x, y = poses[name][:2]
        inside = x_min <= x <= x_max and y_min <= y <= y_max
        assert not inside, f"{name} stands in the robot's corridor at ({x:.2f}, {y:.2f})"


def _model(world, name):
    return next(m for m in world.findall("model") if m.get("name") == name)


def _top(world, name):
    """World z of the inline model `name`'s top_collision surface."""
    model = _model(world, name)
    top = next(c for c in model.iter("collision") if c.get("name") == "top_collision")
    thickness = float(top.findtext("geometry/box/size").split()[2])
    return (float(model.findtext("pose").split()[2]) + float(top.findtext("pose").split()[2])
            + thickness / 2)


@pytest.mark.parametrize("name", [*gen.ON_TABLE_MODELS, *GRASPABLES])
def test_everything_on_the_table_rests_on_its_top(generated, name):
    # Origins at the bottom: the GSO wrappers and the re-seated AWS tableware.
    top = _top(generated, "table")
    assert _poses(generated, "include")[name][2] == pytest.approx(top, abs=1e-6)
    assert gen.TABLE_TOP_M == pytest.approx(top, abs=1e-6)


def test_kitchen_counter_is_inline_and_the_cabinet_dropped(generated):
    counter = _model(generated, "kitchen_counter")
    uris = [u.text for u in counter.iter("uri")]

    assert any(c.get("name") == "top_collision" for c in counter.iter("collision"))
    assert not [c for c in counter.iter("collision") if c.find("geometry/mesh") is not None]
    assert len(uris) == 1 and uris[0].endswith(COUNTER_VISUAL)
    assert "KitchenCabinet_01_001" in gen.DROP_MODELS
    assert "KitchenCabinet_01_001" not in _poses(generated, "include")


def _pose_of(element):
    return [float(v) for v in element.findtext("pose").split()]


def test_counter_visual_stays_where_the_house_had_the_cabinet(source, generated, transform):
    counter = _model(generated, "kitchen_counter")
    placed = gen.pose_matrix(*_pose_of(counter)) @ gen.pose_matrix(*_pose_of(counter.find(
        "link/visual")))
    cabinet = gen.pose_matrix(*_poses(source, "model")["KitchenCabinet_01_001"])

    # The template's copy of the pose is 6-decimal text: 1e-6 per value.
    _assert_same_pose(gen.matrix_pose(placed), gen.matrix_pose(transform @ cabinet), atol=2e-6)


def test_counter_top_constant_matches_the_template(generated):
    assert gen.COUNTER_TOP_M == pytest.approx(_top(generated, "kitchen_counter"), abs=1e-6)


@pytest.mark.parametrize("name", ON_COUNTER_MODELS)
def test_everything_on_the_counter_rests_on_its_top(generated, name):
    z = _poses(generated, "include")[name][2]

    assert z == pytest.approx(gen.COUNTER_TOP_M, abs=COUNTER_RESTING_TOLERANCE_M)


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
