"""
Tests for the pick-and-place scenes and their launch helper.

Covers config/pick_place_scenes.yaml, launch/pick_place_scenes.py and the
world asset check in launch/gazebo.launch.py. No simulator, no network, no
models/third_party.
"""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
SCENES_FILE = PKG / "config" / "pick_place_scenes.yaml"
SCENES = yaml.safe_load(SCENES_FILE.read_text())
GRASPABLE = PKG / "config" / "graspable_objects.yaml"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gazebo_launch = _load("gazebo_launch", PKG / "launch" / "gazebo.launch.py")
scenes = _load("pick_place_scenes", PKG / "launch" / "pick_place_scenes.py")


def _context(**configurations):
    context = LaunchContext()
    context.launch_configurations.update(configurations)
    return context


# ---- config/pick_place_scenes.yaml -----------------------------------------


def test_the_helper_reads_the_package_scenes_file():
    assert Path(scenes.SCENES_FILE).resolve() == SCENES_FILE


def test_garden_is_a_scene():
    assert "garden" in scenes.load_scenes()


@pytest.mark.parametrize("name", sorted(SCENES))
def test_every_scene_has_the_required_keys(name):
    assert scenes.REQUIRED_KEYS <= set(SCENES[name]), (
        f"{name} lacks {sorted(scenes.REQUIRED_KEYS - set(SCENES[name]))}"
    )


@pytest.mark.parametrize("name", sorted(SCENES))
def test_scene_world_files_and_rigs_exist(name):
    assert (PKG / "worlds" / SCENES[name]["world_file"]).is_file()
    assert (PKG / "models" / SCENES[name]["rig_file"]).is_file()


@pytest.mark.parametrize("name", sorted(SCENES))
def test_scene_world_name_matches_the_sdf(name):
    world = ET.parse(PKG / "worlds" / SCENES[name]["world_file"]).getroot().find("world")
    assert world.get("name") == SCENES[name]["world_name"]


@pytest.mark.parametrize("name", sorted(SCENES))
def test_scene_objects_are_graspable(name):
    objects = {o["name"] for o in yaml.safe_load(GRASPABLE.read_text())["objects"]}
    assert SCENES[name]["object_name"] in objects


@pytest.mark.parametrize("name", sorted(SCENES))
def test_scene_numbers_are_floats(name):
    # ParameterValue YAML-parses the joint lists: an int would make the
    # parameter an integer array and the demo node reject it.
    cfg = SCENES[name]
    values = [cfg["base_approach_x"], cfg["gripper_closed"], *cfg["hover_joints"],
              *cfg["grasp_joints"], cfg["standoff_m"], *cfg["place_base_pose"],
              *cfg["place_route"]]
    assert all(isinstance(v, float) for v in values)
    assert len(cfg["hover_joints"]) == len(cfg["grasp_joints"]) == 6


@pytest.mark.parametrize("name", sorted(SCENES))
def test_scene_place_pose_and_route_are_base_poses(name):
    # Poses are [x, y, yaw]; the route is a flat list of them, never empty:
    # a ROS parameter cannot be an empty list.
    cfg = SCENES[name]
    assert len(cfg["place_base_pose"]) == 3
    assert cfg["place_route"] and len(cfg["place_route"]) % 3 == 0
    assert 0.0 < cfg["standoff_m"] < cfg["base_approach_x"]


@pytest.mark.parametrize("name", sorted(SCENES))
def test_place_surface_is_a_world_model(name):
    world = ET.parse(PKG / "worlds" / SCENES[name]["world_file"]).getroot()
    surface = [m for m in world.iter("model") if m.get("name") == SCENES[name]["place_surface"]]

    assert len(surface) == 1
    assert any(c.get("name") == "top_collision" for c in surface[0].iter("collision"))


@pytest.mark.parametrize("key", ["rig_file", "place_route"])
def test_load_scenes_rejects_a_scene_without_a_required_key(tmp_path, key):
    broken = {name: dict(cfg) for name, cfg in SCENES.items()}
    del broken["garden"][key]
    path = tmp_path / "scenes.yaml"
    path.write_text(yaml.safe_dump(broken))

    with pytest.raises(ValueError, match=key):
        scenes.load_scenes(path)


# ---- SceneValue / from_scene (launch/pick_place_scenes.py) ------------------


def test_scene_value_reads_the_named_scene():
    value = scenes.SceneValue(LaunchConfiguration("scene"), "base_approach_x")

    assert value.perform(_context(scene="small_house")) == str(
        SCENES["small_house"]["base_approach_x"]
    )


def test_scene_value_prefers_a_non_empty_override():
    override = LaunchConfiguration("base_approach_x")
    value = scenes.SceneValue("garden", "base_approach_x", override=override)

    assert value.perform(_context(base_approach_x="0.6")) == "0.6"
    assert value.perform(_context(base_approach_x="")) == str(
        SCENES["garden"]["base_approach_x"]
    )


def test_scene_value_formats_joint_lists_as_yaml():
    value = scenes.SceneValue("small_house", "hover_joints")

    assert yaml.safe_load(value.perform(_context())) == SCENES["small_house"]["hover_joints"]


def test_unknown_scene_is_an_error():
    value = scenes.SceneValue(LaunchConfiguration("scene"), "world_name")

    with pytest.raises(RuntimeError, match=r"unknown scene 'nope'.*garden"):
        value.perform(_context(scene="nope"))


def test_from_scene_types_the_parameter():
    context = _context(scene="small_house", base_approach_x="")

    approach = scenes.from_scene(LaunchConfiguration("scene"), "base_approach_x", float)
    hover = scenes.from_scene(LaunchConfiguration("scene"), "hover_joints")
    route = scenes.from_scene(LaunchConfiguration("scene"), "place_route")

    assert approach.evaluate(context) == SCENES["small_house"]["base_approach_x"]
    assert list(hover.evaluate(context)) == SCENES["small_house"]["hover_joints"]
    assert list(route.evaluate(context)) == SCENES["small_house"]["place_route"]
    assert all(isinstance(v, float) for v in route.evaluate(context))


def _bringup_configuration(**given):
    """Launch configuration after sim_bringup's arguments, given `given` on the command line."""
    module = _load("sim_bringup_launch", PKG / "launch" / "sim_bringup.launch.py")
    context = _context(**given)
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)  # in declaration order, like `ros2 launch`
    return context.launch_configurations


@pytest.mark.parametrize("name", sorted(SCENES))
def test_sim_bringup_takes_its_world_from_the_scene(name):
    configuration = _bringup_configuration(scene=name)

    assert configuration["world_name"] == SCENES[name]["world_name"]
    assert Path(configuration["world"]).name == SCENES[name]["world_file"]
    assert Path(configuration["world"]).is_file()


def test_sim_bringup_defaults_to_the_garden():
    assert _bringup_configuration()["world_name"] == SCENES["garden"]["world_name"]


def test_explicit_world_arguments_override_the_scene():
    configuration = _bringup_configuration(scene="small_house", world="/x/y.sdf", world_name="w")

    assert (configuration["world"], configuration["world_name"]) == ("/x/y.sdf", "w")


def test_from_scene_takes_the_launch_argument_of_the_same_name():
    context = _context(scene="small_house", object_name="supplement_bottle")

    name = scenes.from_scene(LaunchConfiguration("scene"), "object_name", str)

    assert name.evaluate(context) == "supplement_bottle"


# ---- world asset check (gazebo.launch.py) ----------------------------------


def _sdf_with_uris(path, *uris):
    path.parent.mkdir(parents=True, exist_ok=True)
    body = "".join(f"<include><uri>{uri}</uri></include>" for uri in uris)
    path.write_text(f"<sdf version='1.8'><world name='w'>{body}</world></sdf>")
    return path


def test_unresolved_model_uris_lists_missing_models(tmp_path):
    world = tmp_path / "world.sdf"
    world.write_text(
        "<sdf version='1.8'><world name='w'>"
        "<include><uri>model://present</uri></include>"
        "<include><uri>model://absent</uri></include>"
        "<model name='m'><link name='l'><visual name='v'><geometry><mesh>"
        "<uri>model://absent/meshes/a.dae</uri></mesh></geometry></visual></link></model>"
        "<include><uri>https://fuel.gazebosim.org/1.0/o/models/x</uri></include>"
        "</world></sdf>"
    )
    _sdf_with_uris(tmp_path / "models" / "present" / "model.sdf")

    missing = gazebo_launch.unresolved_model_uris(
        str(world), [str(tmp_path / "nowhere"), str(tmp_path / "models")]
    )

    assert missing == ["absent"]


def test_unresolved_model_uris_follows_the_models_a_world_includes(tmp_path):
    # A wrapper model that is installed but uses a scan that was not fetched.
    world = _sdf_with_uris(tmp_path / "world.sdf", "model://wrapper")
    _sdf_with_uris(tmp_path / "models" / "wrapper" / "model.sdf", "model://scan/meshes/m.obj")

    assert gazebo_launch.unresolved_model_uris(str(world), [str(tmp_path / "models")]) == ["scan"]


def test_a_model_file_must_exist_not_just_its_directory(tmp_path):
    world = _sdf_with_uris(tmp_path / "world.sdf", "model://empty", "model://mesh/meshes/m.obj")
    (tmp_path / "models" / "empty").mkdir(parents=True)  # e.g. a stale install-space copy
    (tmp_path / "models" / "mesh" / "meshes").mkdir(parents=True)

    missing = gazebo_launch.unresolved_model_uris(str(world), [str(tmp_path / "models")])

    assert missing == ["empty", "mesh"]


def test_the_small_house_without_fetched_assets_misses_the_scans_too():
    # Only the package's own models: the wrappers resolve, the scans they use do not.
    world = PKG / "worlds" / "small_house_world.sdf"

    missing = gazebo_launch.unresolved_model_uris(str(world), [str(PKG / "models")])

    assert {"AllergenFree_JarroDophilus", "Beta_Glucan"} <= set(missing)
    assert "aws_robomaker_residential_KitchenTable_01" in missing
    assert "vitamin_bottle" not in missing


def test_garden_world_needs_no_extra_models():
    garden = PKG / "worlds" / "garden_world.sdf"
    assert gazebo_launch.unresolved_model_uris(str(garden), []) == []


def _gazebo_context(world):
    return _context(world=str(world), headless="true", paused="true")


def test_gazebo_launch_refuses_a_world_with_missing_models(tmp_path):
    world = tmp_path / "world.sdf"
    world.write_text(
        "<sdf version='1.8'><world name='w'>"
        "<include><uri>model://not_fetched_anywhere</uri></include></world></sdf>"
    )

    with pytest.raises(RuntimeError, match=r"1 models .*not_fetched_anywhere.*fetch_sim_assets"):
        gazebo_launch._launch_gazebo(_gazebo_context(world))


def test_gazebo_launch_puts_the_package_models_on_the_resource_path():
    actions = gazebo_launch._launch_gazebo(_gazebo_context(PKG / "worlds" / "garden_world.sdf"))

    env = next(a for a in actions if getattr(a, "name", None) and
               "".join(s.text for s in a.name) == "GZ_SIM_RESOURCE_PATH")
    paths = "".join(s.text for s in env.value).split(":")
    assert any(p.endswith("ranger_sim/models") for p in paths)
    assert any(p.endswith("ranger_sim/models/third_party") for p in paths)
