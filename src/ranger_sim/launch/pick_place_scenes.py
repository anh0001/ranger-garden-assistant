#!/usr/bin/env python3
"""
Pick-and-place scenes for the launch files (a helper module, not a launch file).

config/pick_place_scenes.yaml holds, per scene, the world, the object, the base
approach, the arm waypoints, the place pose and route of the transfer and the
video camera rig; OPTIONAL_KEYS may be left out and take another key's value
(scene_value). The launch files take a
`scene` argument and read their defaults through SceneValue / from_scene. They
import this module by path (it is installed next to them; `ros2 launch` only
lists *.launch.py, so it stays out of the way):

    _spec = importlib.util.spec_from_file_location(
        "pick_place_scenes", Path(__file__).with_name("pick_place_scenes.py"))
    scenes = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(scenes)
"""

from pathlib import Path

from launch import Substitution
from launch.substitutions import LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.parameter_descriptions import ParameterValue
import yaml

# launch/ and config/ are siblings in both the source and the install space.
SCENES_FILE = Path(__file__).parent.parent / "config" / "pick_place_scenes.yaml"
REQUIRED_KEYS = frozenset({
    "world_file", "world_name", "object_name", "base_approach_x",
    "gripper_closed", "hover_joints", "grasp_joints", "rig_file",
    "standoff_m", "place_base_pose", "place_route", "place_surface",
})
# Optional scene keys (arm waypoints, 6 floats) and the key each defaults to:
# a scene whose place surface is as high as its pick table places with the
# pick's hover and grasp.
OPTIONAL_KEYS = {
    "place_hover_joints": "hover_joints",
    "place_grasp_joints": "grasp_joints",
}
ARM_JOINT_COUNT = 6
# Scene keys a launch argument of the same name can override (empty = scene).
OVERRIDABLE = ("object_name", "base_approach_x", "gripper_closed")


def _is_joint_list(value) -> bool:
    return (isinstance(value, list) and len(value) == ARM_JOINT_COUNT
            and all(isinstance(v, float) for v in value))


def load_scenes(path: Path = SCENES_FILE) -> dict:
    """Read the scenes file; ValueError for a missing required key or a bad optional one."""
    scenes = yaml.safe_load(Path(path).read_text())
    if not isinstance(scenes, dict) or not scenes:
        raise ValueError(f"{path}: expected a mapping of scene names")
    for name, cfg in scenes.items():
        if not isinstance(cfg, dict):
            raise ValueError(f"{path}: scene '{name}' must be a mapping")
        missing = sorted(REQUIRED_KEYS - set(cfg))
        if missing:
            raise ValueError(f"{path}: scene '{name}' is missing {', '.join(missing)}")
        for key in sorted(set(OPTIONAL_KEYS) & set(cfg)):
            if not _is_joint_list(cfg[key]):
                raise ValueError(
                    f"{path}: scene '{name}' {key} must be {ARM_JOINT_COUNT} floats, "
                    f"got {cfg[key]!r}"
                )
    return scenes


def scene_value(cfg: dict, key: str):
    """Value of `key` in the scene mapping `cfg`; an absent optional key takes its default's."""
    if key not in cfg and key in OPTIONAL_KEYS:
        return cfg[OPTIONAL_KEYS[key]]
    return cfg[key]


class SceneValue(Substitution):
    """Value of `key` for the scene named by `scene`, unless `override` is non-empty."""

    def __init__(self, scene, key: str, override=None, path: Path = SCENES_FILE) -> None:
        super().__init__()
        self._scene = normalize_to_list_of_substitutions(scene)
        self._key = key
        self._override = (
            None if override is None else normalize_to_list_of_substitutions(override)
        )
        self._path = Path(path)

    def describe(self) -> str:
        return f"SceneValue('{self._key}')"

    def perform(self, context) -> str:
        if self._override is not None:
            value = perform_substitutions(context, self._override)
            if value:
                return value
        name = perform_substitutions(context, self._scene)
        scenes = load_scenes(self._path)
        if name not in scenes:
            raise RuntimeError(f"unknown scene '{name}'; known: {sorted(scenes)}")
        value = scene_value(scenes[name], self._key)
        if isinstance(value, list):
            return yaml.safe_dump(value, default_flow_style=True).strip()
        return str(value)


def from_scene(scene, key: str, value_type=None) -> ParameterValue:
    """Node parameter for `key`: the launch arg of the same name if non-empty, else the scene."""
    override = LaunchConfiguration(key, default="") if key in OVERRIDABLE else None
    return ParameterValue(SceneValue(scene, key, override=override), value_type=value_type)
