#!/usr/bin/env python3
"""Launch Gazebo Fortress with the specified world."""

import glob
import os
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch_ros.substitutions import FindPackageShare

MODEL_SCHEME = "model://"
MODEL_FILES = ("model.config", "model.sdf")  # what makes a directory a model
FETCH_COMMAND = "python3 src/ranger_sim/tools/fetch_sim_assets.py"
MISSING_SHOWN = 3  # missing model names listed in the error before "and N more"


def _model_uris(sdf_file: str) -> set[str]:
    """Return the model:// URIs of an SDF file, without the scheme ('X' or 'X/meshes/a.dae')."""
    uris = set()
    for uri in ET.parse(sdf_file).getroot().iter("uri"):
        text = (uri.text or "").strip()
        if text.startswith(MODEL_SCHEME):
            uris.add(text[len(MODEL_SCHEME):].strip("/"))
    return uris


def _resolve(uri: str, search_dirs: list[str]) -> str | None:
    """Return the path of a model:// URI: a file inside a model, or a model directory."""
    for directory in filter(None, search_dirs):
        path = os.path.join(directory, uri)
        if "/" in uri:
            if os.path.isfile(path):
                return path
        elif any(os.path.isfile(os.path.join(path, name)) for name in MODEL_FILES):
            return path
    return None


def unresolved_model_uris(world_file: str, search_dirs: list[str]) -> list[str]:
    """
    Return the names of the models the world needs that no search dir has.

    Follows the world's model:// URIs (includes and meshes) into the models it
    includes, so a wrapper model whose mesh comes from an unfetched model is
    caught too. A model directory needs a model.config or model.sdf; a URI
    into a model needs that file.
    """
    missing, seen = set(), set()
    pending = list(_model_uris(world_file))
    while pending:
        uri = pending.pop()
        if uri in seen:
            continue
        seen.add(uri)
        path = _resolve(uri, search_dirs)
        if path is None:
            missing.add(uri.split("/")[0])
        elif "/" not in uri and os.path.isfile(os.path.join(path, "model.sdf")):
            pending.extend(_model_uris(os.path.join(path, "model.sdf")))
    return sorted(missing)


def _resource_paths() -> list[str]:
    """Return the Gazebo resource path: the existing one plus the robot and ranger_sim models."""
    # Wrapper models (models/<name>) and the fetched third-party assets
    # (tools/fetch_sim_assets.py): models/third_party/<name>, and asset sets
    # kept in their own layout, models/third_party/<set>/models/<name>.
    ranger_sim_models = os.path.join(get_package_share_directory("ranger_sim"), "models")
    third_party = os.path.join(ranger_sim_models, "third_party")
    paths = [
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        os.path.dirname(get_package_share_directory("piper_description")),
        os.path.dirname(get_package_share_directory("ranger_description")),
        ranger_sim_models,
        third_party,
        *sorted(glob.glob(os.path.join(third_party, "*", "models"))),
    ]
    return [path for path in paths if path]


def _check_world_assets(world: str, search_dirs: list[str]) -> None:
    """Raise RuntimeError naming the fetch tool if the world needs missing models."""
    # A world file that is not on disk (e.g. a name Gazebo resolves itself)
    # is left to Gazebo.
    if not os.path.isfile(world):
        return
    missing = unresolved_model_uris(world, search_dirs)
    if missing:
        shown = ", ".join(missing[:MISSING_SHOWN])
        more = len(missing) - MISSING_SHOWN
        raise RuntimeError(
            f"World {world} needs {len(missing)} models that are not installed "
            f"({shown}{f' and {more} more' if more > 0 else ''}). "
            f"Run `{FETCH_COMMAND}` and rebuild ranger_sim."
        )


def _launch_gazebo(context, *args, **kwargs):
    """Resolve headless flag at launch time and build gz_args accordingly."""
    try:
        get_package_share_directory("gz_ros2_control")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "Missing gz_ros2_control. Install ros-humble-gz-ros2-control "
            "(or ros-humble-ign-ros2-control on Humble Ignition setups) "
            "before launching backend:=gazebo."
        ) from exc

    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    paused = LaunchConfiguration("paused").perform(context)
    resource_paths = _resource_paths()
    gz_resource_path = os.pathsep.join(resource_paths)
    _check_world_assets(world, resource_paths)

    # Paused start (no -r): every DetachableJoint grasp joint is created
    # attached at robot spawn; grasp_manager.py detaches them all and then
    # unpauses the world, so no object is dragged on the first step.
    gz_args = world if paused.lower() == "true" else f"-r {world}"
    if headless.lower() == "true":
        gz_args += " --headless-rendering -s"

    return [
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=gz_resource_path),
        SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=gz_resource_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
                ])
            ),
            launch_arguments={"gz_args": gz_args}.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("ranger_sim"), "worlds", "garden_world.sdf"
            ]),
            description="Path to the SDF world file.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo headless (no GUI) for CI.",
        ),
        DeclareLaunchArgument(
            "paused",
            default_value="true",
            description="Start the world paused; grasp_manager unpauses it "
                        "after detaching the graspable objects.",
        ),
        OpaqueFunction(function=_launch_gazebo),
    ])
