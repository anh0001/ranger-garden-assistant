"""Tests for the demo-video camera rigs and their launch file (no simulator required)."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
SCENES = yaml.safe_load((PKG / "config" / "pick_place_scenes.yaml").read_text())
# Every scene's rig (config/pick_place_scenes.yaml: rig_file).
RIGS = pytest.mark.parametrize(
    "rig_file", sorted({cfg["rig_file"] for cfg in SCENES.values()})
)


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


recorder = _load("record_demo_video", PKG / "scripts" / "record_demo_video.py")
launch_file = _load(
    "record_pick_place_video_launch", PKG / "launch" / "record_pick_place_video.launch.py"
)


def _rig(rig_file):
    model = ET.parse(PKG / "models" / rig_file).getroot().find("model")
    cameras = {s.get("name"): s for s in model.iter("sensor") if s.get("type") == "camera"}
    return model, cameras


def _size(sensor):
    image = sensor.find("camera/image")
    return int(image.findtext("width")), int(image.findtext("height"))


# ---- camera rigs ----------------------------------------------------------


@RIGS
def test_rig_is_a_static_model_named_video_rig(rig_file):
    model, _ = _rig(rig_file)
    assert model.get("name") == "video_rig"
    assert model.findtext("static") == "true"


@RIGS
def test_rig_has_no_collisions(rig_file):
    model, _ = _rig(rig_file)
    assert list(model.iter("collision")) == []


@RIGS
def test_rig_wide_camera_matches_the_recorder_defaults(rig_file):
    _, cameras = _rig(rig_file)
    wide = cameras["wide_cam"]
    assert "/" + wide.findtext("topic") == recorder.DEFAULT_WIDE_TOPIC
    assert _size(wide) == (recorder.DEFAULT_WIDTH, recorder.DEFAULT_HEIGHT)
    assert int(wide.findtext("update_rate")) >= recorder.DEFAULT_FPS


@RIGS
def test_rig_close_camera_matches_the_recorder_defaults(rig_file):
    _, cameras = _rig(rig_file)
    close = cameras["close_cam"]
    assert "/" + close.findtext("topic") == recorder.DEFAULT_CLOSE_TOPIC
    width, height = _size(close)
    assert width * 9 == height * 16  # same 16:9 shape as the inset


# ---- launch file ----------------------------------------------------------


def _entities():
    return launch_file.generate_launch_description().entities


def test_launch_declares_its_arguments():
    names = {e.name for e in _entities() if isinstance(e, DeclareLaunchArgument)}
    assert names == {
        "scene",
        "world_name",
        "output_path",
        "object_name",
        "base_approach_x",
        "gripper_closed",
        "first_frame_timeout",
    }


def test_launch_does_not_start_the_demo_up_front():
    # The demo moves the arm; it may only start from the first-frame gate.
    executables = {e.node_executable for e in _entities() if isinstance(e, Node)}
    assert "pick_place_demo.py" not in executables
    assert "record_demo_video.py" in executables


def test_launch_waits_for_the_recorders_ready_line():
    assert launch_file.READY_MARKER == recorder.READY_MARKER


def test_demo_gate_ignores_other_output():
    gate = launch_file.make_demo_gate(["demo"])
    assert gate(SimpleNamespace(text=b"[INFO] waiting for the first camera frame")) is None


def test_demo_gate_starts_the_demo_once_on_the_ready_line():
    gate = launch_file.make_demo_gate(["demo"])
    ready = SimpleNamespace(text=f"[INFO] {recorder.READY_MARKER}: /x.mp4\n".encode())
    assert gate(ready) == ["demo"]
    assert gate(ready) is None
