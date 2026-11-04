"""Tests for the demo-video camera rigs and their launch file (no simulator required)."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import cv2
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
import numpy as np
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
SCENES = yaml.safe_load((PKG / "config" / "pick_place_scenes.yaml").read_text())
SIM_XACRO = PKG / "urdf" / "ranger_gazebo.urdf.xacro"
BRIDGE = PKG / "config" / "gz_bridge.yaml"
MAX_INSET_STRETCH = 0.01  # wrist image vs. inset aspect ratio
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
def test_rig_has_only_the_wide_camera(rig_file):
    # The inset is the robot's wrist camera: a rig close-up would only cost
    # a render per frame.
    model, cameras = _rig(rig_file)
    assert set(cameras) == {"wide_cam"}
    assert [s.get("name") for s in model.iter("sensor")] == ["wide_cam"]


# ---- the inset: the robot's wrist camera ----------------------------------


def _wrist_sensor():
    """Return the wrist RGBD sensor of the sim xacro (plain XML: no xacro processing)."""
    root = ET.parse(SIM_XACRO).getroot()
    return next(s for s in root.iter("sensor") if s.get("name") == "wrist_rgbd")


def _bridge_entry(gz_topic):
    entries = [e for e in yaml.safe_load(BRIDGE.read_text()) if e.get("gz_topic_name") == gz_topic]
    assert len(entries) == 1, f"{gz_topic} bridged {len(entries)} times"
    return entries[0]


def test_recorder_inset_is_the_bridged_wrist_camera():
    sensor = _wrist_sensor()
    entry = _bridge_entry("/" + sensor.findtext("topic") + "/image")  # rgbd_camera colour

    assert entry["ros_topic_name"] == recorder.DEFAULT_CLOSE_TOPIC
    assert entry["direction"] == "GZ_TO_ROS"
    assert entry["ros_type_name"] == "sensor_msgs/msg/Image"


def test_wrist_depth_image_is_bridged_too():
    sensor = _wrist_sensor()
    entry = _bridge_entry("/" + sensor.findtext("topic") + "/depth_image")

    assert entry["ros_topic_name"] == "/piper/wrist_camera/depth/image_rect_raw"


def test_wrist_camera_has_the_shape_of_the_inset():
    width, height = _size(_wrist_sensor())

    # 848x480, a native D405 colour mode, is 0.6 % from the inset's 16:9.
    inset_aspect = recorder.INSET_W / recorder.INSET_H
    assert abs(width / height - inset_aspect) / inset_aspect <= MAX_INSET_STRETCH


def test_inset_label_fits_its_shade():
    frame = recorder.compose_frame(
        np.zeros((720, 1280, 3), np.uint8), np.full((480, 848, 3), 255, np.uint8),
        size=(1280, 720), title="t", caption="c", grasp_text="cup: DETACHED",
        grasp_attached=False, sim_sec=0.0,
    )
    x0, y0 = recorder.MARGIN, recorder.TITLE_BAR_H + recorder.MARGIN
    label_w = cv2.getTextSize(
        recorder.INSET_LABEL, recorder.FONT, recorder.INSET_LABEL_SCALE, 1
    )[0][0]
    right_of_label = frame[y0 + 2:y0 + recorder.INSET_LABEL_H - 2,
                           x0 + label_w + recorder.INSET_LABEL_PAD + 2]

    assert (right_of_label < 255).all(), "the shade ends before the label does"


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


def _literal_arguments(node):
    """Return the plain-text command-line arguments of a launch_ros Node."""
    return [
        "".join(s.text for s in arg) for arg in node.cmd
        if all(isinstance(s, TextSubstitution) for s in arg)
    ]


def test_launch_bridges_only_the_wide_camera():
    (bridge,) = [
        e for e in _entities() if isinstance(e, Node) and e.node_executable == "parameter_bridge"
    ]
    arguments = _literal_arguments(bridge)

    assert any(a.startswith(recorder.DEFAULT_WIDE_TOPIC + "@") for a in arguments)
    assert not any(a.startswith("/video/close@") for a in arguments)


def test_launch_leaves_the_recorder_on_its_wrist_camera_default():
    (rec,) = [
        e for e in _entities()
        if isinstance(e, Node) and e.node_executable == "record_demo_video.py"
    ]
    names = {"".join(s.text for s in name) for params in rec._Node__parameters for name in params}

    assert "output_path" in names  # the parameters are visible here at all
    assert "close_topic" not in names


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
