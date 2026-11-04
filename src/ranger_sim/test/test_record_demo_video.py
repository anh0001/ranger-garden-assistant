"""Unit tests for scripts/record_demo_video.py (no simulator required)."""

from datetime import datetime
import importlib.util
from pathlib import Path
import shutil

import cv2
import numpy as np
import pytest
from rcl_interfaces.msg import Log
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import String
import yaml

PKG = Path(__file__).resolve().parents[1]
SCRIPT = PKG / "scripts" / "record_demo_video.py"
_spec = importlib.util.spec_from_file_location("record_demo_video", SCRIPT)
rec = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rec)

FPS = 30
SEC = 1_000_000_000


def _image(width, height, encoding="rgb8", fill=(10, 20, 30), sec=0, nanosec=0, pad=0):
    pixels = np.zeros((height, width, 3), np.uint8)
    pixels[:, :] = fill
    rows = pixels.reshape(height, width * 3)
    if pad:
        rows = np.hstack([rows, np.zeros((height, pad), np.uint8)])
    msg = Image()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.width = width
    msg.height = height
    msg.encoding = encoding
    msg.step = width * 3 + pad
    msg.data = rows.tobytes()
    return msg


def _log(name, text):
    msg = Log()
    msg.name = name
    msg.msg = text
    return msg


class FakeSink:
    """Stands in for FfmpegSink: records frames, optionally fails."""

    def __init__(self, cmd, fail_on_write=False, close_code=0):
        self.cmd = cmd
        self.frames = []
        self.fail_on_write = fail_on_write
        self.close_code = close_code
        self.closed = 0

    def write(self, data):
        if self.fail_on_write:
            raise BrokenPipeError("ffmpeg exited")
        self.frames.append(data)

    def close(self, timeout=30.0):
        self.closed += 1
        return self.close_code


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _recorder(tmp_path, **sink_kwargs):
    sinks = []

    def factory(cmd):
        sinks.append(FakeSink(cmd, **sink_kwargs))
        return sinks[-1]

    node = rec.DemoVideoRecorder(
        sink_factory=factory,
        parameter_overrides=[
            Parameter("output_path", value=str(tmp_path / "videos" / "demo.mp4")),
            Parameter("width", value=64),
            Parameter("height", value=36),
        ],
    )
    return node, sinks[0]


# ---- frame timing on the simulation clock --------------------------------


def test_first_frame_is_written_once():
    assert rec.frames_due(start_ns=5 * SEC, stamp_ns=5 * SEC, fps=FPS, written=0) == 1


def test_frame_before_next_slot_is_dropped():
    assert rec.frames_due(5 * SEC, 5 * SEC + SEC // 60, FPS, written=1) == 0


def test_frame_on_next_slot_is_written():
    assert rec.frames_due(0, SEC // FPS + 1, FPS, written=1) == 1


def test_gap_in_camera_frames_is_filled_with_repeats():
    # 0.1 s of simulation time without a frame = 3 more slots at 30 fps.
    assert rec.frames_due(0, SEC // 10, FPS, written=1) == 3


def test_stamp_before_start_writes_nothing():
    assert rec.frames_due(10 * SEC, 9 * SEC, FPS, written=0) == 0


def test_long_recording_does_not_drift():
    # A 60 Hz camera for one simulated hour must give exactly 30 fps.
    written = 0
    for i in range(60 * 3600 + 1):
        written += rec.frames_due(0, i * SEC // 60, FPS, written)
    assert written == FPS * 3600 + 1


def test_stamp_to_ns_combines_seconds_and_nanoseconds():
    assert rec.stamp_to_ns(3, 250_000_000) == 3_250_000_000


# ---- image conversion -----------------------------------------------------


def test_rgb8_image_becomes_bgr():
    bgr = rec.image_to_bgr(_image(4, 2, "rgb8", fill=(255, 0, 0)))
    assert bgr.shape == (2, 4, 3)
    assert tuple(bgr[0, 0]) == (0, 0, 255)


def test_bgr8_image_is_unchanged():
    bgr = rec.image_to_bgr(_image(4, 2, "bgr8", fill=(255, 0, 0)))
    assert tuple(bgr[1, 3]) == (255, 0, 0)


def test_row_padding_is_dropped():
    bgr = rec.image_to_bgr(_image(3, 2, "rgb8", fill=(1, 2, 3), pad=7))
    assert bgr.shape == (2, 3, 3)
    assert tuple(bgr[1, 2]) == (3, 2, 1)


def test_unsupported_encoding_is_rejected():
    with pytest.raises(ValueError, match="mono8"):
        rec.image_to_bgr(_image(4, 2, "mono8"))


def test_truncated_image_data_is_rejected():
    msg = _image(4, 2)
    msg.data = msg.data[:-1]
    with pytest.raises(ValueError, match="size"):
        rec.image_to_bgr(msg)


# ---- overlay text ---------------------------------------------------------


def test_caption_keeps_short_ascii_text():
    assert rec.format_caption("Arm -> 'home'") == "Arm -> 'home'"


def test_caption_replaces_characters_puttext_cannot_draw():
    assert rec.format_caption("starting — mode: EXECUTE") == "starting - mode: EXECUTE"


def test_caption_is_truncated_with_ellipsis():
    assert rec.format_caption("x" * 100, max_chars=20) == "x" * 17 + "..."


def test_caption_comes_from_the_demo_node_only():
    demo_line = _log("pick_place_demo", "Arm -> 'lift'")
    other_line = _log("move_group", "Planning")
    assert rec.caption_from_log(demo_line, "pick_place_demo") == "Arm -> 'lift'"
    assert rec.caption_from_log(other_line, "pick_place_demo") is None


def test_waiting_lines_do_not_replace_the_caption():
    log = _log("pick_place_demo", "Waiting for /move_action ...")
    assert rec.caption_from_log(log, "pick_place_demo") is None


def test_grasp_label_highlights_attached():
    assert rec.grasp_label("cup", "attached") == ("cup: ATTACHED", True)
    assert rec.grasp_label("cup", "detached") == ("cup: DETACHED", False)


# ---- status bar layout ----------------------------------------------------

GRASPABLE_NAMES = [
    o["name"]
    for o in yaml.safe_load((PKG / "config" / "graspable_objects.yaml").read_text())["objects"]
]
LONGEST_CAPTION = rec.format_caption("step: " + "grasp -> attach 'x' (attached) " * 4)
SIM_TEXT = "sim t = 9999.9 s"


@pytest.mark.parametrize("name", GRASPABLE_NAMES)
def test_status_texts_never_overlap(name):
    grasp_text, _ = rec.grasp_label(name, "detached")

    step, grasp_x, sim_x = rec.status_layout(1280, LONGEST_CAPTION, grasp_text, SIM_TEXT)

    step_end = rec.MARGIN + rec.text_width(step, rec.STATUS_SCALE)
    grasp_end = grasp_x + rec.text_width(grasp_text, rec.STATUS_SCALE)
    assert step_end < grasp_x, "the step caption runs into the grasp state"
    assert grasp_end < sim_x, "the grasp state runs into the sim time"
    assert sim_x + rec.text_width(SIM_TEXT, rec.SIM_SCALE) <= 1280 - rec.MARGIN


def test_status_layout_keeps_a_short_caption_whole():
    step, _, _ = rec.status_layout(1280, "step: Arm -> hover", "cup: DETACHED", SIM_TEXT)
    assert step == "step: Arm -> hover"


def test_status_layout_cuts_a_long_caption_with_an_ellipsis():
    step, _, _ = rec.status_layout(1280, LONGEST_CAPTION, "vitamin_bottle: DETACHED", SIM_TEXT)
    assert step.endswith("...") and len(step) < len(LONGEST_CAPTION)


# ---- composition ----------------------------------------------------------


def _compose(wide, close=None):
    return rec.compose_frame(
        wide,
        close,
        size=(1280, 720),
        title="title",
        caption="step",
        grasp_text="cup: DETACHED",
        grasp_attached=False,
        sim_sec=12.5,
    )


def test_composed_frame_has_the_output_size():
    frame = _compose(np.zeros((720, 1280, 3), np.uint8))
    assert frame.shape == (720, 1280, 3)


def test_smaller_wide_frame_is_scaled_to_the_output_size():
    frame = _compose(np.zeros((360, 640, 3), np.uint8))
    assert frame.shape == (720, 1280, 3)


def test_composition_does_not_modify_the_camera_frame():
    wide = np.full((720, 1280, 3), 200, np.uint8)
    before = wide.copy()
    _compose(wide, np.zeros((540, 960, 3), np.uint8))
    assert np.array_equal(wide, before)


def test_close_up_is_drawn_as_an_inset():
    wide = np.full((720, 1280, 3), 200, np.uint8)
    close = np.zeros((540, 960, 3), np.uint8)
    with_inset = _compose(wide, close)
    without = _compose(wide)
    x, y = rec.MARGIN + 200, rec.TITLE_BAR_H + rec.MARGIN + 150
    assert tuple(with_inset[y, x]) == (0, 0, 0)
    assert tuple(without[y, x]) == (200, 200, 200)


# ---- ffmpeg ---------------------------------------------------------------


def test_ffmpeg_command_encodes_raw_bgr_frames_to_h264():
    cmd = rec.ffmpeg_command("/tmp/out.mp4", 1280, 720, 30, crf=20)
    joined = " ".join(cmd)
    assert cmd[0] == "ffmpeg"
    assert cmd[-1] == "/tmp/out.mp4"
    expected = (
        "-f rawvideo",
        "-pix_fmt bgr24",
        "-s 1280x720",
        "-r 30",
        "-i -",
        "-c:v libx264",
        "-crf 20",
        "-pix_fmt yuv420p",
        "-movflags +faststart",
    )
    for part in expected:
        assert part in joined


def test_default_output_path_is_timestamped(tmp_path):
    path = rec.default_output_path(datetime(2026, 11, 1, 16, 5, 9), tmp_path)
    assert path == tmp_path / "pick_place_20261101_160509.mp4"


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg not installed")
def test_ffmpeg_sink_writes_a_playable_mp4(tmp_path):
    out = tmp_path / "clip.mp4"
    sink = rec.FfmpegSink(rec.ffmpeg_command(str(out), 64, 48, FPS, crf=30))
    frame = np.full((48, 64, 3), 128, np.uint8).tobytes()
    for _ in range(15):
        sink.write(frame)

    assert sink.close() == 0
    video = cv2.VideoCapture(str(out))
    assert int(video.get(cv2.CAP_PROP_FRAME_COUNT)) == 15
    video.release()


def test_ffmpeg_sink_reports_the_encoder_exit_code():
    sink = rec.FfmpegSink(["sh", "-c", "cat > /dev/null; exit 3"])
    sink.write(b"frame")
    assert sink.close() == 3


# ---- node -----------------------------------------------------------------


def test_node_writes_one_frame_per_simulation_slot(ros, tmp_path):
    node, sink = _recorder(tmp_path)
    try:
        for i in range(7):  # 60 Hz camera, 0.1 s of simulation time
            node._on_wide(_image(64, 36, sec=100, nanosec=i * SEC // 60))
    finally:
        node.destroy_node()

    assert len(sink.frames) == 4  # slots at 0, 1/30, 2/30 and 3/30 s
    assert all(len(frame) == 64 * 36 * 3 for frame in sink.frames)


def test_node_creates_the_output_directory(ros, tmp_path):
    node, sink = _recorder(tmp_path)
    node.destroy_node()

    assert (tmp_path / "videos").is_dir()
    assert sink.cmd[-1] == str(tmp_path / "videos" / "demo.mp4")


def test_node_is_not_recording_before_the_first_frame(ros, tmp_path):
    node, _ = _recorder(tmp_path)
    try:
        assert not node.recording
        node._on_wide(_image(64, 36, sec=1))
        assert node.recording
    finally:
        node.destroy_node()


def test_unconvertible_frame_does_not_start_the_recording(ros, tmp_path):
    node, sink = _recorder(tmp_path)
    try:
        node._on_wide(_image(64, 36, encoding="mono8", sec=1))
    finally:
        node.destroy_node()

    assert not node.recording
    assert sink.frames == []


def test_overlay_follows_the_demo_log_and_grasp_state(ros, tmp_path):
    node, _ = _recorder(tmp_path)
    try:
        node._on_log(_log("pick_place_demo", "Arm -> 'lift'"))
        node._on_log(_log("move_group", "ignored"))
        node._on_grasp_state(String(data="attached"))
        assert node.caption == "Arm -> 'lift'"
        assert node.grasp_state == "attached"
    finally:
        node.destroy_node()


def test_encoder_failure_stops_the_recording(ros, tmp_path):
    node, _ = _recorder(tmp_path, fail_on_write=True)
    try:
        node._on_wide(_image(64, 36, sec=1))
        node._on_wide(_image(64, 36, sec=2))
    finally:
        node.destroy_node()

    assert node.failed


def test_no_frame_before_the_timeout_fails_the_recording(ros, tmp_path):
    node, _ = _recorder(tmp_path)
    try:
        node._on_first_frame_timeout()
    finally:
        node.destroy_node()

    assert node.failed


def test_timeout_after_the_first_frame_is_ignored(ros, tmp_path):
    node, _ = _recorder(tmp_path)
    try:
        node._on_wide(_image(64, 36, sec=1))
        node._on_first_frame_timeout()
    finally:
        node.destroy_node()

    assert not node.failed


def test_finish_closes_the_encoder_once_and_returns_its_code(ros, tmp_path):
    node, sink = _recorder(tmp_path, close_code=0)
    try:
        assert node.finish() == 0
        assert node.finish() == 0
    finally:
        node.destroy_node()

    assert sink.closed == 1


def test_finish_without_frames_removes_the_empty_file(ros, tmp_path):
    node, _ = _recorder(tmp_path)
    out = tmp_path / "videos" / "demo.mp4"
    out.write_bytes(b"mp4 header, no frames")  # what ffmpeg leaves behind
    try:
        node.finish()
    finally:
        node.destroy_node()

    assert not out.exists()


def test_finish_keeps_a_recorded_file(ros, tmp_path):
    node, _ = _recorder(tmp_path)
    out = tmp_path / "videos" / "demo.mp4"
    out.write_bytes(b"mp4 with frames")
    try:
        node._on_wide(_image(64, 36, sec=1))
        node.finish()
    finally:
        node.destroy_node()

    assert out.exists()


def test_finish_reports_a_failed_encoder(ros, tmp_path):
    node, _ = _recorder(tmp_path, close_code=1)
    try:
        assert node.finish() == 1
    finally:
        node.destroy_node()
