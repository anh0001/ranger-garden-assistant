#!/usr/bin/env python3
"""
Record the pick-and-place demo from the video_rig cameras into an H.264 MP4.

Reads the two Gazebo cameras of models/video_rig.sdf (bridged to ROS as
/video/wide and /video/close), draws the close-up as an inset, overlays the
latest pick_place_demo log line (from /rosout) and the grasp state of one
object (/grasp/<object>/state), and pipes the frames to ffmpeg.

Frames are placed on the simulation clock (image header stamps): one output
frame per 1/fps of simulation time, extra camera frames dropped and gaps
filled with the last frame. Playback therefore runs at simulation speed,
however fast or slow the sim and its cameras happen to run.

The first converted wide frame is logged with READY_MARKER.
record_pick_place_video.launch.py starts the demo only after that line, so
the arm moves only once frames from the rig in a running Gazebo arrive. The
node exits non-zero if no frame arrives within first_frame_timeout_sec or
the encoder fails.
"""

from datetime import datetime
from pathlib import Path
import subprocess
import sys
from typing import Callable, Optional, Protocol

import cv2
import numpy as np
from rcl_interfaces.msg import Log
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import String

READY_MARKER = "video recording started"

DEFAULT_WIDE_TOPIC = "/video/wide"
DEFAULT_CLOSE_TOPIC = "/video/close"
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 30
DEFAULT_CRF = 20
DEFAULT_TITLE = "Ranger Mini 3.0 + PiPER  |  Gazebo Fortress pick & place"
DEFAULT_VIDEO_DIR = Path.home() / ".ros" / "videos"

NS_PER_SEC = 1_000_000_000
CAPTION_MAX_CHARS = 60
IGNORED_CAPTION_PREFIXES = ("Waiting",)  # discovery chatter, not demo steps
ASCII_REPLACEMENTS = {"—": "-", "–": "-", "→": "->"}

# Overlay layout (pixels, at the default 1280x720 output).
TITLE_BAR_H = 44
STATUS_BAR_H = 52
MARGIN = 16
INSET_W, INSET_H = 480, 270
FONT = cv2.FONT_HERSHEY_SIMPLEX
WHITE = (255, 255, 255)
GREY = (210, 210, 210)
GREEN = (60, 220, 60)
SHADE_ALPHA = 0.55

ENCODER_CLOSE_TIMEOUT_SEC = 30.0
LOG_THROTTLE_SEC = 5.0


class FrameSink(Protocol):
    """Destination for raw BGR frames (FfmpegSink, or a fake in tests)."""

    def write(self, data: bytes) -> None: ...

    def close(self, timeout: float = ENCODER_CLOSE_TIMEOUT_SEC) -> int: ...


def stamp_to_ns(sec: int, nanosec: int) -> int:
    """Return a ROS time stamp as integer nanoseconds."""
    return sec * NS_PER_SEC + nanosec


def frames_due(start_ns: int, stamp_ns: int, fps: int, written: int) -> int:
    """
    Return how many output frames a camera frame stamped ``stamp_ns`` fills.

    Output frame k belongs to simulation time ``start_ns + k / fps``. The
    frame fills every slot up to its stamp that is not written yet: 0 drops
    it, more than 1 repeats it over a gap. Integer arithmetic, so the rate
    never drifts.
    """
    if stamp_ns < start_ns:
        return 0
    slots_until_stamp = (stamp_ns - start_ns) * fps // NS_PER_SEC + 1
    return max(0, slots_until_stamp - written)


def image_to_bgr(msg: Image) -> np.ndarray:
    """Convert an rgb8/bgr8 sensor_msgs/Image to a BGR array; ValueError otherwise."""
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported image encoding '{msg.encoding}' (need rgb8 or bgr8)")
    row_bytes = msg.width * 3
    if msg.step < row_bytes or len(msg.data) != msg.step * msg.height:
        raise ValueError(
            f"image data size {len(msg.data)} does not match "
            f"{msg.height} rows x step {msg.step} for width {msg.width}"
        )
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    pixels = rows[:, :row_bytes].reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    return pixels.copy()


def format_caption(text: str, max_chars: int = CAPTION_MAX_CHARS) -> str:
    """Make ``text`` drawable by cv2.putText (ASCII only) and at most ``max_chars`` long."""
    for char, replacement in ASCII_REPLACEMENTS.items():
        text = text.replace(char, replacement)
    text = text.encode("ascii", "ignore").decode("ascii")
    if len(text) <= max_chars:
        return text
    return text[: max_chars - 3] + "..."


def caption_from_log(msg: Log, node_name: str) -> Optional[str]:
    """Return the log text if it is a demo step logged by ``node_name``, else None."""
    if msg.name != node_name or msg.msg.startswith(IGNORED_CAPTION_PREFIXES):
        return None
    return msg.msg


def grasp_label(obj: str, state: str) -> tuple[str, bool]:
    """Return the overlay label for a grasp state and whether it is attached."""
    return f"{obj}: {state.upper()}", state == "attached"


def _shade(frame: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> None:
    roi = frame[y0:y1, x0:x1]
    frame[y0:y1, x0:x1] = (roi * (1.0 - SHADE_ALPHA)).astype(np.uint8)


def _draw_inset(frame: np.ndarray, close: np.ndarray) -> None:
    x0, y0 = MARGIN, TITLE_BAR_H + MARGIN
    x1, y1 = x0 + INSET_W, y0 + INSET_H
    frame[y0:y1, x0:x1] = cv2.resize(close, (INSET_W, INSET_H))
    cv2.rectangle(frame, (x0 - 2, y0 - 2), (x1 + 1, y1 + 1), WHITE, 2)
    _shade(frame, x0, y0, x0 + 120, y0 + 26)
    cv2.putText(frame, "close-up", (x0 + 8, y0 + 19), FONT, 0.6, WHITE, 1, cv2.LINE_AA)


def compose_frame(
    wide: np.ndarray,
    close: Optional[np.ndarray],
    *,
    size: tuple[int, int],
    title: str,
    caption: str,
    grasp_text: str,
    grasp_attached: bool,
    sim_sec: float,
) -> np.ndarray:
    """Return a new ``size`` (w, h) frame: wide view, close-up inset and overlays."""
    width, height = size
    if wide.shape[:2] == (height, width):
        frame = wide.copy()
    else:
        frame = cv2.resize(wide, (width, height))
    _shade(frame, 0, 0, width, TITLE_BAR_H)
    cv2.putText(frame, title, (16, 29), FONT, 0.72, WHITE, 2, cv2.LINE_AA)
    if close is not None:
        _draw_inset(frame, close)
    base = height - 18
    _shade(frame, 0, height - STATUS_BAR_H, width, height)
    step = format_caption(f"step: {caption}")
    cv2.putText(frame, step, (16, base), FONT, 0.75, WHITE, 2, cv2.LINE_AA)
    grasp_color = GREEN if grasp_attached else GREY
    cv2.putText(frame, grasp_text, (width - 420, base), FONT, 0.75, grasp_color, 2, cv2.LINE_AA)
    sim_text = f"sim t = {sim_sec:5.1f} s"
    cv2.putText(frame, sim_text, (width - 200, base), FONT, 0.65, GREY, 2, cv2.LINE_AA)
    return frame


def ffmpeg_command(out_path: str, width: int, height: int, fps: int, crf: int) -> list[str]:
    """Return the ffmpeg command that encodes raw BGR frames on stdin to H.264 MP4."""
    return [
        "ffmpeg", "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{width}x{height}",
        "-r", str(fps), "-i", "-",
        "-c:v", "libx264", "-preset", "medium", "-crf", str(crf),
        "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        out_path,
    ]  # fmt: skip


def default_output_path(now: datetime, base_dir: Path) -> Path:
    """Return ``base_dir/pick_place_<YYYYmmdd_HHMMSS>.mp4``."""
    return base_dir / f"pick_place_{now:%Y%m%d_%H%M%S}.mp4"


class FfmpegSink:
    """Pipe raw frames into an encoder process."""

    def __init__(self, cmd: list[str]) -> None:
        # Own session: a Ctrl-C in the terminal reaches only the recorder,
        # which then closes stdin so the encoder finishes the file cleanly.
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, start_new_session=True)

    def write(self, data: bytes) -> None:
        self._proc.stdin.write(data)

    def close(self, timeout: float = ENCODER_CLOSE_TIMEOUT_SEC) -> int:
        """Finish the stream and return the encoder exit code (-1 if it hangs)."""
        try:
            self._proc.stdin.close()
        except BrokenPipeError:
            pass  # encoder already exited; its exit code tells why
        try:
            return self._proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self._proc.kill()
            self._proc.wait()
            return -1


class DemoVideoRecorder(Node):
    """Turn the rig camera streams into an annotated video at simulation speed."""

    def __init__(
        self,
        sink_factory: Optional[Callable[[list[str]], FrameSink]] = None,
        **node_kwargs,
    ) -> None:
        super().__init__("demo_video_recorder", **node_kwargs)
        self._declare_parameters()
        param = self._param
        self._size = (param("width"), param("height"))
        self._fps = param("fps")
        self._title = param("title")
        self._caption_node = param("caption_node")
        self._grasp_object = param("grasp_object")
        self._out_path = param("output_path") or str(
            default_output_path(datetime.now(), DEFAULT_VIDEO_DIR)
        )
        Path(self._out_path).parent.mkdir(parents=True, exist_ok=True)
        cmd = ffmpeg_command(self._out_path, *self._size, self._fps, param("crf"))
        self._sink = (sink_factory or FfmpegSink)(cmd)

        self.caption = "waiting for the demo to start"
        self.grasp_state = "detached"
        self.failed = False
        self._close: Optional[np.ndarray] = None
        self._start_ns: Optional[int] = None
        self._written = 0
        self._exit_code: Optional[int] = None

        images = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, param("wide_topic"), self._on_wide, images)
        self.create_subscription(Image, param("close_topic"), self._on_close, images)
        self.create_subscription(Log, "/rosout", self._on_log, 100)
        grasp_topic = f"/grasp/{self._grasp_object}/state"
        self.create_subscription(String, grasp_topic, self._on_grasp_state, 10)
        # Wall clock: a stalled or missing sim must still trip the timeout.
        self._first_frame_timer = self.create_timer(
            param("first_frame_timeout_sec"),
            self._on_first_frame_timeout,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.get_logger().info(
            f"waiting for the first camera frame on {param('wide_topic')} -> {self._out_path}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "output_path": "",
            "wide_topic": DEFAULT_WIDE_TOPIC,
            "close_topic": DEFAULT_CLOSE_TOPIC,
            "caption_node": "pick_place_demo",
            "grasp_object": "cup",
            "width": DEFAULT_WIDTH,
            "height": DEFAULT_HEIGHT,
            "fps": DEFAULT_FPS,
            "crf": DEFAULT_CRF,
            "title": DEFAULT_TITLE,
            "first_frame_timeout_sec": 60.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _param(self, name: str):
        return self.get_parameter(name).value

    @property
    def recording(self) -> bool:
        """Report whether the first wide frame was converted and written."""
        return self._start_ns is not None

    def _on_close(self, msg: Image) -> None:
        try:
            self._close = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(
                f"close-up frame skipped: {exc}", throttle_duration_sec=LOG_THROTTLE_SEC
            )

    def _on_log(self, msg: Log) -> None:
        caption = caption_from_log(msg, self._caption_node)
        if caption is not None:
            self.caption = caption

    def _on_grasp_state(self, msg: String) -> None:
        self.grasp_state = msg.data

    def _on_wide(self, msg: Image) -> None:
        if self.failed:
            return
        stamp_ns = stamp_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
        start_ns = stamp_ns if self._start_ns is None else self._start_ns
        due = frames_due(start_ns, stamp_ns, self._fps, self._written)
        if due == 0:
            return
        try:
            wide = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(
                f"wide frame skipped: {exc}", throttle_duration_sec=LOG_THROTTLE_SEC
            )
            return
        if self._start_ns is None:
            self._start_ns = stamp_ns
            self.get_logger().info(
                f"{READY_MARKER}: {self._out_path} (sim t = {stamp_ns / NS_PER_SEC:.2f} s)"
            )
        self._write(self._compose(wide, stamp_ns), due)

    def _compose(self, wide: np.ndarray, stamp_ns: int) -> np.ndarray:
        grasp_text, attached = grasp_label(self._grasp_object, self.grasp_state)
        return compose_frame(
            wide,
            self._close,
            size=self._size,
            title=self._title,
            caption=self.caption,
            grasp_text=grasp_text,
            grasp_attached=attached,
            sim_sec=stamp_ns / NS_PER_SEC,
        )

    def _write(self, frame: np.ndarray, copies: int) -> None:
        data = frame.tobytes()
        try:
            for _ in range(copies):
                self._sink.write(data)
        except OSError as exc:  # BrokenPipeError: the encoder exited
            self.failed = True
            self.get_logger().error(f"video encoder stopped accepting frames: {exc}")
            return
        self._written += copies

    def _on_first_frame_timeout(self) -> None:
        self._first_frame_timer.cancel()
        if self.recording:
            return
        self.failed = True
        self.get_logger().error(
            "no camera frame received in time: is the sim running and video_rig "
            "spawned and bridged? Not starting the demo."
        )

    def finish(self) -> int:
        """Close the encoder once; return its exit code (0 = file is complete)."""
        if self._exit_code is not None:
            return self._exit_code
        self._exit_code = self._sink.close()
        if self._written == 0:
            # ffmpeg still writes a header-only, unplayable file.
            Path(self._out_path).unlink(missing_ok=True)
            self.get_logger().warn(f"no frames recorded; removed {self._out_path}")
            return self._exit_code
        seconds = self._written / self._fps
        if self._exit_code == 0:
            self.get_logger().info(
                f"video written: {self._out_path} ({self._written} frames, {seconds:.1f} s)"
            )
        else:
            self.get_logger().error(
                f"video encoder exited with code {self._exit_code}; "
                f"{self._out_path} may be incomplete"
            )
        return self._exit_code


def main() -> int:
    # Plain KeyboardInterrupt on SIGINT (launch shutdown, Ctrl-C) instead of
    # rclpy's handler, which shuts the context down first so finish() could
    # no longer log to /rosout. On SIGTERM the process just dies; ffmpeg then
    # sees EOF on its stdin and still finalizes the file.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DemoVideoRecorder()
    try:
        while rclpy.ok() and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # launch shutdown (demo finished) or Ctrl-C: finish the file
    finally:
        encoder_code = node.finish()
        failed = node.failed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if failed or encoder_code != 0 else 0


if __name__ == "__main__":
    sys.exit(main())
