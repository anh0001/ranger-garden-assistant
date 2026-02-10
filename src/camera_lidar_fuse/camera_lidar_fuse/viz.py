#!/usr/bin/env python3

# CLI command to run this node:
# ros2 run camera_lidar_fuse projection_visualizer --ros-args \
#   -p image_topic:=/image_raw \
#   -p projected_topic:=/lidar_points_projected \
#   -p output_image_topic:=/camera/projected_overlay \
#   -p slop:=10.0

"""Overlay `/lidar_points_projected` onto camera images for quick visualization."""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField

# Reuse the PointField type mapping used in projection_node.
POINT_FIELD_TO_DTYPE = {
    PointField.INT8: np.dtype(np.int8),
    PointField.UINT8: np.dtype(np.uint8),
    PointField.INT16: np.dtype(np.int16),
    PointField.UINT16: np.dtype(np.uint16),
    PointField.INT32: np.dtype(np.int32),
    PointField.UINT32: np.dtype(np.uint32),
    PointField.FLOAT32: np.dtype(np.float32),
    PointField.FLOAT64: np.dtype(np.float64),
}


class ProjectedCloudVisualizer(Node):
    """Draw projected cloud pixels on top of the camera image."""

    def __init__(self) -> None:
        super().__init__("projected_cloud_visualizer")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("projected_topic", "/lidar_points_projected")
        self.declare_parameter("output_image_topic", "/camera/projected_overlay")
        self.declare_parameter("slop", 0.1)
        self.declare_parameter("max_points", 120000)
        self.declare_parameter("point_radius", 2)
        self.declare_parameter("image_encoding", "bgr8")
        self.declare_parameter("force_yuv422", False)
        self.declare_parameter("yuv422_layout", "uyvy")
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.projected_topic = self.get_parameter("projected_topic").get_parameter_value().string_value
        self.output_image_topic = self.get_parameter("output_image_topic").get_parameter_value().string_value
        self.slop = float(self.get_parameter("slop").get_parameter_value().double_value)
        self.max_points = max(1, int(self.get_parameter("max_points").get_parameter_value().integer_value))
        self.point_radius = max(1, int(self.get_parameter("point_radius").get_parameter_value().integer_value))
        self.image_encoding = self.get_parameter("image_encoding").get_parameter_value().string_value
        self.force_yuv422 = self.get_parameter("force_yuv422").get_parameter_value().bool_value
        self.yuv422_layout = self.get_parameter("yuv422_layout").get_parameter_value().string_value
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"Projected cloud topic: {self.projected_topic}")
        self.get_logger().info(f"Overlay topic: {self.output_image_topic}")
        self.get_logger().info(
            f"Sync slop: {self.slop}s  Max points: {self.max_points}  Radius: {self.point_radius}"
        )
        self.get_logger().info(
            f"Image encoding: {self.image_encoding}  Force YUV422: {self.force_yuv422}  "
            f"YUV422 layout: {self.yuv422_layout}"
        )

        self.bridge = CvBridge()
        self.pub_overlay = self.create_publisher(Image, self.output_image_topic, 1)

        self.image_sub = Subscriber(self, Image, self.image_topic)
        self.cloud_sub = Subscriber(self, PointCloud2, self.projected_topic)

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.cloud_sub],
            queue_size=5,
            slop=self.slop,
        )
        self.ts.registerCallback(self._sync_callback)

        self._input_dtype: Optional[np.dtype] = None
        self._fields_signature: Optional[Tuple[Tuple[str, int, int, int], ...]] = None
        self._has_range = False
        self._has_intensity = False
        self._has_xyz = False

    def _sync_callback(self, image_msg: Image, cloud_msg: PointCloud2) -> None:
        try:
            cloud = self._pointcloud_to_array(cloud_msg)
        except ValueError as exc:
            self.get_logger().error(f"Failed to parse projected cloud: {exc}")
            return

        if cloud.size == 0:
            return

        try:
            cv_image = self._decode_image(image_msg)
        except Exception as exc:
            self.get_logger().error(f"Failed to decode image: {exc}")
            return
        h, w = cv_image.shape[:2]

        u = cloud["u"].astype(np.float64)
        v = cloud["v"].astype(np.float64)
        finite_mask = np.isfinite(u) & np.isfinite(v)
        if not np.any(finite_mask):
            return

        u = u[finite_mask]
        v = v[finite_mask]

        inside_mask = (u >= 0.0) & (u < w) & (v >= 0.0) & (v < h)
        if not np.any(inside_mask):
            return

        u = u[inside_mask]
        v = v[inside_mask]

        cloud = cloud[finite_mask][inside_mask]

        if cloud.shape[0] > self.max_points:
            step = math.ceil(cloud.shape[0] / self.max_points)
            cloud = cloud[::step]
            u = u[::step]
            v = v[::step]

        distances = self._extract_range(cloud)
        colors = self._colorize(distances)

        # Define a small region of interest to report a dominant distance/point.
        roi_width = max(int(0.14 * w), 60)
        roi_height = max(int(0.12 * h), 40)
        center_x = w / 2.0
        center_y = min(h - roi_height // 2 - 1, int(h * 0.6))
        roi_x1 = max(0, int(round(center_x - roi_width / 2.0)))
        roi_y1 = max(0, center_y - roi_height // 2)
        roi_x2 = min(w, roi_x1 + roi_width)
        roi_y2 = min(h, roi_y1 + roi_height)
        roi_mask = (u >= roi_x1) & (u < roi_x2) & (v >= roi_y1) & (v < roi_y2)

        roi_distances = distances[roi_mask] if distances.size else np.array([], dtype=np.float64)
        roi_xyz = None
        if self._has_xyz:
            roi_xyz = np.stack((cloud["x"], cloud["y"], cloud["z"]), axis=1).astype(np.float64, copy=False)
            roi_xyz = roi_xyz[roi_mask]
        elif all(name in cloud.dtype.names for name in ("x_lidar", "y_lidar", "z_lidar")):
            roi_xyz = np.stack(
                (cloud["x_lidar"], cloud["y_lidar"], cloud["z_lidar"]),
                axis=1
            ).astype(np.float64, copy=False)
            roi_xyz = roi_xyz[roi_mask]

        distance_text, text_color = self._summarize_distance(roi_distances)
        xyz_text, xyz_text_color = self._summarize_xyz(roi_xyz)

        if roi_x2 - roi_x1 > 1 and roi_y2 - roi_y1 > 1:
            cv2.rectangle(cv_image, (roi_x1, roi_y1), (roi_x2 - 1, roi_y2 - 1), (0, 255, 0), 2)

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        (distance_width, distance_height), _ = cv2.getTextSize(distance_text, font, font_scale, thickness)
        (xyz_width, xyz_height), _ = cv2.getTextSize(xyz_text, font, font_scale, thickness)

        distance_x = max(0, int(round((roi_x1 + roi_x2) * 0.5 - distance_width * 0.5)))
        distance_y = min(h - 6, roi_y2 - 8)
        distance_origin = (distance_x, max(roi_y1 + distance_height + 4, distance_y))

        xyz_x = max(0, int(round((roi_x1 + roi_x2) * 0.5 - xyz_width * 0.5)))
        xyz_target_y = distance_origin[1] - distance_height - 6
        xyz_y = max(roi_y1 + xyz_height + 4, xyz_target_y)
        xyz_origin = (xyz_x, xyz_y)

        cv2.putText(cv_image, xyz_text, xyz_origin, font, font_scale, xyz_text_color, thickness, cv2.LINE_AA)
        cv2.putText(cv_image, distance_text, distance_origin, font, font_scale, text_color, thickness, cv2.LINE_AA)

        for (u_val, v_val), color in zip(zip(u, v), colors):
            u_int = int(round(u_val))
            v_int = int(round(v_val))
            cv2.circle(cv_image, (u_int, v_int), self.point_radius, tuple(int(c) for c in color.tolist()), -1)

        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        out_msg.header = image_msg.header
        self.pub_overlay.publish(out_msg)

    def _decode_image(self, image_msg: Image) -> np.ndarray:
        encoding = (image_msg.encoding or "").lower()
        if self.force_yuv422 or encoding in ("yuv422", "uyvy", "uyvy422", "yuyv", "yuy2"):
            return self._decode_yuv422(image_msg, encoding)
        return self.bridge.imgmsg_to_cv2(image_msg, desired_encoding=self.image_encoding)

    def _decode_yuv422(self, image_msg: Image, encoding: str) -> np.ndarray:
        layout = (self.yuv422_layout or "uyvy").strip().lower()
        if encoding in ("uyvy", "uyvy422"):
            layout = "uyvy"
        elif encoding in ("yuyv", "yuy2"):
            layout = "yuyv"

        height = int(image_msg.height)
        width = int(image_msg.width)
        if height <= 0 or width <= 0:
            raise ValueError("Image size is invalid for YUV422 decoding.")

        row_stride = int(image_msg.step) if image_msg.step else width * 2
        expected_stride = width * 2
        if row_stride < expected_stride:
            raise ValueError(
                f"Image step ({row_stride}) is smaller than expected ({expected_stride}) for YUV422."
            )

        data = np.frombuffer(image_msg.data, dtype=np.uint8)
        required = row_stride * height
        if data.size < required:
            raise ValueError(f"Image data is too small ({data.size} bytes) for YUV422 decoding.")

        data = data[:required].reshape((height, row_stride))[:, :expected_stride]
        yuv = np.ascontiguousarray(data.reshape((height, width, 2)))

        if layout == "uyvy":
            return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_UYVY)
        if layout == "yuyv":
            return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUY2)

        raise ValueError(f"Unsupported YUV422 layout '{layout}'. Use 'uyvy' or 'yuyv'.")

    def _extract_range(self, cloud: np.ndarray) -> np.ndarray:
        if self._has_range:
            distances = cloud["range"].astype(np.float64, copy=False)
        elif self._has_xyz:
            distances = np.sqrt(
                np.square(cloud["x"], dtype=np.float64)
                + np.square(cloud["y"], dtype=np.float64)
                + np.square(cloud["z"], dtype=np.float64)
            )
        else:
            distances = np.zeros(cloud.shape[0], dtype=np.float64)
        return distances

    @staticmethod
    def _colorize(distances: np.ndarray) -> np.ndarray:
        if distances.size == 0:
            return np.zeros((0, 3), dtype=np.uint8)

        finite_mask = np.isfinite(distances)
        if not np.any(finite_mask):
            return np.zeros((distances.size, 3), dtype=np.uint8)

        valid = distances[finite_mask]
        d_min = float(np.min(valid))
        d_max = float(np.max(valid))
        span = d_max - d_min
        if span < 1e-6:
            norm = np.zeros_like(valid, dtype=np.float64)
        else:
            norm = (valid - d_min) / span

        colormap_input = (norm * 255.0).astype(np.uint8).reshape(-1, 1)
        colors_valid = cv2.applyColorMap(colormap_input, cv2.COLORMAP_TURBO).reshape(-1, 3)

        colors = np.zeros((distances.size, 3), dtype=np.uint8)
        colors[finite_mask] = colors_valid
        return colors

    def _pointcloud_to_array(self, cloud_msg: PointCloud2) -> np.ndarray:
        self._prepare_field_layout(cloud_msg)
        if self._input_dtype is None:
            raise ValueError("PointCloud2 type information was not initialized.")
        array = np.frombuffer(cloud_msg.data, dtype=self._input_dtype)
        if cloud_msg.is_bigendian:
            array = array.byteswap().newbyteorder()
        return array

    def _prepare_field_layout(self, cloud_msg: PointCloud2) -> None:
        signature = tuple((f.name, f.offset, f.datatype, f.count) for f in cloud_msg.fields)
        if signature and signature == self._fields_signature:
            return

        if not cloud_msg.fields:
            raise ValueError("PointCloud2 message contains no fields.")

        field_names = {field.name for field in cloud_msg.fields}
        required = {"u", "v"}
        if not required.issubset(field_names):
            missing = required - field_names
            raise ValueError(f"Projected cloud missing fields: {missing}")

        self._has_range = "range" in field_names
        self._has_intensity = "intensity" in field_names
        self._has_xyz = {"x", "y", "z"}.issubset(field_names)

        self._input_dtype = self._build_input_dtype(cloud_msg.fields, cloud_msg.point_step)
        self._fields_signature = signature

    @staticmethod
    def _build_input_dtype(fields: List[PointField], point_step: int) -> np.dtype:
        names: List[str] = []
        formats: List[np.dtype] = []
        offsets: List[int] = []

        for field in fields:
            base_dtype = POINT_FIELD_TO_DTYPE.get(field.datatype)
            if base_dtype is None:
                raise ValueError(f"Unsupported PointField datatype: {field.datatype}")
            count = field.count if field.count > 0 else 1
            dtype_entry = base_dtype if count == 1 else np.dtype((base_dtype, count))
            names.append(field.name)
            formats.append(dtype_entry)
            offsets.append(field.offset)

        return np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": point_step})

    @staticmethod
    def _summarize_distance(distances: np.ndarray) -> Tuple[str, Tuple[int, int, int]]:
        if distances.size == 0:
            return "N/A", (0, 255, 255)
        precision_step = 0.02  # 2 cm buckets
        rounded = np.round(distances / precision_step) * precision_step
        unique_vals, counts = np.unique(rounded, return_counts=True)
        dominant_idx = int(np.argmax(counts))
        dominant_distance = float(unique_vals[dominant_idx])
        return f"{dominant_distance:.2f} m", (0, 255, 0)

    @staticmethod
    def _summarize_xyz(points: Optional[np.ndarray]) -> Tuple[str, Tuple[int, int, int]]:
        if points is None or points.size == 0:
            return "x:N/A y:N/A z:N/A", (0, 255, 255)
        mode_resolution = 0.05  # 5 cm buckets
        mode_buckets = np.round(points / mode_resolution) * mode_resolution
        unique_points, point_counts = np.unique(mode_buckets, axis=0, return_counts=True)
        dominant_idx = int(np.argmax(point_counts))
        dominant_xyz = unique_points[dominant_idx]
        return f"x:{dominant_xyz[0]:.2f} y:{dominant_xyz[1]:.2f} z:{dominant_xyz[2]:.2f} m", (0, 255, 0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ProjectedCloudVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
