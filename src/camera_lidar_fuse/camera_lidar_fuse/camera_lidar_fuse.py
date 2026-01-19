#!/usr/bin/env python3

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


def _reshape_camera_matrix(mat_any) -> np.ndarray:
    """Convert YAML camera matrix data into a 3x3 numpy array."""
    array = np.array(mat_any, dtype=np.float64)
    if array.size == 9:
        return array.reshape(3, 3)
    if array.shape == (3, 3):
        return array
    raise ValueError('camera_matrix must contain 9 values.')


def load_camera_calibration(yaml_path: str) -> Tuple[np.ndarray, np.ndarray, str, int, int]:
    """Load camera intrinsics, distortion, and image size from a calibration YAML."""
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f'Camera calibration file not found: {yaml_path}')

    with open(yaml_path, 'r', encoding='utf-8') as file:
        calib_data = yaml.safe_load(file)

    camera_matrix = _reshape_camera_matrix(calib_data['camera_matrix']['data'])
    distortion_data = calib_data['distortion_coefficients']['data']
    distortion_coeffs = np.array(distortion_data, dtype=np.float64).reshape((1, -1))
    distortion_model = str(calib_data.get('distortion_model', 'plumb_bob'))
    image_width = int(calib_data.get('image_width', 0))
    image_height = int(calib_data.get('image_height', 0))
    if image_width <= 0 or image_height <= 0:
        raise ValueError('Camera calibration file must define positive image_width and image_height values.')

    return camera_matrix, distortion_coeffs, distortion_model, image_width, image_height


def load_extrinsic_matrix(yaml_path: str) -> np.ndarray:
    """Load a 4x4 transformation matrix from YAML."""
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f'Extrinsic calibration file not found: {yaml_path}')

    with open(yaml_path, 'r', encoding='utf-8') as file:
        data = yaml.safe_load(file)

    if 'extrinsic_matrix' not in data:
        raise KeyError("Expected key 'extrinsic_matrix' in extrinsic calibration YAML.")

    matrix = np.array(data['extrinsic_matrix'], dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError('Extrinsic matrix must be 4x4.')
    return matrix


def resolve_path(path: str, base_dir: str) -> str:
    """Resolve absolute path using base_dir when the supplied path is relative."""
    if not path:
        return ''
    if os.path.isabs(path):
        return path
    return os.path.abspath(os.path.join(base_dir, path))


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


@dataclass(frozen=True)
class FieldSpec:
    name: str
    datatype: int
    count: int
    numpy_dtype: np.dtype
    source_name: Optional[str] = None


class CalibrationProjectionNode(Node):
    """ROS 2 node that attaches image coordinates to LiDAR points."""

    def __init__(self):
        super().__init__('calibration_projection_node')

        # Declare parameters before reading configuration.
        self.declare_parameter('calibration_config_path', '')
        self.declare_parameter('camera_info_path', '')
        self.declare_parameter('extrinsic_path', '')
        self.declare_parameter('config_folder', '')
        self.declare_parameter('lidar_topic', '')
        self.declare_parameter('projected_topic', '')
        self.declare_parameter('skip_rate', 0)
        self.declare_parameter('output_frame', '')

        configuration = self._load_configuration()

        try:
            self._load_calibration_files()
        except Exception as exc:  # pragma: no cover - fatal configuration issue
            self.get_logger().fatal(f'Failed to load calibration data: {exc}')
            raise

        self.skip_rate = max(1, int(configuration.get('skip_rate', 1)))
        self.lidar_topic = configuration['lidar_topic']
        self.projected_topic = configuration['projected_topic']
        self.output_frame = configuration['output_frame']

        self.get_logger().info(f'Camera intrinsics: {self.camera_info_path}')
        self.get_logger().info(f'Extrinsics: {self.extrinsic_path}')
        self.get_logger().info(f'LiDAR topic: {self.lidar_topic}')
        self.get_logger().info(f'Projected cloud topic: {self.projected_topic}')
        self.get_logger().info(
            f'Output frame: {self.output_frame}  Skip rate: {self.skip_rate}  '
            'xyz/range remain in the LiDAR frame from the incoming message.'
        )

        self._use_fisheye = self.distortion_model.lower() == 'equidistant'
        self._zero_rvec = np.zeros((3, 1), dtype=np.float64)
        self._zero_tvec = np.zeros((3, 1), dtype=np.float64)

        self._input_dtype: Optional[np.dtype] = None
        self._fields_signature: Optional[Tuple[Tuple[str, int, int, int], ...]] = None
        self._output_dtype: Optional[np.dtype] = None
        self._output_field_specs: List[FieldSpec] = []
        self._passthrough_specs: List[FieldSpec] = []
        self._point_fields: List[PointField] = []
        self._point_step: int = 0

        self.projected_pub = self.create_publisher(PointCloud2, self.projected_topic, 10)
        self.create_subscription(PointCloud2, self.lidar_topic, self._lidar_callback, 10)

    def _load_configuration(self) -> Dict[str, str]:
        """Gather configuration from parameters and optional YAML file."""
        config_path = self.get_parameter('calibration_config_path').get_parameter_value().string_value
        camera_info_path = self.get_parameter('camera_info_path').get_parameter_value().string_value
        extrinsic_path = self.get_parameter('extrinsic_path').get_parameter_value().string_value
        config_folder_param = self.get_parameter('config_folder').get_parameter_value().string_value
        lidar_topic_param = self.get_parameter('lidar_topic').get_parameter_value().string_value
        projected_topic_param = self.get_parameter('projected_topic').get_parameter_value().string_value
        skip_rate_param = self.get_parameter('skip_rate').get_parameter_value().integer_value
        output_frame_param = self.get_parameter('output_frame').get_parameter_value().string_value

        base_dir = config_folder_param or os.path.dirname(config_path) or os.getcwd()
        base_dir = resolve_path(base_dir, os.getcwd())

        if config_path:
            config = self._read_yaml(config_path)
            config_general = config.get('general', {})
            config_camera = config.get('camera', {})
            config_lidar = config.get('lidar', {})
            config_projection = config.get('projection', {})

            base_dir_from_yaml = config_general.get('config_folder', '') or base_dir
            base_dir = resolve_path(base_dir_from_yaml, os.path.dirname(config_path))

            camera_info_path = camera_info_path or resolve_path(
                config_general.get('camera_intrinsic_calibration', ''),
                base_dir
            )
            extrinsic_path = extrinsic_path or resolve_path(
                config_general.get('camera_extrinsic_calibration', ''),
                base_dir
            )

            lidar_topic_param = lidar_topic_param or config_lidar.get('lidar_topic', '')
            projected_topic_param = projected_topic_param or config_projection.get('projected_topic', '')
            skip_rate_param = skip_rate_param or int(config_projection.get('skip_rate', 1))

            frame_id_from_yaml = (
                config_projection.get('output_frame') or
                config_lidar.get('frame_id', '') or
                config_camera.get('frame_id', '')
            )
            output_frame_param = output_frame_param or frame_id_from_yaml

        if not camera_info_path:
            raise ValueError('camera_info_path must be provided via parameter or config file.')
        if not extrinsic_path:
            raise ValueError('extrinsic_path must be provided via parameter or config file.')

        if not lidar_topic_param:
            lidar_topic_param = '/lidar_points'
        if not projected_topic_param:
            projected_topic_param = '/lidar_points_projected'
        if not output_frame_param:
            output_frame_param = 'camera_frame'

        self.camera_info_path = camera_info_path
        self.extrinsic_path = extrinsic_path
        self.base_dir = base_dir

        skip_rate = int(skip_rate_param) if skip_rate_param > 0 else 1

        return {
            'lidar_topic': lidar_topic_param,
            'projected_topic': projected_topic_param,
            'skip_rate': skip_rate,
            'output_frame': output_frame_param,
        }

    @staticmethod
    def _read_yaml(path: str) -> Dict:
        if not os.path.isfile(path):
            raise FileNotFoundError(f'Configuration YAML not found: {path}')
        with open(path, 'r', encoding='utf-8') as file:
            return yaml.safe_load(file) or {}

    def _load_calibration_files(self) -> None:
        (
            self.camera_matrix,
            self.dist_coeffs,
            self.distortion_model,
            self.image_width,
            self.image_height
        ) = load_camera_calibration(self.camera_info_path)
        self.extrinsic_matrix = load_extrinsic_matrix(self.extrinsic_path)

        self.get_logger().debug(f'Camera matrix:\n{self.camera_matrix}')
        self.get_logger().debug(f'Distortion coefficients:\n{self.dist_coeffs}')
        self.get_logger().debug(f'Distortion model: {self.distortion_model}')
        self.get_logger().debug(f'Extrinsic matrix:\n{self.extrinsic_matrix}')
        self.get_logger().debug(f'Image size: {self.image_width}x{self.image_height}')

    def _lidar_callback(self, cloud_msg: PointCloud2) -> None:
        if self.projected_pub is None:
            return

        try:
            structured = self._pointcloud_to_array(cloud_msg)
        except ValueError as exc:
            self.get_logger().error(f'Failed to parse PointCloud2: {exc}')
            return

        if structured.size == 0:
            self._publish_empty_cloud(cloud_msg.header.stamp)
            return

        data = structured
        if self.skip_rate > 1:
            data = data[::self.skip_rate]
            if data.size == 0:
                self._publish_empty_cloud(cloud_msg.header.stamp)
                return

        xyz = np.stack((data['x'], data['y'], data['z']), axis=1).astype(np.float64, copy=False)
        finite_mask = np.isfinite(xyz).all(axis=1)
        if not np.any(finite_mask):
            self._publish_empty_cloud(cloud_msg.header.stamp)
            return

        xyz_lidar = xyz[finite_mask]
        valid_indices = np.nonzero(finite_mask)[0]

        ones = np.ones((xyz_lidar.shape[0], 1), dtype=np.float64)
        xyz_cam = np.hstack((xyz_lidar, ones)) @ self.extrinsic_matrix.T
        xyz_cam = xyz_cam[:, :3]

        cam_indices, uv = self._project_into_image(xyz_cam)
        if cam_indices.size == 0 or uv.size == 0:
            self._publish_empty_cloud(cloud_msg.header.stamp)
            return

        xyz_selected = xyz_cam[cam_indices]
        xyz_lidar_selected = xyz_lidar[cam_indices]
        source_indices = valid_indices[cam_indices]

        ranges = np.sqrt(np.sum(np.square(xyz_lidar_selected), axis=1, dtype=np.float64))
        projected = np.zeros(xyz_selected.shape[0], dtype=self._output_dtype)
        projected['x'] = xyz_lidar_selected[:, 0].astype(np.float32, copy=False)
        projected['y'] = xyz_lidar_selected[:, 1].astype(np.float32, copy=False)
        projected['z'] = xyz_lidar_selected[:, 2].astype(np.float32, copy=False)
        projected['u'] = uv[:, 0].astype(np.float32, copy=False)
        projected['v'] = uv[:, 1].astype(np.float32, copy=False)
        projected['range'] = ranges.astype(np.float32, copy=False)

        for spec in self._passthrough_specs:
            projected[spec.name] = data[spec.source_name][source_indices]

        projected_msg = self._build_cloud_message(projected, cloud_msg.header.stamp)
        self.projected_pub.publish(projected_msg)

    def _project_into_image(self, xyz_cam: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Return indices of points that fall inside the image along with their (u, v)."""
        if xyz_cam.size == 0:
            return np.empty(0, dtype=np.int64), np.empty((0, 2), dtype=np.float32)

        in_front_mask = xyz_cam[:, 2] > 0.0
        if not np.any(in_front_mask):
            return np.empty(0, dtype=np.int64), np.empty((0, 2), dtype=np.float32)

        xyz_front = xyz_cam[in_front_mask]
        indices_front = np.nonzero(in_front_mask)[0]

        if self._use_fisheye:
            object_points = xyz_front.reshape(-1, 1, 3)
            image_points, _ = cv2.fisheye.projectPoints(
                object_points,
                self._zero_rvec,
                self._zero_tvec,
                self.camera_matrix,
                self.dist_coeffs
            )
        else:
            image_points, _ = cv2.projectPoints(
                xyz_front,
                self._zero_rvec,
                self._zero_tvec,
                self.camera_matrix,
                self.dist_coeffs
            )

        uv_all = image_points.reshape(-1, 2)
        finite_mask = np.isfinite(uv_all).all(axis=1)
        if not np.any(finite_mask):
            return np.empty(0, dtype=np.int64), np.empty((0, 2), dtype=np.float32)

        uv_all = uv_all[finite_mask]
        indices_front = indices_front[finite_mask]

        inside_mask = (
            (uv_all[:, 0] >= 0.0) & (uv_all[:, 0] < self.image_width) &
            (uv_all[:, 1] >= 0.0) & (uv_all[:, 1] < self.image_height)
        )
        if not np.any(inside_mask):
            return np.empty(0, dtype=np.int64), np.empty((0, 2), dtype=np.float32)

        return indices_front[inside_mask], uv_all[inside_mask]

    def _build_cloud_message(self, payload: np.ndarray, stamp) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.output_frame
        cloud.height = 1
        cloud.width = int(payload.shape[0])
        cloud.fields = self._point_fields
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.point_step = self._point_step
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = payload.tobytes()
        return cloud

    def _publish_empty_cloud(self, stamp) -> None:
        """Publish an empty PointCloud2 with the expected layout."""
        payload = np.zeros(0, dtype=self._output_dtype)
        self.projected_pub.publish(self._build_cloud_message(payload, stamp))

    def _pointcloud_to_array(self, cloud_msg: PointCloud2) -> np.ndarray:
        """Convert a PointCloud2 message into a structured numpy array."""
        self._prepare_field_layout(cloud_msg)
        if self._input_dtype is None:
            raise ValueError('PointCloud2 type information was not initialized.')
        array = np.frombuffer(cloud_msg.data, dtype=self._input_dtype)
        if cloud_msg.is_bigendian:
            array = array.byteswap().newbyteorder()
        return array

    def _prepare_field_layout(self, cloud_msg: PointCloud2) -> None:
        signature = tuple((f.name, f.offset, f.datatype, f.count) for f in cloud_msg.fields)
        if signature and signature == self._fields_signature:
            return

        if not cloud_msg.fields:
            raise ValueError('PointCloud2 message contains no fields.')

        required_axes = {'x', 'y', 'z'}
        field_names = {field.name for field in cloud_msg.fields}
        if not required_axes.issubset(field_names):
            missing = required_axes - field_names
            raise ValueError(f'PointCloud2 message is missing required fields: {missing}')

        input_dtype = self._build_input_dtype(cloud_msg.fields, cloud_msg.point_step)

        field_map = {field.name: field for field in cloud_msg.fields}
        extra_field_names = [field.name for field in cloud_msg.fields if field.name not in ('x', 'y', 'z')]

        specs: List[FieldSpec] = [
            FieldSpec('x', PointField.FLOAT32, 1, np.dtype(np.float32)),
            FieldSpec('y', PointField.FLOAT32, 1, np.dtype(np.float32)),
            FieldSpec('z', PointField.FLOAT32, 1, np.dtype(np.float32)),
            FieldSpec('u', PointField.FLOAT32, 1, np.dtype(np.float32)),
            FieldSpec('v', PointField.FLOAT32, 1, np.dtype(np.float32)),
        ]

        passthrough_specs: List[FieldSpec] = []
        if 'intensity' in extra_field_names:
            intensity_field = field_map['intensity']
            intensity_spec = self._spec_from_field(intensity_field, source_name='intensity')
            specs.append(intensity_spec)
            passthrough_specs.append(intensity_spec)
            extra_field_names.remove('intensity')

        range_spec = FieldSpec('range', PointField.FLOAT32, 1, np.dtype(np.float32))
        specs.append(range_spec)

        for name in extra_field_names:
            spec = self._spec_from_field(field_map[name], source_name=name)
            specs.append(spec)
            passthrough_specs.append(spec)

        dtype_entries = []
        for spec in specs:
            if spec.count <= 1:
                dtype_entries.append((spec.name, spec.numpy_dtype))
            else:
                dtype_entries.append((spec.name, (spec.numpy_dtype, spec.count)))

        output_dtype = np.dtype(dtype_entries)
        point_fields: List[PointField] = []
        offset = 0
        for spec in specs:
            pf = PointField()
            pf.name = spec.name
            pf.count = max(1, spec.count)
            pf.datatype = spec.datatype
            pf.offset = offset
            point_fields.append(pf)
            offset += spec.numpy_dtype.itemsize * pf.count

        self._input_dtype = input_dtype
        self._output_dtype = output_dtype
        self._point_fields = point_fields
        self._point_step = offset
        self._passthrough_specs = passthrough_specs
        self._output_field_specs = specs
        self._fields_signature = signature

    @staticmethod
    def _build_input_dtype(fields: List[PointField], point_step: int) -> np.dtype:
        names: List[str] = []
        formats: List[np.dtype] = []
        offsets: List[int] = []

        for field in fields:
            base_dtype = POINT_FIELD_TO_DTYPE.get(field.datatype)
            if base_dtype is None:
                raise ValueError(f'Unsupported PointField datatype: {field.datatype}')
            count = field.count if field.count > 0 else 1
            dtype_entry = base_dtype if count == 1 else np.dtype((base_dtype, count))
            names.append(field.name)
            formats.append(dtype_entry)
            offsets.append(field.offset)

        return np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': point_step})

    @staticmethod
    def _spec_from_field(field: PointField, source_name: str) -> FieldSpec:
        base_dtype = POINT_FIELD_TO_DTYPE.get(field.datatype)
        if base_dtype is None:
            raise ValueError(f'Unsupported PointField datatype: {field.datatype}')
        count = field.count if field.count > 0 else 1
        return FieldSpec(
            name=field.name,
            datatype=field.datatype,
            count=count,
            numpy_dtype=base_dtype,
            source_name=source_name
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CalibrationProjectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
