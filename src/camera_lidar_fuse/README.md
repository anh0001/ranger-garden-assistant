# camera_lidar_fuse

ROS 2 Humble package that projects LiDAR point clouds into camera frame with per-point image coordinates.

## Features
- Supports pinhole and fisheye distortion models
- Publishes `/lidar_points_projected` with `x,y,z,u,v,range,intensity,ring` fields
- Configurable via YAML or ROS parameters

## Quick Start
```bash
# Build
colcon build --packages-select camera_lidar_fuse

# Launch
ros2 launch camera_lidar_fuse calibration_projection.launch.py \
  calibration_config:=/path/to/config.yaml
```

## Key Parameters
- `camera_info_path` - Camera intrinsics YAML
- `extrinsic_path` - LiDAR-to-camera extrinsics YAML
- `lidar_topic` - Input point cloud topic
- `projected_topic` - Output topic (default: `/lidar_points_projected`)
- `skip_rate` - Point cloud decimation factor

See [config/projection_config.yaml](config/projection_config.yaml) for full configuration.
