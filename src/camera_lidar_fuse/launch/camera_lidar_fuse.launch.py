from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'camera_lidar_fuse'
    default_config = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'projection_config.yaml',
    ])

    calibration_config_arg = DeclareLaunchArgument(
        'calibration_config',
        default_value=default_config,
        description='Path to a YAML file describing calibration outputs and topics.'
    )

    projection_node = Node(
        package=package_name,
        executable='projection_node',
        name='calibration_projection_node',
        output='screen',
        parameters=[{
            'calibration_config_path': LaunchConfiguration('calibration_config'),
        }],
    )

    return LaunchDescription([
        calibration_config_arg,
        projection_node,
    ])
