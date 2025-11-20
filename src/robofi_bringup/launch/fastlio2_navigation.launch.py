#!/usr/bin/env python3

"""
Ranger Garden Assistant bringup that wires FASTLIO2_ROS2 (LIO + PGO + Localizer),
OctoMap server, and Nav2 on top of the standard robot + Livox stack.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock for every node in this bringup.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Optional ROS namespace for the Nav2 stack.",
        ),
        DeclareLaunchArgument(
            "lio_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robofi_bringup"), "config", "fastlio2_lio.yaml"]
            ),
            description="FASTLIO2 LIO configuration YAML.",
        ),
        DeclareLaunchArgument(
            "pgo_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robofi_bringup"), "config", "fastlio2_pgo.yaml"]
            ),
            description="Pose graph optimization configuration YAML.",
        ),
        DeclareLaunchArgument(
            "localizer_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("robofi_bringup"),
                    "config",
                    "fastlio2_localizer.yaml",
                ]
            ),
            description="FASTLIO2 relocalization configuration YAML.",
        ),
        DeclareLaunchArgument(
            "octomap_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robofi_bringup"), "config", "octomap_server.yaml"]
            ),
            description="OctoMap server parameter YAML.",
        ),
        DeclareLaunchArgument(
            "octomap_point_topic",
            default_value="/fastlio2/world_cloud",
            description="PointCloud2 topic fed into OctoMap.",
        ),
        DeclareLaunchArgument(
            "launch_pgo",
            default_value="true",
            description="Start the pose graph optimization node.",
        ),
        DeclareLaunchArgument(
            "launch_localizer",
            default_value="true",
            description="Start the relocalizer node (load maps via /localizer/relocalize).",
        ),
        DeclareLaunchArgument(
            "launch_octomap",
            default_value="true",
            description="Enable OctoMap server for 3D + projected 2D maps.",
        ),
        DeclareLaunchArgument(
            "launch_nav2",
            default_value="true",
            description="Launch Nav2 navigation stack.",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robofi_bringup"), "config", "nav2_params.yaml"]
            ),
            description="Nav2 parameters YAML.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Optional map YAML to pass straight to Nav2.",
        ),
        DeclareLaunchArgument(
            "map_path",
            default_value="",
            description="FASTLIO2 .pcd map used to auto-trigger /localizer/relocalize.",
        ),
        DeclareLaunchArgument(
            "map_initial_x",
            default_value="0.0",
            description="Initial pose X (meters) used when relocalizing against the saved map.",
        ),
        DeclareLaunchArgument(
            "map_initial_y",
            default_value="0.0",
            description="Initial pose Y (meters) used when relocalizing against the saved map.",
        ),
        DeclareLaunchArgument(
            "map_initial_z",
            default_value="0.0",
            description="Initial pose Z (meters) used when relocalizing against the saved map.",
        ),
        DeclareLaunchArgument(
            "map_initial_roll",
            default_value="0.0",
            description="Initial pose roll (radians) for relocalization.",
        ),
        DeclareLaunchArgument(
            "map_initial_pitch",
            default_value="0.0",
            description="Initial pose pitch (radians) for relocalization.",
        ),
        DeclareLaunchArgument(
            "map_initial_yaw",
            default_value="0.0",
            description="Initial pose yaw (radians) for relocalization.",
        ),
        DeclareLaunchArgument(
            "nav2_autostart",
            default_value="true",
            description="Autostart Nav2 lifecycle managers.",
        ),
        DeclareLaunchArgument(
            "nav2_use_amcl",
            default_value="false",
            description="Also start AMCL localization alongside FASTLIO2 stack.",
        ),
        DeclareLaunchArgument(
            "auto_relocalize",
            default_value="true",
            description="Automatically call /localizer/relocalize when map_path is provided.",
        ),
        DeclareLaunchArgument(
            "auto_relocalize_timeout",
            default_value="30.0",
            description="Seconds to wait for /localizer/relocalize before giving up.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Start RViz preloaded with FASTLIO2 visualization config.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robofi_bringup"), "rviz", "fastlio2_mapping.rviz"]
            ),
            description="RViz configuration file used when launch_rviz is true.",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    lio_config = LaunchConfiguration("lio_config")
    pgo_config = LaunchConfiguration("pgo_config")
    localizer_config = LaunchConfiguration("localizer_config")
    octomap_config = LaunchConfiguration("octomap_config")
    octomap_point_topic = LaunchConfiguration("octomap_point_topic")
    launch_pgo = LaunchConfiguration("launch_pgo")
    launch_localizer = LaunchConfiguration("launch_localizer")
    launch_octomap = LaunchConfiguration("launch_octomap")
    launch_nav2 = LaunchConfiguration("launch_nav2")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nav2_map = LaunchConfiguration("map")
    map_path = LaunchConfiguration("map_path")
    map_initial_x = LaunchConfiguration("map_initial_x")
    map_initial_y = LaunchConfiguration("map_initial_y")
    map_initial_z = LaunchConfiguration("map_initial_z")
    map_initial_roll = LaunchConfiguration("map_initial_roll")
    map_initial_pitch = LaunchConfiguration("map_initial_pitch")
    map_initial_yaw = LaunchConfiguration("map_initial_yaw")
    auto_relocalize = LaunchConfiguration("auto_relocalize")
    auto_relocalize_timeout = LaunchConfiguration("auto_relocalize_timeout")
    nav2_autostart = LaunchConfiguration("nav2_autostart")
    nav2_use_amcl = LaunchConfiguration("nav2_use_amcl")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    robofi_share = FindPackageShare("robofi_bringup")

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robofi_share, "launch", "ranger_complete_bringup.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time, "use_rviz": "false"}.items(),
    )

    fastlio2_node = Node(
        package="fastlio2",
        executable="lio_node",
        name="lio_node",
        namespace="fastlio2",
        output="screen",
        parameters=[{"config_path": lio_config}, {"use_sim_time": use_sim_time}],
    )

    # Bridge FASTLIO2's body frame (mounted at 45° pitch) to the robot base frame
    # so the URDF stays level in RViz and controllers consume base_footprint/base_link.
    # Transform derived from ranger_complete.urdf.xacro defaults:
    # base_footprint -> base_link: (0, 0, 0.33), rpy (0, 0, 0)
    # base_link -> lidar_link: (0.33, 0, 0), rpy (0, pi/4, 0)
    # Inverse gives fastlio2_body -> base_footprint: (0, 0, -0.46669), rpy (0, -pi/4, 0).
    static_tf_fastlio2_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fastlio2_body_to_base",
        output="screen",
        arguments=["0", "0", "-0.46669", "0", "-0.785398", "0", "fastlio2_body", "base_footprint"],
    )

    pgo_node = Node(
        package="pgo",
        executable="pgo_node",
        name="pgo_node",
        namespace="pgo",
        output="screen",
        parameters=[{"config_path": pgo_config}, {"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_pgo),
    )

    localizer_node = Node(
        package="localizer",
        executable="localizer_node",
        name="localizer_node",
        namespace="localizer",
        output="screen",
        parameters=[{"config_path": localizer_config}, {"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_localizer),
    )

    auto_relocalize_condition = IfCondition(
        PythonExpression(
            [
                "'",
                launch_localizer,
                "' == 'true' and '",
                auto_relocalize,
                "' == 'true' and len('",
                map_path,
                "'.strip()) > 0",
            ]
        )
    )

    auto_relocalize_node = Node(
        package="robofi_bringup",
        executable="fastlio2_auto_relocalize.py",
        name="fastlio2_auto_relocalize",
        output="screen",
        parameters=[
            {
                "map_path": map_path,
                "initial_x": map_initial_x,
                "initial_y": map_initial_y,
                "initial_z": map_initial_z,
                "initial_roll": map_initial_roll,
                "initial_pitch": map_initial_pitch,
                "initial_yaw": map_initial_yaw,
                "timeout_sec": auto_relocalize_timeout,
            }
        ],
        condition=auto_relocalize_condition,
    )

    octomap_node = Node(
        package="octomap_server2",
        executable="octomap_server",
        name="octomap_server",
        namespace="",
        output="screen",
        parameters=[octomap_config, {"use_sim_time": use_sim_time}],
        remappings=[("cloud_in", octomap_point_topic)],
        condition=IfCondition(launch_octomap),
        respawn=True,
        respawn_delay=2.0,
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robofi_share, "launch", "navigation.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "map": nav2_map,
            "autostart": nav2_autostart,
            "namespace": namespace,
            "use_amcl": nav2_use_amcl,
        }.items(),
        condition=IfCondition(launch_nav2),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="fastlio2_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        declared_arguments
        + [
            base_bringup,
            fastlio2_node,
            pgo_node,
            localizer_node,
            auto_relocalize_node,
            octomap_node,
            nav2_launch,
            rviz_node,
            static_tf_fastlio2_to_base,
        ]
    )
