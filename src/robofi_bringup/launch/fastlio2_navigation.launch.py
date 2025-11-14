#!/usr/bin/env python3

"""
Ranger Garden Assistant bringup that wires FASTLIO2_ROS2 (LIO + PGO + Localizer),
OctoMap server, and Nav2 on top of the standard robot + Livox stack.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            "nav2_autostart",
            default_value="true",
            description="Autostart Nav2 lifecycle managers.",
        ),
        DeclareLaunchArgument(
            "nav2_use_amcl",
            default_value="false",
            description="Also start AMCL localization alongside FASTLIO2 stack.",
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
    nav2_autostart = LaunchConfiguration("nav2_autostart")
    nav2_use_amcl = LaunchConfiguration("nav2_use_amcl")

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

    octomap_node = Node(
        package="octomap_server2",
        executable="octomap_server",
        name="octomap_server",
        output="screen",
        parameters=[octomap_config, {"use_sim_time": use_sim_time}],
        remappings=[("cloud_in", octomap_point_topic)],
        condition=IfCondition(launch_octomap),
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

    return LaunchDescription(
        declared_arguments
        + [
            base_bringup,
            fastlio2_node,
            pgo_node,
            localizer_node,
            octomap_node,
            nav2_launch,
        ]
    )
