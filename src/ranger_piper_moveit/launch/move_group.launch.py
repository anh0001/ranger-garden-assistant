from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ranger_mini_complete", package_name="ranger_piper_moveit").to_moveit_configs()
    move_group_launch = generate_move_group_launch(moveit_config)

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock for move_group.",
        ),
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
                *move_group_launch.entities,
            ]
        ),
    ])
