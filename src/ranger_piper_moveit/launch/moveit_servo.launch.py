from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock for MoveIt Servo.",
    )
    cartesian_command_in_topic_arg = DeclareLaunchArgument(
        "cartesian_command_in_topic",
        default_value="/servo_node/delta_twist_cmds",
        description="Cartesian command topic consumed by MoveIt Servo.",
    )
    joint_command_in_topic_arg = DeclareLaunchArgument(
        "joint_command_in_topic",
        default_value="/servo_node/delta_joint_cmds",
        description="JointJog command topic consumed by MoveIt Servo.",
    )

    moveit_config = MoveItConfigsBuilder(
        "ranger_mini_complete",
        package_name="ranger_piper_moveit",
    ).to_moveit_configs()

    servo_config = PathJoinSubstitution(
        [FindPackageShare("ranger_piper_moveit"), "config", "moveit_servo.yaml"]
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_config,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                "moveit_servo.cartesian_command_in_topic": ParameterValue(
                    LaunchConfiguration("cartesian_command_in_topic"), value_type=str
                ),
                "moveit_servo.joint_command_in_topic": ParameterValue(
                    LaunchConfiguration("joint_command_in_topic"), value_type=str
                ),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )
            },
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        cartesian_command_in_topic_arg,
        joint_command_in_topic_arg,
        servo_node,
    ])
