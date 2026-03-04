from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('openarm_moveit2_pose_executor')
    params_file = os.path.join(pkg_share, 'config', 'path.yaml')
    group_name = LaunchConfiguration('group_name')
    pose_topic_mode = LaunchConfiguration('pose_topic_mode')
    pose_topic = LaunchConfiguration('pose_topic')
    eef_step = LaunchConfiguration('eef_step')
    min_fraction = LaunchConfiguration('min_fraction')
    fallback_to_pose_plan = LaunchConfiguration('fallback_to_pose_plan')
    planning_time = LaunchConfiguration('planning_time')
    num_planning_attempts = LaunchConfiguration('num_planning_attempts')
    goal_position_tolerance = LaunchConfiguration('goal_position_tolerance')
    goal_orientation_tolerance = LaunchConfiguration('goal_orientation_tolerance')

    # 加载 MoveIt 配置（包括机器人描述、语义描述和运动学配置）
    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarm_bimanual_moveit_config").to_moveit_configs()

    motion_node = Node(
        package='openarm_moveit2_pose_executor',
        executable='motion_planner_node',
        name='openarm_move_group',
        output='screen',
        parameters=[
            {'group_name': group_name},
            {'pose_topic_mode': pose_topic_mode},
            {'pose_topic': pose_topic},
            {'eef_step': eef_step},
            {'min_fraction': min_fraction},
            {'fallback_to_pose_plan': fallback_to_pose_plan},
            {'planning_time': planning_time},
            {'num_planning_attempts': num_planning_attempts},
            {'goal_position_tolerance': goal_position_tolerance},
            {'goal_orientation_tolerance': goal_orientation_tolerance},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            params_file,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('group_name', default_value='left_arm'),
        DeclareLaunchArgument('pose_topic_mode', default_value='true'),
        DeclareLaunchArgument('pose_topic', default_value='/target_pose'),
        DeclareLaunchArgument('eef_step', default_value='0.005'),
        DeclareLaunchArgument('min_fraction', default_value='0.9'),
        DeclareLaunchArgument('fallback_to_pose_plan', default_value='true'),
        DeclareLaunchArgument('planning_time', default_value='10.0'),
        DeclareLaunchArgument('num_planning_attempts', default_value='10'),
        DeclareLaunchArgument('goal_position_tolerance', default_value='0.01'),
        DeclareLaunchArgument('goal_orientation_tolerance', default_value='0.05'),
        motion_node,
    ])
