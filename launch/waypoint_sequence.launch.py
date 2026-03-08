from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    group_name = LaunchConfiguration('group_name')
    planning_time = LaunchConfiguration('planning_time')
    num_planning_attempts = LaunchConfiguration('num_planning_attempts')
    goal_position_tolerance = LaunchConfiguration('goal_position_tolerance')
    goal_orientation_tolerance = LaunchConfiguration('goal_orientation_tolerance')
    max_velocity_scaling_factor = LaunchConfiguration('max_velocity_scaling_factor')
    max_acceleration_scaling_factor = LaunchConfiguration('max_acceleration_scaling_factor')
    try_cartesian_first = LaunchConfiguration('try_cartesian_first')
    cartesian_eef_step = LaunchConfiguration('cartesian_eef_step')
    cartesian_min_fraction = LaunchConfiguration('cartesian_min_fraction')
    wait_seconds_between_targets = LaunchConfiguration('wait_seconds_between_targets')
    stop_on_failure = LaunchConfiguration('stop_on_failure')
    orientation_is_wxyz = LaunchConfiguration('orientation_is_wxyz')
    target_poses_raw = LaunchConfiguration('target_poses_raw')

    moveit_config = MoveItConfigsBuilder(
        'openarm', package_name='openarm_bimanual_moveit_config').to_moveit_configs()

    sequence_node = Node(
        package='openarm_moveit2_pose_executor',
        executable='waypoint_sequence_node',
        name='waypoint_sequence_node',
        output='screen',
        parameters=[
            {'group_name': group_name},
            {'planning_time': planning_time},
            {'num_planning_attempts': num_planning_attempts},
            {'goal_position_tolerance': goal_position_tolerance},
            {'goal_orientation_tolerance': goal_orientation_tolerance},
            {'max_velocity_scaling_factor': max_velocity_scaling_factor},
            {'max_acceleration_scaling_factor': max_acceleration_scaling_factor},
            {'try_cartesian_first': try_cartesian_first},
            {'cartesian_eef_step': cartesian_eef_step},
            {'cartesian_min_fraction': cartesian_min_fraction},
            {'wait_seconds_between_targets': wait_seconds_between_targets},
            {'stop_on_failure': stop_on_failure},
            {'orientation_is_wxyz': orientation_is_wxyz},
            {'target_poses_raw': target_poses_raw},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('group_name', default_value='right_arm'),
        DeclareLaunchArgument('planning_time', default_value='3.0'),
        DeclareLaunchArgument('num_planning_attempts', default_value='2'),
        DeclareLaunchArgument('goal_position_tolerance', default_value='0.01'),
        DeclareLaunchArgument('goal_orientation_tolerance', default_value='3.14159'),
        DeclareLaunchArgument('max_velocity_scaling_factor', default_value='0.8'),
        DeclareLaunchArgument('max_acceleration_scaling_factor', default_value='0.8'),
        DeclareLaunchArgument('try_cartesian_first', default_value='true'),
        DeclareLaunchArgument('cartesian_eef_step', default_value='0.01'),
        DeclareLaunchArgument('cartesian_min_fraction', default_value='0.9'),
        DeclareLaunchArgument('wait_seconds_between_targets', default_value='0.2'),
        DeclareLaunchArgument('stop_on_failure', default_value='false'),
        DeclareLaunchArgument('orientation_is_wxyz', default_value='true'),
        DeclareLaunchArgument('target_poses_raw', default_value=''),
        sequence_node,
    ])
