from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarm_bimanual_moveit_config").to_moveit_configs()

    left_node = Node(
        package='openarm_moveit2_pose_executor',
        executable='motion_planner_node',
        name='openarm_left_move_group',
        output='screen',
        parameters=[
            {'group_name': 'left_arm'},
            {'pose_topic_mode': True},
            {'pose_topic': '/left_target_pose'},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    right_node = Node(
        package='openarm_moveit2_pose_executor',
        executable='motion_planner_node',
        name='openarm_right_move_group',
        output='screen',
        parameters=[
            {'group_name': 'right_arm'},
            {'pose_topic_mode': True},
            {'pose_topic': '/right_target_pose'},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([left_node, right_node])