from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_launch_description():
    node_name = LaunchConfiguration('node_name')
    namespace = LaunchConfiguration('namespace')
    arm_group = LaunchConfiguration('arm_group')
    hand_group = LaunchConfiguration('hand_group')
    eef_name = LaunchConfiguration('eef_name')
    hand_frame = LaunchConfiguration('hand_frame')
    ik_frame_link = LaunchConfiguration('ik_frame_link')
    service_name = LaunchConfiguration('service_name')
    gripper_open_pose = LaunchConfiguration('gripper_open_pose')
    gripper_close_pose = LaunchConfiguration('gripper_close_pose')
    arm_home_pose = LaunchConfiguration('arm_home_pose')

    moveit_config = MoveItConfigsBuilder(
        'openarm', package_name='openarm_bimanual_moveit_config').to_moveit_configs()
    moveit_config_pkg = get_package_share_directory('openarm_bimanual_moveit_config')
    joint_limits_file = os.path.join(moveit_config_pkg, 'config', 'joint_limits.yaml')
    with open(joint_limits_file, 'r', encoding='utf-8') as file:
        joint_limits = yaml.safe_load(file)

    node = Node(
        package='openarm_moveit2_pose_executor',
        executable='mtc_pick_place_node',
        name=node_name,
        namespace=namespace,
        output='screen',
        parameters=[
            {'arm_group': arm_group},
            {'hand_group': hand_group},
            {'eef_name': eef_name},
            {'hand_frame': hand_frame},
            {'ik_frame_link': ik_frame_link},
            {'service_name': service_name},
            {'gripper_open_pose': gripper_open_pose},
            {'gripper_close_pose': gripper_close_pose},
            {'arm_home_pose': arm_home_pose},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {'robot_description_planning': joint_limits},
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='mtc_pick_place_node'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('arm_group', default_value='right_arm'),
        DeclareLaunchArgument('hand_group', default_value='right_gripper'),
        DeclareLaunchArgument('eef_name', default_value='right_ee'),
        DeclareLaunchArgument('hand_frame', default_value='openarm_right_gripper_tip'),
        DeclareLaunchArgument('ik_frame_link', default_value='openarm_right_gripper_tip'),
        DeclareLaunchArgument('service_name', default_value='/execute_pick_place'),
        DeclareLaunchArgument('gripper_open_pose', default_value='open'),
        DeclareLaunchArgument('gripper_close_pose', default_value='closed'),
        DeclareLaunchArgument('arm_home_pose', default_value='home'),
        node,
    ])
