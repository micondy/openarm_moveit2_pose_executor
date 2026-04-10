from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    launch_dir = get_package_share_directory('openarm_moveit2_pose_executor')
    single_launch = os.path.join(launch_dir, 'launch', 'mtc_pick_place.launch.py')

    right_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_launch),
        launch_arguments={
            'node_name': 'mtc_pick_place_right_node',
            'namespace': '',
            'arm_group': 'right_arm',
            'hand_group': 'right_gripper',
            'eef_name': 'right_ee',
            'hand_frame': 'openarm_right_gripper_tip',
            'ik_frame_link': 'openarm_right_gripper_tip',
            'service_name': '/execute_pick_place_right',
        }.items(),
    )

    left_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_launch),
        launch_arguments={
            'node_name': 'mtc_pick_place_left_node',
            'namespace': '',
            'arm_group': 'left_arm',
            'hand_group': 'left_gripper',
            'eef_name': 'left_ee',
            'hand_frame': 'openarm_left_gripper_tip',
            'ik_frame_link': 'openarm_left_gripper_tip',
            'service_name': '/execute_pick_place_left',
        }.items(),
    )

    return LaunchDescription([
        right_node,
        left_node,
    ])
