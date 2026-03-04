from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('openarm_moveit2_pose_executor')
    params_file = os.path.join(pkg_share, 'config', 'path.yaml')

    # 加载 MoveIt 配置（包括机器人描述、语义描述和运动学配置）
    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarm_bimanual_moveit_config").to_moveit_configs()

    motion_node = Node(
        package='openarm_moveit2_pose_executor',
        executable='openarm_moveit2_pose_executor_node',
        name='openarm_move_group',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            params_file,
        ],
    )

    return LaunchDescription([motion_node])
