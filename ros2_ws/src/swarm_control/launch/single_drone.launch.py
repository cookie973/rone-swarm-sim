"""single_drone.launch.py — 启动单架无人机控制节点（调试用）

用法：
    ros2 launch swarm_control single_drone.launch.py drone_id:=1
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('swarm_control')
    params_file = os.path.join(pkg_share, 'config', 'swarm_params.yaml')

    drone_id_arg = DeclareLaunchArgument(
        'drone_id', default_value='1',
        description='无人机编号 (1-6)',
    )

    drone_node = Node(
        package='swarm_control',
        executable='drone_controller_node',
        name=['drone_controller_', LaunchConfiguration('drone_id')],
        parameters=[
            params_file,
            {'drone_id': LaunchConfiguration('drone_id')},
        ],
        output='screen',
    )

    return LaunchDescription([drone_id_arg, drone_node])
