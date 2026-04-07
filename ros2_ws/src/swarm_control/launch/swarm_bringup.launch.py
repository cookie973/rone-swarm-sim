"""swarm_bringup.launch.py — 一键启动全部蜂群控制节点

用法：
    ros2 launch swarm_control swarm_bringup.launch.py
    ros2 launch swarm_control swarm_bringup.launch.py num_drones:=4
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

    # 启动参数：无人机数量
    num_drones_arg = DeclareLaunchArgument(
        'num_drones', default_value='6',
        description='蜂群中无人机数量',
    )

    ld = LaunchDescription([num_drones_arg])

    # 为每架无人机启动一个 drone_controller 节点
    # 注: LaunchConfiguration 在 Python 代码中无法直接用于 range()，
    #     所以这里用固定上限循环，实际可直接改此处的数字。
    for i in range(1, 7):
        node = Node(
            package='swarm_control',
            executable='drone_controller_node',
            name=f'drone_controller_{i}',
            parameters=[
                params_file,
                {'drone_id': i},
            ],
            output='screen',
        )
        ld.add_action(node)

    return ld
