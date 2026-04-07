from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_control',
            executable='swarm_ground_station',
            name='swarm_ground_station',
            output='screen',
        ),
    ])
