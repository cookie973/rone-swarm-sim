"""formation_cmd.launch.py — 集中式队形指挥官启动

用法：
    ros2 launch swarm_control formation_cmd.launch.py
    ros2 launch swarm_control formation_cmd.launch.py spacing:=8.0 formation:=t_shape
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value='6'),
        DeclareLaunchArgument('target_altitude', default_value='5.0'),
        DeclareLaunchArgument('control_rate', default_value='6.0'),
        DeclareLaunchArgument('spacing', default_value='6.0'),
        DeclareLaunchArgument('formation', default_value='line_x'),
        DeclareLaunchArgument('hold_cycles', default_value='1'),
        DeclareLaunchArgument('xy_tolerance_reconfig', default_value='1.2'),
        DeclareLaunchArgument('z_tolerance_reconfig', default_value='0.45'),
        DeclareLaunchArgument('max_step_xy', default_value='2.5'),
        DeclareLaunchArgument('max_step_z', default_value='1.0'),
        DeclareLaunchArgument('max_step_xy_reconfig', default_value='6.0'),
        DeclareLaunchArgument('max_step_z_reconfig', default_value='2.0'),
        DeclareLaunchArgument('path_conflict_distance', default_value='4.0'),
        DeclareLaunchArgument('path_conflict_penalty', default_value='1000.0'),
        DeclareLaunchArgument('planning_snap_tolerance', default_value='2.0'),
        DeclareLaunchArgument('search_group_spacing', default_value='5.0'),
        DeclareLaunchArgument('search_snap_tolerance', default_value='3.0'),
        DeclareLaunchArgument('orbit_radius_default', default_value='12.0'),
        DeclareLaunchArgument('orbit_angular_speed_default', default_value='0.12'),
        DeclareLaunchArgument('mission_step_xy_reconfig_scale', default_value='0.25'),
        DeclareLaunchArgument('mission_step_xy_xy_scale', default_value='0.15'),
        DeclareLaunchArgument('mission_step_xy_queue_scale', default_value='0.25'),

        Node(
            package='swarm_control',
            executable='formation_commander_node',
            name='formation_commander',
            parameters=[{
                'num_drones':        LaunchConfiguration('num_drones'),
                'target_altitude':   LaunchConfiguration('target_altitude'),
                'control_rate':      LaunchConfiguration('control_rate'),
                'spacing':           LaunchConfiguration('spacing'),
                'hold_cycles':       LaunchConfiguration('hold_cycles'),
                'xy_tolerance_reconfig': LaunchConfiguration('xy_tolerance_reconfig'),
                'z_tolerance_reconfig': LaunchConfiguration('z_tolerance_reconfig'),
                'max_step_xy':       LaunchConfiguration('max_step_xy'),
                'max_step_z':        LaunchConfiguration('max_step_z'),
                'max_step_xy_reconfig': LaunchConfiguration('max_step_xy_reconfig'),
                'max_step_z_reconfig':  LaunchConfiguration('max_step_z_reconfig'),
                'path_conflict_distance': LaunchConfiguration('path_conflict_distance'),
                'path_conflict_penalty': LaunchConfiguration('path_conflict_penalty'),
                'planning_snap_tolerance': LaunchConfiguration('planning_snap_tolerance'),
                'search_group_spacing': LaunchConfiguration('search_group_spacing'),
                'search_snap_tolerance': LaunchConfiguration('search_snap_tolerance'),
                'orbit_radius_default': LaunchConfiguration('orbit_radius_default'),
                'orbit_angular_speed_default': LaunchConfiguration('orbit_angular_speed_default'),
                'mission_step_xy_reconfig_scale': LaunchConfiguration('mission_step_xy_reconfig_scale'),
                'mission_step_xy_xy_scale': LaunchConfiguration('mission_step_xy_xy_scale'),
                'mission_step_xy_queue_scale': LaunchConfiguration('mission_step_xy_queue_scale'),
                'default_formation': LaunchConfiguration('formation'),
            }],
            output='screen',
        ),
    ])
