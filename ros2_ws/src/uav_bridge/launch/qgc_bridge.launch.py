#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    num_uav = LaunchConfiguration('num_uav')
    ns_prefix = LaunchConfiguration('ns_prefix')
    qgc_base_port = LaunchConfiguration('qgc_base_port')
    vehicle_base_port = LaunchConfiguration('vehicle_base_port')
    reference_uav = LaunchConfiguration('reference_uav')
    poll_rate_hz = LaunchConfiguration('poll_rate_hz')
    default_search_weight = LaunchConfiguration('default_search_weight')
    default_orbit_radius = LaunchConfiguration('default_orbit_radius')
    default_orbit_angular_speed = LaunchConfiguration('default_orbit_angular_speed')

    return LaunchDescription([
        DeclareLaunchArgument('num_uav', default_value='6', description='Swarm UAV count.'),
        DeclareLaunchArgument('ns_prefix', default_value='uav', description='Per-UAV namespace prefix.'),
        DeclareLaunchArgument('qgc_base_port', default_value='14540', description='QGC UDP base port.'),
        DeclareLaunchArgument('vehicle_base_port', default_value='14740', description='Vehicle UDP base port.'),
        DeclareLaunchArgument('reference_uav', default_value='1', description='Reference UAV index starting from 1.'),
        DeclareLaunchArgument('poll_rate_hz', default_value='100.0', description='UDP polling frequency.'),
        DeclareLaunchArgument('default_search_weight', default_value='1.0', description='Fallback weight for multi-point search.'),
        DeclareLaunchArgument('default_orbit_radius', default_value='12.0', description='Fallback orbit radius in meters.'),
        DeclareLaunchArgument('default_orbit_angular_speed', default_value='0.12', description='Fallback orbit angular speed in rad/s.'),
        Node(
            package='uav_bridge',
            executable='qgc_bridge',
            name='qgc_bridge',
            output='screen',
            parameters=[
                {'num_uav': num_uav},
                {'ns_prefix': ns_prefix},
                {'qgc_base_port': qgc_base_port},
                {'vehicle_base_port': vehicle_base_port},
                {'reference_uav': reference_uav},
                {'poll_rate_hz': poll_rate_hz},
                {'default_search_weight': default_search_weight},
                {'default_orbit_radius': default_orbit_radius},
                {'default_orbit_angular_speed': default_orbit_angular_speed},
            ],
        ),
    ])