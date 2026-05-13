#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ap_ngps_ros2'),
            'config',
            'ngps_config.yaml',
        ]),
        description='Path to the NGPS configuration file',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )
    ngps_node = Node(
        package='ap_ngps_ros2',
        executable='ngps_localization_node.py',
        name='ngps_localization_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )
    return LaunchDescription([config_arg, use_sim_time_arg, ngps_node])
