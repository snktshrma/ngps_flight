#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
    )

    ref_tile = DeclareLaunchArgument(
        'ref_tile_path',
        default_value='',
        description='Absolute path to the reference GeoTIFF (UTM projection)',
    )

    config_file = PathJoinSubstitution([
        FindPackageShare('ap_vo2'),
        'config',
        'map_match_config.yaml',
    ])

    node = Node(
        package='ap_vo2',
        executable='map_match_node.py',
        name='map_match_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'ref_tile_path': LaunchConfiguration('ref_tile_path'),
            },
        ],
    )

    return LaunchDescription([
        use_sim_time,
        ref_tile,
        node,
    ])
