#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    reference_image_arg = DeclareLaunchArgument(
        'reference_image_path',
        default_value='',
        description='Path to the reference image for NGPS localization'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera topic to subscribe to'
    )
    
    ngps_config_arg = DeclareLaunchArgument(
        'ngps_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ap_ngps_ros2'),
            'config',
            'ngps_config.yaml'
        ]),
        description='Path to the NGPS configuration file'
    )
    
    vips_config_arg = DeclareLaunchArgument(
        'vips_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('vins'),
            'config',
            'high_alt',
            'high_alt_mono_imu_config.yaml'
        ]),
        description='Path to the VIPS configuration file'
    )
    
    ukf_config_arg = DeclareLaunchArgument(
        'ukf_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ap_ukf'),
            'params',
            'estimator_config.yaml'
        ]),
        description='Path to the ap_ukf (fusion_ros) configuration file'
    )

    fusion_backend_arg = DeclareLaunchArgument(
        'fusion_backend',
        default_value='ap_ukf',
        description='Fusion: ap_ukf (custom UKF) or robot_localization (ekf_node)'
    )

    rl_config_arg = DeclareLaunchArgument(
        'robot_localization_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ap_ngps_ros2'),
            'config',
            'rl_ekf.yaml'
        ]),
        description='robot_localization ekf_node YAML'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
    )
    
    ngps_node = Node(
        package='ap_ngps_ros2',
        executable='ngps_localization_node.py',
        name='ngps_localization_node',
        output='screen',
        parameters=[
            LaunchConfiguration('ngps_config_file'),
            {
                'reference_image_path': LaunchConfiguration('reference_image_path'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('camera_topic')),
        ]
    )
    
    vips_node = Node(
        package='vins',
        executable='vins_node',
        name='vips_node',
        output='screen',
        parameters=[
            LaunchConfiguration('vips_config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        arguments=[LaunchConfiguration('vips_config_file')],
        remappings=[
            ('/vins_estimator/odometry', '/odometry/vio_raw'),
            ('/vins_estimator/path', '/vips/path'),
            ('/vins_estimator/pose', '/vips/pose'),
        ]
    )

    vio_relay_node = Node(
        package='ap_ngps_ros2',
        executable='vio_origin_relay.py',
        name='vio_origin_relay',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    ukf_node = Node(
        package='ap_ukf',
        executable='fusion_ros',
        name='fusion_ros',
        output='screen',
        parameters=[
            LaunchConfiguration('ukf_config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('/odometry/vio', '/odometry/vio'),
            ('/odometry/vps', '/odometry/vps'),
            ('/topic/sensor/odom_state', '/fused/odometry'),
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('fusion_backend'), "' == 'ap_ukf'"])
        ),
    )

    rl_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('robot_localization_config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings=[
            ('odometry/filtered', '/fused/odometry'),
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('fusion_backend'), "' == 'robot_localization'"])
        ),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('vins'),
            'config',
            'vins_rviz_config.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    return LaunchDescription([
        reference_image_arg,
        camera_topic_arg,
        ngps_config_arg,
        vips_config_arg,
        ukf_config_arg,
        fusion_backend_arg,
        rl_config_arg,
        use_sim_time_arg,
        enable_rviz_arg,
        
        ngps_node,
        vips_node,
        vio_relay_node,
        ukf_node,
        rl_ekf_node,
        rviz_node,
    ])
