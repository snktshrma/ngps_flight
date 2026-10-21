#!/usr/bin/env python3
"""Capture a flight dataset: RTSP camera + ArduPilot topics into one mcap.

    ros2 launch ap_dataset_logger dataset.launch.py
    ros2 launch ap_dataset_logger dataset.launch.py bag_dir:=/media/ssd/bags
    ros2 launch ap_dataset_logger dataset.launch.py archive_h265:=true
    ros2 launch ap_dataset_logger dataset.launch.py record:=false   # camera only

    # also connect to the flight controller, over DDS (default) or MAVLink:
    ros2 launch ap_dataset_logger dataset.launch.py fc:=serial:/dev/ttyTHS1:921600
    ros2 launch ap_dataset_logger dataset.launch.py link:=mavlink fc:=serial:/dev/ttyTHS1:921600
    ros2 launch ap_dataset_logger dataset.launch.py link:=mavlink fc:=udp:14550
    ros2 launch ap_dataset_logger dataset.launch.py link:=mavlink fc:=tcp:127.0.0.1:5760  # SITL

The flight controller's serial port must speak the matching protocol: SERIALx_PROTOCOL
45 for DDS, 2 for MAVLink. Switching means changing that parameter and rebooting it.

Recording is delegated to record.launch.py in this package, which owns the topic
allowlist and the RAM-tuned rosbag2 flags.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource, PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ap_dataset_logger'), 'config', 'dataset_logger.yaml'
        ]),
        description='Camera/sync parameters',
    )
    record_arg = DeclareLaunchArgument(
        'record', default_value='true',
        description='Record an mcap alongside the camera (separate process)')
    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir', default_value='',
        description='Bag output directory; empty uses the repo bags/ directory')
    bag_name_arg = DeclareLaunchArgument(
        'bag_name', default_value='flight',
        description='Bag name prefix; a timestamp is appended')
    archive_arg = DeclareLaunchArgument(
        'archive_h265', default_value='false',
        description='Also write the untouched H265 elementary stream to .mkv')
    url_arg = DeclareLaunchArgument(
        'rtsp_url', default_value='',
        description='Override the RTSP URL from the config file')

    def _camera(context, *_a, **_k):
        # rtsp_url is applied only when non-empty; passing it through unconditionally
        # would overwrite the config file's value with an empty string.
        overrides = {
            'archive_h265': LaunchConfiguration('archive_h265'),
            'archive_dir': LaunchConfiguration('bag_dir'),
        }
        url = LaunchConfiguration('rtsp_url').perform(context).strip()
        if url:
            overrides['rtsp_url'] = url
        return [Node(
            package='ap_dataset_logger',
            executable='rtsp_camera_node.py',
            name='rtsp_camera_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file'), overrides],
        )]

    camera = OpaqueFunction(function=_camera)

    def _fc_link(context, *_a, **_k):
        """Start whatever carries the flight controller's data: DDS or MAVLink."""
        link = LaunchConfiguration('link').perform(context).strip().lower()
        spec = LaunchConfiguration('fc').perform(context).strip()
        if not spec:
            spec = LaunchConfiguration('agent').perform(context).strip()   # older name
        if spec in ('', 'none'):
            return []

        kind, _, rest = spec.partition(':')
        if kind == 'serial':
            dev, _, baud = rest.rpartition(':')
            if not dev or not baud.isdigit():
                raise ValueError(f'fc:=serial:<device>:<baud>, got {spec!r}')
        elif kind == 'udp':
            if not rest.isdigit():
                raise ValueError(f'fc:=udp:<port>, got {spec!r}')
        elif kind == 'tcp':
            host, _, port = rest.rpartition(':')
            if not host or not port.isdigit():
                raise ValueError(f'fc:=tcp:<host>:<port>, got {spec!r}')
        else:
            raise ValueError(
                f'fc must be serial:<dev>:<baud>, udp:<port> or tcp:<host>:<port>; got {spec!r}')

        if link == 'dds':
            # AP_DDS reaches this machine through a micro-ROS agent. Nothing in /ap/*
            # exists until it runs.
            if kind == 'serial':
                args = ['serial', '--dev', dev, '-b', baud]
            elif kind == 'udp':
                args = ['udp4', '-p', rest]
            else:
                args = ['tcp4', '-p', port]
            return [
                LogInfo(msg=f'flight controller over DDS: micro_ros_agent {" ".join(args)}'),
                ExecuteProcess(
                    cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', *args],
                    output='screen',
                    # The agent's own restart is what recovers a dropped serial link, so
                    # keep it alive rather than letting one glitch end the flight's data.
                    respawn=True, respawn_delay=2.0,
                ),
            ]

        if link == 'mavlink':
            # MAVROS publishes /mavros/*, and keeps its timestamps on this machine's
            # clock via MAVLink TIMESYNC -- which is what lines them up with the camera.
            if kind == 'serial':
                fcu_url = f'{dev}:{baud}'
            elif kind == 'udp':
                fcu_url = f'udp://:{rest}@'
            else:
                fcu_url = f'tcp://{host}:{port}'
            from ament_index_python.packages import get_package_share_directory
            apm = os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch')
            return [
                LogInfo(msg=f'flight controller over MAVLink: fcu_url={fcu_url}'),
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(apm),
                    launch_arguments={
                        'fcu_url': fcu_url,
                        'gcs_url': '',
                        'tgt_system': '1',
                        'tgt_component': '1',
                        'respawn_mavros': 'true',
                    }.items(),
                ),
                # ArduPilot's default stream rates on a telemetry port are a few Hz; this
                # raises them for this link only, without touching saved parameters.
                Node(package='ap_dataset_logger', executable='mavlink_rates.py',
                     name='mavlink_rates', output='screen'),
            ]

        raise ValueError(f'link must be dds or mavlink, got {link!r}')

    fc_link = OpaqueFunction(function=_fc_link)
    link_arg = DeclareLaunchArgument(
        'link', default_value='dds',
        description='dds | mavlink -- how the flight controller\'s data reaches this machine')
    fc_arg = DeclareLaunchArgument(
        'fc', default_value='',
        description='none | serial:<device>:<baud> | udp:<port> | tcp:<host>:<port>')
    agent_arg = DeclareLaunchArgument(
        'agent', default_value='none',
        description='older name for fc:= with link:=dds; kept so existing commands work')
    ngps_arg = DeclareLaunchArgument(
        'ngps', default_value='',
        description='Run NGPS localization live with this parameter file (empty: do not run it)')

    def _ngps(context, *_a, **_k):
        # Live NGPS: its outputs land in the same bag, and with VISO_TYPE set the flight
        # controller logs what it receives on /ap/tf as VISP.
        params = LaunchConfiguration('ngps').perform(context).strip()
        if not params:
            return []
        return [
            LogInfo(msg=f'NGPS localization running live with {params}'),
            Node(
                package='ap_ngps_ros2',
                executable='ngps_localization_node.py',
                name='ngps_localization_node',
                output='screen',
                parameters=[params],
                respawn=True,
                respawn_delay=5.0,
            ),
        ]

    ngps_node = OpaqueFunction(function=_ngps)

    recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ap_dataset_logger'), 'launch', 'record.launch.py'
            ])
        ]),
        launch_arguments={
            'bag_dir': LaunchConfiguration('bag_dir'),
            'bag_name': LaunchConfiguration('bag_name'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('record')),
    )

    return LaunchDescription([
        config_arg, record_arg, bag_dir_arg, bag_name_arg, archive_arg, url_arg,
        link_arg, fc_arg, agent_arg, ngps_arg,
        fc_link, camera, recorder, ngps_node,
    ])
