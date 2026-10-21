#!/usr/bin/env python3
"""Record the flight topics to an mcap bag.

Runs `ros2 bag record` as a *separate process* on purpose. The localization node holds
the reference mosaic in RAM on an 8 GB Orin Nano with no swap, so the recorder's write
buffers must never share that heap.

Included by dataset.launch.py, or run standalone against an already-running stack:

    ros2 launch ap_dataset_logger record.launch.py
    ros2 launch ap_dataset_logger record.launch.py bag_dir:=/media/ssd/bags
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Recorded by pattern, not by list. A list silently misses whatever it forgot -- the
# first version here left out /ap/twist/filtered, the EKF velocity, which is exactly the
# ground truth a VIO/VO comparison needs. A pattern also picks up anything ArduPilot
# adds in a later firmware.
#
# What AP_DDS publishes, and how often (from AP_DDS_Client.cpp):
#   /ap/imu/experimental/data  200 Hz   /ap/pose/filtered           30 Hz
#   /ap/time, /ap/clock        100 Hz   /ap/geopose/filtered        30 Hz
#   /ap/navsat              GPS rate    /ap/twist/filtered          30 Hz
#   /ap/status, /ap/rc          10 Hz   /ap/gps_global_origin/...    1 Hz
#   /ap/battery                  1 Hz   /ap/tf_static        once, latched
# /ap/time is what lines this bag up with the ArduCopter dataflash log afterwards: it
# carries ArduPilot's GPS-derived UTC, stamped on arrival with this machine's clock.
#
# The ngps/odometry/fused entries only matter when those nodes run; harmless otherwise.
#
# Deliberately excluded by the anchors:
#   /camera/image_raw  -- uncompressed, roughly 30x the bytes of the compressed topic
#   ngps/match_pair    -- uncompressed debug panel images
# /mavros/* is the MAVLink fallback's equivalent of /ap/*; whichever link is in use, the
# other simply has no publishers and records nothing.
TOPIC_REGEX = (
    r'^/(ap/.*'
    r'|mavros/.*'
    r'|camera/image_raw/compressed'
    r'|diagnostics'
    r'|ngps/(pose|position|rotation|global_position|ecef_position)'
    r'|odometry/(vps|vio|vio_raw)'
    r'|fused/odometry'
    r')$'
)

# rosbag2 defaults to a 100 MB in-process message cache. That is the single largest RAM
# lever here, and 16 MB is ample at this stack's ~2.5 MB/s ingest.
MAX_CACHE_BYTES = '16777216'

# Split at 500 MB so a long flight stays deletable file by file.
MAX_BAG_BYTES = '500000000'

# No compression: the payload is already JPEG, so --compression-mode would buy almost
# nothing in size while spending real CPU and buffer memory on an Orin Nano.

#: Default output root. Matches the existing (gitignored) bags/ location.
DEFAULT_BAG_DIR = os.path.join(
    os.path.expanduser('~'), 'ngps_ws', 'src', 'ngps_flight', 'bags'
)


def _recorder(context, *args, **kwargs):
    # An OpaqueFunction so the timestamp and the empty-bag_dir fallback resolve with real
    # Python. `ros2 bag record -o` does not append a timestamp and refuses to overwrite
    # an existing directory, so a unique name has to be built here.
    bag_dir = LaunchConfiguration('bag_dir').perform(context).strip() or DEFAULT_BAG_DIR
    bag_name = LaunchConfiguration('bag_name').perform(context).strip() or 'flight'
    out = os.path.join(bag_dir, f'{bag_name}_{datetime.now():%Y%m%d_%H%M%S}')
    os.makedirs(bag_dir, exist_ok=True)

    return [
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-s', 'mcap',
                '-o', out,
                '--max-cache-size', MAX_CACHE_BYTES,
                '--max-bag-size', MAX_BAG_BYTES,
                '--regex', TOPIC_REGEX,
            ],
            output='screen',
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_dir', default_value='',
            description=f'Output directory for the bag. Empty uses {DEFAULT_BAG_DIR}.'),
        DeclareLaunchArgument(
            'bag_name', default_value='flight',
            description='Bag name prefix; a _YYYYmmdd_HHMMSS stamp is appended.'),
        OpaqueFunction(function=_recorder),
    ])
