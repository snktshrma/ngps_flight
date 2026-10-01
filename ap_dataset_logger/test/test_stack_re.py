#!/usr/bin/env python3
"""What flight_log.sh's orphan cleanup may kill.

The vps-jp7 container shares the Jetson's process table, so `pgrep -f` inside it sees
every process on the machine. An earlier loose pattern killed the caller's own shell.
STACK_RE must match the stack's own programs and nothing that merely mentions them.
"""

import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'flight_log.sh'
WS = '/home/rmackay9/ngps_ws'


def pattern(name: str = 'STACK_RE') -> str:
    line = next(ln for ln in SCRIPT.read_text().splitlines() if ln.startswith(f'{name}='))
    return subprocess.run(['bash', '-c', f'{line}; printf %s "${name}"'],
                          capture_output=True, text=True, check=True).stdout


def matches(cmdline: str, name: str = 'STACK_RE') -> bool:
    # pgrep -f uses POSIX extended regexes, as grep -E does
    return subprocess.run(['grep', '-qE', pattern(name)], input=cmdline + '\n', text=True).returncode == 0


STACK = [
    f'python3 {WS}/install-jazzy/ap_ngps_ros2/lib/ap_ngps_ros2/ngps_localization_node.py --ros-args -r __node:=ngps_localization_node',
    f'/usr/bin/python3 {WS}/install-jazzy/ap_dataset_logger/lib/ap_dataset_logger/rtsp_camera_node.py --ros-args',
    'python3 src/ap_ngps_ros2/vo_ngps_fusion_node.py --ros-args --params-file /x/fusion_nagano0.yaml',
    f'bash {WS}/src/ngps_flight/ap_ngps_ros2/scripts/run_fusion.sh /x/fusion_nagano0.yaml',
    f'{WS}/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent udp4 -p 2019',
    '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019',
    '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 bag record -s mcap -o /bags/flight_1 --regex /ap/.*',
    '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch ap_dataset_logger dataset.launch.py link:=dds fc:=none',
]

NOT_STACK = [
    'python3 /home/rmackay9/_getlog.py',                         # the dataflash download
    '/bin/bash -c source snapshot.sh && grep -n vo_ngps_fusion_node flight_log.sh',
    'bash /home/rmackay9/ngps_ws/src/ngps_flight/ap_dataset_logger/scripts/flight_log.sh stop',
    'SCREEN -dmS vo_ngps -L docker exec vps-jp7 bash /x/run_fusion.sh /x/f.yaml',
    'docker exec -u rmackay9 vps-jp7 bash /x/scripts/run_fusion.sh /x/f.yaml',   # host side
    'pgrep -f micro_ros_agent|ngps_localization_node',
    'vim /x/launch/dataset.launch.py',
    'less /x/ngps_localization_node.py',
    'tail -f /bags/vo_ngps_fusion_node.log',
    '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 bag play /bags/flight3 --topics /ap/pose/filtered',
    '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 topic echo /ap/tf',
    'python3 -m pytest test/test_ngps_node.py',
]


@pytest.mark.parametrize('cmd', STACK)
def test_matches_stack_processes(cmd):
    assert matches(cmd)


@pytest.mark.parametrize('cmd', NOT_STACK)
def test_leaves_everything_else_alone(cmd):
    assert not matches(cmd)


@pytest.mark.parametrize('cmd', STACK)
def test_fusion_subset_is_only_the_vo_node_and_its_loop(cmd):
    assert matches(cmd, 'FUSION_RE') == ('vo_ngps_fusion_node.py' in cmd or 'run_fusion.sh' in cmd)


@pytest.mark.parametrize('cmd', NOT_STACK)
def test_fusion_subset_leaves_everything_else_alone(cmd):
    assert not matches(cmd, 'FUSION_RE')
