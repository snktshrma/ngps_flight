"""system/ramguard.py: who gets stopped at which RAM level (the flight stack only as a last resort)."""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('ramguard', Path(__file__).resolve().parents[2] / 'system' / 'ramguard.py')
rg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rg)

FUSION = 'python3 src/ap_ngps_ros2/vo_ngps_fusion_node.py --ros-args --params-file fusion_ottawa1.yaml'
CAMERA = 'python3 /home/u/ngps_ws/install-jazzy/ap_dataset_logger/lib/ap_dataset_logger/rtsp_camera_node.py'
RECORD = '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 bag record -s mcap -o flight11 /camera/image_raw/compressed'
AGENT = 'micro_ros_agent udp4 -p 2019'
MAPCMP = 'python3 ngps_config/tools/map_compare.py --bag bags/flight11 --maps ottawa1 ottawa1_city'
CLAUDE = 'claude --resume'


def procs():
    rows = [(1500, 1, 'python3', FUSION), (300, 2, 'python3', CAMERA), (400, 3, 'python3', RECORD),
            (900, 4, 'micro_ros_agent', AGENT), (1300, 5, 'python3', MAPCMP), (2000, 6, 'claude', CLAUDE)]
    return [(r, p, c, cmd, rg.tier_of(c, cmd)) for r, p, c, cmd in rows]


def pick(used, ps=None):
    p = rg.choose(used, procs() if ps is None else ps, 90, 95, 97)
    return None if p is None else p[1]


def test_tiers():
    t = {p[1]: p[4] for p in procs()}
    assert t == {1: 'flight', 2: 'flight', 3: 'recorder', 4: 'protected', 5: 'other', 6: 'protected'}


def test_below_threshold_nothing():
    assert pick(89.9) is None


def test_analysis_goes_first_even_though_the_fusion_node_is_bigger():
    assert pick(91.0) == 5
    assert pick(99.0) == 5


def test_flight_stack_is_kept_until_95_then_recorder_then_97():
    ps = [p for p in procs() if p[1] != 5]          # no analysis left
    assert pick(94.0, ps) is None
    assert pick(95.5, ps) == 3                      # recorder: data lost, flight fine
    no_rec = [p for p in ps if p[1] != 3]
    assert pick(96.0, no_rec) is None
    assert pick(97.5, no_rec) == 1                  # last resort: largest flight process


def test_dds_agent_and_session_never():
    only = [p for p in procs() if p[1] in (4, 6)]
    assert pick(99.9, only) is None


def test_analysis_scripts_are_not_mistaken_for_flight():
    for cmd in ('python3 ap_dataset_logger/scripts/fc_log_check.py --download',
                'python3 -m pytest ap_ngps_ros2/test', 'python3 ngps_config/tools/make_mosaic.py --bag b'):
        assert rg.tier_of('python3', cmd) == 'other', cmd
