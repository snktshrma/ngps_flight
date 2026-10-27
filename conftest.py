"""Repo-wide pytest setup for the autotest CI (see .github/workflows/autotest.yml)."""

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent

# Import the packages from source so the tests also run without a colcon install
for src in ('ap_ngps_ros2/src', 'ap_dataset_logger/src'):
    sys.path.insert(0, str(ROOT / src))

# Manual scripts named test_*: a demo that only prints, and a node that needs a live graph
SKIP = {
    'ap_ngps_ros2/test/test_rotation_detection.py',
    'ap_ngps_ros2/test/test_ngps_node.py',
}

# TensorRT and pycuda only exist on the Jetson
if not all(importlib.util.find_spec(m) for m in ('tensorrt', 'pycuda')):
    SKIP.add('ap_ngps_ros2/test/test_trt_unit.py')


# A hook rather than collect_ignore: ROS's pytest plugins (launch_testing, ament_*) answer
# pytest_ignore_collect first, so pytest's own collect_ignore check never runs under ROS.
def pytest_ignore_collect(collection_path, config):
    try:
        rel = Path(collection_path).resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return None
    return True if rel in SKIP else None
