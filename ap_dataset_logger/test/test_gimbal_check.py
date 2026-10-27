"""The gimbal nadir check reads pitch from GIMBAL_DEVICE_ATTITUDE_STATUS quaternions; pin the maths."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
pc = pytest.importorskip('preflight_check')


def quat(roll, pitch, yaw):
    r, p, y = (math.radians(a) / 2 for a in (roll, pitch, yaw))
    return [math.cos(r) * math.cos(p) * math.cos(y) + math.sin(r) * math.sin(p) * math.sin(y),
            math.sin(r) * math.cos(p) * math.cos(y) - math.cos(r) * math.sin(p) * math.sin(y),
            math.cos(r) * math.sin(p) * math.cos(y) + math.sin(r) * math.cos(p) * math.sin(y),
            math.cos(r) * math.cos(p) * math.sin(y) - math.sin(r) * math.sin(p) * math.cos(y)]


@pytest.mark.parametrize('pitch', [-90.0, -89.9, -85.0, -45.0, 0.0, 20.0])
def test_pitch_from_quaternion(pitch):
    assert abs(pc.pitch_from_quaternion(quat(0.0, pitch, 165.0)) - pitch) < 0.05


def test_nadir_tolerance_matches_flight8_case():
    # flight 8 flew at 0 deg (horizon) -> must be far outside the tolerance; flight 7 at -89.9 inside
    assert abs(pc.pitch_from_quaternion(quat(0, 0.0, 0)) + 90.0) > pc.NADIR_TOL_DEG
    assert abs(pc.pitch_from_quaternion(quat(0, -89.9, 0)) + 90.0) <= pc.NADIR_TOL_DEG
