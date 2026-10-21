"""Terrain correction: pinned to the uncorrected height on flat ground, exact on a known slope, off when unknown."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from ap_ngps_ros2 import ap_frame  # noqa: E402
from ap_ngps_ros2.terrain import TerrainModel, height_above_ground  # noqa: E402

ORIGIN = (36.323136, 138.618168, 921.0)
B = [138.61, 36.316, 138.626, 36.330]     # ~1.4 x 1.6 km around the origin


def plane(north_slope=0.0, east_slope=0.0, base=921.0, n=200):
    """Ground rising north_slope m per m north and east_slope m per m east of ORIGIN, sampled at cell centres."""
    rows, cols = n, n
    lat = B[3] - (np.arange(rows) + 0.5) / rows * (B[3] - B[1])
    lon = B[0] + (np.arange(cols) + 0.5) / cols * (B[2] - B[0])
    north = (lat - ORIGIN[0]) * ap_frame.LATLON_TO_M
    east = (lon - ORIGIN[1]) * ap_frame.LATLON_TO_M * math.cos(math.radians(ORIGIN[0]))
    return TerrainModel(base + north_slope * north[:, None] + east_slope * east[None, :], B, 'test')


def test_flat_ground_changes_nothing():
    m = plane()
    for here in ((0, 0), (120, -80), (-300, 250)):
        h, rise = height_above_ground(50.0, m, ORIGIN, (0.0, 0.0), here)
        assert abs(h - 50.0) < 1e-6 and abs(rise) < 1e-6


def test_slope_is_corrected_exactly():
    m = plane(north_slope=0.1, east_slope=-0.05)      # 150 m north: +15 m; 100 m east: -5 m
    h, rise = height_above_ground(50.0, m, ORIGIN, (0.0, 0.0), (100.0, 150.0))
    assert abs(rise - 10.0) < 0.05 and abs(h - 40.0) < 0.05


def test_takeoff_point_is_the_reference_not_the_origin():
    m = plane(north_slope=0.1)
    h, _ = height_above_ground(50.0, m, ORIGIN, (0.0, 100.0), (0.0, 100.0))   # took off 100 m north, still there
    assert abs(h - 50.0) < 1e-6


def test_unknown_ground_falls_back_to_height_above_takeoff():
    m = plane(north_slope=0.1)
    m.h[:100, :] = np.nan                                     # no data in the northern half
    assert height_above_ground(50.0, m, ORIGIN, (0.0, 0.0), (0.0, 300.0)) == (50.0, None)
    far = height_above_ground(50.0, m, ORIGIN, (0.0, 0.0), (5000.0, 0.0))  # outside the grid
    assert far == (50.0, None)


def test_off_and_on_the_ground():
    assert height_above_ground(50.0, None, ORIGIN, (0.0, 0.0), (10.0, 10.0)) == (50.0, None)
    m = plane(north_slope=0.1)
    assert height_above_ground(0.0, m, ORIGIN, (0.0, 0.0), (0.0, 100.0)) == (0.0, None)   # before seeding
    h, _ = height_above_ground(5.0, m, ORIGIN, (0.0, 0.0), (0.0, 100.0))   # ground 10 m up: never negative
    assert h == 0.0


def test_npz_round_trip(tmp_path):
    m = plane(north_slope=0.1)
    p = tmp_path / 'dem.npz'
    np.savez_compressed(p, height_m=m.h.astype(np.float32), bounds=np.array(B), source='test')
    m2 = TerrainModel.load(str(p))
    assert m2.source == 'test'
    assert abs(m2.height(ORIGIN[0] + 0.001, ORIGIN[1]) - m.height(ORIGIN[0] + 0.001, ORIGIN[1])) < 0.01
