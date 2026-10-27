"""make_mosaic.frame_to_map_affine: pinned to the fusion node's camera conventions (nadir pixel, scale, heading).

The end-to-end check (frames rendered with this affine, matched by VoNgpsFusionNode._ngps_candidate,
fixes within 0.26 m of truth at 80-110 m) needs TensorRT; it was run on 2026-10-24 for the ottawa1 map.
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ngps_config' / 'tools'))
sys.path.insert(0, str(ROOT / 'ap_ngps_ros2' / 'src'))
import make_mosaic as mm  # noqa: E402
from ap_ngps_ros2 import camera  # noqa: E402

W, H = 1280, 720


def apply(A, u, v):
    return A @ np.array([u, v, 1.0])


def test_nadir_pixel_lands_on_the_aircraft():
    A = mm.frame_to_map_affine(W, H, 110.0, 37.0, (5000.0, 4000.0), 0.105)
    P = (W * (0.5 + camera.NADIR_OFFSET_X_FRAC), H * (0.5 + camera.NADIR_OFFSET_Y_FRAC))
    assert np.allclose(apply(A, *P), (5000.0, 4000.0))


def test_ground_width_matches_the_hfov():
    A = mm.frame_to_map_affine(W, H, 100.0, 0.0, (0.0, 0.0), 0.1)
    left, right = apply(A, 0, H / 2), apply(A, W, H / 2)
    want_m = 2 * 100.0 * math.tan(math.radians(camera.HFOV_DEG / 2))
    assert abs(np.linalg.norm(right - left) * 0.1 - want_m) < 1e-6


def test_image_up_points_along_the_heading():
    for heading, (dc, dr) in ((0.0, (0, -1)), (90.0, (1, 0)), (180.0, (0, 1)), (270.0, (-1, 0))):
        A = mm.frame_to_map_affine(W, H, 100.0, heading, (0.0, 0.0), 0.1)
        up = apply(A, W / 2, 0) - apply(A, W / 2, H / 2)          # towards the top of the image
        assert np.allclose(up / np.linalg.norm(up), (dc, dr), atol=1e-9), heading   # map rows grow south


def test_frame_size_does_not_change_the_ground_footprint():
    a = mm.frame_to_map_affine(1280, 720, 90.0, 20.0, (100.0, 200.0), 0.105)
    b = mm.frame_to_map_affine(640, 360, 90.0, 20.0, (100.0, 200.0), 0.105)
    assert np.allclose(apply(a, 1280, 720), apply(b, 640, 360))
