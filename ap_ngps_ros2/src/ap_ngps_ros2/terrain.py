"""Height above the ground under the aircraft, from a stored terrain model.

VO scale and the NGPS map-crop size both need the camera's height above what it sees. The EKF only gives
height above the takeoff point, which is that only over flat ground: at the nagano1 site the ground is
12-17 m higher 150 m north and south of takeoff, so at 50 m above takeoff over it VO would read distances
~30 % long and every map crop would be oversized past the matcher's 0.8-1.25 scale gate.

    height = height above takeoff - (ground here - ground at takeoff)

Ground heights come from the same model at both points, so its datum and any GPS altitude error cancel.
The model is a north-up grid in an .npz (ngps_config/tools/make_terrain.py writes one from GSI dem5a):
    height_m  float32 [rows, cols], metres, NaN = unknown; row 0 = north edge
    bounds    [min_lon, min_lat, max_lon, max_lat] of the grid's outer edges
Unknown or out-of-model ground: no correction (plain height above takeoff), never a guess.
"""
from __future__ import annotations

import math
from typing import Optional, Sequence

import numpy as np

from ap_ngps_ros2 import ap_frame

MAX_RISE_M = 150.0   # a ground difference beyond this is a broken model, not terrain


class TerrainModel:
    def __init__(self, height_m: np.ndarray, bounds: Sequence[float], source: str = ''):
        self.h = np.asarray(height_m, np.float64)
        self.min_lon, self.min_lat, self.max_lon, self.max_lat = map(float, bounds)
        self.source = str(source)
        rows, cols = self.h.shape
        if rows < 2 or cols < 2 or not (self.max_lon > self.min_lon and self.max_lat > self.min_lat):
            raise ValueError('terrain model needs a grid of at least 2x2 and valid bounds')

    @classmethod
    def load(cls, path: str) -> 'TerrainModel':
        with np.load(path, allow_pickle=False) as z:
            return cls(z['height_m'], z['bounds'], str(z['source']) if 'source' in z else '')

    def height(self, lat: float, lon: float) -> float:
        """Bilinear ground height at lat/lon (cell centres), NaN outside the grid or next to unknown cells."""
        rows, cols = self.h.shape
        r = (self.max_lat - lat) / (self.max_lat - self.min_lat) * rows - 0.5
        c = (lon - self.min_lon) / (self.max_lon - self.min_lon) * cols - 0.5
        if not (0.0 <= r <= rows - 1 and 0.0 <= c <= cols - 1):
            return math.nan
        r0, c0 = min(int(r), rows - 2), min(int(c), cols - 2)
        fr, fc = r - r0, c - c0
        q = self.h[r0:r0 + 2, c0:c0 + 2]
        return float((q[0, 0] * (1 - fc) + q[0, 1] * fc) * (1 - fr) + (q[1, 0] * (1 - fc) + q[1, 1] * fc) * fr)

    def rise(self, origin, from_en, to_en) -> Optional[float]:
        """Ground at to_en minus ground at from_en (ENU metres from origin); None if either is unknown."""
        a = self.height(*ap_frame.en_to_ll(origin, float(from_en[0]), float(from_en[1])))
        b = self.height(*ap_frame.en_to_ll(origin, float(to_en[0]), float(to_en[1])))
        d = b - a
        if not math.isfinite(d) or abs(d) > MAX_RISE_M:
            return None
        return d


def height_above_ground(height_above_takeoff: float, model: Optional[TerrainModel], origin, takeoff_en, here_en):
    """(height, rise): the terrain-corrected height and the ground rise used (None = no correction)."""
    if model is None or origin is None or takeoff_en is None or here_en is None or height_above_takeoff <= 0.0:
        return height_above_takeoff, None
    rise = model.rise(origin, takeoff_en, here_en)
    if rise is None:
        return height_above_takeoff, None
    return max(height_above_takeoff - rise, 0.0), rise
