#!/usr/bin/env python3
"""Ground-height model for the fusion node's terrain correction (ap_ngps_ros2/terrain.py).

    python3 ngps_config/tools/make_terrain.py --name nagano1 --lat 36.323136 --lon 138.618168 [--size-m 1200]

Writes ngps_config/terrain/<name>/dem.npz (height_m, bounds, source) from GSI dem5a (5 m laser-survey
bare-ground model, z15 tiles at ~3.9 m/px), filling its gaps (water, survey holes) from DEM10B (layer id 'dem').
Bare ground: over forest the camera sees the treetops, which this model does not include.
Source: GSI tiles (出典：地理院タイル).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import urllib.request

import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
URL = 'https://cyberjapandata.gsi.go.jp/xyz/{layer}/{z}/{x}/{y}.txt'
UA = {'User-Agent': 'ngps-flight-terrain/1.0'}


def tile_xy(lat, lon, z):
    n = 2 ** z
    return (lon + 180.0) / 360.0 * n, (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n


def tile_lonlat(x, y, z):
    n = 2 ** z
    return x / n * 360.0 - 180.0, math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))


def tile(layer, z, x, y):
    try:
        with urllib.request.urlopen(urllib.request.Request(URL.format(layer=layer, z=z, x=x, y=y), headers=UA),
                                    timeout=20) as r:
            txt = r.read().decode()
    except Exception as exc:  # noqa: BLE001
        if getattr(exc, 'code', None) == 404:
            return np.full((256, 256), np.nan)
        raise
    return np.array([[float(v) if v != 'e' else np.nan for v in row.split(',')] for row in txt.strip().split('\n')])


def grid(layer, z, x0, x1, y0, y1):
    return np.block([[tile(layer, z, x, y) for x in range(x0, x1)] for y in range(y0, y1)])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--name', required=True)
    ap.add_argument('--lat', type=float, required=True)
    ap.add_argument('--lon', type=float, required=True)
    ap.add_argument('--size-m', type=float, default=1200.0)
    a = ap.parse_args(argv)

    z = 15
    mpp = 156543.03392 * math.cos(math.radians(a.lat)) / 2 ** z
    cx, cy = tile_xy(a.lat, a.lon, z)
    half = a.size_m / 2 / (256 * mpp)
    x0, x1 = int(math.floor(cx - half)), int(math.ceil(cx + half))
    y0, y1 = int(math.floor(cy - half)), int(math.ceil(cy + half))
    h = grid('dem5a', z, x0, x1, y0, y1)
    holes = int(np.isnan(h).sum())
    if holes:
        # dem10b is z14 max: each of its cells covers 2x2 of ours
        h10 = grid('dem', 14, x0 // 2, (x1 + 1) // 2, y0 // 2, (y1 + 1) // 2)
        h10 = np.repeat(np.repeat(h10, 2, 0), 2, 1)
        oy, ox = (y0 - 2 * (y0 // 2)) * 256, (x0 - 2 * (x0 // 2)) * 256
        h10 = h10[oy:oy + h.shape[0], ox:ox + h.shape[1]]
        h = np.where(np.isnan(h), h10, h)
    left = int(np.isnan(h).sum())
    min_lon, max_lat = tile_lonlat(x0, y0, z)
    max_lon, min_lat = tile_lonlat(x1, y1, z)
    bounds = np.array([min_lon, min_lat, max_lon, max_lat])
    # The tile grid is web-mercator; over ~1 km the rows are within 0.01 % of evenly spaced in latitude,
    # which is what terrain.py assumes.
    out = os.path.join(ROOT, 'terrain', a.name)
    os.makedirs(out, exist_ok=True)
    src = f'GSI dem5a z15 (~{mpp:.1f} m/px), gaps from DEM10B; 出典：地理院タイル'
    np.savez_compressed(os.path.join(out, 'dem.npz'), height_m=h.astype(np.float32), bounds=bounds, source=src)

    sys.path.insert(0, os.path.join(ROOT, '..', 'ap_ngps_ros2', 'src'))
    from ap_ngps_ros2.terrain import TerrainModel
    m = TerrainModel.load(os.path.join(out, 'dem.npz'))
    centre = m.height(a.lat, a.lon)
    info = {'source': src, 'bounds': bounds.tolist(), 'shape': list(h.shape), 'centre_lat_lon': [a.lat, a.lon],
            'ground_at_centre_m': round(centre, 1), 'dem5a_holes_filled': holes - left, 'unknown_cells': left}
    for r in (50, 150, 300):
        k = int(r / mpp)
        rr, cc = int((cy - y0) * 256), int((cx - x0) * 256)
        w = h[max(rr - k, 0):rr + k + 1, max(cc - k, 0):cc + k + 1]
        info[f'within_{r}m'] = {'min': round(float(np.nanmin(w)) - centre, 1), 'max': round(float(np.nanmax(w)) - centre, 1)}
    with open(os.path.join(out, 'dem.json'), 'w') as f:
        json.dump(info, f, indent=1, ensure_ascii=False)
    print(json.dumps(info, indent=1, ensure_ascii=False))
    print('wrote', out)


if __name__ == '__main__':
    main()
