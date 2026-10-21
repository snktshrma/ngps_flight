#!/usr/bin/env python3
"""Build an NGPS reference map (mosaic.tif + metadata.json + preview.jpg) around a point.

    python3 ngps_config/tools/make_map.py --name nagano1 --lat 36.323136 --lon 138.618168 --source mapbox
    python3 ngps_config/tools/make_map.py --name nagano1_gsi_ort --lat ... --lon ... --source gsi_ort

Same layout as ngps_config/tiff/nagano0: an untagged RGB TIFF on the web-mercator tile grid, bounds in
metadata.json (min_lon,min_lat,max_lon,max_lat of the tile edges), zoom_level. The NGPS configs take
reference_min/max_lat/lon from those bounds and mosaic_meters_per_pixel from the zoom.

Mapbox needs MAPBOX_API_KEY in the environment. GSI tiles: attribution "出典：地理院タイル".
Imagery is for this aircraft's own localisation only; do not redistribute it.
Also writes the GSI 5 m DEM relief around the point into metadata (terrain_m), because NGPS sizes its
map crop from height above home: ground 20 m higher than home at 50 m AGL is a 40 % scale error.
"""
from __future__ import annotations

import argparse
import io
import json
import math
import os
import sys
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from PIL import Image

SOURCES = {
    # name: (url template, max zoom, description)
    'mapbox': ('https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}.jpg90?access_token={token}', 20,
               'Mapbox satellite (Maxar-based)'),
    'gsi_ort': ('https://cyberjapandata.gsi.go.jp/xyz/ort/{z}/{x}/{y}.jpg', 18,
                'GSI latest aerial ortho "ort" (Source: GSI tiles / 出典：地理院タイル)'),
    'gsi_seamless': ('https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{z}/{x}/{y}.jpg', 18,
                     'GSI seamlessphoto (Source: GSI tiles / 出典：地理院タイル)'),
    'gsi_2019': ('https://cyberjapandata.gsi.go.jp/xyz/nendophoto2019/{z}/{x}/{y}.png', 18,
                 'GSI nendophoto2019 aerial ortho (Source: GSI tiles / 出典：地理院タイル)'),
}
UA = {'User-Agent': 'ngps-flight-map/1.0'}


def tile_xy(lat, lon, z):
    n = 2 ** z
    return (lon + 180.0) / 360.0 * n, (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n


def tile_lonlat(x, y, z):
    n = 2 ** z
    return x / n * 360.0 - 180.0, math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))


def fetch(url, tries=4):
    for i in range(tries):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=20) as r:
                return r.read()
        except Exception as exc:  # noqa: BLE001
            if getattr(exc, 'code', None) == 404 or i == tries - 1:
                raise
            time.sleep(1.5 * (i + 1))


def terrain(lat, lon, radius_m):
    """GSI dem5a heights around the point: centre, min, max, and the 4 compass edges (make_terrain.py fills gaps)."""
    z = 15
    x, y = tile_xy(lat, lon, z)
    m_per_px = 156543.03392 * math.cos(math.radians(lat)) / 2 ** z
    r = int(math.ceil(radius_m / m_per_px))
    px, py = int(x * 256), int(y * 256)
    for layer in ('dem5a',):
        grid = {}
        try:
            for ty in range((py - r) // 256, (py + r) // 256 + 1):
                for tx in range((px - r) // 256, (px + r) // 256 + 1):
                    txt = fetch(f'https://cyberjapandata.gsi.go.jp/xyz/{layer}/{z}/{tx}/{ty}.txt').decode()
                    grid[tx, ty] = np.array([[float(v) if v != 'e' else np.nan for v in row.split(',')]
                                             for row in txt.strip().split('\n')])
        except Exception:  # noqa: BLE001
            continue

        def h(gx, gy):
            return float(grid[gx // 256, gy // 256][gy % 256, gx % 256])
        win = np.array([[h(gx, gy) for gx in range(px - r, px + r + 1)] for gy in range(py - r, py + r + 1)])
        c = h(px, py)
        return {'dem': layer, 'radius_m': radius_m, 'centre': round(c, 1),
                'min': round(float(np.nanmin(win)), 1), 'max': round(float(np.nanmax(win)), 1),
                'north': round(h(px, py - r), 1), 'south': round(h(px, py + r), 1),
                'east': round(h(px + r, py), 1), 'west': round(h(px - r, py), 1)}
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--name', required=True, help='output folder under ngps_config/tiff/')
    ap.add_argument('--lat', type=float, required=True)
    ap.add_argument('--lon', type=float, required=True)
    ap.add_argument('--source', choices=sorted(SOURCES), default='mapbox')
    ap.add_argument('--zoom', type=int, help='default: the source maximum')
    ap.add_argument('--size-m', type=float, default=600.0, help='square side, metres (rounded up to whole tiles)')
    ap.add_argument('--note', default='')
    a = ap.parse_args(argv)

    tmpl, zmax, desc = SOURCES[a.source]
    z = min(a.zoom or zmax, zmax)
    token = os.environ.get('MAPBOX_API_KEY', '')
    if a.source == 'mapbox' and not token:
        sys.exit('MAPBOX_API_KEY is not set')
    m_per_px = 156543.03392 * math.cos(math.radians(a.lat)) / 2 ** z
    half_tiles = a.size_m / 2 / (256 * m_per_px)
    cx, cy = tile_xy(a.lat, a.lon, z)
    x0, x1 = int(math.floor(cx - half_tiles)), int(math.ceil(cx + half_tiles))
    y0, y1 = int(math.floor(cy - half_tiles)), int(math.ceil(cy + half_tiles))
    nx, ny = x1 - x0, y1 - y0
    print(f'{a.source} z{z}: {nx}x{ny} tiles, {m_per_px:.4f} m/px, {nx * 256 * m_per_px:.0f} m wide')

    def get(xy):
        x, y = xy
        try:
            return xy, Image.open(io.BytesIO(fetch(tmpl.format(z=z, x=x, y=y, token=token)))).convert('RGB')
        except Exception as exc:  # noqa: BLE001
            return xy, exc
    mosaic = Image.new('RGB', (nx * 256, ny * 256))
    missing = []
    with ThreadPoolExecutor(8) as ex:
        for (x, y), im in ex.map(get, [(x, y) for y in range(y0, y1) for x in range(x0, x1)]):
            if isinstance(im, Exception):
                missing.append((x, y))
            else:
                mosaic.paste(im.resize((256, 256)) if im.size != (256, 256) else im, ((x - x0) * 256, (y - y0) * 256))
    if missing:
        print(f'WARNING: {len(missing)}/{nx * ny} tiles missing (black)')
    if len(missing) > 0.2 * nx * ny:
        sys.exit('too many tiles missing; this source does not cover the area')

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tiff', a.name)
    out = os.path.normpath(out)
    os.makedirs(out, exist_ok=True)
    mosaic.save(os.path.join(out, 'mosaic.tif'))
    pv = mosaic.copy()
    pv.thumbnail((1200, 1200))
    pv.save(os.path.join(out, 'preview.jpg'), quality=85)

    min_lon, max_lat = tile_lonlat(x0, y0, z)
    max_lon, min_lat = tile_lonlat(x1, y1, z)
    meta = {
        'name': '{z}/{x}/{y}',
        'description': f'{desc} z{z}' + (f', {a.note}' if a.note else ''),
        'source': a.source,
        'bounds': f'{min_lon},{min_lat},{max_lon},{max_lat}',
        'center': f'{a.lon},{a.lat}',
        'zoom_level': z,
        'meters_per_pixel': round(156543.03392 * math.cos(math.radians(0.5 * (min_lat + max_lat))) / 2 ** z, 5),
        'size_px': [nx * 256, ny * 256],
        'site_px': [round((cx - x0) * 256), round((cy - y0) * 256)],
        'missing_tiles': len(missing),
        'downloaded': time.strftime('%Y-%m-%d'),
    }
    t = terrain(a.lat, a.lon, 150.0)
    if t:
        meta['terrain_m'] = t
    with open(os.path.join(out, 'metadata.json'), 'w') as f:
        json.dump(meta, f, indent=1, ensure_ascii=False)
    print(json.dumps(meta, indent=1, ensure_ascii=False))
    print('wrote', out)


if __name__ == '__main__':
    main()
