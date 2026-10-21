#!/usr/bin/env python3
"""Which map works best here, and what is its offset? Matches flight frames against each map with GPS as truth.

    python3 ngps_config/tools/map_compare.py --bag bags/flight9_... \
        --maps nagano1 nagano1_gsi_seamless nagano1_gsi_2019 nagano1_gsi_ort [--period 1.0] [--min-alt 20]

Runs inside vps-jp7 (TensorRT + ROS). For each sampled camera frame above --min-alt it builds the map
crop at the EKF's own position and heading (a GPS flight, so that is truth) and runs the fusion node's own
match (VoNgpsFusionNode._ngps_candidate: same crop size, matcher and acceptance checks). Per map:
  * match rate, overall and by height above home, and why the others failed;
  * map minus truth (east, north): the map's offset, i.e. the offset_en_m to put in its metadata.json
    (median of the slow samples: at speed the camera lag smears it), with its spread.
Any offset_en_m already in a map's metadata is ignored here; the numbers are the raw map error.
--dem ngps_config/terrain/<site>/dem.npz [surface.npz ...] also runs every map with the node's terrain
correction (terrain.py) from each model, as "<map>+<model>": compare match rate and fit scale (true/assumed
height; ~1.0 = height right) with and without. surface.npz = ground + canopy (treetops).
Writes <bag>_map_compare.json next to the bag.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import types
from collections import Counter

import cv2 as cv
import numpy as np
import rosbag2_py
import yaml
from geographic_msgs.msg import GeoPointStamped, GeoPoseStamped
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'ap_ngps_ros2', 'src'))
from ap_ngps_ros2 import ap_frame, camera  # noqa: E402
from ap_ngps_ros2.terrain import TerrainModel, height_above_ground  # noqa: E402
from ap_ngps_ros2.vo_ngps_fusion_node import VoNgpsFusionNode  # noqa: E402

TIFF = os.path.join(ROOT, 'ngps_config', 'tiff')
CFG = os.path.join(ROOT, 'ap_ngps_ros2', 'config', 'fusion_nagano0.yaml')
DEFAULTS = {'min_agree': 25, 'max_reproj_px': 2.0, 'max_rot_deg': 10.0, 'hfov_deg': camera.HFOV_DEG}


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def read_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/ap/geopose/filtered', '/ap/gps_global_origin/filtered',
                                                  '/camera/image_raw/compressed']))
    geo, frames, origin = [], [], None
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/ap/geopose/filtered':
            m = deserialize_message(data, GeoPoseStamped)
            p = m.pose.position
            if ap_frame.valid_ll(p.latitude, p.longitude):
                geo.append((stamp(m.header), p.latitude, p.longitude, p.altitude,
                            ap_frame.yaw_enu_deg(m.pose.orientation)))
        elif topic == '/ap/gps_global_origin/filtered':
            m = deserialize_message(data, GeoPointStamped).position
            if ap_frame.valid_ll(m.latitude, m.longitude):
                origin = (m.latitude, m.longitude, m.altitude)
        else:
            m = deserialize_message(data, CompressedImage)
            frames.append((stamp(m.header), bytes(m.data)))
    return np.array(geo), frames, origin


def matcher_ctx(map_name, params, trt, origin):
    """Just enough of a VoNgpsFusionNode for _ngps_candidate."""
    d = os.path.join(TIFF, map_name)
    meta = json.load(open(os.path.join(d, 'metadata.json')))
    b = meta['bounds']
    lat_mid = 0.5 * (float(b.split(',')[1]) + float(b.split(',')[3])) if isinstance(b, str) else 0.5 * (b[1] + b[3])
    ctx = types.SimpleNamespace(
        map=cv.imread(os.path.join(d, 'mosaic.tif')),
        bounds=list(map(float, b.split(','))) if isinstance(b, str) else list(map(float, b)),
        mpp=156543.03392 * math.cos(math.radians(lat_mid)) / 2 ** int(meta['zoom_level']),
        map_offset=np.zeros(2), vo=types.SimpleNamespace(scale=1.0), trt=trt,
        P=np.array([640 * (0.5 + camera.NADIR_OFFSET_X_FRAC), 360 * (0.5 + camera.NADIR_OFFSET_Y_FRAC)]),
        track=types.SimpleNamespace(origin=origin),
        get_parameter=lambda n: types.SimpleNamespace(value=params.get(n, DEFAULTS.get(n))),
        desc=meta.get('description', ''))
    for name in ('_enu_to_ll', '_ll_to_enu', '_ngps_candidate'):
        setattr(ctx, name, types.MethodType(getattr(VoNgpsFusionNode, name), ctx))
    return ctx


def summarise(rows):
    rows = [r for r in rows if r is not None]
    n = len(rows)
    ok = [r for r in rows if r['why'] == 'ok']
    out = {'attempts': n, 'ok': len(ok), 'rate': round(len(ok) / n, 3) if n else 0.0,
           'why': dict(Counter(r['why'] for r in rows).most_common())}
    bins = {}
    for lo in range(0, 200, 20):
        sel = [r for r in rows if lo <= r['alt'] < lo + 20]
        if sel:
            bins[f'{lo}-{lo + 20}m'] = f"{sum(r['why'] == 'ok' for r in sel)}/{len(sel)}"
    out['by_height'] = bins
    slow = [r for r in ok if r['speed'] < 1.0]
    use = slow if len(slow) >= 5 else ok
    if use:
        e = np.array([r['err'] for r in use])
        med = np.median(e, axis=0)
        out.update({
            'offset_en_m': [round(float(med[0]), 2), round(float(med[1]), 2)],
            'offset_from': f"{len(use)} {'slow (<1 m/s)' if use is slow else 'all (too few slow)'} fixes",
            'spread_m_p50': round(float(np.median(np.linalg.norm(e - med, axis=1))), 2),
            'spread_m_p90': round(float(np.percentile(np.linalg.norm(e - med, axis=1), 90)), 2),
            'fit_scale_median': round(float(np.median([r['scale'] for r in use])), 3),
            'fit_scale_p10_p90_all': [round(float(np.percentile([r['scale'] for r in ok], q)), 3) for q in (10, 90)],
        })
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--maps', nargs='+', required=True, help='folders under ngps_config/tiff/')
    ap.add_argument('--config', default=CFG, help='fusion config for the engine and acceptance checks')
    ap.add_argument('--period', type=float, default=1.0, help='seconds between sampled frames')
    ap.add_argument('--min-alt', type=float, default=20.0, help='metres above the EKF origin')
    ap.add_argument('--dem', nargs='*', default=[],
                    help='terrain/surface model .npz files: also run each map corrected with each, as <map>+<file stem>')
    a = ap.parse_args(argv)

    params = next(iter(yaml.safe_load(open(a.config)).values()))['ros__parameters']
    geo, frames, origin = read_bag(os.path.abspath(a.bag))
    if origin is None or len(geo) == 0:
        sys.exit('bag has no EKF origin or geopose')
    print(f'{len(frames)} frames, {len(geo)} poses, origin {origin}')

    from ap_ngps_ros2.trt_matcher import TrtMatcher
    trt = TrtMatcher(engine_path=params['tensorrt_engine_path'], match_threshold=float(params.get('match_threshold', 0.1)),
                     warmup=True)
    ctxs = {m: matcher_ctx(m, params, trt, origin) for m in a.maps}
    models = {os.path.splitext(os.path.basename(p))[0]: TerrainModel.load(p) for p in a.dem}
    variants = [(m, m, None) for m in a.maps] + [(f'{m}+{k}', m, k) for k in models for m in a.maps]

    t_geo = geo[:, 0]
    en = np.array([ap_frame.ll_to_en(origin, la, lo) for la, lo in geo[:, 1:3]])
    vel = np.gradient(en, t_geo, axis=0)
    rows = {v: [] for v, _, _ in variants}
    # takeoff point: where the EKF sat while armed on the ground is unknown here; the first pose below 1 m
    low = np.flatnonzero(geo[:, 3] - origin[2] < 1.0)
    takeoff = en[low[0]] if len(low) else en[0]
    next_t, decode = -1e18, cv.IMREAD_COLOR
    for ts, data in frames:
        if ts < next_t or ts < t_geo[0] or ts > t_geo[-1]:
            continue
        i = int(np.clip(np.searchsorted(t_geo, ts), 1, len(t_geo) - 1))
        if abs(t_geo[i] - ts) > 0.2:
            continue
        w = (ts - t_geo[i - 1]) / max(t_geo[i] - t_geo[i - 1], 1e-6)
        truth = en[i - 1] + w * (en[i] - en[i - 1])
        alt = geo[i, 3] - origin[2]
        if alt < a.min_alt:
            continue
        next_t = ts + a.period
        img = cv.imdecode(np.frombuffer(data, np.uint8), decode)
        if decode == cv.IMREAD_COLOR and img.shape[1] >= 1280:
            decode = cv.IMREAD_REDUCED_COLOR_2
        if img.shape[:2] != (360, 640):
            img = cv.resize(img, (640, 360), interpolation=cv.INTER_AREA)
        heading = (90.0 - geo[i, 4] + camera.YAW_OFFSET_DEG) % 360.0
        speed = float(np.linalg.norm(vel[i]))
        for v, m, k in variants:
            h, rise = height_above_ground(alt, models[k], origin, takeoff, truth) if k else (alt, None)
            fix, why, sc = ctxs[m]._ngps_candidate(img, truth, heading, h)
            r = {'t': ts, 'alt': alt, 'height': h, 'rise': rise, 'speed': speed, 'why': why}
            if fix is not None:
                r['err'] = (fix - truth).tolist()
                r['scale'] = sc
            rows[v].append(r)
    res = {'bag': os.path.abspath(a.bag), 'origin': origin, 'period_s': a.period, 'min_alt_m': a.min_alt,
           'dem': a.dem, 'maps': {v: dict(summarise(rows[v]), description=ctxs[m].desc) for v, m, _ in variants}}
    out = os.path.abspath(a.bag).rstrip('/') + '_map_compare.json'
    with open(out, 'w') as f:
        json.dump(dict(res, samples=rows), f)
    for m, s in sorted(res['maps'].items(), key=lambda kv: -kv[1]['rate']):
        print(f"\n{m}: {s['ok']}/{s['attempts']} matched ({100 * s['rate']:.0f}%)  {s['description']}")
        print(f"   by height {s['by_height']}")
        print(f"   failures  {s['why']}")
        if 'offset_en_m' in s:
            print(f"   offset_en_m {s['offset_en_m']} from {s['offset_from']}; spread p50 {s['spread_m_p50']} m, "
                  f"p90 {s['spread_m_p90']} m; fit scale {s['fit_scale_median']} (all p10-p90 {s['fit_scale_p10_p90_all']})")
    print('\nwrote', out)


if __name__ == '__main__':
    main()
