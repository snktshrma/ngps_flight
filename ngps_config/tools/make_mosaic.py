#!/usr/bin/env python3
"""Today's map: paste a GPS flight's camera frames onto a base map, as a new NGPS reference map.

    python3 ngps_config/tools/make_mosaic.py --bag bags/<gps flight> --base ottawa1 --name ottawa1_today \
        [--base-offset E N] [--min-alt 60] [--period 0.5] [--max-speed 6] [--t-start S] [--t-end S]

Runs inside vps-jp7 (ROS bag reading). Each sampled frame is placed with the EKF's own position, heading and
height at the frame's timestamp (a GPS flight, so that is GPS truth), using the fusion node's camera model
(camera.py: HFOV at 640 px, nadir pixel, image-up = heading + yaw offset) -- the same geometry the node uses
to cut its map crops, so a frame pasted here lands where the node will look for it. Where several frames
cover a pixel, the one whose nadir is closest wins (sharpest, least tilt/parallax).

The base map's measured offset (--base-offset, its offset_en_m from map_compare.py) is removed by shifting the
base image, so the composite is GPS-consistent everywhere: its metadata gets offset_en_m [0, 0].
Writes ngps_config/tiff/<name>/{mosaic.tif, metadata.json, preview.jpg, coverage.png}.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import cv2 as cv
import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'ap_ngps_ros2', 'src'))
from ap_ngps_ros2 import ap_frame, camera  # noqa: E402

TIFF = os.path.join(ROOT, 'ngps_config', 'tiff')
W640, H640 = 640, 360


def mercator_y(lat):
    return math.log(math.tan(math.pi / 4 + math.radians(lat) / 2))


class MapGrid:
    """Pixel <-> lat/lon for a make_map.py mosaic (web-mercator tile grid; square pixels of mpp metres)."""

    def __init__(self, meta):
        b = meta['bounds']
        self.min_lon, self.min_lat, self.max_lon, self.max_lat = (
            [float(v) for v in b.split(',')] if isinstance(b, str) else [float(v) for v in b])
        self.W, self.H = meta['size_px']
        self.mpp = 156543.03392 * math.cos(math.radians(0.5 * (self.min_lat + self.max_lat))) / 2 ** int(meta['zoom_level'])

    def px(self, lat, lon):
        c = (lon - self.min_lon) / (self.max_lon - self.min_lon) * self.W
        r = (mercator_y(self.max_lat) - mercator_y(lat)) / (mercator_y(self.max_lat) - mercator_y(self.min_lat)) * self.H
        return c, r

    def local_mpp(self, lat):
        """Metres per pixel at lat (web-mercator pixels shrink with cos(lat) across the map)."""
        lat_mid = 0.5 * (self.min_lat + self.max_lat)
        return self.mpp * math.cos(math.radians(lat)) / math.cos(math.radians(lat_mid))


def frame_to_map_affine(width, height, alt, heading_deg, centre_px, mpp, hfov_deg=camera.HFOV_DEG,
                        nadir_x=camera.NADIR_OFFSET_X_FRAC, nadir_y=camera.NADIR_OFFSET_Y_FRAC):
    """2x3 affine taking frame pixels (width x height image) to map pixels.

    heading_deg: compass direction of the image's up (the node's _heading(): vehicle heading + yaw offset).
    centre_px: map pixel (col, row) under the aircraft (the nadir pixel P of the frame).
    Same conventions as VoNgpsFusionNode._ngps_candidate: ground metres per 640-px-frame pixel = alt / F,
    F = 320 / tan(hfov/2); frame right = heading + 90 deg; map rows grow southwards.
    """
    F640 = (W640 / 2) / math.tan(math.radians(hfov_deg / 2))
    s = alt / F640 * (W640 / width)                    # metres per pixel of this frame
    P = np.array([width * (0.5 + nadir_x), height * (0.5 + nadir_y)])
    h = math.radians(heading_deg)
    k = s / mpp
    R = k * np.array([[math.cos(h), -math.sin(h)], [math.sin(h), math.cos(h)]])
    t = np.asarray(centre_px, float) - R @ P
    return np.hstack([R, t[:, None]])


def read_frames(bag, period, min_alt, max_speed, time_offset, t_start=-1e9, t_end=1e9):
    import rosbag2_py
    from geographic_msgs.msg import GeoPointStamped, GeoPoseStamped
    from geometry_msgs.msg import TwistStamped
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/ap/geopose/filtered', '/ap/gps_global_origin/filtered',
                                                  '/camera/image_raw/compressed', '/ap/twist/filtered']))
    st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    geo, frames, origin, tw = [], [], None, []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/ap/geopose/filtered':
            m = deserialize_message(data, GeoPoseStamped)
            p = m.pose.position
            if ap_frame.valid_ll(p.latitude, p.longitude):
                geo.append((st(m.header), p.latitude, p.longitude, p.altitude, ap_frame.yaw_enu_deg(m.pose.orientation)))
        elif topic == '/ap/gps_global_origin/filtered':
            m = deserialize_message(data, GeoPointStamped).position
            if ap_frame.valid_ll(m.latitude, m.longitude):
                origin = (m.latitude, m.longitude, m.altitude)
        elif topic == '/ap/twist/filtered':
            m = deserialize_message(data, TwistStamped)
            tw.append((st(m.header), math.hypot(m.twist.linear.x, m.twist.linear.y)))
        else:
            m = deserialize_message(data, CompressedImage)
            frames.append((st(m.header) + time_offset, bytes(m.data)))
    if origin is None or not geo:
        sys.exit('bag has no EKF origin or geopose')
    G = np.array(geo)
    en = np.array([ap_frame.ll_to_en(origin, la, lo) for la, lo in G[:, 1:3]])
    # Horizontal speed from the EKF velocity, not differentiated positions (AP_DDS lat/lon are float32 steps).
    TW = np.array(tw)
    if len(TW):
        speed_at = lambda t: float(np.interp(t, TW[:, 0], TW[:, 1]))
    else:
        speed_at = lambda t: float(np.hypot(*(np.array([np.interp(t + 0.5, G[:, 0], en[:, k]) for k in (0, 1)])
                                             - np.array([np.interp(t - 0.5, G[:, 0], en[:, k]) for k in (0, 1)]))))
    out, next_t = [], -1e18
    yaw_unwrapped = np.degrees(np.unwrap(np.radians(G[:, 4])))
    for ts, data in frames:
        if ts < next_t or ts < G[0, 0] or ts > G[-1, 0] or not t_start <= ts - G[0, 0] <= t_end:
            continue
        i = int(np.clip(np.searchsorted(G[:, 0], ts), 1, len(G) - 1))
        if abs(G[i, 0] - ts) > 0.2:
            continue
        alt = float(np.interp(ts, G[:, 0], G[:, 3])) - origin[2]
        speed = speed_at(ts)
        if alt < min_alt or speed > max_speed:
            continue
        lat = float(np.interp(ts, G[:, 0], G[:, 1]))
        lon = float(np.interp(ts, G[:, 0], G[:, 2]))
        yaw = float(np.interp(ts, G[:, 0], yaw_unwrapped))
        out.append({'t': ts, 'lat': lat, 'lon': lon, 'alt': alt, 'yaw_enu': yaw, 'speed': speed, 'jpeg': data})
        next_t = ts + period
    return out, origin


def paste(canvas, best, frame_bgr, A, footprint_pad=2):
    """Warp frame onto canvas where it is the closest-nadir frame so far. best: per-pixel squared distance (float32)."""
    h, w = frame_bgr.shape[:2]
    corners = cv.transform(np.float32([[[0, 0], [w, 0], [w, h], [0, h]]]), A)[0]
    c0, r0 = np.floor(corners.min(0)).astype(int) - footprint_pad
    c1, r1 = np.ceil(corners.max(0)).astype(int) + footprint_pad
    c0, r0 = max(c0, 0), max(r0, 0)
    c1, r1 = min(c1, canvas.shape[1]), min(r1, canvas.shape[0])
    if c1 <= c0 or r1 <= r0:
        return 0
    A_roi = A.copy()
    A_roi[:, 2] -= (c0, r0)
    size = (c1 - c0, r1 - r0)
    warped = cv.warpAffine(frame_bgr, A_roi, size, flags=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)
    valid = cv.warpAffine(np.full((h, w), 255, np.uint8), A_roi, size, flags=cv.INTER_NEAREST) > 0
    # distance of each ROI pixel from this frame's nadir (map px)
    P = A_roi @ np.array([w * (0.5 + camera.NADIR_OFFSET_X_FRAC), h * (0.5 + camera.NADIR_OFFSET_Y_FRAC), 1.0])
    yy, xx = np.mgrid[0:size[1], 0:size[0]].astype(np.float32)
    d2 = (xx - P[0]) ** 2 + (yy - P[1]) ** 2
    roi_best = best[r0:r1, c0:c1]
    take = valid & (d2 < roi_best)
    canvas[r0:r1, c0:c1][take] = warped[take]
    roi_best[take] = d2[take]
    return int(take.sum())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--base', required=True, help='base map folder under ngps_config/tiff/ (e.g. ottawa1)')
    ap.add_argument('--name', required=True, help='output map folder')
    ap.add_argument('--base-offset', nargs=2, type=float, metavar=('E', 'N'),
                    help="base map's offset_en_m (map minus truth) to remove; default: its metadata's, else 0")
    ap.add_argument('--min-alt', type=float, default=60.0)
    ap.add_argument('--period', type=float, default=0.5, help='seconds between frames used')
    ap.add_argument('--max-speed', type=float, default=6.0, help='m/s; faster frames are skipped (lag smear)')
    ap.add_argument('--time-offset', type=float, default=0.0, help='s added to camera stamps (residual camera lag)')
    ap.add_argument('--t-start', type=float, default=-1e9, help='s after the first EKF pose: use frames from here')
    ap.add_argument('--t-end', type=float, default=1e9, help='s after the first EKF pose: use frames up to here')
    a = ap.parse_args(argv)

    base_dir = os.path.join(TIFF, a.base)
    meta = json.load(open(os.path.join(base_dir, 'metadata.json')))
    grid = MapGrid(meta)
    t0 = time.time()
    canvas = cv.imread(os.path.join(base_dir, 'mosaic.tif'))
    off = a.base_offset if a.base_offset is not None else meta.get('offset_en_m', [0.0, 0.0])
    dc, dr = int(round(-off[0] / grid.mpp)), int(round(off[1] / grid.mpp))
    if dc or dr:   # move the base's content by -offset: new[r, c] = old[r - dr, c - dc]
        shifted = np.zeros_like(canvas)
        H, W = canvas.shape[:2]
        shifted[max(dr, 0):H + min(dr, 0), max(dc, 0):W + min(dc, 0)] = canvas[max(-dr, 0):H + min(-dr, 0), max(-dc, 0):W + min(-dc, 0)]
        canvas = shifted
    print(f'base {a.base} {canvas.shape[1]}x{canvas.shape[0]}, offset removed E {off[0]:+.2f} N {off[1]:+.2f} m '
          f'({dc:+d}, {dr:+d} px), {time.time() - t0:.0f} s', flush=True)

    frames, origin = read_frames(os.path.abspath(a.bag), a.period, a.min_alt, a.max_speed, a.time_offset,
                                 a.t_start, a.t_end)
    print(f'{len(frames)} frames to paste (>= {a.min_alt:.0f} m, <= {a.max_speed:.0f} m/s, every {a.period} s)', flush=True)
    if not frames:
        sys.exit('no usable frames')
    # placements first, so the per-pixel bookkeeping only spans the flown area, not the whole map
    jobs, lo, hi = [], np.array([np.inf, np.inf]), np.array([-np.inf, -np.inf])
    for fr in frames:
        img = cv.imdecode(np.frombuffer(fr['jpeg'], np.uint8), cv.IMREAD_COLOR)
        heading = (90.0 - fr['yaw_enu'] + camera.YAW_OFFSET_DEG) % 360.0
        A = frame_to_map_affine(img.shape[1], img.shape[0], fr['alt'], heading, grid.px(fr['lat'], fr['lon']),
                                grid.local_mpp(fr['lat']))
        c = cv.transform(np.float32([[[0, 0], [img.shape[1], 0], [img.shape[1], img.shape[0]], [0, img.shape[0]]]]), A)[0]
        lo, hi = np.minimum(lo, c.min(0)), np.maximum(hi, c.max(0))
        jobs.append((img, A))
    c0, r0 = [max(int(v) - 4, 0) for v in lo]
    c1, r1 = min(int(hi[0]) + 4, canvas.shape[1]), min(int(hi[1]) + 4, canvas.shape[0])
    if c1 <= c0 or r1 <= r0:
        sys.exit('the flight is outside the base map')
    sub = canvas[r0:r1, c0:c1]                        # a view: pasting into it writes the canvas
    best = np.full(sub.shape[:2], np.inf, np.float32)
    for img, A in jobs:
        A = A.copy()
        A[:, 2] -= (c0, r0)
        paste(sub, best, img, A)
    cover = np.zeros(canvas.shape[:2], bool)
    cover[r0:r1, c0:c1] = np.isfinite(best)
    print(f'pasted: {cover.sum() * grid.mpp ** 2 / 1e4:.1f} ha of today\'s imagery', flush=True)

    out = os.path.join(TIFF, a.name)
    os.makedirs(out, exist_ok=True)
    cv.imwrite(os.path.join(out, 'mosaic.tif'), canvas)
    pv = cv.resize(canvas, (1200, int(1200 * canvas.shape[0] / canvas.shape[1])), interpolation=cv.INTER_AREA)
    cv.imwrite(os.path.join(out, 'preview.jpg'), pv, [cv.IMWRITE_JPEG_QUALITY, 85])
    cv.imwrite(os.path.join(out, 'coverage.png'),
               cv.resize(cover.astype(np.uint8) * 255, (pv.shape[1], pv.shape[0]), interpolation=cv.INTER_AREA))
    m2 = dict(meta)
    m2.update({
        'description': f"today's camera mosaic ({len(frames)} frames, {os.path.basename(a.bag.rstrip('/'))}) "
                       f"over {a.base} (offset removed); {meta.get('description', '')}",
        'source': f'mosaic+{meta.get("source", a.base)}',
        'offset_en_m': [0.0, 0.0],
        'offset_note': f'GPS-consistent by construction: frames placed with GPS poses; base {a.base} shifted by '
                       f'its offset E {off[0]:+.2f} N {off[1]:+.2f} m. Re-check with map_compare.py on a later flight.',
        'mosaic_frames': len(frames), 'mosaic_coverage_ha': round(float(cover.sum() * grid.mpp ** 2 / 1e4), 1),
        'downloaded': time.strftime('%Y-%m-%d'),
    })
    with open(os.path.join(out, 'metadata.json'), 'w') as f:
        json.dump(m2, f, indent=1, ensure_ascii=False)
    print('wrote', out, f'({time.time() - t0:.0f} s)')


if __name__ == '__main__':
    main()
