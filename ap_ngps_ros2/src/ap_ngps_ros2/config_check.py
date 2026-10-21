"""Reject a live-flight parameter file before it can cost a flight.

    python3 -m ap_ngps_ros2.config_check config/fusion_nagano0.yaml

Every rule here is a failure that has already happened once:
  * flight 2: altitude_topic pointed at /mavros/... (this aircraft is DDS only, so
    the topic never existed) and publish_ap_dds_tf was false -- NGPS produced nothing.
  * an altitude source in AMSL (/ap/geopose/filtered reads ~943 m here) instead of
    height above home, which sizes the map crop 10x too big.
  * the Maxar config carried HFOV 71.8 deg after 49.2 had been measured.
The nodes run the same checks on their live parameters at startup and report them
over statustext; flight_log.sh runs this on the file before it starts anything.

Exit code 0 = no errors (warnings allowed), 1 = errors.
"""

from __future__ import annotations

import json
import math
import os
import sys
from typing import Iterable, List, Tuple

from ap_ngps_ros2 import camera

ERROR, WARN = 'ERROR', 'WARN'
Finding = Tuple[str, str]

NODES = ('vo_ngps_fusion_node', 'ngps_localization_node')
AP_TF = '/ap/tf'
MIN_TF_HZ = 5.0   # ArduPilot calls visual odometry unhealthy after 300 ms without data

# Node defaults that matter for the rules below, for keys a file leaves out.
DEFAULTS = {
    'vo_ngps_fusion_node': {
        'geopose_topic': '/ap/geopose/filtered', 'origin_topic': '/ap/gps_global_origin/filtered',
        'status_topic': '/ap/status', 'tf_topic': AP_TF, 'tf_rate_hz': 10.0,
    },
    'ngps_localization_node': {
        'publish_ap_dds_tf': False, 'ap_dds_tf_topic': AP_TF, 'dds_tf_rate_hz': 10.0,
        'altitude_topic': '/ap/pose/filtered', 'altitude_source': 'pose',
        'dds_tf_before_first_fix': False,
    },
}
# Height above home. geopose and navsat altitudes are AMSL.
AGL_SOURCES = {'pose'}


def tf_topic_of(node: str, params: dict) -> str:
    key = 'tf_topic' if node == 'vo_ngps_fusion_node' else 'ap_dds_tf_topic'
    return str(params.get(key, DEFAULTS[node][key]))


def altitude_of(node: str, params: dict) -> Tuple[str, str]:
    """(topic, source) the NGPS node takes its height above home from.

    The fusion node has none to check: it works in the EKF-origin frame from
    /ap/geopose/filtered and /ap/gps_global_origin/filtered (see ap_frame.py).
    """
    if node == 'vo_ngps_fusion_node':
        return '', 'origin'
    d = DEFAULTS[node]
    return (str(params.get('altitude_topic', d['altitude_topic'])),
            str(params.get('altitude_source', d['altitude_source'])).strip().lower())


def _walk_strings(obj, path=''):
    if isinstance(obj, str):
        yield path, obj
    elif isinstance(obj, dict):
        for k, v in obj.items():
            yield from _walk_strings(v, f'{path}.{k}' if path else str(k))
    elif isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            yield from _walk_strings(v, f'{path}[{i}]')


def _metadata(node: str, params: dict):
    if node == 'vo_ngps_fusion_node':
        path = params.get('metadata_path', '')
    else:
        ref = str(params.get('reference_image_path', ''))
        path = os.path.join(os.path.dirname(ref), 'metadata.json') if os.path.isabs(ref) else ''
    if not path or not os.path.isabs(str(path)) or not os.path.isfile(str(path)):
        return None
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def check_params(node: str, params: dict) -> List[Finding]:
    """Rules for one node's parameters (file values, or the node's live values)."""
    out: List[Finding] = []
    if node not in NODES:
        return [(WARN, f'{node}: not a live node this checker knows; skipped')]

    for key, value in _walk_strings(params):
        if '/mavros' in value:
            out.append((ERROR, f'{key}: {value!r} is a MAVROS topic; this aircraft is DDS only (use /ap/...)'))

    tf_topic = tf_topic_of(node, params)
    if node == 'ngps_localization_node':
        if not bool(params.get('publish_ap_dds_tf', DEFAULTS[node]['publish_ap_dds_tf'])):
            out.append((ERROR, 'publish_ap_dds_tf is false (the default): ArduPilot would receive nothing'))
        rate = float(params.get('dds_tf_rate_hz', DEFAULTS[node]['dds_tf_rate_hz']))
    else:
        rate = float(params.get('tf_rate_hz', DEFAULTS[node]['tf_rate_hz']))
        for key in ('geopose_topic', 'origin_topic', 'status_topic'):
            topic = str(params.get(key, DEFAULTS[node][key]))
            if not topic.startswith('/ap/'):
                out.append((ERROR, f'{key}: {topic!r} is not an ArduPilot DDS topic (/ap/...)'))
        for key in ('pose_topic', 'gps_topic', 'home_jump_m'):
            if key in params:
                out.append((WARN, f'{key}: no longer used (the frame is the EKF origin, not home); remove it'))
        backend = str(params.get('vo_backend', 'opencv'))
        if backend not in ('opencv', 'vpi_cuda'):
            out.append((ERROR, f"vo_backend {backend!r}: must be 'opencv' or 'vpi_cuda'"))
        fpl = int(params.get('vpi_features_per_level', 500))
        if not 100 <= fpl <= 2000:
            out.append((ERROR, f'vpi_features_per_level {fpl}: outside 100-2000'))
        pm = float(params.get('predict_ms', 0.0))
        if not 0.0 <= pm <= 400.0:
            out.append((ERROR, f'predict_ms {pm:g}: outside 0-400 (it should be measured lag - VISO_DELAY_MS)'))
        pw = float(params.get('predict_window_s', 0.3))
        if not 0.1 <= pw <= 1.0:
            out.append((ERROR, f'predict_window_s {pw:g}: outside 0.1-1.0 s'))
        nb = float(params.get('ngps_blend_mps', 0.4))
        if not 0.0 <= nb <= 5.0:
            out.append((ERROR, f'ngps_blend_mps {nb:g}: outside 0-5 m/s'))
        source = str(params.get('camera_source', 'topic'))
        if source not in ('topic', 'rtsp'):
            out.append((ERROR, f"camera_source {source!r}: must be 'topic' or 'rtsp'"))
        if source == 'rtsp':
            url = str(params.get('rtsp_url', 'rtsp://192.168.144.25:8554/video1'))
            if not url.startswith('rtsp://'):
                out.append((ERROR, f'rtsp_url {url!r} is not an rtsp:// URL'))
            if str(params.get('rtsp_codec', 'h264')) not in ('h264', 'h265'):
                out.append((ERROR, f"rtsp_codec {params.get('rtsp_codec')!r}: must be 'h264' or 'h265'"))
    if rate < MIN_TF_HZ:
        out.append((ERROR, f'{tf_topic} rate {rate:g} Hz: below {MIN_TF_HZ:g} Hz ArduPilot times visual odometry out'))
    if tf_topic != AP_TF:
        out.append((WARN, f'publishing on {tf_topic!r}, not {AP_TF}: ArduPilot will not see it (test config?)'))

    alt_topic, alt_source = altitude_of(node, params)
    if node == 'vo_ngps_fusion_node':
        pass
    elif not alt_topic:
        out.append((ERROR, 'altitude_topic is empty: every frame would use default_altitude_m'))
    elif not alt_topic.startswith('/ap/'):
        out.append((ERROR, f'altitude topic {alt_topic!r} is not an ArduPilot DDS topic (/ap/...)'))
    if node == 'ngps_localization_node' and alt_source not in AGL_SOURCES:
        why = 'AMSL (~943 m here), not height above home' if alt_source in ('geopose', 'navsat') else 'unknown datum'
        out.append((ERROR, f'altitude_source {alt_source!r} is {why}; use altitude_source: pose on /ap/pose/filtered'))

    # Camera intrinsics: single-sourced in camera.py.
    for key, want in camera.PARAMS[node].items():
        if key not in params:
            continue
        got = float(params[key])
        if key == 'kernel_hfov_deg' and got == 0.0:
            out.append((WARN, 'kernel_hfov_deg 0: map crop is not sized from altitude'))
        elif not math.isclose(got, want, rel_tol=1e-6, abs_tol=1e-9):
            out.append((ERROR, f'{key} {got:g} differs from the measured camera ({want:g}, camera.py); '
                               'remove it from the config'))

    for key in ('reference_image_path', 'metadata_path', 'tensorrt_engine_path', 'terrain_dem_path'):
        path = str(params.get(key, '') or '')
        if path and os.path.isabs(os.path.expanduser(path)) and not os.path.exists(os.path.expanduser(path)):
            out.append((ERROR, f'{key}: {path} does not exist'))

    meta = _metadata(node, params)
    if meta is not None:
        b = meta.get('bounds')
        bounds = [float(v) for v in (b.split(',') if isinstance(b, str) else b)] if b else None
        mpp = float(params.get('mosaic_meters_per_pixel', 0.0) or 0.0)
        zoom = meta.get('zoom_level')
        if bounds and zoom is not None and mpp > 0:
            lat = 0.5 * (bounds[1] + bounds[3])
            want = 156543.03392 * math.cos(math.radians(lat)) / 2 ** int(zoom)
            if abs(mpp - want) / want > 0.02:
                out.append((ERROR, f'mosaic_meters_per_pixel {mpp:g} but the map is zoom {zoom} ({want:.4f} m/px)'))
        off = meta.get('offset_en_m')
        if off is not None:
            if len(off) != 2 or math.hypot(float(off[0]), float(off[1])) > 10.0:
                out.append((ERROR, f'map metadata offset_en_m {off}: must be [east, north] and under 10 m'))
            elif node != 'vo_ngps_fusion_node':
                out.append((WARN, f'map offset_en_m {off} is applied by vo_ngps_fusion_node only'))
        if bounds and node == 'ngps_localization_node':
            keys = ('reference_min_lon', 'reference_min_lat', 'reference_max_lon', 'reference_max_lat')
            if all(k in params for k in keys):
                if max(abs(float(params[k]) - v) for k, v in zip(keys, bounds)) > 1e-6:
                    out.append((ERROR, 'reference_min/max_lat/lon disagree with the map metadata.json bounds'))
    return out


def load(path: str) -> dict:
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    out = {}
    for name, body in data.items():
        params = (body or {}).get('ros__parameters') if isinstance(body, dict) else None
        if params is not None:
            out[str(name).lstrip('/')] = params
    return out


def check_file(path: str) -> List[Finding]:
    try:
        nodes = load(path)
    except Exception as exc:  # noqa: BLE001 - any parse failure is a NO-GO
        return [(ERROR, f'{path}: cannot read: {exc}')]
    if not nodes:
        return [(ERROR, f'{path}: no <node>: ros__parameters: section')]
    out: List[Finding] = []
    for node, params in nodes.items():
        out += check_params(node, params)
    return out


def report(findings: Iterable[Finding], log=print) -> bool:
    ok = True
    for level, text in findings:
        ok = ok and level != ERROR
        log(f'  {"XX" if level == ERROR else "!!"} {level:<5} {text}')
    return ok


def main(argv=None) -> int:
    paths = list(sys.argv[1:] if argv is None else argv)
    if not paths:
        print(__doc__)
        return 1
    ok = True
    for path in paths:
        findings = check_file(path)
        print(f'{path}: {"OK" if not findings else ""}')
        ok = report(findings) and ok
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
