#!/usr/bin/env python3
"""Go / no-go check before takeoff: is everything you need actually being recorded?

    ros2 run ap_dataset_logger preflight_check.py
    ros2 run ap_dataset_logger preflight_check.py --seconds 15
    ros2 run ap_dataset_logger preflight_check.py --node-config FILE   # + live NGPS / VO checks

Listens to the live topics for a few seconds and checks each against what an offline
NGPS / VIO comparison needs. Run it with the vehicle powered, the camera streaming and
the recorder already started -- it checks the recorder too.

With --node-config (flight_log.sh passes the --ngps / --fusion file) it also checks
what the live position node needs: the file itself (no /mavros topics, /ap/tf
enabled, intrinsics match camera.py), its altitude source is publishing a height
above home (not AMSL), and exactly one /ap/tf publisher at a steady >= 8 Hz.

Exit code: 0 = GO, 2 = GO with warnings, 1 = NO-GO.
"""

import argparse
import glob
import math
import os
import re
import shutil
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

GO, WARN, NOGO = 'GO', 'WARN', 'NO-GO'

# Worth having, but not worth grounding a flight over: missing or slow is a warning.
OPTIONAL = {'/mavros/gimbal_control/device/attitude_status'}

# (topic, why it matters, go_hz, warn_hz). Below warn_hz is NO-GO.
# Thresholds are set from what the hardware actually delivers, not from nominal rates:
# a check that warns every time teaches you to ignore it.
#   camera: 30 fps in daylight, 24.95 in low light -- both fine for NGPS and VIO.
#   DDS IMU: AP_DDS is configured for 200 Hz but delivered ~112 Hz even in SITL over local
#            UDP, and ~120 Hz in an earlier recording -- the publisher's ceiling, not a
#            fault. High-rate IMU for VIO comes from the dataflash log.
#   MAVLink: mavlink_rates.py asks for 200 Hz IMU, 30 Hz position and gimbal attitude;
#            the CubeOrange+ over its PPP/Ethernet link delivered all of it in full.
CAMERA = ('/camera/image_raw/compressed', 'camera frames', 24.0, 15.0)

LINKS = {
    'dds': {
        'rates': [
            CAMERA,
            ('/ap/imu/experimental/data', 'IMU (DDS tops out ~110 Hz; use dataflash IMU for VIO)',
             90.0, 50.0),
            ('/ap/time',             'aligns bag with dataflash log', 50.0, 20.0),
            ('/ap/pose/filtered',    'EKF local position (truth)',    20.0, 10.0),
            ('/ap/geopose/filtered', 'EKF global position (truth)',   20.0, 10.0),
            ('/ap/twist/filtered',   'EKF velocity (VIO/VO truth)',   20.0, 10.0),
            # ~2 Hz from AP_DDS. The live VO seeds on arming, so without it VO never starts.
            ('/ap/status',           'arming state (VO seeds on arm)', 1.5, 0.5),
        ],
        'gps': '/ap/navsat',
        'origin': '/ap/gps_global_origin/filtered',
        'state': None,
    },
    'mavlink': {
        'rates': [
            CAMERA,
            ('/mavros/imu/data', 'IMU (200 Hz requested; VIO source)',           150.0, 30.0),
            ('/mavros/time_reference',               'aligns bag with dataflash log',  2.0, 0.5),
            ('/mavros/local_position/pose',          'EKF local position (truth)',    15.0, 5.0),
            ('/mavros/global_position/global',       'EKF global position (truth)',   15.0, 5.0),
            ('/mavros/local_position/velocity_local', 'EKF velocity (VIO/VO truth)',  15.0, 5.0),
            ('/mavros/gimbal_control/device/attitude_status',
             'gimbal attitude (where the camera points)', 20.0, 5.0),
        ],
        'gps': '/mavros/global_position/raw/fix',
        'origin': '/mavros/global_position/gp_origin',
        'state': '/mavros/state',
    },
}

# Topics only the flight controller publishes over DDS: no messages on any of them means no
# DDS session. (Merely existing proves nothing: our own nodes subscribe to them.)
FC_DDS_TOPICS = ('/ap/time', '/ap/pose/filtered', '/ap/status', '/ap/navsat')
DDS_HINT = (
    'no DDS session with the flight controller. Check: micro_ros_agent is running here '
    '(flight_log.sh start --fc udp:2019); FC parameters DDS_ENABLE=1, DDS_IP0..3 = 192.168.144.20 '
    '(this Jetson; the firmware default 192.168.144.2 reaches nothing), DDS_UDP_PORT=2019, '
    f'DDS_DOMAIN_ID = ROS_DOMAIN_ID here ({os.environ.get("ROS_DOMAIN_ID") or "0"})')

DEFAULT_BAG_ROOT = os.path.join(os.path.expanduser('~'), 'ngps_ws', 'src', 'ngps_flight', 'bags')


class Probe(Node):
    def __init__(self, link):
        super().__init__('preflight_check')
        self.link = link
        # Best effort accepts both reliable and best-effort publishers.
        self.qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)
        self.counts = {}
        self.navsat_status = None
        self.fc_connected = None
        self.camera_diag = {}
        self.subscribed = set()
        self.extra = {}            # topic -> decoder callback for --node-config checks
        self.armed = None
        self.altitude = None
        self.tf_stamps = []
        self.tf_last = None        # (east, north, up) of the live node's latest /ap/tf
        self.ekf_origin = None     # (lat, lon, alt)
        self.ekf_geo = None        # (lat, lon, alt)

    def subscribe_available(self):
        """Subscribe to whatever check topics exist now; call repeatedly while waiting."""
        types = dict(self.get_topic_names_and_types())
        cfg = LINKS[self.link]
        wanted = [t for t, *_ in cfg['rates']] + [cfg['gps'], cfg['origin'], '/diagnostics']
        if cfg['state']:
            wanted.append(cfg['state'])
        wanted += [t for t in self.extra if t not in wanted]
        self.decoded = {cfg['gps'], '/diagnostics', cfg['state'], '/ap/status', *self.extra}
        for topic in wanted:
            if topic in self.subscribed or topic not in types:
                continue
            msg_type = get_message(types[topic][0])
            self.counts[topic] = 0
            raw = topic not in self.decoded
            self.create_subscription(msg_type, topic, self._cb(topic), self.qos, raw=raw)
            self.subscribed.add(topic)

    def _cb(self, topic):
        def f(msg):
            self.counts[topic] += 1
            if topic in self.extra:
                self.extra[topic](msg)
            if topic == '/ap/status':
                self.armed = bool(msg.armed)
            if topic == LINKS[self.link]['gps']:
                self.navsat_status = msg.status.status
            elif topic == LINKS[self.link]['state']:
                self.fc_connected = msg.connected
            elif topic == '/diagnostics':
                for st in msg.status:
                    if st.name.startswith('rtsp_camera'):
                        self.camera_diag = {kv.key: kv.value for kv in st.values}
                        self.camera_diag['_message'] = st.message
        return f


def newest_bag_file(roots):
    files = []
    for root in roots:
        files += glob.glob(os.path.join(root, '**', '*.mcap'), recursive=True)
    return max(files, key=os.path.getmtime) if files else None


def recorder_running():
    try:
        out = subprocess.run(['ps', '-eo', 'args'], capture_output=True, text=True).stdout
    except OSError:
        return False
    # The program itself, not any shell or editor whose command line mentions it (the
    # container shares the host's process table).
    rec = re.compile(r'^\S*python3(\.\d+)? \S*/ros2 bag record( |$)')
    return any(rec.match(line.strip()) for line in out.splitlines())


def load_node_config(path):
    """(node name, params, findings) for a --ngps / --fusion file, via ap_ngps_ros2."""
    try:
        from ap_ngps_ros2 import config_check
    except ImportError as exc:
        return None, {}, [('ERROR', f'cannot import ap_ngps_ros2.config_check ({exc}) -- rebuild')], None
    findings = config_check.check_file(path)
    try:
        nodes = config_check.load(path)
    except Exception:  # noqa: BLE001 - already reported by check_file
        nodes = {}
    node = next((n for n in nodes if n in config_check.NODES), None)
    return node, nodes.get(node, {}), findings, config_check


def watch_node_inputs(probe, node, params, cc):
    """Subscribe to the live node's altitude and /ap/tf output; returns (alt topic, source, tf topic)."""
    alt_topic, source = cc.altitude_of(node, params)
    tf_topic = cc.tf_topic_of(node, params)

    def on_alt(msg):
        if source == 'pose':
            probe.altitude = abs(float(msg.pose.position.z))
        elif source == 'geopose':
            probe.altitude = float(msg.pose.position.altitude)
        elif source == 'navsat':
            probe.altitude = float(msg.altitude)
        elif source == 'float64':
            probe.altitude = float(msg.data)

    def on_tf(msg):
        if not msg.transforms:
            return
        st = msg.transforms[0].header.stamp
        t = st.sec + st.nanosec * 1e-9 if (st.sec or st.nanosec) else time.time()
        probe.tf_stamps.append(t)
        tr = msg.transforms[0].transform.translation
        probe.tf_last = (-tr.y, tr.x, tr.z)   # sent as x=north, y=west, z=up

    def on_origin(msg):
        p = msg.position
        probe.ekf_origin = (p.latitude, p.longitude, p.altitude)

    def on_geo(msg):
        p = msg.pose.position
        probe.ekf_geo = (p.latitude, p.longitude, p.altitude)

    if alt_topic:
        probe.extra[alt_topic] = on_alt
    probe.extra[tf_topic] = on_tf
    probe.extra['/ap/gps_global_origin/filtered'] = on_origin
    probe.extra['/ap/geopose/filtered'] = on_geo
    return alt_topic, source, tf_topic


NADIR_TOL_DEG = 5.0
FC_MAVLINK = 'tcp:192.168.144.15:5760'


def pitch_from_quaternion(q):
    """Pitch (deg, nose/lens down negative, ArduPilot convention) from a MAVLink [w, x, y, z] quaternion."""
    w, x, y, z = q
    return math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))))


def gimbal_pitch(addr, timeout=6.0):
    """The gimbal's pitch as ArduPilot reports it (GIMBAL_DEVICE_ATTITUDE_STATUS), or None.

    Not on DDS, so this reads the FC's MAVLink link. Flight 8 (2026-10-06) flew with the gimbal at 0 deg,
    looking at the horizon: VO tracked hills and clouds and the drone wandered 8 m in 30 s on ExtNav.
    """
    try:
        from pymavlink import mavutil
        m = mavutil.mavlink_connection(addr, source_system=240, source_component=192)
        if not m.wait_heartbeat(timeout=timeout):
            return None
        m.mav.command_long_send(1, 1, 511, 0, 285, 200000, 0, 0, 0, 0, 0)   # 5 Hz
        m.mav.command_long_send(1, 1, 512, 0, 285, 0, 0, 0, 0, 0, 0)
        end = time.time() + timeout
        while time.time() < end:
            x = m.recv_match(type='GIMBAL_DEVICE_ATTITUDE_STATUS', blocking=True, timeout=1)
            if x is not None:
                return pitch_from_quaternion(x.q)
    except Exception:
        return None
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--link', choices=sorted(LINKS), default='dds',
                    help='which flight-controller link is in use')
    ap.add_argument('--bag-dir', default=DEFAULT_BAG_ROOT,
                    help='where the recorder writes (to check the bag is growing)')
    ap.add_argument('--mavlink', default=FC_MAVLINK,
                    help="FC MAVLink link for the gimbal-pitch check ('' to skip)")
    ap.add_argument('--node-config', default='',
                    help='parameter file of the live NGPS / VO node (flight_log.sh --ngps/--fusion)')
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.link)
    live = None
    if args.node_config:
        live_node, live_params, live_findings, cc = load_node_config(args.node_config)
        if live_node is not None:
            live = (live_node, live_params, *watch_node_inputs(node, live_node, live_params, cc))
    cfg = LINKS[args.link]
    roots = [args.bag_dir]
    bag_before = newest_bag_file(roots)
    size_before = os.path.getsize(bag_before) if bag_before else 0

    print(f'link: {args.link}   listening for {args.seconds:.0f} s ...\n')
    t_end = time.time() + args.seconds + 2.0
    t_count = None
    while time.time() < t_end:
        node.subscribe_available()
        rclpy.spin_once(node, timeout_sec=0.05)
        # Give discovery ~2 s before the rate window starts, then zero the counters so
        # the rates are not diluted by the time it took to find the topics.
        if t_count is None and time.time() > t_end - args.seconds:
            t_count = time.time()
            for k in node.counts:
                node.counts[k] = 0
    window = time.time() - (t_count or time.time() - args.seconds)

    results = []

    def add(item, status, detail):
        results.append((item, status, detail))

    if args.link == 'dds' and not any(node.counts.get(t) for t in FC_DDS_TOPICS):
        add('flight controller', NOGO, 'no /ap/* topics from the FC (see hint below)')
    if cfg['state']:
        if node.fc_connected is None:
            add('flight controller', NOGO, f'no {cfg["state"]} -- is MAVROS running?')
        elif not node.fc_connected:
            add('flight controller', NOGO, 'MAVROS up but NOT connected -- check fc:= port and baud')
        else:
            add('flight controller', GO, 'MAVLink heartbeat received')

    for topic, why, go_hz, warn_hz in cfg['rates']:
        worst = WARN if topic in OPTIONAL else NOGO
        if topic not in node.counts:
            add(topic, worst, f'not published -- {why}')
            continue
        hz = node.counts[topic] / window
        status = GO if hz >= go_hz else WARN if hz >= warn_hz else worst
        if hz == 0 and 'EKF' in why:
            # The flight controller advertises these but only fills them once the EKF has
            # a position relative to its origin -- typically ~30 s after GPS lock.
            add(topic, status, f'   0.0 Hz   {why} -- advertised but silent: EKF still '
                               'settling after GPS lock, re-run in ~30 s')
        else:
            add(topic, status, f'{hz:6.1f} Hz   {why}')

    # GPS: present, and does it have a fix? NavSatStatus: -1 no fix, >=0 fix.
    gps, origin = cfg['gps'], cfg['origin']
    if gps not in node.counts:
        add(gps, NOGO, 'not published -- GPS truth for the NGPS comparison')
    else:
        hz = node.counts[gps] / window
        if node.navsat_status is None:
            add(gps, NOGO, 'no messages received')
        elif node.navsat_status < 0:
            add(gps, WARN, f'{hz:6.1f} Hz   NO GPS FIX yet -- wait for lock')
        else:
            add(gps, GO, f'{hz:6.1f} Hz   GPS fix (status {node.navsat_status})')

    if node.counts.get(origin):
        add(origin, GO, 'EKF origin set')
    else:
        add(origin, WARN,
            'EKF origin not set yet (usually appears after GPS lock) -- needed to put '
            'local pose on the map')

    # The live NGPS / VO node: its config, its altitude input and its /ap/tf output.
    tf_pubs = node.get_publishers_info_by_topic('/ap/tf')
    if args.node_config:
        if live is None:
            for level, text in live_findings:
                add('node config', NOGO if level == 'ERROR' else WARN, text)
            if cc is not None:
                add('node config', NOGO, f'no vo_ngps_fusion_node / ngps_localization_node section in {args.node_config}')
        else:
            live_node, live_params, alt_topic, source, tf_topic = live
            for level, text in live_findings:
                add('node config', NOGO if level == 'ERROR' else WARN, text)
            if not live_findings:
                add('node config', GO, f'{os.path.basename(args.node_config)} ({live_node})')
            # Altitude: must be publishing, and on the ground read as height above home.
            if not alt_topic:
                pass   # fusion node: height comes from the EKF origin and geopose (frame check below)
            elif node.altitude is None:
                add('altitude', NOGO, f'nothing on {alt_topic or "(unset)"} -- NGPS/VO cannot size the map crop')
            elif node.armed:
                add('altitude', GO, f'{node.altitude:6.1f} m on {alt_topic} (armed)')
            elif abs(node.altitude) <= 15.0:
                add('altitude', GO, f'{node.altitude:6.1f} m above home on {alt_topic} (on the ground)')
            elif abs(node.altitude) <= 100.0:
                add('altitude', WARN, f'{node.altitude:6.1f} m on the ground? {alt_topic} ({source}) -- EKF settling?')
            else:
                add('altitude', NOGO, f'{node.altitude:6.1f} m on the ground: {alt_topic} ({source}) looks like AMSL, '
                                      'not height above home')
            # Output rate and gaps, from the publisher's own stamps.
            stamps = sorted(node.tf_stamps)
            if len(stamps) < 2:
                add(f'{tf_topic} rate', NOGO, 'not publishing -- ArduPilot VisOdom will be unhealthy, cannot arm')
            else:
                span = max(stamps[-1] - stamps[0], 1e-3)
                hz = (len(stamps) - 1) / span
                gap = max(b - a for a, b in zip(stamps, stamps[1:]))
                detail = f'{hz:6.1f} Hz   max gap {gap * 1000:4.0f} ms (VisOdom times out at 300 ms)'
                status = GO if hz >= 8.0 and gap <= 0.2 else WARN if hz >= 5.0 and gap <= 0.3 else NOGO
                add(f'{tf_topic} rate', status, detail)
            # Frame: /ap/tf must be ENU from the EKF ORIGIN -- ArduPilot does not re-offset
            # it. Before VO/NGPS takes over the node echoes the EKF, so on the ground the
            # two must agree; home-relative output is off by the home-origin distance
            # (15 m / 5.2 m on flight 3).
            from ap_ngps_ros2 import ap_frame
            ekf = ap_frame.ekf_en_up(node.ekf_origin, *node.ekf_geo) if node.ekf_geo and node.ekf_origin else None
            if node.ekf_origin is None or not ap_frame.valid_ll(*node.ekf_origin[:2]):
                add('/ap/tf frame', NOGO, 'no EKF origin (/ap/gps_global_origin/filtered): /ap/tf cannot be '
                                          'placed in ArduPilot\'s frame -- wait for GPS lock')
            elif ekf is None or node.tf_last is None:
                add('/ap/tf frame', NOGO if node.tf_last is None else WARN,
                    'cannot compare: ' + ('no /ap/tf' if node.tf_last is None else 'no /ap/geopose/filtered'))
            else:
                dh = math.hypot(node.tf_last[0] - ekf[0], node.tf_last[1] - ekf[1])
                dz = node.tf_last[2] - ekf[2]
                detail = (f'vs EKF from origin: horizontal {dh:5.1f} m, up {dz:+5.1f} m  '
                          f'(tf E {node.tf_last[0]:.1f} N {node.tf_last[1]:.1f} U {node.tf_last[2]:.1f})')
                if node.armed:
                    add('/ap/tf frame', GO if dh < 10 and abs(dz) < 3 else WARN, detail + ' (armed)')
                elif dh < 3.0 and abs(dz) < 1.5:
                    add('/ap/tf frame', GO, detail)
                elif dh < 10.0 and abs(dz) < 3.0:
                    add('/ap/tf frame', WARN, detail)
                else:
                    add('/ap/tf frame', NOGO, detail + ' -- not the EKF-origin frame (home-relative?)')
    names = ', '.join(sorted(f'{i.node_namespace.rstrip("/")}/{i.node_name}' for i in tf_pubs)) or '-'
    expect_one = live is not None and live[4] == '/ap/tf'
    if len(tf_pubs) == 1:
        add('/ap/tf publishers', GO, f'exactly one ({names})')
    elif len(tf_pubs) > 1:
        add('/ap/tf publishers', NOGO, f'{len(tf_pubs)} publishers ({names}): ArduPilot would get interleaved '
                                      'positions -- an orphan from an earlier start?')
    elif expect_one:
        add('/ap/tf publishers', NOGO, 'none: the live node is not running')
    else:
        add('/ap/tf publishers', GO, 'none (no NGPS/VO running)')
    if args.link == 'dds' and node.count_subscribers('/ap/statustext') == 0:
        add('/ap/statustext', WARN, 'no subscriber: ArduPilot has no DDS reader, NGPS/VO warnings will not reach the GCS')

    # Camera clock sync, from the camera node's own diagnostics.
    d = node.camera_diag
    if not d:
        add('camera sync', NOGO, 'no rtsp_camera diagnostics -- is the camera node running?')
    else:
        src, ready = d.get('sync_source', '?'), d.get('sync_ready', 'False') == 'True'
        fallback = int(d.get('frames_arrival_stamped', '0') or 0)
        breaks = d.get('sync_rate_breaks', '0')
        if src in ('rtcp', 'rtp') and ready and fallback == 0:
            add('camera sync', GO, f'source={src} slope={d.get("sync_slope", "?")} '
                                   f'residual={d.get("sync_residual_ms", "?")} ms'
                                   + (f' rate_changes={breaks}' if breaks not in ('0', '') else ''))
        elif src in ('rtcp', 'rtp', 'deciding'):
            add('camera sync', WARN, f'source={src} ready={ready} fallbacks={fallback} '
                                     '-- wait a few seconds and re-run')
        else:
            add('camera sync', WARN, f'source={src}: timestamps carry network jitter')

    # Recorder: running, and actually writing.
    if not recorder_running():
        add('recorder', NOGO, 'no `ros2 bag record` process -- nothing is being saved')
    else:
        bag_after = newest_bag_file(roots)
        if not bag_after:
            add('recorder', NOGO, f'running, but no .mcap under {args.bag_dir}')
        else:
            size_after = os.path.getsize(bag_after)
            grew = (size_after - size_before) if bag_after == bag_before else size_after
            rate = grew / window / 1e6
            status = GO if rate > 0.5 else WARN if rate > 0 else NOGO
            node.measured_mb_s = rate
            add('recorder', status, f'{rate:5.2f} MB/s into {os.path.relpath(bag_after, args.bag_dir) if bag_after.startswith(args.bag_dir) else bag_after}')

    rate = getattr(node, 'measured_mb_s', 0) or 8.5   # MB/s; 8.5 measured in daylight at q85
    free_gb = shutil.disk_usage(args.bag_dir if os.path.isdir(args.bag_dir) else '/').free / 1e9
    add('disk', GO if free_gb >= 10 else WARN if free_gb >= 4 else NOGO,
        f'{free_gb:.1f} GB free  (~{free_gb / (rate * 3.6):.1f} h at {rate:.1f} MB/s)')

    if args.mavlink and args.node_config:
        # VO and NGPS assume a straight-down (nadir) camera
        pitch = gimbal_pitch(args.mavlink)
        if pitch is None:
            add('gimbal', WARN, f'could not read the gimbal attitude over {args.mavlink}: check by eye that it points DOWN')
        elif abs(pitch + 90.0) <= NADIR_TOL_DEG:
            add('gimbal', GO, f'pitch {pitch:.1f} deg: pointing straight down')
        else:
            add('gimbal', WARN, f'pitch {pitch:.1f} deg, NOT pointing down (need -90 +/- {NADIR_TOL_DEG:g}): '
                                'VO/NGPS would track the horizon; point the gimbal down (MNT1_NEUTRAL_Y -90)')

    node.destroy_node()
    rclpy.shutdown()

    width = max(len(r[0]) for r in results)
    for item, status, detail in results:
        mark = {GO: 'ok ', WARN: '!! ', NOGO: 'XX '}[status]
        print(f'  {mark} {item:<{width}}  {status:<5}  {detail}')

    nogo = [r for r in results if r[1] == NOGO]
    warn = [r for r in results if r[1] == WARN]
    print()
    if args.link == 'dds' and not any(node.counts.get(t) for t in FC_DDS_TOPICS):
        print(f'  hint: {DDS_HINT}\n')
    if nogo:
        print(f'  NO-GO  ({len(nogo)} blocking, {len(warn)} warnings)')
        return 1
    if warn:
        print(f'  GO WITH WARNINGS  ({len(warn)})')
        return 2
    print('  GO')
    return 0


if __name__ == '__main__':
    sys.exit(main())
