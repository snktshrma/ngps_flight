#!/usr/bin/env python3
"""Live ORB keyframe VO, corrected by strictly gated NGPS, published to ArduPilot.

This is the pipeline scored offline on flights 1 and 2, run live:
  * VO at 10 Hz: every frame matched to a keyframe, metric from height above takeoff,
    direction from the autopilot heading, measured at the pixel below the aircraft
    (camera tilt compensation).
  * NGPS once a second: map crop at the current estimate; a fix only moves the
    estimate after passing every check and a second confirming attempt, and then by
    a Kalman-weighted amount, never a jump.
  * /ap/tf odom->base_link at a steady rate so ArduPilot's visual odometry stays
    healthy from power-on (it holds the seed position until VO starts).

Frame: ENU metres from the EKF ORIGIN (/ap/gps_global_origin/filtered), which is what
ArduPilot's external navigation expects and does not correct for. The vehicle's own
position comes from /ap/geopose/filtered; /ap/pose/filtered is relative to HOME (it
moves at arming: 15 m and 5.24 m in height on flight 3) and is deliberately not used.
See ap_frame.py and seed_tracker.py.
"""
import json
import math
import threading
import time
from collections import Counter, deque
from typing import Optional

import cv2 as cv
import numpy as np
import rclpy
from geographic_msgs.msg import GeoPointStamped, GeoPoseStamped
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSPresetProfiles, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from tf2_msgs.msg import TFMessage

from ardupilot_msgs.msg import Status

from ap_ngps_ros2 import ap_frame
from ap_ngps_ros2 import camera
from ap_ngps_ros2 import config_check
from ap_ngps_ros2 import statustext as st
from ap_ngps_ros2.seed_tracker import SeedTracker
from ap_ngps_ros2.statustext import StatusTextReporter

VMAX_MPS = 15.0


class KeyframeVO:
    """ORB keyframe visual odometry in ENU metres."""

    def __init__(self, focal_px: float, nadir_px: np.ndarray, vmax: float):
        self.orb = cv.ORB_create(nfeatures=2000, fastThreshold=10)
        self.bf = cv.BFMatcher(cv.NORM_HAMMING)
        self.F, self.P, self.vmax = focal_px, nadir_px, vmax
        self.kf = None
        self.pose: Optional[np.ndarray] = None
        self.last = 0.0
        self.path = 0.0
        self.keyframes = 0

    def _feat(self, img):
        return self.orb.detectAndCompute(cv.cvtColor(img, cv.COLOR_BGR2GRAY), None)

    def _pair(self, a, b):
        (ka, da), (kb, db) = a, b
        if da is None or db is None or len(da) < 10 or len(db) < 10:
            return None, None
        good = [m for m, n in (p for p in self.bf.knnMatch(da, db, k=2) if len(p) == 2)
                if m.distance < 0.75 * n.distance]
        if len(good) < 8:
            return None, None
        return (np.float32([ka[m.queryIdx].pt for m in good]),
                np.float32([kb[m.trainIdx].pt for m in good]))

    def start(self, t, img, xy, alt, heading):
        self.kf = (self._feat(img), np.array(xy, float), alt, heading)
        self.pose = np.array(xy, float)
        self.last = t
        self.keyframes += 1

    def step(self, t, img, alt, heading) -> bool:
        feat = self._feat(img)
        kfeat, kxy, ka, kh = self.kf
        p0, p1 = self._pair(kfeat, feat)
        ok = p0 is not None and len(p0) >= 25
        if ok:
            A, inl = cv.estimateAffinePartial2D(p0, p1, method=cv.RANSAC, ransacReprojThreshold=3.0)
            ok = A is not None and inl.sum() >= 25
        if not ok:
            if t - self.last > 1.0:
                self.start(t, img, self.pose, alt, heading)
            return False
        Ai = cv.invertAffineTransform(A)
        pk = Ai[:, :2] @ self.P + Ai[:, 2]
        sc = max(ka, 0.0) / self.F
        right, fwd = (pk[0] - self.P[0]) * sc, -(pk[1] - self.P[1]) * sc
        ps = math.radians(kh)
        cand = kxy + np.array([right * math.cos(ps) + fwd * math.sin(ps),
                               -right * math.sin(ps) + fwd * math.cos(ps)])
        if np.linalg.norm(cand - self.pose) > self.vmax * (t - self.last) + 1.0:
            return False  # physically impossible jump: keep the last good pose
        self.path += float(np.linalg.norm(cand - self.pose))
        self.pose = cand
        self.last = t
        zoom = math.sqrt(abs(np.linalg.det(A[:, :2])))
        if inl.sum() < 62 or np.linalg.norm(pk - self.P) > 160 or not (0.75 < zoom < 1.33):
            self.start(t, img, self.pose, alt, heading)
        return True


class VoNgpsFusionNode(Node):
    def __init__(self):
        super().__init__('vo_ngps_fusion_node')
        p = self.declare_parameter
        p('camera_topic', '/camera/image_raw/compressed')
        p('geopose_topic', '/ap/geopose/filtered')        # EKF position, attitude, altitude
        p('origin_topic', '/ap/gps_global_origin/filtered')  # the frame /ap/tf must be in
        p('status_topic', '/ap/status')
        p('tf_topic', '/ap/tf')
        p('tf_rate_hz', 10.0)
        p('vo_rate_hz', 10.0)
        p('ngps_period_s', 1.0)
        p('ngps_enable', True)   # false: pure VO, no map matching (bench comparison)
        # Defaults are the measured flight camera; see camera.py (the only copy).
        p('hfov_deg', camera.HFOV_DEG)
        p('camera_yaw_offset_deg', camera.YAW_OFFSET_DEG)
        p('camera_nadir_offset_x_frac', camera.NADIR_OFFSET_X_FRAC)
        p('camera_nadir_offset_y_frac', camera.NADIR_OFFSET_Y_FRAC)
        p('seed_samples', 5)
        p('reference_image_path', '')
        p('metadata_path', '')
        p('mosaic_meters_per_pixel', 0.1203)
        p('tensorrt_engine_path', '')
        p('match_threshold', 0.1)
        # Acceptance checks, as used on flights 1 and 2
        p('min_agree', 25)
        p('max_reproj_px', 2.0)
        p('max_rot_deg', 10.0)
        p('sigma_fix_m', 3.0)
        p('drift_frac', 0.02)
        p('statustext_topic', st.TOPIC_DEFAULT)
        g = lambda n: self.get_parameter(n).value

        self._status = StatusTextReporter(self, topic=g('statustext_topic'))
        self._config_errors = []
        for level, text in config_check.check_params(
                'vo_ngps_fusion_node', {n: q.value for n, q in self.get_parameters_by_prefix('').items()}):
            (self.get_logger().error if level == config_check.ERROR else self.get_logger().warning)(f'config: {text}')
            if level == config_check.ERROR:
                self._config_errors.append(text)
        W, H = 640, 360
        self.F = (W / 2) / math.tan(math.radians(g('hfov_deg') / 2))
        self.P = np.array([W * (0.5 + g('camera_nadir_offset_x_frac')), H * (0.5 + g('camera_nadir_offset_y_frac'))])
        self.vo = KeyframeVO(self.F, self.P, vmax=VMAX_MPS)

        # map
        self.map = cv.imread(g('reference_image_path'))
        b = json.load(open(g('metadata_path')))['bounds']
        self.bounds = list(map(float, b.split(','))) if isinstance(b, str) else list(map(float, b))
        self.mpp = float(g('mosaic_meters_per_pixel'))
        from ap_ngps_ros2.trt_matcher import TrtMatcher
        self.trt = TrtMatcher(engine_path=g('tensorrt_engine_path'), match_threshold=float(g('match_threshold')), warmup=True)

        # state. Arming, seeding and the EKF-origin frame: see seed_tracker.py.
        self.track = SeedTracker(seed_samples=int(g('seed_samples')))
        self.alt = 0.0              # height above the takeoff point (VO scale, map crop)
        self.yaw_enu = None
        self.frames = 0
        self.t_start = time.monotonic()
        self.offset = np.zeros(2)   # map -> odom correction
        self.Pvar = float(g('sigma_fix_m')) ** 2
        self.d0 = 0.0
        self.cand = None
        self.next_vo = 0.0
        self.next_ngps = 0.0
        self.accepted = 0
        self.attempts = 0
        self.last_tf_s = 0.0
        self.tf_gap_max = 0.0
        self.tf_lock = threading.Lock()
        self.fuse_lock = threading.Lock()   # offset / candidate: NGPS worker vs frame callback
        self.ngps_job = None                # latest frame handed to the NGPS worker
        self.decode_flag = cv.IMREAD_COLOR  # switches to a half-size JPEG decode for 1280x720
        self.lat = {k: deque(maxlen=200) for k in ('age_rx', 'decode', 'vo', 'age_pub', 'ngps')}
        self.ngps_why = Counter()   # outcome of each map-match attempt, logged with the latency
        # Frame handling: received, run through VO, skipped by the rate gate, and frames that
        # never reached the callback (overwritten in the depth-1 queue while it was busy).
        self.fcount = Counter()
        self.last_stamp = None

        qos = QoSPresetProfiles.SENSOR_DATA.value
        # The frame pipeline (ORB + TensorRT, up to a few hundred ms on an NGPS frame)
        # and the pose/status inputs share the node's default mutually exclusive group,
        # so a frame shift is never applied half-way through a VO step. The /ap/tf
        # keep-alive has a group of its own, and so does NGPS map matching (TensorRT,
        # ~100+ ms once a second), which works on a snapshot of the frame and VO pose.
        # main() spins three threads: neither can delay a VO frame or the keep-alive.
        # (Flight 3, single-threaded: /ap/tf averaged 7.6 Hz with 88 gaps over 300 ms.)
        self.tf_group = MutuallyExclusiveCallbackGroup()
        self.ngps_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Status, g('status_topic'), self._status_cb, qos)
        self.create_subscription(GeoPointStamped, g('origin_topic'), self._origin_cb, qos)
        self.create_subscription(GeoPoseStamped, g('geopose_topic'), self._geopose_cb, qos)
        # Depth 1: only ever the newest frame. SENSOR_DATA's depth 5 let frames queue
        # behind a slow callback, and VO then worked on one ~100-170 ms old.
        latest_only = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(CompressedImage, g('camera_topic'), self._image_cb, latest_only)
        self.tf_pub = self.create_publisher(TFMessage, g('tf_topic'), qos)
        self.tf_period = 1.0 / max(1.0, float(g('tf_rate_hz')))
        self.create_timer(self.tf_period, self._tf_keepalive, callback_group=self.tf_group)
        self.create_timer(0.02, self._ngps_worker, callback_group=self.ngps_group)
        self.create_timer(1.0, self._health_tick)
        self.get_logger().info('VO+NGPS fusion ready; waiting for EKF origin and position')
        self._status.send('VO+NGPS started', st.INFO)

    # ---------------- inputs ----------------
    def _status_cb(self, msg: Status):
        was = self.track.armed_since
        self.track.on_status(bool(msg.armed), time.monotonic())
        if was is None and self.track.armed_since is not None:
            self.get_logger().info('armed: seeding from the EKF position')

    def _shift_frame(self, delta: np.ndarray):
        """The EKF origin moved (an EKF reset): keep everything in ArduPilot's new frame."""
        if self.vo.pose is not None:
            self.vo.pose = self.vo.pose + delta
        if self.vo.kf is not None:
            f, kxy, ka, kh = self.vo.kf
            self.vo.kf = (f, kxy + delta, ka, kh)
        with self.fuse_lock:
            if self.cand is not None:
                self.cand = (self.cand[0], self.cand[1] + delta, self.cand[2] + delta)
        self.get_logger().warning(f'EKF origin moved: shifted frame by E {delta[0]:+.2f} N {delta[1]:+.2f} m')
        self._status.send(f'EKF origin moved {np.linalg.norm(delta):.1f} m, frame followed', st.WARNING,
                          key='originmove', repeat_suppress_s=5.0)

    def _origin_cb(self, msg: GeoPointStamped):
        had = self.track.origin is not None
        moved = self.track.on_origin(msg.position.latitude, msg.position.longitude, msg.position.altitude)
        if not had and self.track.origin is not None:
            la, lo, al = self.track.origin
            self.get_logger().info(f'EKF origin {la:.7f}, {lo:.7f}, {al:.2f} m')
        if moved is not None:
            self._shift_frame(moved[0])

    def _geopose_cb(self, msg: GeoPoseStamped):
        # The EKF's own location (origin + its position estimate) and attitude. Home,
        # and so /ap/pose/filtered, never enters the frame.
        self.yaw_enu = ap_frame.yaw_enu_deg(msg.pose.orientation)
        pos = msg.pose.position
        self.track.on_geopose(pos.latitude, pos.longitude, pos.altitude, time.monotonic())
        self.alt = self.track.height()

    def _vehicle_heading(self) -> float:
        """Compass heading of the vehicle's nose (what /ap/tf's orientation carries)."""
        return (90.0 - self.yaw_enu) % 360.0

    def _heading(self) -> float:
        """Compass heading of the image's up direction (VO and map-matching maths only)."""
        return (90.0 - self.yaw_enu + float(self.get_parameter('camera_yaw_offset_deg').value)) % 360.0

    # ---------------- NGPS ----------------
    def _enu_to_ll(self, xy):
        return ap_frame.en_to_ll(self.track.origin, xy[0], xy[1])

    def _ll_to_enu(self, la, lo):
        return np.array(ap_frame.ll_to_en(self.track.origin, la, lo))

    def _ngps_candidate(self, frame, centre, heading, alt):
        img = self.map
        minlon, minlat, maxlon, maxlat = self.bounds
        H, W = img.shape[:2]
        la, lo = self._enu_to_ll(centre)
        row = (maxlat - la) / (maxlat - minlat) * H
        col = (lo - minlon) / (maxlon - minlon) * W
        hf = float(self.get_parameter('hfov_deg').value)
        k = int(2 * alt * math.tan(math.radians(hf / 2)) / self.mpp)
        kh = int(k * 360 / 640)
        pad = int(math.hypot(k, kh) / 2) + 4
        if k < 32:
            return None, 'too low'
        r0, c0 = int(row) - pad, int(col) - pad
        if r0 < 0 or c0 < 0 or r0 + 2 * pad > H or c0 + 2 * pad > W:
            return None, 'outside map'
        Rm = cv.getRotationMatrix2D((pad, pad), heading, 1.0)
        crop = cv.warpAffine(img[r0:r0 + 2 * pad, c0:c0 + 2 * pad], Rm, (2 * pad, 2 * pad))
        crop = cv.resize(crop[pad - kh // 2:pad - kh // 2 + kh, pad - k // 2:pad - k // 2 + k], (640, 360),
                         interpolation=cv.INTER_CUBIC if k < 640 else cv.INTER_AREA)
        k0, k1, _ = self.trt.match(crop, frame)
        k0, k1 = np.float32(k0), np.float32(k1)
        min_agree = int(self.get_parameter('min_agree').value)
        if len(k0) < min_agree:
            return None, 'few matches'
        Hm, mask = cv.findHomography(k1, k0, cv.RANSAC, 3.0)
        if Hm is None:
            return None, 'few agreeing'
        inl = mask.ravel().astype(bool)
        if inl.sum() < min_agree:
            return None, 'few agreeing'
        rep = np.linalg.norm(cv.perspectiveTransform(k1[inl].reshape(-1, 1, 2), Hm).reshape(-1, 2) - k0[inl], axis=1)
        if np.median(rep) > float(self.get_parameter('max_reproj_px').value):
            return None, 'poor fit'
        A = Hm[:2, :2] / Hm[2, 2]
        sc = math.sqrt(abs(np.linalg.det(A)))
        rot = math.degrees(math.atan2(A[1, 0], A[0, 0]))
        persp = max(abs(Hm[2, 0]), abs(Hm[2, 1])) / abs(Hm[2, 2])
        if not (0.8 < sc < 1.25) or abs(rot) > float(self.get_parameter('max_rot_deg').value) or persp > 1e-3:
            return None, 'implausible geometry'
        cen = cv.perspectiveTransform(np.float32([[self.P]]), Hm)[0][0]
        kx, ky = cen[0] * k / 640 + pad - k / 2, cen[1] * kh / 360 + pad - kh / 2
        px, py = cv.invertAffineTransform(Rm) @ np.array([kx, ky, 1.0])
        mr, mc = r0 + py, c0 + px
        return self._ll_to_enu(maxlat - mr / H * (maxlat - minlat), minlon + mc / W * (maxlon - minlon)), 'ok'

    def _ngps_worker(self):
        """NGPS thread: match the latest handed-over frame, off the VO path."""
        job, self.ngps_job = self.ngps_job, None
        if job is None:
            return
        t0 = time.monotonic()
        try:
            self._ngps_attempt(*job)
        except Exception as exc:
            self.get_logger().warning(f'ngps: {exc}')
            self._status.send(f'NGPS error: {type(exc).__name__}', st.WARNING, key='ngpsexc', repeat_suppress_s=120.0)
        self.lat['ngps'].append(time.monotonic() - t0)

    def _ngps_attempt(self, t, frame, vo_pose, vo_path, heading, alt, jumps):
        """One map match. vo_pose/vo_path/heading/alt are the VO state at this frame."""
        self.attempts += 1
        sig_fix = float(self.get_parameter('sigma_fix_m').value)
        drift = float(self.get_parameter('drift_frac').value)
        pred = vo_pose + self.offset
        fix, why = self._ngps_candidate(frame, pred, heading, alt)
        self.ngps_why[why] += 1
        if fix is None:
            return
        with self.fuse_lock:
            if self.track.origin_moves != jumps:
                self.cand = None   # the frame moved while matching: this fix is in the old one
                return
            sp = math.sqrt(self.Pvar + (drift * (vo_path - self.d0)) ** 2)
            if np.linalg.norm(fix - pred) > 3 * math.hypot(sp, sig_fix):
                self.cand = None
                return
            if self.cand is None or t - self.cand[0] > 2.5:
                self.cand = (t, fix, vo_pose.copy())
                return
            if np.linalg.norm((fix - self.cand[1]) - (vo_pose - self.cand[2])) > 3.0:
                self.cand = (t, fix, vo_pose.copy())
                return
            z = 0.5 * (fix + self.cand[1] + (vo_pose - self.cand[2]))
            R = sig_fix ** 2 / 2
            Pp = sp ** 2
            K = Pp / (Pp + R)
            shift = K * (z - pred)
            self.offset = self.offset + shift
            self.Pvar = (1 - K) * Pp
            self.d0 = vo_path
            self.cand = None
            self.accepted += 1
        if self.accepted == 1:
            self._status.send(f'first NGPS correction at {self.alt:.0f}m', st.INFO)
        self.get_logger().info(
            f'NGPS correction #{self.accepted} at {self.alt:.0f} m: moved {np.linalg.norm(shift):.2f} m '
            f'(gain {K:.2f}, {self.attempts} attempts so far)')

    # ---------------- camera ----------------
    def _image_cb(self, msg: CompressedImage):
        try:
            t = time.monotonic()
            wall = time.time()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.frames += 1
            self.fcount['rx'] += 1
            if self.last_stamp is not None and stamp - self.last_stamp > 0.05:   # > 1.5 frames at 30 fps
                self.fcount['missed'] += int(round((stamp - self.last_stamp) * 30.0)) - 1
            self.last_stamp = stamp
            if self.yaw_enu is None:
                return
            if not self.track.seeded:
                if not self.track.try_seed():
                    return
                xy = self.track.seed_xy
                self.get_logger().info(f'seeded at E {xy[0]:.2f} N {xy[1]:.2f}, heading {self._heading():.1f}')
                self._status.send(f'VO seeded, heading {self._heading():.0f} deg', st.INFO)
            # 20 ms early is close enough: a 30 fps camera otherwise turns 10 Hz into 7.5.
            if t < self.next_vo - 0.02:
                self.fcount['rate_skip'] += 1
                return
            self.fcount['vo'] += 1
            self.next_vo = t + 1.0 / float(self.get_parameter('vo_rate_hz').value)
            self.lat['age_rx'].append(wall - stamp)
            img = cv.imdecode(np.frombuffer(msg.data, np.uint8), self.decode_flag)
            if self.decode_flag == cv.IMREAD_COLOR and img.shape[1] >= 1280:
                self.decode_flag = cv.IMREAD_REDUCED_COLOR_2   # JPEG-domain half size: ~2x faster
            if img.shape[:2] != (360, 640):
                img = cv.resize(img, (640, 360), interpolation=cv.INTER_AREA)
            t1 = time.monotonic()
            self.lat['decode'].append(t1 - t)
            heading = self._heading()
            if self.vo.kf is None:
                self.vo.start(t, img, self.track.seed_xy, self.alt, heading)
                return
            self.vo.step(t, img, self.alt, heading)
            # Publish this frame's pose now; map matching happens on another thread.
            self._publish_tf()
            self.lat['vo'].append(time.monotonic() - t1)
            self.lat['age_pub'].append(time.time() - stamp)
            if self.get_parameter('ngps_enable').value and t >= self.next_ngps and self.ngps_job is None:
                self.next_ngps = t + float(self.get_parameter('ngps_period_s').value)
                self.ngps_job = (t, img, self.vo.pose.copy(), self.vo.path, heading, self.alt, self.track.origin_moves)
        except Exception as exc:
            self.get_logger().warning(f'frame: {exc}')
            self._status.send(f'fusion error: {type(exc).__name__}', st.WARNING, key='exc', repeat_suppress_s=120.0)

    # ---------------- output ----------------
    def _publish_tf(self):
        # Called from the frame callback and from the keep-alive thread.
        with self.tf_lock:
            self._publish_tf_locked()

    def _publish_tf_locked(self):
        # Nothing rather than a guess: ArduPilot aligns its VisOdom yaw to the first
        # sample it gets and never re-offsets position, so a (0,0) or a zero heading
        # here would stick for the whole flight. VisOdom stays unhealthy (no arming)
        # until the EKF has an origin, a position and an attitude.
        if self.yaw_enu is None or not self.track.output_ready():
            return
        east, north = self.track.output_en(None if self.vo.pose is None else self.vo.pose + self.offset)
        # The vehicle's heading, not the camera's: camera_yaw_offset_deg is image-to-map
        # maths only (it was adding 1.3 deg to every yaw ArduPilot got).
        heading = self._vehicle_heading()
        # ArduPilot turns this into NED by negating y and z only
        # (AP_DDS_External_Odom::convert_transform): send x=north, y=west, z=up.
        yaw = math.radians(((-heading) + 180.0) % 360.0 - 180.0)
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'odom'
        ts.child_frame_id = 'base_link'
        ts.transform.translation.x = float(north)
        ts.transform.translation.y = float(-east)
        # Height above the ORIGIN, from the EKF (VO is horizontal only).
        ts.transform.translation.z = float(self.track.up)
        ts.transform.rotation.z = math.sin(yaw / 2)
        ts.transform.rotation.w = math.cos(yaw / 2)
        self.tf_pub.publish(TFMessage(transforms=[ts]))
        now = time.monotonic()
        if self.last_tf_s:
            self.tf_gap_max = max(self.tf_gap_max, now - self.last_tf_s)
        self.last_tf_s = now

    def _tf_keepalive(self):
        try:
            if time.monotonic() - self.last_tf_s >= self.tf_period * 0.9:
                self._publish_tf()
        except Exception as exc:  # the keep-alive must outlive any one bad sample
            self.get_logger().warning(f'tf keep-alive: {exc}')

    def _log_latency(self):
        def ms(k):
            v = self.lat[k]
            return f'{np.median(v) * 1e3:4.0f}/{max(v) * 1e3:4.0f}' if v else '   -/   -'
        self.get_logger().info(
            f'latency p50/max ms: frame age at receive {ms("age_rx")}, decode {ms("decode")}, '
            f'VO {ms("vo")}, frame age at /ap/tf {ms("age_pub")}, NGPS {ms("ngps")} (off the VO path); '
            f'NGPS outcomes {dict(self.ngps_why)}, accepted {self.accepted}')
        a = np.array(self.lat['age_rx']) if self.lat['age_rx'] else None
        if a is not None:
            self.get_logger().info(
                f'frames {dict(self.fcount)}; frame age at receive p5/p50/p95/max '
                f'{" / ".join(f"{np.percentile(a, q) * 1e3:.0f}" for q in (5, 50, 95, 100))} ms')

    def _health_tick(self):
        """1 Hz: say on the GCS what is wrong, so nothing fails silently before takeoff."""
        try:
            now = time.monotonic()
            for i, text in enumerate(self._config_errors):
                self._status.send(f'CONFIG: {text}', st.ERROR, key=f'cfg{i}', repeat_suppress_s=60.0)
            gap, self.tf_gap_max = self.tf_gap_max, 0.0
            topic = self.tf_pub.topic_name   # resolved, so a remap is honoured
            others = sorted({i.node_name for i in self.get_publishers_info_by_topic(topic)
                             if i.node_name != self.get_name()})
            if others:
                self._status.send(f'{topic} ALSO published by {",".join(others)}: kill it', st.ERROR,
                                  key='tfothers', repeat_suppress_s=20.0)
            if int(now - self.t_start) % 10 == 0:
                self._log_latency()
            if gap > 0.25:
                self._status.send(f'/ap/tf stalled {gap * 1000:.0f} ms (VisOdom times out at 300)', st.WARNING,
                                  key='tfgap', repeat_suppress_s=30.0)
            why = self.track.waiting_for(now, self.frames)
            if why is not None:
                if self.track.armed_for(now) > 5.0:
                    self._status.send(f'armed but VO NOT seeded: {why}', st.WARNING, key='noseed',
                                      repeat_suppress_s=10.0)
                elif now - self.t_start > 15.0 and why not in ('not armed', 'collecting samples',
                                                               'waiting for a camera frame'):
                    self._status.send(f'VO cannot seed: {why}', st.WARNING, key='noseed', repeat_suppress_s=60.0)
            self._status.spin_once()
        except Exception as exc:  # a diagnostic channel must never take the node down
            self.get_logger().debug(f'health tick: {exc}')


def main():
    rclpy.init()
    node = VoNgpsFusionNode()
    executor = MultiThreadedExecutor(num_threads=3)   # frames, NGPS matching, /ap/tf keep-alive
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
