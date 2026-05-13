#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CompressedImage, Image, NavSatFix, NavSatStatus
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64
from builtin_interfaces.msg import Time as BuiltinTime
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import torch
from PIL import Image as PILImage
from lightglue import LightGlue, SuperPoint
from lightglue.utils import load_image_arr, rbd
import gc
import json
import math
import os
import time
from pathlib import Path
from typing import Optional, Tuple
from collections import deque

from ap_ngps_ros2 import transformations as tf_


class NGPSLocalizationNode(Node):
    
    def __init__(self):
        super().__init__('ngps_localization_node')
        
        self.bridge = CvBridge()
        
        self.tot_rot = 0.0
        self._pending_kernel_rot_step_deg = 0.0
        self.base_x = 0
        self.base_y = 0
        self.x = 0
        self.y = 0
        self.theta_deg = 0.0
        self.frame_count = 0

        self.rot_hist = []
        self.rot_std_thresh = 15.0
        self.max_rot_change = 30.0
        self.last_valid_rot = 0.0
        self._anchor_origin_rowcol: Optional[Tuple[float, float]] = None
        self._auto_georef_bounds: Optional[Tuple[float, float, float, float]] = None
        self._source_mosaic_w = 0
        self._source_mosaic_h = 0
        self._pixel_utm_map = None
        self._vps_origin_en = None
        self._local_metric_utm = None
        self._first_vps_odom_published = False
        self._last_num_matches = 0
        self._last_mean_score = 0.0
        self._last_inlier_ratio = 0.0
        self._first_fix_gate_log_t = -1e9
        self._altitude_m = 0.0
        self._prev_kernel_xy: Optional[Tuple[float, float]] = None

        self.declare_parameter('reference_image_path', '')
        self.declare_parameter('reference_max_edge', 4096)
        self.declare_parameter('kernel_size', 300)
        self.declare_parameter('kernel_max_size', 3000)
        self.declare_parameter('kernel_grow_fill_ratio', 0.85)
        self.declare_parameter('kernel_grow_factor', 1.25)
        self.declare_parameter('match_threshold', 0.5)
        self.declare_parameter('min_matches', 20)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_compressed', False)
        self.declare_parameter('initial_base_x', 0)
        self.declare_parameter('initial_base_y', 0)
        self.declare_parameter('max_rotation_change', 30.0)
        self.declare_parameter('rotation_std_threshold', 15.0)
        self.declare_parameter('max_rotation_history', 10)
        self.declare_parameter('enable_rotation_smoothing', True)
        self.declare_parameter('enable_rotation_validation', True)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('reference_min_lon', 0.0)
        self.declare_parameter('reference_min_lat', 0.0)
        self.declare_parameter('reference_max_lon', 0.0)
        self.declare_parameter('reference_max_lat', 0.0)
        self.declare_parameter('reference_altitude', 0.0)
        self.declare_parameter('altitude_topic', '/ap/geopose/filtered')
        self.declare_parameter('altitude_source', 'geopose')
        self.declare_parameter('default_altitude_m', 10.0)
        self.declare_parameter('mosaic_meters_per_pixel', 0.0)
        self.declare_parameter('mosaic_meters_per_pixel_source_full_res', True)
        self.declare_parameter('enable_global_coordinates', False)
        self.declare_parameter('pixel_utm_map_path', '')
        self.declare_parameter('first_fix_min_matches', 45)
        self.declare_parameter('first_fix_min_mean_score', 0.48)
        self.declare_parameter('first_fix_min_inlier_ratio', 0.35)
        self.declare_parameter(
            'min_frame_inlier_ratio',
            0.0,
        )
        self.declare_parameter(
            'max_kernel_translation_step_px',
            0.0,
        )
        self.declare_parameter('reject_degenerate_homography', True)
        self.declare_parameter('kernel_rotation_use_rect_tilt', True)
        self.declare_parameter('kernel_rotation_k', 0.35)
        self.declare_parameter('kernel_rotation_max_step_deg', 3.0)
        self.declare_parameter('kernel_rotation_deadband_deg', 0.5)
        self.declare_parameter('kernel_rotation_rect_sign', 1.0)
        self.declare_parameter('vps_pose_variance_base_m2', 4.0)
        self.declare_parameter('max_keypoints', 1024)
        self.declare_parameter('enable_gpu', True)
        self.declare_parameter('publish_match_pair', True)
        self.declare_parameter('debug_match_pair_topic', 'ngps/match_pair')
        self.declare_parameter('publish_ap_dds_tf', False)
        self.declare_parameter('ap_dds_tf_topic', '/ap/tf')
        self.declare_parameter('dds_tf_use_ap_time', True)
        self.declare_parameter('dds_tf_use_unix_wall_time', False)
        self.declare_parameter('ap_time_topic', '/ap/time')

        max_keypoints = int(self.get_parameter('max_keypoints').get_parameter_value().integer_value)
        enable_gpu = self.get_parameter('enable_gpu').get_parameter_value().bool_value
        if enable_gpu and torch.cuda.is_available():
            self.device = torch.device('cuda')
        else:
            if enable_gpu:
                self.get_logger().warn('enable_gpu is true but CUDA is unavailable; using CPU')
            self.device = torch.device('cpu')
        self.get_logger().info(f'LightGlue device: {self.device}')
        self.extractor = SuperPoint(max_num_keypoints=max_keypoints).eval().to(self.device)
        self.matcher = LightGlue(features='superpoint').eval().to(self.device)

        self.load_reference_image()
        mosaic_scale = float(getattr(self, '_mosaic_load_scale', 1.0))
        base_kernel = int(self.get_parameter('kernel_size').get_parameter_value().integer_value)
        max_kernel = int(self.get_parameter('kernel_max_size').get_parameter_value().integer_value)
        self._active_kernel_size = max(1, int(round(base_kernel * mosaic_scale)))
        self._kernel_max_size_active = max(
            self._active_kernel_size,
            int(round(max_kernel * mosaic_scale)),
        )
        init_row = int(self.get_parameter('initial_base_x').get_parameter_value().integer_value)
        init_col = int(self.get_parameter('initial_base_y').get_parameter_value().integer_value)
        if mosaic_scale != 1.0 and init_row > 0 and init_col > 0:
            init_row = int(round(init_row * mosaic_scale))
            init_col = int(round(init_col * mosaic_scale))
        if init_row > 0 and init_col > 0:
            self.base_x = init_row
            self.base_y = init_col
        else:
            self.base_x = self.h // 2
            self.base_y = self.w // 2

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        camera_compressed = (
            self.get_parameter('camera_compressed').get_parameter_value().bool_value
        )

        self._publish_match_pair = self.get_parameter(
            'publish_match_pair'
        ).get_parameter_value().bool_value
        match_pair_topic = (
            self.get_parameter('debug_match_pair_topic').get_parameter_value().string_value
        )

        self.pose_pub = self.create_publisher(PoseStamped, 'ngps/pose', 10)
        self.position_pub = self.create_publisher(PointStamped, 'ngps/position', 10)
        self.rotation_pub = self.create_publisher(Float64, 'ngps/rotation', 10)
        self.match_pair_pub = None
        if self._publish_match_pair:
            self.match_pair_pub = self.create_publisher(Image, match_pair_topic, 10)
        self.global_position_pub = self.create_publisher(NavSatFix, 'ngps/global_position', 10)
        self.ecef_position_pub = self.create_publisher(PointStamped, 'ngps/ecef_position', 10)
        self.odom_vps_pub = self.create_publisher(Odometry, 'odometry/vps', 10)

        self._ap_dds_tf_pub = None
        self._dds_tf_hist_x = None
        self._dds_tf_use_ap_time = False
        self._dds_tf_use_unix_wall_time = False
        self._ap_time_stamp_for_tf: Optional[BuiltinTime] = None
        if self.get_parameter('publish_ap_dds_tf').get_parameter_value().bool_value:
            ap_tf_topic = self.get_parameter('ap_dds_tf_topic').get_parameter_value().string_value
            self._ap_dds_tf_pub = self.create_publisher(
                TFMessage, ap_tf_topic, QoSPresetProfiles.SENSOR_DATA.value
            )
            self._dds_tf_hist_x = deque(maxlen=5)
            self._dds_tf_hist_y = deque(maxlen=5)
            self._dds_tf_hist_z = deque(maxlen=5)
            self._dds_tf_hist_yaw_deg = deque(maxlen=5)
            self._dds_tf_use_unix_wall_time = (
                self.get_parameter('dds_tf_use_unix_wall_time').get_parameter_value().bool_value
            )
            self._dds_tf_use_ap_time = (
                self.get_parameter('dds_tf_use_ap_time').get_parameter_value().bool_value
            )
            if self._dds_tf_use_unix_wall_time and self._dds_tf_use_ap_time:
                self.get_logger().warn(
                    'dds_tf_use_unix_wall_time is true: ignoring dds_tf_use_ap_time '
                    '(/ap/time not used for /ap/tf stamp)'
                )
            if self._dds_tf_use_ap_time and not self._dds_tf_use_unix_wall_time:
                ap_time_topic = (
                    self.get_parameter('ap_time_topic').get_parameter_value().string_value
                )
                self.create_subscription(
                    BuiltinTime,
                    ap_time_topic,
                    self._ap_time_for_tf_callback,
                    QoSPresetProfiles.SENSOR_DATA.value,
                )
                self.get_logger().info(
                    f'ArduPilot DDS: TF header stamp from {ap_time_topic} when available '
                    '(else ROS node clock)'
                )
            elif self._dds_tf_use_unix_wall_time:
                self.get_logger().info(
                    'ArduPilot DDS: TF header stamp from host Unix wall clock (time.time_ns)'
                )
            self.get_logger().info(
                f'ArduPilot DDS: publishing {ap_tf_topic} as tf2_msgs/TFMessage '
                '(odom to base_link, median window 5; requires AP visual odometry / DDS enabled)'
            )

        image_qos = QoSPresetProfiles.SENSOR_DATA.value
        if camera_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage,
                camera_topic,
                self.compressed_image_callback,
                image_qos,
            )
        else:
            self.image_sub = self.create_subscription(
                Image,
                camera_topic,
                self.image_callback,
                image_qos,
            )

        default_altitude_m = self.get_parameter('default_altitude_m').get_parameter_value().double_value
        reference_altitude = self.get_parameter('reference_altitude').get_parameter_value().double_value
        self._altitude_m = float(default_altitude_m if default_altitude_m > 0.0 else reference_altitude)
        self._create_altitude_subscription()

        self.get_logger().info('NGPS localization node ready')

    def _create_altitude_subscription(self) -> None:
        alt_topic = self.get_parameter('altitude_topic').get_parameter_value().string_value
        if not alt_topic:
            self.get_logger().info(
                f'altitude_topic unset; using default altitude {self._altitude_m:.2f} m'
            )
            return

        source = self.get_parameter('altitude_source').get_parameter_value().string_value.strip().lower()
        qos = QoSPresetProfiles.SENSOR_DATA.value
        if source == 'float64':
            self.create_subscription(Float64, alt_topic, self._alt_float_cb, qos)
        elif source == 'geopose':
            self.create_subscription(GeoPoseStamped, alt_topic, self._alt_geopose_cb, qos)
        elif source == 'pose':
            self.create_subscription(PoseStamped, alt_topic, self._alt_pose_cb, qos)
        elif source == 'navsat':
            self.create_subscription(NavSatFix, alt_topic, self._alt_navsat_cb, qos)
        else:
            raise RuntimeError(
                f'unsupported altitude_source={source!r} '
                '(expected float64, geopose, pose, or navsat)'
            )
        self.get_logger().info(f'altitude from {alt_topic} ({source})')

    def _set_altitude_m(self, altitude_m: float) -> None:
        alt = max(0.0, float(altitude_m))
        if alt <= 0.0:
            return
        self._altitude_m = alt

    def _alt_float_cb(self, msg: Float64) -> None:
        self._set_altitude_m(msg.data)

    def _alt_geopose_cb(self, msg: GeoPoseStamped) -> None:
        self._set_altitude_m(msg.pose.position.altitude)

    def _alt_pose_cb(self, msg: PoseStamped) -> None:
        self._set_altitude_m(abs(msg.pose.position.z))

    def _alt_navsat_cb(self, msg: NavSatFix) -> None:
        self._set_altitude_m(msg.altitude)

    def _published_altitude_m(self) -> float:
        return float(self._altitude_m)
    
    def _downsample_mosaic(self, mosaic_bgr: np.ndarray) -> np.ndarray:
        max_edge = int(self.get_parameter('reference_max_edge').get_parameter_value().integer_value)
        if max_edge <= 0:
            self._mosaic_load_scale = 1.0
            return mosaic_bgr
        h0, w0 = mosaic_bgr.shape[:2]
        long_edge = max(h0, w0)
        if long_edge <= max_edge:
            self._mosaic_load_scale = 1.0
            return mosaic_bgr
        scale = max_edge / float(long_edge)
        new_w = max(1, int(round(w0 * scale)))
        new_h = max(1, int(round(h0 * scale)))
        self._mosaic_load_scale = scale
        return cv.resize(mosaic_bgr, (new_w, new_h), interpolation=cv.INTER_AREA)

    def _reference_image_candidates(self) -> list[str]:
        candidates: list[str] = []
        try:
            from ament_index_python.packages import get_package_share_directory

            candidates.append(
                os.path.join(
                    get_package_share_directory('ap_ngps_ros2'),
                    'ngps_config',
                    'tiff',
                    'osrf0',
                    'osrf.tif',
                )
            )
        except Exception:
            pass
        candidates.append(
            str(
                Path(__file__).resolve().parents[3]
                / 'ngps_config'
                / 'tiff'
                / 'osrf0'
                / 'osrf.tif'
            )
        )
        return candidates

    def _resolve_reference_image_path(self, configured: str) -> str:
        raw = (configured or '').strip()
        expanded = os.path.expanduser(os.path.expandvars(raw)) if raw else ''
        if expanded and os.path.exists(expanded):
            return expanded
        for candidate in self._reference_image_candidates():
            if os.path.exists(candidate):
                return candidate
        return expanded or raw

    def load_reference_image(self):
        configured = self.get_parameter('reference_image_path').get_parameter_value().string_value
        ref_img_path = self._resolve_reference_image_path(configured)
        self._mosaic_load_scale = 1.0

        if not ref_img_path or not os.path.exists(ref_img_path):
            self.get_logger().error(
                'reference_image_path is missing or not found; mosaic will be blank. '
                'Set reference_image_path to the OSRF GeoTIFF (e.g. osrf.tif).'
            )
            self.mosaic_bgr = np.zeros((1440, 1024, 3), dtype=np.uint8)
        else:
            try:
                PILImage.MAX_IMAGE_PIXELS = None
                arr = np.array(PILImage.open(ref_img_path))
                if arr.ndim == 2:
                    self.mosaic_bgr = cv.cvtColor(arr, cv.COLOR_GRAY2BGR)
                else:
                    self.mosaic_bgr = cv.cvtColor(arr[:, :, :3], cv.COLOR_RGB2BGR)
                src_h, src_w = self.mosaic_bgr.shape[:2]
                self._source_mosaic_w = int(src_w)
                self._source_mosaic_h = int(src_h)
                self.mosaic_bgr = self._downsample_mosaic(self.mosaic_bgr)
                self._try_load_bounds_from_metadata(ref_img_path)
                self.get_logger().info(
                    f'Loaded reference mosaic {ref_img_path} '
                    f'source={src_w}x{src_h} working={self.mosaic_bgr.shape[1]}x{self.mosaic_bgr.shape[0]} '
                    f'scale={self._mosaic_load_scale:.4f}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to load reference image {ref_img_path}: {e}')
                self.mosaic_bgr = np.zeros((1440, 1024, 3), dtype=np.uint8)

        self.h, self.w, self.c = self.mosaic_bgr.shape
        self._try_load_pixel_utm_map()

    def _try_load_bounds_from_metadata(self, ref_img_path: str) -> None:
        meta_path = Path(ref_img_path).with_name('metadata.json')
        if not meta_path.is_file():
            return
        try:
            payload = json.loads(meta_path.read_text(encoding='utf-8'))
            bounds = payload.get('bounds')
            if not isinstance(bounds, str):
                return
            parts = [float(x.strip()) for x in bounds.split(',')]
            if len(parts) != 4:
                return
            self._auto_georef_bounds = (parts[0], parts[1], parts[2], parts[3])
            self.get_logger().info(
                f'Loaded reference bounds from {meta_path}: '
                f'west={parts[0]:.8f} south={parts[1]:.8f} '
                f'east={parts[2]:.8f} north={parts[3]:.8f}'
            )
        except Exception as exc:
            self.get_logger().warn(f'Failed to parse {meta_path}: {exc}')

    def _georef_bounds(self) -> Optional[Tuple[float, float, float, float]]:
        min_lon = self.get_parameter('reference_min_lon').get_parameter_value().double_value
        min_lat = self.get_parameter('reference_min_lat').get_parameter_value().double_value
        max_lon = self.get_parameter('reference_max_lon').get_parameter_value().double_value
        max_lat = self.get_parameter('reference_max_lat').get_parameter_value().double_value
        if not (min_lon == 0.0 and min_lat == 0.0 and max_lon == 0.0 and max_lat == 0.0):
            return min_lon, min_lat, max_lon, max_lat
        return self._auto_georef_bounds

    def _reference_bbox_configured(self) -> bool:
        bounds = self._georef_bounds()
        if bounds is None:
            return False
        min_lon, min_lat, max_lon, max_lat = bounds
        return min_lon < max_lon and min_lat < max_lat

    def _working_meters_per_pixel(self) -> Optional[float]:
        mpp = self.get_parameter('mosaic_meters_per_pixel').get_parameter_value().double_value
        if mpp <= 0.0:
            return None
        if self.get_parameter('mosaic_meters_per_pixel_source_full_res').get_parameter_value().bool_value:
            return mpp * float(getattr(self, '_mosaic_load_scale', 1.0))
        return mpp

    def _publish_anchor_rowcol(self) -> Tuple[float, float]:
        row, col = self.map_kernel_img(self.y, self.x, self.base_x, self.base_y)
        return float(row), float(col)

    def _try_load_pixel_utm_map(self):
        path = self.get_parameter('pixel_utm_map_path').get_parameter_value().string_value
        if not path or not os.path.exists(path):
            return
        try:
            arr = np.load(path)
            if arr.shape[0] != self.h or arr.shape[1] != self.w:
                if arr.shape[0] > 0 and arr.shape[1] > 0:
                    arr = cv.resize(
                        arr,
                        (self.w, self.h),
                        interpolation=cv.INTER_AREA,
                    )
                else:
                    self.get_logger().error(f'pixel_utm_map has invalid shape {arr.shape}')
                    return
            if arr.shape[2] != 2:
                self.get_logger().error(
                    f'pixel_utm_map shape {arr.shape} must have 2 channels (E, N)'
                )
                return
            self._pixel_utm_map = arr.astype(np.float64)
            self.get_logger().info(f'Loaded pixel_utm_map from {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load pixel_utm_map: {e}')

    def _bilinear_utm(self, px, py):
        if self._pixel_utm_map is None:
            return None
        H, W, _ = self._pixel_utm_map.shape
        x = float(np.clip(px, 0.0, W - 1.001))
        y = float(np.clip(py, 0.0, H - 1.001))
        x0 = int(np.floor(x))
        y0 = int(np.floor(y))
        x1 = min(x0 + 1, W - 1)
        y1 = min(y0 + 1, H - 1)
        fx = x - x0
        fy = y - y0
        e00 = self._pixel_utm_map[y0, x0, 0]
        e10 = self._pixel_utm_map[y0, x1, 0]
        e01 = self._pixel_utm_map[y1, x0, 0]
        e11 = self._pixel_utm_map[y1, x1, 0]
        n00 = self._pixel_utm_map[y0, x0, 1]
        n10 = self._pixel_utm_map[y0, x1, 1]
        n01 = self._pixel_utm_map[y1, x0, 1]
        n11 = self._pixel_utm_map[y1, x1, 1]
        E = (1 - fx) * (1 - fy) * e00 + fx * (1 - fy) * e10 + (1 - fx) * fy * e01 + fx * fy * e11
        N = (1 - fx) * (1 - fy) * n00 + fx * (1 - fy) * n10 + (1 - fx) * fy * n01 + fx * fy * n11
        return (E, N)

    def _update_local_metric_for_publish(self):
        self._local_metric_utm = None
        if self._pixel_utm_map is None:
            return
        row, col = self._publish_anchor_rowcol()
        en = self._bilinear_utm(col, row)
        if en is None:
            return
        if self._vps_origin_en is None:
            self._vps_origin_en = (en[0], en[1])
        self._local_metric_utm = (
            en[0] - self._vps_origin_en[0],
            en[1] - self._vps_origin_en[1],
        )

    def _local_xy_enu_m(self) -> Tuple[float, float]:
        self._update_local_metric_for_publish()
        if self._local_metric_utm is not None:
            return float(self._local_metric_utm[0]), float(self._local_metric_utm[1])

        row, col = self._publish_anchor_rowcol()
        if self._anchor_origin_rowcol is None:
            self._anchor_origin_rowcol = (row, col)
        o_row, o_col = self._anchor_origin_rowcol
        d_row = row - o_row
        d_col = col - o_col

        if self._reference_bbox_configured():
            bounds = self._georef_bounds()
            if bounds is None:
                return d_col, -d_row
            min_lon, min_lat, max_lon, max_lat = bounds

            def lonlat(c: float, r: float) -> Tuple[float, float]:
                return tf_.pixel_to_geodetic(
                    c,
                    r,
                    self.w,
                    self.h,
                    min_lon,
                    min_lat,
                    max_lon,
                    max_lat,
                )

            lon0, lat0 = lonlat(o_col, o_row)
            lon, lat = lonlat(col, row)
            east = (lon - lon0) * tf_.meters_per_degree_lon(lat0)
            north = (lat - lat0) * tf_.meters_per_degree_lat(lat0)
            return east, north

        mpp = self._working_meters_per_pixel()
        if mpp is not None:
            return d_col * mpp, -d_row * mpp
        return d_col, -d_row

    def _passes_first_fix_gate(self) -> bool:
        n = int(self._last_num_matches)
        ms = float(self._last_mean_score)
        ir = float(self._last_inlier_ratio)
        pmin = self.get_parameter('first_fix_min_matches').get_parameter_value().integer_value
        smin = self.get_parameter('first_fix_min_mean_score').get_parameter_value().double_value
        irmin = self.get_parameter('first_fix_min_inlier_ratio').get_parameter_value().double_value
        return n >= pmin and ms >= smin and ir >= irmin

    def _vps_xy_variance_m2(self) -> tuple:
        base = self.get_parameter('vps_pose_variance_base_m2').get_parameter_value().double_value
        n = max(1, int(self._last_num_matches))
        ms = max(0.01, float(self._last_mean_score))
        ir = max(0.01, float(self._last_inlier_ratio))
        q = (n / 40.0) * ms * ir
        q = max(0.15, min(2.0, q))
        v = base / q
        v = max(0.25, min(25.0, v))
        return (v, v)

    def _max_kernel_long_edge(self, out_h: int, out_w: int) -> int:
        if out_w >= out_h:
            return min(self.h - 2, int((self.w - 2) * out_h / float(out_w)))
        return min(self.w - 2, int((self.h - 2) * out_w / float(out_h)))

    def _footprint_fill_ratio(self, dst: np.ndarray, out_w: int, out_h: int) -> float:
        rect = cv.minAreaRect(dst.reshape(-1, 2).astype(np.float32))
        rw, rh = rect[1]
        if rw <= 1.0 or rh <= 1.0:
            return 0.0
        return (rw * rh) / float(max(1, out_w) * max(1, out_h))

    def _maybe_grow_kernel_size(self, dst: np.ndarray, out_w: int, out_h: int) -> None:
        grow_ratio = float(
            self.get_parameter('kernel_grow_fill_ratio').get_parameter_value().double_value
        )
        grow_factor = float(
            self.get_parameter('kernel_grow_factor').get_parameter_value().double_value
        )
        max_size = min(self._kernel_max_size_active, self._max_kernel_long_edge(out_h, out_w))
        fill = self._footprint_fill_ratio(dst, out_w, out_h)
        if fill < grow_ratio or grow_factor <= 1.0:
            return
        new_size = int(round(self._active_kernel_size * grow_factor))
        capped = min(max_size, max(self._active_kernel_size + 1, new_size))
        if capped <= self._active_kernel_size:
            return
        self.get_logger().info(
            f'kernel_size {self._active_kernel_size} -> {capped} '
            f'(footprint fill {fill:.2f})'
        )
        self._active_kernel_size = capped

    def kernel_show(
        self,
        center_row: int,
        center_col: int,
        out_h: int,
        out_w: int,
    ) -> Tuple[np.ndarray, float, float]:
        out_h = max(1, int(out_h))
        out_w = max(1, int(out_w))
        ksize = max(1, int(self._active_kernel_size))
        ksize = min(ksize, self._max_kernel_long_edge(out_h, out_w))

        if center_row == 0 and center_col == 0:
            center_row = self.h // 2
            center_col = self.w // 2

        if out_w >= out_h:
            crop_w = ksize
            crop_h = max(1, int(round(ksize * out_h / float(out_w))))
        else:
            crop_h = ksize
            crop_w = max(1, int(round(ksize * out_w / float(out_h))))

        margin_row = crop_h // 2
        margin_col = crop_w // 2
        min_row, max_row = margin_row, self.h - margin_row
        min_col, max_col = margin_col, self.w - margin_col

        center_row = int(np.clip(center_row, min_row, max_row))
        center_col = int(np.clip(center_col, min_col, max_col))

        step = float(self._pending_kernel_rot_step_deg)
        self.tot_rot += step
        self._pending_kernel_rot_step_deg = 0.0
        rot_mat = cv.getRotationMatrix2D((int(center_col), int(center_row)), self.tot_rot, 1.0)
        mosaic = cv.warpAffine(self.mosaic_bgr, rot_mat, (self.w, self.h))

        kernel_bgr = mosaic[
            center_row - margin_row : center_row + margin_row,
            center_col - margin_col : center_col + margin_col,
            :,
        ]
        if kernel_bgr.shape[0] != out_h or kernel_bgr.shape[1] != out_w:
            kernel_bgr = cv.resize(kernel_bgr, (out_w, out_h), interpolation=cv.INTER_LINEAR)

        scale_row = crop_h / float(out_h)
        scale_col = crop_w / float(out_w)
        return kernel_bgr, scale_row, scale_col
    
    def map_kernel_img(self, x, y, base_x, base_y):
        if base_x == 0 and base_y == 0:
            base_x = self.h // 2
            base_y = self.w // 2
        
        rot_mat = cv.getRotationMatrix2D((int(0), int(0)), self.tot_rot, 1)
        rot3d = np.vstack((rot_mat, np.array([0, 0, 1])))
        inv_rot3d = np.linalg.inv(rot3d)
        corr_pt = np.matmul(inv_rot3d, np.array([y, x, 1]))
        
        new_x = base_x + corr_pt[1]
        new_y = base_y + corr_pt[0]
        
        return new_x, new_y
    
    def calculate_rotation_from_homography(self, M):
        try:
            M_norm = M / M[2, 2]
            R = M_norm[:2, :2]
            
            scale_x = np.linalg.norm(R[:, 0])
            scale_y = np.linalg.norm(R[:, 1])
            
            if scale_x > 0 and scale_y > 0:
                R_norm = R / np.array([[scale_x], [scale_y]])
                theta_rad = np.arctan2(R_norm[1, 0], R_norm[0, 0])
                theta_deg = np.degrees(theta_rad)
            else:
                theta_deg = 0.0
                
        except Exception as e:
            self.get_logger().debug(f'homography rotation: {e}')
            theta_deg = 0.0

        return theta_deg

    @staticmethod
    def _rect_tilt_deg_from_dst(dst: np.ndarray) -> float:
        """Tilt of camera footprint quad in kernel image coords (minAreaRect), ~(-45, 45] deg."""
        pts = np.int32(dst.reshape(-1, 2))
        rect = cv.minAreaRect(pts)
        theta_deg = float(rect[2])
        if theta_deg < -45.0:
            theta_deg = 90.0 + theta_deg
        if theta_deg > 45.0:
            theta_deg = -90.0 + theta_deg
        return theta_deg

    def _kernel_rotation_step_from_dst(self, dst: np.ndarray, theta_hom: float) -> float:
        k = float(self.get_parameter('kernel_rotation_k').get_parameter_value().double_value)
        mx = float(self.get_parameter('kernel_rotation_max_step_deg').get_parameter_value().double_value)
        dead = float(self.get_parameter('kernel_rotation_deadband_deg').get_parameter_value().double_value)
        sg = float(self.get_parameter('kernel_rotation_rect_sign').get_parameter_value().double_value)
        k = max(0.0, k)
        mx = max(0.0, mx)

        if self.get_parameter('kernel_rotation_use_rect_tilt').get_parameter_value().bool_value:
            tilt = self._rect_tilt_deg_from_dst(dst)
            if abs(tilt) < dead:
                return 0.0
            step_mag = min(abs(tilt) * k, mx)
            return float(np.copysign(step_mag, tilt) * sg)

        if abs(theta_hom) < dead:
            return 0.0
        step_mag = min(abs(theta_hom) * k, mx)
        return float(np.copysign(step_mag, theta_hom) * sg)

    def _angle_delta_deg(self, a: float, b: float) -> float:
        return (a - b + 180.0) % 360.0 - 180.0

    @staticmethod
    def _wrap_deg180(deg: float) -> float:
        return (float(deg) + 180.0) % 360.0 - 180.0

    def validate_rotation(self, theta_deg):
        if not self.get_parameter('enable_rotation_validation').get_parameter_value().bool_value:
            return theta_deg

        max_change = self.get_parameter('max_rotation_change').get_parameter_value().double_value
        std_threshold = self.get_parameter('rotation_std_threshold').get_parameter_value().double_value

        if abs(self._angle_delta_deg(theta_deg, self.last_valid_rot)) > max_change:
            self.get_logger().debug(
                f'rotation change clamped: {theta_deg:.2f} vs last {self.last_valid_rot:.2f}'
            )
            return self.last_valid_rot

        self.rot_hist.append(theta_deg)
        max_history = self.get_parameter('max_rotation_history').get_parameter_value().integer_value
        if len(self.rot_hist) > max_history:
            self.rot_hist.pop(0)

        if len(self.rot_hist) >= 3:
            recent = np.deg2rad(np.asarray(self.rot_hist[-3:], dtype=np.float64))
            recent_std = float(np.degrees(np.std(np.unwrap(recent))))
            if recent_std > std_threshold:
                self.get_logger().debug(f'rotation variance clamped: std={recent_std:.2f}')
                return self.last_valid_rot

        self.last_valid_rot = theta_deg
        return theta_deg

    def smooth_rotation(self, theta_deg):
        if not self.get_parameter('enable_rotation_smoothing').get_parameter_value().bool_value:
            return theta_deg

        if len(self.rot_hist) == 0:
            return theta_deg

        angles = np.deg2rad(np.asarray(self.rot_hist, dtype=np.float64))
        weights = np.linspace(0.5, 1.0, len(angles))
        weights = weights / np.sum(weights)
        sin_sum = float(np.sum(weights * np.sin(angles)))
        cos_sum = float(np.sum(weights * np.cos(angles)))
        return float(np.degrees(np.arctan2(sin_sum, cos_sum)))
    
    def _cv_bgr_from_image_msg(self, msg: Image) -> np.ndarray:
        return self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _cv_bgr_from_compressed_msg(self, msg: CompressedImage) -> np.ndarray:
        return self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')

    def _ap_time_for_tf_callback(self, msg: BuiltinTime) -> None:
        self._ap_time_stamp_for_tf = msg

    @staticmethod
    def _builtin_time_from_unix_now() -> BuiltinTime:
        ns = time.time_ns()
        out = BuiltinTime()
        out.sec = int(ns // 1_000_000_000)
        out.nanosec = int(ns % 1_000_000_000)
        return out

    def compressed_image_callback(self, msg: CompressedImage) -> None:
        try:
            self._process_localization_frame(self._cv_bgr_from_compressed_msg(msg))
        except Exception as e:
            self.get_logger().warning(f'compressed image callback: {e}')

    def image_callback(self, msg: Image) -> None:
        try:
            self._process_localization_frame(self._cv_bgr_from_image_msg(msg))
        except Exception as e:
            self.get_logger().warning(f'image callback: {e}')

    def _process_localization_frame(self, frame):
        try:
            frame = cv.resize(frame, (0, 0), fx=0.6, fy=0.6, interpolation=cv.INTER_AREA)
            hh, ww = frame.shape[:2]
            
            self.base_x, self.base_y = self.map_kernel_img(self.y, self.x, self.base_x, self.base_y)
            
            img_kl, kernel_scale_row, kernel_scale_col = self.kernel_show(
                int(self.base_x),
                int(self.base_y),
                hh,
                ww,
            )

            image0 = load_image_arr(cv.cvtColor(img_kl, cv.COLOR_BGR2RGB))
            feats0 = self.extractor.extract(image0.to(self.device))
            
            if frame.ndim == 3:
                frame_tensor = frame.transpose((2, 0, 1))
            elif frame.ndim == 2:
                frame_tensor = frame[None]
            
            image1 = torch.tensor(frame_tensor / 255.0, dtype=torch.float)
            feats1 = self.extractor.extract(image1.to(self.device))
            
            if self.frame_count % 23 == 0:
                self.get_logger().debug(f'frame {self.frame_count}')

            matches01 = self.matcher({"image0": feats0, "image1": feats1})
            
            feats0, feats1, matches01 = [rbd(x) for x in [feats0, feats1, matches01]]
            
            kpts0, kpts1 = feats0["keypoints"], feats1["keypoints"]
            matches0 = matches01["matches0"]
            scores0 = matches01["matching_scores0"]
            
            threshold = self.get_parameter('match_threshold').get_parameter_value().double_value
            valid = (scores0 > threshold)
            
            idx0 = torch.nonzero(valid).squeeze()
            idx1 = matches0[valid]
            
            m_kpts0 = kpts0[idx0]
            m_kpts1 = kpts1[idx1]
            
            num_matches = len(m_kpts0)
            if self.frame_count % 10 == 0:
                self.get_logger().debug(f'matches={num_matches}')
            if num_matches > 100:
                self.get_logger().debug('high match count')

            dst = None
            
            min_matches = self.get_parameter('min_matches').get_parameter_value().integer_value
            if num_matches < min_matches:
                self._publish_match_pair_panel(frame, img_kl, num_matches, dst)
                return
            
            M, mask = cv.findHomography(
                m_kpts1.cpu().numpy(), 
                m_kpts0.cpu().numpy(), 
                cv.RANSAC, 
                5.0
            )
            
            if M is None:
                self._publish_match_pair_panel(frame, img_kl, num_matches, dst)
                return
            if mask is not None:
                inlier_ratio = float(np.count_nonzero(mask)) / float(mask.size)
            else:
                inlier_ratio = 1.0
            mean_score = float(scores0[valid].float().mean().cpu().item())
            self._last_num_matches = num_matches
            self._last_mean_score = mean_score
            self._last_inlier_ratio = inlier_ratio

            min_ir = float(self.get_parameter('min_frame_inlier_ratio').get_parameter_value().double_value)
            if min_ir > 0.0 and inlier_ratio < min_ir:
                self.get_logger().debug(
                    f'skip frame: inlier_ratio={inlier_ratio:.3f} < min_frame_inlier_ratio={min_ir:.3f}'
                )
                self._publish_match_pair_panel(frame, img_kl, num_matches, dst)
                return

            pts = np.float32([[0, 0], [ww, 0], [ww, hh], [0, hh]]).reshape(-1, 1, 2)
            dst = cv.perspectiveTransform(pts, M)
            if self.get_parameter('reject_degenerate_homography').get_parameter_value().bool_value:
                ref_a = float(max(1, ww * hh))
                q_area = abs(cv.contourArea(dst.reshape(-1, 2).astype(np.float32)))
                ar = q_area / ref_a
                if ar < 0.12 or ar > 6.0:
                    self.get_logger().debug(
                        f'skip frame: homography area ratio {ar:.3f} (expected ~1 for similarity)'
                    )
                    self._publish_match_pair_panel(frame, img_kl, num_matches, dst)
                    return
            self._maybe_grow_kernel_size(dst, ww, hh)
            self._publish_match_pair_panel(frame, img_kl, num_matches, dst)

            theta_hom = self.calculate_rotation_from_homography(M)
            if abs(theta_hom) >= 180:
                self._publish_match_pair_panel(frame, img_kl, num_matches, dst)
                return

            rect_tilt = self._rect_tilt_deg_from_dst(dst)
            if abs(rect_tilt) > 75.0:
                self.get_logger().debug(f'skip frame: rect tilt {rect_tilt:.1f} deg out of range')
                self._publish_match_pair_panel(frame, img_kl, num_matches, dst)
                return

            x, y = np.mean(dst, axis=0).astype(int)[0]
            x_new = int(round((x - (ww * 0.5)) * kernel_scale_col))
            y_new = int(round((y - (hh * 0.5)) * kernel_scale_row))
            max_step = float(self.get_parameter('max_kernel_translation_step_px').get_parameter_value().double_value)
            if max_step > 0.0 and self._prev_kernel_xy is not None:
                step = math.hypot(x_new - self._prev_kernel_xy[0], y_new - self._prev_kernel_xy[1])
                if step > max_step:
                    self.get_logger().debug(
                        f'skip frame: kernel translation step {step:.1f}px > max_kernel_translation_step_px={max_step:.1f}'
                    )
                    self._publish_match_pair_panel(frame, img_kl, num_matches, dst)
                    return

            theta_pose = self.validate_rotation(theta_hom)
            theta_pose = self.smooth_rotation(theta_pose)

            if abs(theta_pose) > 50:
                self.get_logger().debug(f'large homography rotation: {theta_pose:.1f} deg')

            self.theta_deg = float(theta_pose)
            self._pending_kernel_rot_step_deg = self._kernel_rotation_step_from_dst(dst, theta_hom)
            self.x = x_new
            self.y = y_new
            self._prev_kernel_xy = (float(self.x), float(self.y))

            self.publish_results()

            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            gc.collect()

            self.frame_count += 1

        except Exception as e:
            self.get_logger().warning(f'localization frame: {e}')

    def publish_results(self):
        current_time = self.get_clock().now().to_msg()
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        lx, ly = self._local_xy_enu_m()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = lx
        pose_msg.pose.position.y = ly
        pose_msg.pose.position.z = self._published_altitude_m()
        yaw_pub = self._wrap_deg180(self.theta_deg)
        pose_msg.pose.orientation.z = np.sin(np.radians(yaw_pub) / 2.0)
        pose_msg.pose.orientation.w = np.cos(np.radians(yaw_pub) / 2.0)
        self.pose_pub.publish(pose_msg)

        enable_global = self.get_parameter('enable_global_coordinates').get_parameter_value().bool_value
        if enable_global:
            self._publish_global_coordinates(current_time)
        
        position_msg = PointStamped()
        position_msg.header.stamp = current_time
        position_msg.header.frame_id = frame_id
        position_msg.point.x = float(self.x)
        position_msg.point.y = float(self.y)
        position_msg.point.z = 0.0
        self.position_pub.publish(position_msg)
        
        rotation_msg = Float64()
        rotation_msg.data = float(yaw_pub)
        self.rotation_pub.publish(rotation_msg)

        if not self._first_vps_odom_published and not self._passes_first_fix_gate():
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._first_fix_gate_log_t >= 8.0:
                self._first_fix_gate_log_t = now
                self.get_logger().info(
                    f'first-fix gate: skip /odometry/vps (matches={self._last_num_matches} '
                    f'mean_score={self._last_mean_score:.3f} inlier_ratio={self._last_inlier_ratio:.2f})'
                )
            return

        vx, vy = self._vps_xy_variance_m2()
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = frame_id
        odom.child_frame_id = frame_id + '_vps'
        odom.pose.pose.position.x = lx
        odom.pose.pose.position.y = ly
        odom.pose.pose.position.z = self._published_altitude_m()
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        for i in range(36):
            odom.pose.covariance[i] = 0.0
        odom.pose.covariance[0] = vx
        odom.pose.covariance[7] = vy
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 1e6
        self.odom_vps_pub.publish(odom)
        if self._ap_dds_tf_pub is not None and self._dds_tf_hist_x is not None:
            alt_z = float(self._published_altitude_m())
            self._dds_tf_hist_x.append(float(lx))
            self._dds_tf_hist_y.append(float(ly))
            self._dds_tf_hist_z.append(alt_z)
            self._dds_tf_hist_yaw_deg.append(float(yaw_pub))
            mx = float(np.median(self._dds_tf_hist_x))
            my = float(np.median(self._dds_tf_hist_y))
            mz = float(np.median(self._dds_tf_hist_z))
            myaw = float(np.median(self._dds_tf_hist_yaw_deg))
            zs = float(np.sin(np.radians(myaw) / 2.0))
            wc = float(np.cos(np.radians(myaw) / 2.0))
            ts = TransformStamped()
            if self._dds_tf_use_unix_wall_time:
                tf_stamp = self._builtin_time_from_unix_now()
            elif self._dds_tf_use_ap_time and self._ap_time_stamp_for_tf is not None:
                tf_stamp = self._ap_time_stamp_for_tf
            else:
                tf_stamp = current_time
            ts.header.stamp = tf_stamp
            ts.header.frame_id = 'odom'
            ts.child_frame_id = 'base_link'
            ts.transform.translation.x = my
            ts.transform.translation.y = -mx
            ts.transform.translation.z = mz
            ts.transform.rotation.x = 0.0
            ts.transform.rotation.y = 0.0
            ts.transform.rotation.z = zs
            ts.transform.rotation.w = wc
            self._ap_dds_tf_pub.publish(TFMessage(transforms=[ts]))
        if not self._first_vps_odom_published:
            self._first_vps_odom_published = True
            self.get_logger().info(
                f'first VPS /odometry/vps published (matches={self._last_num_matches} '
                f'mean_score={self._last_mean_score:.3f} inlier_ratio={self._last_inlier_ratio:.2f})'
            )
    
    def _publish_global_coordinates(self, current_time):
        try:
            bounds = self._georef_bounds()
            if bounds is None:
                if not hasattr(self, '_georef_warned'):
                    self.get_logger().debug(
                        'Global coordinates disabled: reference bounds not configured '
                        '(YAML corners or metadata.json beside the reference TIFF).'
                    )
                    self._georef_warned = True
                return
            min_lon, min_lat, max_lon, max_lat = bounds
            ref_alt = self._published_altitude_m()

            row, col = self._publish_anchor_rowcol()
            lon, lat = tf_.pixel_to_geodetic(
                pixel_x=col,
                pixel_y=row,
                image_width=self.w,
                image_height=self.h,
                min_lon=min_lon,
                min_lat=min_lat,
                max_lon=max_lon,
                max_lat=max_lat,
            )
            
            navsat_msg = NavSatFix()
            navsat_msg.header.stamp = current_time
            navsat_msg.header.frame_id = "earth"
            navsat_msg.latitude = lat
            navsat_msg.longitude = lon
            navsat_msg.altitude = ref_alt
            navsat_msg.status.status = NavSatStatus.STATUS_FIX
            navsat_msg.status.service = NavSatStatus.SERVICE_GPS
            navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.global_position_pub.publish(navsat_msg)
            
            ecef_x, ecef_y, ecef_z = tf_.wgs84_to_ecef(lon, lat, ref_alt)
            ecef_msg = PointStamped()
            ecef_msg.header.stamp = current_time
            ecef_msg.header.frame_id = "earth"
            ecef_msg.point.x = ecef_x
            ecef_msg.point.y = ecef_y
            ecef_msg.point.z = ecef_z
            self.ecef_position_pub.publish(ecef_msg)
            
            if self.frame_count % 50 == 0:
                self.get_logger().info(
                    f"Global position: lat={lat:.6f}, lon={lon:.6f}, "
                    f"ECEF=({ecef_x:.2f}, {ecef_y:.2f}, {ecef_z:.2f})"
                )
                
        except Exception as e:
            self.get_logger().warn(f"Failed to publish global coordinates: {e}")

    def _stack_match_panels(self, panels: list[np.ndarray]) -> np.ndarray:
        if not panels:
            return np.zeros((64, 64, 3), dtype=np.uint8)
        target_h = max(panel.shape[0] for panel in panels)
        resized = []
        for panel in panels:
            if panel.shape[0] == target_h:
                resized.append(panel)
                continue
            scale = target_h / float(panel.shape[0])
            new_w = max(1, int(round(panel.shape[1] * scale)))
            resized.append(cv.resize(panel, (new_w, target_h), interpolation=cv.INTER_AREA))
        return cv.hconcat(resized)

    def _label_match_panel(self, image_bgr: np.ndarray, title: str) -> np.ndarray:
        labeled = image_bgr.copy()
        cv.putText(
            labeled,
            title,
            (8, 24),
            cv.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
            cv.LINE_AA,
        )
        return labeled

    def _kernel_overlay_bgr(self, kernel_bgr: np.ndarray, dst: Optional[np.ndarray]) -> np.ndarray:
        if dst is None:
            return kernel_bgr.copy()
        rect = cv.minAreaRect(np.int32(dst))
        box = cv.boxPoints(rect)
        box = np.int32(box)
        overlay = cv.drawContours(kernel_bgr.copy(), [box], 0, (0, 255, 255), 2, cv.LINE_AA)
        x, y = np.mean(dst, axis=0).astype(int)[0]
        return cv.circle(overlay, (x, y), 8, (0, 0, 255), -1)

    def _publish_match_pair_panel(
        self,
        camera_bgr: np.ndarray,
        kernel_bgr: np.ndarray,
        num_matches: int,
        dst: Optional[np.ndarray],
    ) -> None:
        if not self._publish_match_pair or self.match_pair_pub is None:
            return
        try:
            kernel_vis = self._kernel_overlay_bgr(kernel_bgr, dst)
            camera_vis = camera_bgr.copy()
            if camera_vis.ndim == 2:
                camera_vis = cv.cvtColor(camera_vis, cv.COLOR_GRAY2BGR)
            panel = self._stack_match_panels([
                self._label_match_panel(camera_vis, 'camera'),
                self._label_match_panel(kernel_vis, 'mosaic kernel'),
            ])
            footer = (
                f'matches={num_matches} '
                f'anchor=({int(self.base_x)},{int(self.base_y)})'
            )
            cv.putText(
                panel,
                footer,
                (8, panel.shape[0] - 12),
                cv.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv.LINE_AA,
            )
            msg = self.bridge.cv2_to_imgmsg(panel, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
            self.match_pair_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'failed to publish match pair debug image: {exc}')


def main(args=None):
    rclpy.init(args=args)
    
    node = NGPSLocalizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
