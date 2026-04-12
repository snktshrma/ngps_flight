#!/usr/bin/env python3
"""
Map-matching VPS node.

Pipeline (every match_interval_ms):
  1. Crop 512x512 ROI from the centre of the latest camera frame
  2. AKAZE detect + describe on the ROI
  3. Brute-force Hamming match against pre-computed reference tile descriptors
  4. USAC_MAGSAC homography + inlier count gate
  5. H * live_centre_px  ->  ref_px
  6. GDAL geotransform   ->  UTM (E, N)
  7. UTM - (E0, N0)      ->  local (x, y) in metres

Outputs:
  - geometry_msgs/PointStamped  on  ~/fix/local_xy    (local metres, z=0)
  - nav_msgs/Odometry           on  ~/fix/odometry    (same position, with covariance)

The node is fully self-contained; it does not depend on any other NGPS package.

Reference tile requirements:
  - GeoTIFF with a valid GeoTransform in a UTM projection (e.g. EPSG:326xx / EPSG:327xx)
  - Set via ROS param  ref_tile_path
  - The origin (E0, N0) is taken from the tile's own top-left corner by default, or
    overridden via params  origin_easting / origin_northing
"""

import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image

try:
    from osgeo import gdal

    gdal.UseExceptions()
    _GDAL_OK = True
except ImportError:
    _GDAL_OK = False


class MapMatchNode(Node):
    """AKAZE map-matching VPS: reference GeoTIFF -> local (x, y) position fixes."""

    def __init__(self):
        super().__init__('map_match_node')

        # --- params -----------------------------------------------------------
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('ref_tile_path', '')
        self.declare_parameter('match_interval_ms', 200.0)
        self.declare_parameter('roi_size', 512)
        self.declare_parameter('min_inliers', 12)
        # UTM origin override; 0.0 means "use tile top-left"
        self.declare_parameter('origin_easting', 0.0)
        self.declare_parameter('origin_northing', 0.0)
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('position_covariance_xy', 2.0)
        # sliding window: half-width in reference tile pixels around last known position
        # 0 means always use full tile (cold-start behaviour forever)
        self.declare_parameter('search_window_px', 1500)

        self._bridge = CvBridge()
        self._akaze = cv2.AKAZE_create()
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # latest camera frame (protected by GIL - single-threaded spin)
        self._latest_frame: np.ndarray | None = None
        self._latest_stamp = None

        # reference tile data (loaded once)
        self._ref_gray: np.ndarray | None = None
        self._ref_kp = None
        self._ref_desc: np.ndarray | None = None
        self._ref_pts: np.ndarray | None = None   # (N,2) float32 - keypoint xy for fast window filter
        self._geotransform: tuple | None = None   # GDAL 6-tuple
        self._origin_e: float = 0.0
        self._origin_n: float = 0.0

        # sliding window state - set after first successful fix
        self._last_ref_col: float | None = None
        self._last_ref_row: float | None = None

        # publishers
        self._fix_pub = self.create_publisher(PointStamped, '~/fix/local_xy', 10)
        self._odom_pub = self.create_publisher(Odometry, '~/fix/odometry', 10)

        # subscriber
        cam_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self._img_sub = self.create_subscription(
            Image,
            cam_topic,
            self._image_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        # matching timer
        interval_ms = self.get_parameter('match_interval_ms').get_parameter_value().double_value
        self._timer = self.create_timer(interval_ms / 1000.0, self._match_cb)

        # load reference tile
        self._load_ref_tile()

        self.get_logger().info(
            f'MapMatchNode ready | interval={interval_ms:.0f} ms | '
            f'ref_tile={"loaded" if self._ref_desc is not None else "MISSING"}'
        )

    # -------------------------------------------------------------------------
    # Reference tile loading
    # -------------------------------------------------------------------------

    def _load_ref_tile(self):
        path = self.get_parameter('ref_tile_path').get_parameter_value().string_value
        if not path:
            self.get_logger().warn('ref_tile_path not set - node will not produce fixes')
            return

        if not _GDAL_OK:
            self.get_logger().error('GDAL (osgeo) not installed - cannot load reference tile')
            return

        if not Path(path).exists():
            self.get_logger().error(f'Reference tile not found: {path}')
            return

        ds = gdal.Open(path, gdal.GA_ReadOnly)
        if ds is None:
            self.get_logger().error(f'GDAL could not open: {path}')
            return

        self._geotransform = ds.GetGeoTransform()  # (GT0, GT1, GT2, GT3, GT4, GT5)
        band = ds.GetRasterBand(1)
        arr = band.ReadAsArray()  # single-band grey, or we use band 1 of RGB

        # if multi-band, read as BGR and convert
        if ds.RasterCount >= 3:
            r = ds.GetRasterBand(1).ReadAsArray().astype(np.uint8)
            g = ds.GetRasterBand(2).ReadAsArray().astype(np.uint8)
            b = ds.GetRasterBand(3).ReadAsArray().astype(np.uint8)
            bgr = np.stack([b, g, r], axis=-1)
            arr = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        else:
            arr = arr.astype(np.uint8)

        self._ref_gray = arr
        self._ref_kp, self._ref_desc = self._akaze.detectAndCompute(arr, None)
        self._ref_pts = np.array([kp.pt for kp in self._ref_kp], dtype=np.float32)
        ds = None  # close

        # UTM origin
        gt = self._geotransform
        e0_param = self.get_parameter('origin_easting').get_parameter_value().double_value
        n0_param = self.get_parameter('origin_northing').get_parameter_value().double_value
        self._origin_e = e0_param if e0_param != 0.0 else gt[0]
        self._origin_n = n0_param if n0_param != 0.0 else gt[3]

        self.get_logger().info(
            f'Ref tile loaded: {arr.shape[1]}x{arr.shape[0]} px | '
            f'{len(self._ref_kp)} keypoints | '
            f'origin E={self._origin_e:.1f} N={self._origin_n:.1f}'
        )

    # -------------------------------------------------------------------------
    # Camera callback - just cache the latest frame
    # -------------------------------------------------------------------------

    def _image_cb(self, msg: Image):
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            self._latest_stamp = msg.header.stamp
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'imgmsg_to_cv2 failed: {exc}', throttle_duration_sec=5.0)

    # -------------------------------------------------------------------------
    # Matching timer callback
    # -------------------------------------------------------------------------

    def _match_cb(self):
        if self._latest_frame is None:
            return
        if self._ref_desc is None or self._ref_kp is None:
            return

        frame = self._latest_frame
        stamp = self._latest_stamp
        roi_size = int(self.get_parameter('roi_size').get_parameter_value().integer_value)

        # --- 1. crop centre ROI ----------------------------------------------
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        half = roi_size // 2
        x0 = max(0, cx - half)
        y0 = max(0, cy - half)
        x1 = min(w, x0 + roi_size)
        y1 = min(h, y0 + roi_size)
        roi = frame[y0:y1, x0:x1]

        # --- 2. AKAZE on ROI -------------------------------------------------
        t0 = time.monotonic()
        live_kp, live_desc = self._akaze.detectAndCompute(roi, None)

        if live_desc is None or len(live_kp) < 4:
            self.get_logger().debug('AKAZE: too few keypoints in ROI', throttle_duration_sec=2.0)
            return

        # --- 3. Brute-force Hamming match ------------------------------------
        # After first fix: filter reference keypoints to a search window around
        # the last known position. AKAZE on the tile was run once at startup;
        # we just slice the stored arrays - no recomputation needed.
        win_px = int(self.get_parameter('search_window_px').get_parameter_value().integer_value)
        if win_px > 0 and self._last_ref_col is not None:
            col_min = self._last_ref_col - win_px
            col_max = self._last_ref_col + win_px
            row_min = self._last_ref_row - win_px
            row_max = self._last_ref_row + win_px
            idx = np.where(
                (self._ref_pts[:, 0] >= col_min) & (self._ref_pts[:, 0] < col_max) &
                (self._ref_pts[:, 1] >= row_min) & (self._ref_pts[:, 1] < row_max)
            )[0]
            if len(idx) < 4:
                # window produced too few keypoints - fall back to full tile
                match_kp   = self._ref_kp
                match_desc = self._ref_desc
                idx = None
            else:
                match_kp   = [self._ref_kp[i] for i in idx]
                match_desc = self._ref_desc[idx]
        else:
            match_kp   = self._ref_kp
            match_desc = self._ref_desc
            idx = None

        try:
            raw_matches = self._bf.knnMatch(live_desc, match_desc, k=2)
        except cv2.error as exc:
            self.get_logger().warn(f'BFMatcher failed: {exc}', throttle_duration_sec=5.0)
            return

        # Lowe ratio test (guard against pairs where k=1 was returned)
        good = [m for pair in raw_matches if len(pair) == 2 for m, n in [pair] if m.distance < 0.75 * n.distance]

        min_inliers = int(self.get_parameter('min_inliers').get_parameter_value().integer_value)
        if len(good) < min_inliers:
            self.get_logger().debug(
                f'Too few good matches: {len(good)} < {min_inliers}',
                throttle_duration_sec=2.0,
            )
            return

        # --- 4. USAC_MAGSAC homography ---------------------------------------
        pts_live = np.array([live_kp[m.queryIdx].pt for m in good], dtype=np.float32)
        pts_ref  = np.array([match_kp[m.trainIdx].pt for m in good], dtype=np.float32)

        H, mask = cv2.findHomography(
            pts_live,
            pts_ref,
            method=cv2.USAC_MAGSAC,
            ransacReprojThreshold=3.0,
            confidence=0.995,
            maxIters=5000,
        )

        if H is None or mask is None:
            self.get_logger().debug('Homography estimation failed', throttle_duration_sec=2.0)
            return

        n_inliers = int(mask.sum())
        if n_inliers < min_inliers:
            self.get_logger().debug(
                f'Inlier count too low: {n_inliers} < {min_inliers}',
                throttle_duration_sec=2.0,
            )
            return

        elapsed_ms = (time.monotonic() - t0) * 1000.0

        # --- 5. Map live centre pixel -> reference tile pixel ----------------
        # ROI centre in ROI coords
        roi_h, roi_w = roi.shape[:2]
        live_centre = np.array([[[roi_w / 2.0, roi_h / 2.0]]], dtype=np.float32)
        ref_px = cv2.perspectiveTransform(live_centre, H)   # shape (1,1,2)
        ref_col = float(ref_px[0, 0, 0])
        ref_row = float(ref_px[0, 0, 1])

        # --- 6. GDAL geotransform -> UTM -------------------------------------
        gt = self._geotransform
        # X_utm = GT[0] + col*GT[1] + row*GT[2]
        # Y_utm = GT[3] + col*GT[4] + row*GT[5]
        utm_e = gt[0] + ref_col * gt[1] + ref_row * gt[2]
        utm_n = gt[3] + ref_col * gt[4] + ref_row * gt[5]

        # --- 7. local (x, y) in metres ---------------------------------------
        local_x = utm_e - self._origin_e
        local_y = utm_n - self._origin_n

        # update sliding window anchor for next cycle
        self._last_ref_col = ref_col
        self._last_ref_row = ref_row

        windowed = self._last_ref_col is not None and win_px > 0
        self.get_logger().info(
            f'Fix | inliers={n_inliers} | x={local_x:.2f} y={local_y:.2f} m | '
            f'{elapsed_ms:.0f} ms | {"windowed" if windowed else "full-tile"}',
            throttle_duration_sec=1.0,
        )

        self._publish(local_x, local_y, stamp)

    # -------------------------------------------------------------------------
    # Publishers
    # -------------------------------------------------------------------------

    def _publish(self, x: float, y: float, stamp):
        frame = self.get_parameter('output_frame').get_parameter_value().string_value
        cov_xy = float(self.get_parameter('position_covariance_xy').get_parameter_value().double_value)

        # PointStamped - simple local xy fix
        pt = PointStamped()
        pt.header.stamp = stamp
        pt.header.frame_id = frame
        pt.point.x = x
        pt.point.y = y
        pt.point.z = 0.0
        self._fix_pub.publish(pt)

        # Odometry - for robot_localization / EKF fusion
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = frame
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0  # heading not provided by this node
        # diagonal covariance, x and y only; rest are very large (unknown)
        cov = [0.0] * 36
        cov[0]  = cov_xy   # x
        cov[7]  = cov_xy   # y
        cov[14] = 9999.0   # z - not estimated
        cov[21] = 9999.0   # roll
        cov[28] = 9999.0   # pitch
        cov[35] = 9999.0   # yaw
        odom.pose.covariance = cov
        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MapMatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
