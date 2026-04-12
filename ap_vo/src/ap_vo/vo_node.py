#!/usr/bin/env python3

import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
import tf_transformations


class VONode(Node):
    """Two-frame monocular VO: SIFT, Lowe ratio test, metric PnP on a ground plane, odom integration."""

    def __init__(self):
        super().__init__('vo_node')

        self._cv_bridge = CvBridge()
        self._sift = cv2.SIFT_create(nfeatures=1024)
        self._bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)

        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('min_matches', 30)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('child_frame_id', 'camera_optical')
        self.declare_parameter('use_tf_for_scale', False)
        self.declare_parameter('plane_distance_m', 50.0)
        self.declare_parameter('publish_odometry', True)
        self.declare_parameter('odometry_topic', '/odometry/vo')
        self.declare_parameter('pnp_reprojection_error', 8.0)

        self._cached_ref_msg = None
        self._cached_kps_desc = None

        self._camera_info = None
        self._R_wc = np.eye(3)
        self._t_wc = np.zeros(3)
        self._prev_image_stamp = None
        self._prev_t_wc = None
        self._tf_warn_last = -1e9

        self._tf_buffer = tf2_ros.Buffer(Duration(seconds=30))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)

        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'vo/pose', 10)
        odom_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        cam_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        cam_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self._camera_info_sub = self.create_subscription(
            CameraInfo, cam_info_topic, self._camera_info_cb, QoSPresetProfiles.SENSOR_DATA.value
        )
        self._image_sub = self.create_subscription(
            Image, cam_topic, self._image_cb, QoSPresetProfiles.SENSOR_DATA.value
        )

        self.get_logger().info(
            'VO: standalone uses plane_distance_m for metric PnP; set use_tf_for_scale for map->base_link Z'
        )

    def _camera_info_cb(self, msg: CameraInfo):
        self._camera_info = msg

    def _K_D(self):
        k = np.array(self._camera_info.k, dtype=np.float64).reshape(3, 3)
        d = np.array(self._camera_info.d, dtype=np.float64) if self._camera_info.d else np.zeros(5, dtype=np.float64)
        if d.size < 5:
            d = np.pad(d, (0, 5 - d.size))
        return k, d

    def _unproject_plane(self, uv: np.ndarray, z_m: float) -> np.ndarray:
        k, _ = self._K_D()
        fx, fy = k[0, 0], k[1, 1]
        cx, cy = k[0, 2], k[1, 2]
        out = np.zeros((len(uv), 3), dtype=np.float64)
        out[:, 0] = (uv[:, 0] - cx) * z_m / fx
        out[:, 1] = (uv[:, 1] - cy) * z_m / fy
        out[:, 2] = z_m
        return out

    def _plane_distance_m(self) -> float:
        if self.get_parameter('use_tf_for_scale').get_parameter_value().bool_value:
            try:
                q = self.get_clock().now()
                tf = self._tf_buffer.lookup_transform(
                    'map', 'base_link', q, Duration(seconds=0.2)
                )
                return float(abs(tf.transform.translation.z))
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                now = self.get_clock().now().nanoseconds * 1e-9
                if now - self._tf_warn_last >= 5.0:
                    self._tf_warn_last = now
                    self.get_logger().warn(
                        'use_tf_for_scale true but TF map->base_link failed; using plane_distance_m'
                    )
        return float(self.get_parameter('plane_distance_m').get_parameter_value().double_value)

    def _image_cb(self, msg: Image):
        if self._cached_ref_msg is None:
            self._cached_ref_msg = msg
            self._prev_image_stamp = msg.header.stamp
            return

        if self._camera_info is None:
            return

        out = self._compute_pose(msg, self._cached_ref_msg)
        if out is None:
            return

        pose_msg, _ = out
        self._pose_pub.publish(pose_msg)

        if self.get_parameter('publish_odometry').get_parameter_value().bool_value:
            odom = self._make_odometry(msg.header.stamp)
            self._odom_pub.publish(odom)

        self._cached_ref_msg = msg

    def _compute_pose(self, query: Image, reference: Image):
        qry = self._cv_bridge.imgmsg_to_cv2(query, desired_encoding='mono8')
        ref = self._cv_bridge.imgmsg_to_cv2(reference, desired_encoding='mono8')

        kp_qry, desc_qry = self._sift.detectAndCompute(qry, None)

        if self._cached_kps_desc is None:
            kp_ref, desc_ref = self._sift.detectAndCompute(ref, None)
        else:
            kp_ref, desc_ref = self._cached_kps_desc

        if desc_qry is None or desc_ref is None or len(kp_qry) < 4 or len(kp_ref) < 4:
            return None

        try:
            matches = self._bf.knnMatch(desc_qry, desc_ref, k=2)
        except cv2.error:
            return None

        min_m = int(self.get_parameter('min_matches').get_parameter_value().integer_value)
        if len(matches) < min_m:
            return None

        ratio = float(self.get_parameter('confidence_threshold').get_parameter_value().double_value)
        good = [m for m, n in matches if len(n) == 2 and m.distance < ratio * n.distance]
        if len(good) < min_m:
            return None

        mkp_qry = np.array([kp_qry[m.queryIdx].pt for m in good], dtype=np.float64)
        mkp_ref = np.array([kp_ref[m.trainIdx].pt for m in good], dtype=np.float64)

        k, d = self._K_D()
        z = self._plane_distance_m()
        pts3 = self._unproject_plane(mkp_ref, z)

        reproj = float(self.get_parameter('pnp_reprojection_error').get_parameter_value().double_value)
        try:
            ok, rvec, tvec, _inliers = cv2.solvePnPRansac(
                pts3.astype(np.float32),
                mkp_qry.astype(np.float32),
                k.astype(np.float32),
                d.astype(np.float32),
                reprojectionError=reproj,
                confidence=0.999,
                iterationsCount=200,
                flags=cv2.SOLVEPNP_EPNP,
            )
        except cv2.error:
            return None

        if not ok or rvec is None or tvec is None:
            return None

        R, _ = cv2.Rodrigues(rvec)
        t = tvec.reshape(3)

        c_q_in_ref = (-R.T @ t.reshape(3, 1)).ravel()
        self._t_wc = self._R_wc @ c_q_in_ref + self._t_wc
        self._R_wc = self._R_wc @ R.T

        self._cached_kps_desc = (kp_qry, desc_qry)

        pose_msg = self._make_pose_stamped(query.header.stamp)
        return (pose_msg, None)

    def _make_pose_stamped(self, stamp):
        out = PoseWithCovarianceStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.get_parameter('output_frame').get_parameter_value().string_value

        T = np.eye(4)
        T[:3, :3] = self._R_wc
        T[:3, 3] = self._t_wc
        q = tf_transformations.quaternion_from_matrix(T)
        out.pose.pose.orientation.x = float(q[0])
        out.pose.pose.orientation.y = float(q[1])
        out.pose.pose.orientation.z = float(q[2])
        out.pose.pose.orientation.w = float(q[3])
        out.pose.pose.position.x = float(self._t_wc[0])
        out.pose.pose.position.y = float(self._t_wc[1])
        out.pose.pose.position.z = float(self._t_wc[2])

        for i in range(36):
            out.pose.covariance[i] = 0.0
        out.pose.covariance[0] = 0.25
        out.pose.covariance[7] = 0.25
        out.pose.covariance[14] = 0.25
        out.pose.covariance[21] = 0.01
        out.pose.covariance[28] = 0.01
        out.pose.covariance[35] = 0.01
        return out

    def _make_odometry(self, stamp):
        child = self.get_parameter('child_frame_id').get_parameter_value().string_value
        frame = self.get_parameter('output_frame').get_parameter_value().string_value

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = frame
        odom.child_frame_id = child

        odom.pose.pose.position.x = float(self._t_wc[0])
        odom.pose.pose.position.y = float(self._t_wc[1])
        odom.pose.pose.position.z = float(self._t_wc[2])
        T = np.eye(4)
        T[:3, :3] = self._R_wc
        T[:3, 3] = self._t_wc
        q = tf_transformations.quaternion_from_matrix(T)
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])

        for i in range(36):
            odom.pose.covariance[i] = 0.0
        odom.pose.covariance[0] = 0.25
        odom.pose.covariance[7] = 0.25
        odom.pose.covariance[14] = 0.25
        odom.pose.covariance[21] = 0.05
        odom.pose.covariance[28] = 0.05
        odom.pose.covariance[35] = 0.05

        dt = 0.05
        if self._prev_image_stamp is not None:
            t0 = self._prev_image_stamp.sec + self._prev_image_stamp.nanosec * 1e-9
            t1 = stamp.sec + stamp.nanosec * 1e-9
            dt = max(1e-3, t1 - t0)
        self._prev_image_stamp = stamp

        if self._prev_t_wc is not None:
            v = (self._t_wc - self._prev_t_wc) / dt
            odom.twist.twist.linear.x = float(v[0])
            odom.twist.twist.linear.y = float(v[1])
            odom.twist.twist.linear.z = float(v[2])
        self._prev_t_wc = self._t_wc.copy()

        for i in range(36):
            odom.twist.covariance[i] = 0.0
        odom.twist.covariance[0] = 0.5
        odom.twist.covariance[7] = 0.5
        odom.twist.covariance[14] = 0.5

        return odom


def main(args=None):
    rclpy.init(args=args)
    node = VONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
