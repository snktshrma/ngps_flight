#!/usr/bin/env python3
"""Fused horizontal odometry + IMU yaw + optional altitude -> single Odometry for FCU/MAVROS."""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseAssemblyNode(Node):
    def __init__(self):
        super().__init__('pose_assembly')
        self.declare_parameter('fused_in', '/fused/odometry')
        self.declare_parameter('out', '/odometry/fused_full')
        self.declare_parameter('imu_heading_in', '')
        self.declare_parameter('altitude_in', '')

        fused_in = self.get_parameter('fused_in').get_parameter_value().string_value
        out_topic = self.get_parameter('out').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_heading_in').get_parameter_value().string_value
        alt_topic = self.get_parameter('altitude_in').get_parameter_value().string_value

        self._last_yaw = None
        self._last_z = None

        self._pub = self.create_publisher(Odometry, out_topic, 50)
        self.create_subscription(Odometry, fused_in, self._cb_fused, 50)
        if imu_topic:
            self.create_subscription(Imu, imu_topic, self._cb_imu, 50)
        if alt_topic:
            self.create_subscription(Float64, alt_topic, self._cb_alt, 10)

        self.get_logger().info(
            f'pose_assembly: fused={fused_in} -> {out_topic} '
            f'(imu={imu_topic or "none"}, alt={alt_topic or "none"})'
        )

    def _cb_imu(self, msg: Imu):
        self._last_yaw = yaw_from_quat(msg.orientation)

    def _cb_alt(self, msg: Float64):
        self._last_z = float(msg.data)

    def _cb_fused(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = msg.pose.pose.position.x
        out.pose.pose.position.y = msg.pose.pose.position.y
        out.pose.pose.position.z = (
            self._last_z if self._last_z is not None else msg.pose.pose.position.z
        )
        if self._last_yaw is not None:
            cy = math.cos(self._last_yaw * 0.5)
            sy = math.sin(self._last_yaw * 0.5)
            out.pose.pose.orientation.x = 0.0
            out.pose.pose.orientation.y = 0.0
            out.pose.pose.orientation.z = sy
            out.pose.pose.orientation.w = cy
        else:
            out.pose.pose.orientation = msg.pose.pose.orientation

        out.twist = msg.twist
        out.pose.covariance = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PoseAssemblyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
