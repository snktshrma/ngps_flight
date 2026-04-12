#!/usr/bin/env python3
"""Subtract VIO position at first VPS fix so local frame matches SYSTEM_PLAN (shared origin).

First VPS message should already pass NGPS first-fix gating. Optional origin_max_pose_variance_xy
defers alignment if pose covariance is unexpectedly large.
"""
import copy

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class VioOriginRelay(Node):
    def __init__(self):
        super().__init__('vio_origin_relay')
        self.declare_parameter('vio_raw_topic', '/odometry/vio_raw')
        self.declare_parameter('vio_out_topic', '/odometry/vio')
        self.declare_parameter('vps_topic', '/odometry/vps')
        self.declare_parameter('origin_max_pose_variance_xy', 1e9)

        raw_topic = self.get_parameter('vio_raw_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('vio_out_topic').get_parameter_value().string_value
        vps_topic = self.get_parameter('vps_topic').get_parameter_value().string_value

        self._aligned = False
        self._off_x = 0.0
        self._off_y = 0.0
        self._last_vio = None
        self._origin_defer_log_t = -1e9

        self._pub = self.create_publisher(Odometry, out_topic, 50)
        self.create_subscription(Odometry, raw_topic, self._cb_vio, 50)
        self.create_subscription(Odometry, vps_topic, self._cb_vps, 10)
        self.get_logger().info(
            f'vio_origin_relay: {raw_topic} + {vps_topic} -> {out_topic}'
        )

    def _cb_vio(self, msg: Odometry):
        self._last_vio = msg
        out = copy.deepcopy(msg)
        if self._aligned:
            out.pose.pose.position.x = float(msg.pose.pose.position.x - self._off_x)
            out.pose.pose.position.y = float(msg.pose.pose.position.y - self._off_y)
        self._pub.publish(out)

    def _cb_vps(self, msg: Odometry):
        if self._aligned:
            return
        if self._last_vio is None:
            self.get_logger().warn(
                'VPS received before any VIO; waiting for VIO to align origin.'
            )
            return
        max_var = self.get_parameter('origin_max_pose_variance_xy').get_parameter_value().double_value
        vx = float(msg.pose.covariance[0])
        vy = float(msg.pose.covariance[7])
        if vx > max_var or vy > max_var:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._origin_defer_log_t >= 6.0:
                self._origin_defer_log_t = now
                self.get_logger().info(
                    f'VPS origin deferred: pose variance (x={vx:.2f}, y={vy:.2f}) m^2 > max {max_var:.2f}'
                )
            return
        self._off_x = float(self._last_vio.pose.pose.position.x)
        self._off_y = float(self._last_vio.pose.pose.position.y)
        self._aligned = True
        self.get_logger().info(
            f'VIO XY origin aligned to first VPS fix: offset ({self._off_x:.3f}, {self._off_y:.3f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VioOriginRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
