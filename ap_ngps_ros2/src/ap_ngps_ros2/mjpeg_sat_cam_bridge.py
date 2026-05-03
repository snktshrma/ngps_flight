#!/usr/bin/env python3
"""Publish sensor_msgs/Image for NGPS from sat_cam_emulator MJPEG HTTP (/video).

Same ROS 2 domain as ArduPilot: camera and IMU share DDS via the ROS 2 middleware.
"""
import sys

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class MjpegSatCamBridge(Node):
    def __init__(self):
        super().__init__('mjpeg_sat_cam_bridge')
        self.declare_parameter('mjpeg_url', 'http://127.0.0.1:8090/video')
        self.declare_parameter('output_topic', '/camera/image_raw')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('capture_fps', 15.0)

        url = self.get_parameter('mjpeg_url').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        fps = max(1.0, self.get_parameter('capture_fps').get_parameter_value().double_value)

        self._cap = cv2.VideoCapture(url)
        if not self._cap.isOpened():
            raise RuntimeError(f'cannot open MJPEG URL: {url}')

        self._pub = self.create_publisher(Image, out_topic, 10)
        period = 1.0 / fps
        self.create_timer(period, self._tick)
        self.get_logger().info(f'{url} -> {out_topic} ({fps:.1f} Hz cap)')

    def _tick(self) -> None:
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        h, w = frame.shape[:2]
        msg.height = int(h)
        msg.width = int(w)
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = int(w * 3)
        msg.data = frame.tobytes()
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = MjpegSatCamBridge()
    except RuntimeError as e:
        print(e, file=sys.stderr)
        rclpy.shutdown()
        raise SystemExit(1) from e
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
