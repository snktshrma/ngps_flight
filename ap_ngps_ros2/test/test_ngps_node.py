#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np


class NGPSTestNode(Node):
    
    def __init__(self):
        super().__init__('ngps_test_node')
        
        self.bridge = CvBridge()
        self.pose_cnt = 0
        self.pos_cnt = 0
        self.rot_cnt = 0
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ngps/pose',
            self.pose_callback,
            10
        )
        
        self.position_sub = self.create_subscription(
            PointStamped,
            '/ngps/position',
            self.position_callback,
            10
        )
        
        self.rotation_sub = self.create_subscription(
            Float64,
            '/ngps/rotation',
            self.rotation_callback,
            10
        )
        
        self.debug_image_sub = self.create_subscription(
            Image,
            '/ngps/debug_image',
            self.debug_image_callback,
            10
        )
        
        print('NGPS Test Node started')
    
    def pose_callback(self, msg):
        self.pose_cnt += 1
        ts = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        if self.pose_cnt % 5 == 0:
            print(f'pose #{self.pose_cnt}: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
    
    def position_callback(self, msg):
        self.pos_cnt += 1
        ts = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        print(f'pos #{self.pos_cnt}: x={msg.point.x:.2f}, y={msg.point.y:.2f}')
    
    def rotation_callback(self, msg):
        self.rot_cnt += 1
        if self.rot_cnt % 7 == 0:
            print(f'rot #{self.rot_cnt}: {msg.data:.2f} deg')
    
    def debug_image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    
    node = NGPSTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
