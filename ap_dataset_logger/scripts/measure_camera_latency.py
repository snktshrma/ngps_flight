#!/usr/bin/env python3
"""Measure the constant glass-to-timestamp latency the clock fit cannot see.

    ros2 run ap_dataset_logger measure_camera_latency.py

Fitting the camera's RTCP clock to host time removes jitter and drift, but it cannot
tell whether the camera stamps a frame at the moment of exposure or some fixed interval
afterwards. That residual is a constant, and the only way to observe it is to show the
camera an event whose host time you already know.

This strobes the screen between black and white at recorded times, watches the mean
brightness of the incoming frames, and reports how long after each flip the change
showed up. Point the gimbal at the monitor, fill its view with the window, and run it.

The number it prints goes into ``latency_offset_ms`` in dataset_logger.yaml.

Needs a display. Over SSH without one, use --manual: it saves a frame and prints its
timestamp, so you can point the camera at a phone stopwatch and subtract by hand.
"""

import argparse
import os
import random
import statistics
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

try:
    import cv2
except ImportError:
    cv2 = None


class LatencyProbe(Node):
    def __init__(self, topic):
        super().__init__('measure_camera_latency')
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.frames = []          # (header_stamp_s, mean_luminance)
        self.create_subscription(CompressedImage, topic, self._cb, qos)

    def _cb(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        # 1/8-scale greyscale decode: we only need mean brightness, and this keeps the
        # callback far cheaper than the frame interval.
        img = cv2.imdecode(buf, cv2.IMREAD_REDUCED_GRAYSCALE_8)
        if img is None:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.frames.append((stamp, float(img.mean())))


def find_response(frames, flip_t, window=1.0):
    """First frame after ``flip_t`` whose brightness jumps decisively."""
    seg = [(t, v) for t, v in frames if flip_t - 0.2 <= t <= flip_t + window]
    if len(seg) < 6:
        return None
    before = [v for t, v in seg if t < flip_t]
    after = [v for t, v in seg if t >= flip_t]
    if len(before) < 2 or len(after) < 3:
        return None
    base = statistics.median(before)
    swing = max(abs(v - base) for v in after)
    if swing < 12.0:          # screen change must dominate ordinary scene noise
        return None
    thresh = base + 0.5 * swing * (1 if max(after) - base > base - min(after) else -1)
    for t, v in seg:
        if t < flip_t:
            continue
        if (thresh > base and v >= thresh) or (thresh < base and v <= thresh):
            return t - flip_t
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--topic', default='/camera/image_raw/compressed')
    ap.add_argument('--flips', type=int, default=12)
    ap.add_argument('--manual', action='store_true',
                    help='no display: save one frame and print its timestamp')
    args = ap.parse_args()

    if cv2 is None:
        print('opencv is required', file=sys.stderr)
        return 1

    rclpy.init()
    node = LatencyProbe(args.topic)

    def spin(seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    print(f'waiting for frames on {args.topic} ...')
    spin(5.0)
    if not node.frames:
        print(f'no frames on {args.topic}. Is rtsp_camera_node running?', file=sys.stderr)
        node.destroy_node(); rclpy.shutdown()
        return 1
    print(f'receiving frames ({len(node.frames)} in 5 s)\n')

    if args.manual:
        stamp, _ = node.frames[-1]
        spin(0.3)
        stamp, _ = node.frames[-1]
        print('Point the camera at a phone stopwatch, then compare:')
        print(f'  frame header stamp : {stamp:.3f}  ({time.strftime("%H:%M:%S", time.localtime(stamp))}.'
              f'{int((stamp % 1) * 1000):03d})')
        print(f'  host time now      : {time.time():.3f}')
        print('\nlatency_offset_ms = (stopwatch reading in the image) - (header stamp)')
        node.destroy_node(); rclpy.shutdown()
        return 0

    # Check for a display before touching cv2: with no DISPLAY, Qt calls abort() rather
    # than raising, so there is nothing to catch and the process dies outright.
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print('no display available (DISPLAY and WAYLAND_DISPLAY are both unset).\n'
              'The strobe method needs a screen the camera can see. '
              'Run with --manual instead.', file=sys.stderr)
        node.destroy_node(); rclpy.shutdown()
        return 1

    win = 'latency-strobe'
    try:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    except cv2.error as exc:
        print(f'cannot open a window ({exc}).\nRun with --manual instead.', file=sys.stderr)
        node.destroy_node(); rclpy.shutdown()
        return 1

    black = np.zeros((720, 1280), np.uint8)
    white = np.full((720, 1280), 255, np.uint8)
    print('Fill the camera view with the strobe window. Measuring ...')

    offsets = []
    state = False
    for i in range(args.flips):
        spin(0.8 + random.uniform(0.0, 0.5))     # de-correlate flips from the frame clock
        state = not state
        cv2.imshow(win, white if state else black)
        cv2.waitKey(1)
        flip_t = time.clock_gettime(time.CLOCK_REALTIME)
        spin(1.0)
        d = find_response(node.frames, flip_t)
        if d is not None:
            offsets.append(d)
            print(f'  flip {i + 1:>2}/{args.flips}: response after {d * 1e3:7.1f} ms')
        else:
            print(f'  flip {i + 1:>2}/{args.flips}: no clear response (is the screen filling the view?)')

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

    if len(offsets) < 3:
        print('\nToo few clean responses to trust. Fill more of the frame with the '
              'window, dim the room, and retry.', file=sys.stderr)
        return 1

    med = statistics.median(offsets)
    # A frame can only reveal a flip at its own capture instant, so the measurement is
    # quantised by the frame interval; half of it is the expected bias.
    print(f'\n  samples      : {len(offsets)}')
    print(f'  median       : {med * 1e3:.1f} ms')
    print(f'  spread       : {min(offsets) * 1e3:.1f} .. {max(offsets) * 1e3:.1f} ms')
    print(f'  stdev        : {statistics.pstdev(offsets) * 1e3:.1f} ms')
    corrected = med - (1.0 / 60.0)
    print(f'\nSubtracting half a frame interval for sampling bias:')
    print(f'  latency_offset_ms: {corrected * 1e3:.1f}')
    print('\nPut that in config/dataset_logger.yaml. It shifts header stamps earlier by')
    print('that amount, so they land on the exposure rather than after it.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
