#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

import cv2 as cv
import numpy as np

PKG_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PKG_SRC))

from ap_ngps_ros2.trt_matcher import TrtMatcher  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke test TrtMatcher on Jetson")
    parser.add_argument("--engine", required=True, help="Path to .engine file")
    parser.add_argument("--kernel", required=True, help="Kernel BGR image path")
    parser.add_argument("--camera", required=True, help="Camera BGR image path")
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--width", type=int, default=384, help="Resize width (must match engine)")
    parser.add_argument("--height", type=int, default=216, help="Resize height (must match engine)")
    args = parser.parse_args()

    kernel = cv.imread(args.kernel)
    camera = cv.imread(args.camera)
    if kernel is None or camera is None:
        print("Failed to read input images", file=sys.stderr)
        return 1

    kernel = cv.resize(kernel, (args.width, args.height), interpolation=cv.INTER_AREA)
    camera = cv.resize(camera, (args.width, args.height), interpolation=cv.INTER_AREA)

    matcher = TrtMatcher(args.engine, match_threshold=args.threshold, warmup=True)
    k0, k1, stats = matcher.match(kernel, camera)
    print(
        f"matches={int(stats['num_matches'])} "
        f"mean_score={stats['mean_score']:.3f} "
        f"latency_ms={stats['latency_ms']:.1f}"
    )
    return 0 if stats["num_matches"] > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
