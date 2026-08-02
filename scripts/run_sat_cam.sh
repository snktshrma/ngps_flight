#!/usr/bin/env bash
# Terminal 2: sat cam (inside distrobox). Waits for /ap/navsat before starting.
set -euo pipefail

SAT_CAM_PORT="${SAT_CAM_PORT:-14550}"
SAT_CAM_RADIUS_M="${SAT_CAM_RADIUS_M:-200}"
SAT_CAM_ZOOM="${SAT_CAM_ZOOM:-20}"
SAT_CAM_HTTP_PORT="${SAT_CAM_HTTP_PORT:-8090}"
SAT_CAM_SIZE="${SAT_CAM_SIZE:-640x360}"
GPS_WAIT_SEC="${GPS_WAIT_SEC:-120}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/_distrobox_ros.sh" bash -c \
  "echo 'Waiting for GPS on /ap/navsat (timeout ${GPS_WAIT_SEC}s)...'; timeout ${GPS_WAIT_SEC} ros2 topic echo /ap/navsat --once; cd ~/ngps_ws/src/ardupilot; exec python3 ./Tools/autotest/sat_cam_emulator.py --port ${SAT_CAM_PORT} --airfield-radius-m ${SAT_CAM_RADIUS_M} --airfield-zoom ${SAT_CAM_ZOOM} --http-mjpeg-port ${SAT_CAM_HTTP_PORT} --no-hud --ros --no-display --ros-compressed --ros-size ${SAT_CAM_SIZE} --pose-source sim"
