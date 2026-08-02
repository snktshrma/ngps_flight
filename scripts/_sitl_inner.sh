#!/usr/bin/env bash
# Runs inside the container: micro-ROS agent + ArduPilot SITL.
# Kept as a real script rather than an inline `bash -c` string, because the distrobox
# wrapper shell-escapes its command and $-expansions would not survive.
#
# Usage: _sitl_inner.sh <location> [extra sim_vehicle.py args...]
set -uo pipefail

LOCATION="${1:-OSRF0}"
shift || true

ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019 &
MICRO_PID=$!
trap 'kill "${MICRO_PID}" 2>/dev/null || true' EXIT INT TERM

cd ~/ngps_ws/src/ardupilot
exec ./Tools/autotest/sim_vehicle.py -v ArduCopter --enable-DDS --location "${LOCATION}" "$@"
