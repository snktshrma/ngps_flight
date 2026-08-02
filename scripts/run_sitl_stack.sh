#!/usr/bin/env bash
# Terminal 1: micro-ROS agent + ArduPilot SITL (inside distrobox).
#
# SITL_LOCATION    start location (default OSRF0)
# SITL_EXTRA_ARGS  extra sim_vehicle.py flags, e.g. debug build under gdb:
#                    SITL_EXTRA_ARGS=-DG ./run_sitl_stack.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Values are expanded here, in the host shell: the distrobox wrapper escapes the command
# it forwards, so anything left for the container shell to expand arrives as literal text.
exec "${SCRIPT_DIR}/_distrobox_ros.sh" \
  "${SCRIPT_DIR}/_sitl_inner.sh" "${SITL_LOCATION:-OSRF0}" ${SITL_EXTRA_ARGS:-}
