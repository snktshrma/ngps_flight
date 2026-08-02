#!/usr/bin/env bash
# Run a command inside distrobox with ROS Jazzy + ngps_ws sourced.
# Usage: _distrobox_ros.sh <command...>
set -euo pipefail

DISTROBOX_NAME="${DISTROBOX_NAME:-vps-jp7}"

if [[ $# -eq 0 ]]; then
  echo "Usage: $0 <command...>" >&2
  exit 1
fi

# Distrobox drops multiline bash -c strings; keep this on one line.
distrobox enter "${DISTROBOX_NAME}" -- bash -lc \
  "set -e; source /opt/ros/jazzy/setup.bash; source ~/ngps_ws/install/setup.bash; $(printf '%q ' "$@")"
