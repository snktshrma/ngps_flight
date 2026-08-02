#!/usr/bin/env bash
# Terminal 4: NGPS localization (inside distrobox).
# Full stack: LAUNCH=unified_localization_simple.launch.py ./run_ngps.sh
set -euo pipefail

LAUNCH="${LAUNCH:-ngps_localization.launch.py}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/_distrobox_ros.sh" bash -c \
  "exec ros2 launch ap_ngps_ros2 ${LAUNCH}"
