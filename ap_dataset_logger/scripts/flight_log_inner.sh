#!/usr/bin/env bash
# Runs inside the vps-jp7 container; called by flight_log.sh. Not meant to be run by hand.
set -eo pipefail   # no -u: the ROS setup scripts reference unset variables

WS="${NGPS_WS:-$HOME/ngps_ws}"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# Both overlays are needed, in this order. install/ holds micro_ros_agent and
# ardupilot_msgs: without it there is no agent (so no /ap/* at all) and /ap/status and
# /ap/rc cannot be recorded. install-jazzy/ holds this logger and is sourced last so its
# newer ap_ngps_ros2 wins over the older copy in install/.
for overlay in "$WS/install" "$WS/install-jazzy"; do
  # shellcheck disable=SC1091
  [ -f "$overlay/setup.bash" ] && source "$overlay/setup.bash"
done

mode="${1:-}"; shift || true
case "$mode" in
  launch) exec ros2 launch ap_dataset_logger dataset.launch.py "$@" ;;
  check)  exec ros2 run ap_dataset_logger preflight_check.py "$@" ;;
  validate) exec python3 -m ap_ngps_ros2.config_check "$@" ;;
  *)      echo "flight_log_inner.sh: unknown mode '$mode'" >&2; exit 1 ;;
esac
