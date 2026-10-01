#!/usr/bin/env bash
# Live VO+NGPS to ArduPilot. Restarts itself so a crash cannot leave VisOdom unhealthy.
# Started by flight_log.sh start --fusion FILE, which validates FILE first.
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
source /home/rmackay9/ngps_ws/install/setup.bash
export PYTHONPATH=/home/rmackay9/ngps_ws/src/ngps_flight/ap_ngps_ros2/src:$PYTHONPATH
cd /home/rmackay9/ngps_ws/src/ngps_flight/ap_ngps_ros2 || exit 1
while true; do
  python3 src/ap_ngps_ros2/vo_ngps_fusion_node.py --ros-args --params-file "${1:-config/fusion_nagano0.yaml}"
  echo "fusion node exited ($?), restarting in 3 s"; sleep 3
done
