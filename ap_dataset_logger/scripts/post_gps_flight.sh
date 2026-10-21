#!/usr/bin/env bash
# After a GPS flight at a new site: FC log verdict + every map compared, raw / terrain / terrain+canopy.
#   ap_dataset_logger/scripts/post_gps_flight.sh <flight-name> [site]      e.g. flight9 nagano1
# Runs ~10-20 min; safe to run detached: nohup ... &  Output: bags/<flight-name>_post.log
NAME=${1:?flight name, e.g. flight9}
SITE=${2:-nagano1}
WS=/home/rmackay9/ngps_ws
R=$WS/src/ngps_flight
OUT=$R/bags/${NAME}_post.log
exec > >(tee "$OUT") 2>&1
echo "=== $(date +%T) FC log"
(cd $R/ap_dataset_logger/scripts && python3 fc_log_check.py --download --name "$NAME")
BAG=$(ls -td $R/bags/*/ | while read d; do [ -f "$d/metadata.yaml" ] && ls "$d"/*.mcap >/dev/null 2>&1 && echo "$d" && break; done)
BAG=${BAG%/}
echo "=== $(date +%T) map compare on $BAG"
MAPS=$(cd $R/ngps_config/tiff && ls -d ${SITE} ${SITE}_* 2>/dev/null | tr '\n' ' ')
DEMS=$(ls $R/ngps_config/terrain/$SITE/dem.npz $R/ngps_config/terrain/$SITE/surface.npz 2>/dev/null | tr '\n' ' ')
docker start vps-jp7 >/dev/null
docker exec vps-jp7 bash -c "source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash; source $WS/install-jazzy/setup.bash; \
  cd $R && python3 ngps_config/tools/map_compare.py --bag '$BAG' --maps $MAPS --dem $DEMS --period 1.0 --min-alt 20" \
  2>&1 | grep -v 'TRT\] \[W'
echo "=== $(date +%T) DONE"
