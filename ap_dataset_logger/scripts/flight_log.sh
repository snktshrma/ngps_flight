#!/usr/bin/env bash
# Field wrapper for flight data logging. Run on the Jetson host (over SSH is fine).
#
#   ./flight_log.sh start --fc serial:/dev/ttyTHS1:921600                 # DDS (default)
#   ./flight_log.sh start --link mavlink --fc serial:/dev/ttyTHS1:921600  # MAVLink fallback
#   ./flight_log.sh check                                      # go / no-go before takeoff
#   ./flight_log.sh stop                                       # finalise the bag cleanly
#
#   ./flight_log.sh attach     # watch the live output (detach again: Ctrl-A then D)
#   ./flight_log.sh status     # is it running?
#   ./flight_log.sh view       # live camera preview on the Jetson's monitor
#
# start options:
#   --link L       dds | mavlink   how the flight controller's data arrives (default: dds)
#   --fc SPEC      serial:<device>:<baud> | udp:<port> | tcp:<host>:<port> | none
#   --agent SPEC   older spelling of --link dds --fc SPEC
#
#   The flight controller port must speak the same protocol: SERIALx_PROTOCOL = 45 for
#   DDS, 2 for MAVLink. Switching means changing that parameter and rebooting the FC.
#   --name NAME    bag name prefix (default: flight)
#   --bag-dir DIR  where to write (default: ~/ngps_ws/src/ngps_flight/bags)
#   --archive      also keep the camera's untouched H264 stream (.mkv, ~0.7 GB/h)
#   --fusion FILE  run live VO+NGPS (vo_ngps_fusion_node) with this parameter file and
#                  send it to ArduPilot on /ap/tf. Preferred over --ngps.
#   --ngps FILE    also run NGPS localization live with this parameter file. Its
#                  outputs are recorded, and with VISO_TYPE set the flight controller
#                  logs what it receives on /ap/tf as VISP.
#   --check-timeout S  how long start keeps re-running `check` before giving up (default 120)
#   --no-check     skip that (bench use only: nothing then stops a NO-GO setup flying)
#
#   start validates the --ngps/--fusion file before launching anything, then runs
#   `check` until it is GO; if it is still NO-GO at the timeout -- or start fails in any
#   other way -- it stops everything it started, so nothing is left publishing /ap/tf.
#   A stopped vps-jp7 container (it does not come back after a power cycle) is started.
#
# Everything runs inside a detached `screen` session, so a dropped SSH or Wi-Fi link
# does NOT stop the recording. Always finish with `stop`: it sends Ctrl-C so rosbag2
# writes metadata.yaml. If power is cut instead, the .mcap is still intact; repair it
# with `ros2 bag reindex <bag_dir> -s mcap`.

set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${NGPS_CONTAINER:-vps-jp7}"
SESSION=ngps_log
MIN_FREE_GB=8
export DBX_CONTAINER_MANAGER="${DBX_CONTAINER_MANAGER:-docker}"

BAG_DIR="$HOME/ngps_ws/src/ngps_flight/bags"
NAME=flight
LINK=dds
FC=none
ARCHIVE=false
NGPS=
FUSION=
NODE_CONFIG=        # the --ngps or --fusion file, so `check` can check the live node
CHECK=true
CHECK_TIMEOUT=120

# `start` records where it is writing, so `check` and `stop` look at the right bag
# without being told again.
STATE="$HOME/.ngps_flight_log"
# shellcheck source=/dev/null
[ -f "$STATE" ] && . "$STATE"

running() { screen -list 2>/dev/null | grep -qE "[0-9]+\.${SESSION}[[:space:]]"; }
FUSION_SESSION=vo_ngps
fusion_running() { screen -list 2>/dev/null | grep -qE "[0-9]+\.${FUSION_SESSION}[[:space:]]"; }

# Everything this script starts inside the container. A session that dies can leave
# these behind, and a leftover publisher on /ap/tf interleaves its data with the live
# one -- which is exactly what corrupted flight 3's VISP log.
# The container shares the host's process table, so pgrep sees every process on the
# Jetson, including shells and editors whose command line merely mentions a node name
# (another agent's `grep vo_ngps_fusion_node`, the caller's own shell, ...). So each
# pattern is anchored to the program that actually runs: python3 <path>/<node>.py,
# bash <path>/run_fusion.sh, micro_ros_agent, python3 <path>/ros2 bag record|run|launch.
# Bracketed letters additionally stop the pattern matching its own text.
# test/test_stack_re.py pins what this does and does not match.
STACK_RE='^([^ ]*/)?python3(\.[0-9]+)? [^ ]*([r]tsp_camera_node|[n]gps_localization_node|[v]o_ngps_fusion_node)\.py( |$)|^([^ ]*/)?bash [^ ]*[r]un_fusion\.sh( |$)|^([^ ]*/)?[m]icro_ros_agent( |$)|^([^ ]*/)?python3(\.[0-9]+)? [^ ]*/ros2 ([b]ag record|[r]un micro_ros_agent|[l]aunch ap_dataset_logger)( |$)'
# The subset that publishes /ap/tf, stopped first.
FUSION_RE='^([^ ]*/)?python3(\.[0-9]+)? [^ ]*[v]o_ngps_fusion_node\.py( |$)|^([^ ]*/)?bash [^ ]*[r]un_fusion\.sh( |$)'
ancestors() { local p=$$; while [ "${p:-1}" -gt 1 ]; do echo "$p"; p=$(awk '{print $4}' "/proc/$p/stat" 2>/dev/null || echo 1); done; }
stack_pids() {
  local mine; mine=" $(ancestors | tr '\n' ' ') "
  for pid in $(docker exec "$CONTAINER" pgrep -f "${1:-$STACK_RE}" 2>/dev/null); do
    case "$mine" in *" $pid "*) ;; *) printf '%s ' "$pid" ;; esac   # never our own caller
  done
}
cleanup_orphans() {   # cleanup_orphans LABEL [PATTERN]
  # A stopped container has no processes left to clean up.
  [ "$(container_state)" = running ] || return 0
  local re="${2:-$STACK_RE}"
  local pids; pids=$(stack_pids "$re")
  [ -z "${pids// /}" ] && return 0
  case "$1" in
    start) echo "start: killing leftover processes from an earlier session:" ;;
    *)     echo "$1: stopping:" ;;
  esac
  local list; list=$(tr ' ' ',' <<<"${pids% }")
  docker exec "$CONTAINER" ps -o pid,args -p "$list" 2>/dev/null | cut -c1-110 | sed 's/^/    /' || true
  # shellcheck disable=SC2086  # $pids is a list of numbers, split on purpose
  docker exec "$CONTAINER" kill $pids 2>/dev/null || true
  sleep 3
  pids=$(stack_pids "$re")
  # shellcheck disable=SC2086
  if [ -n "${pids// /}" ]; then docker exec "$CONTAINER" kill -9 $pids 2>/dev/null || true; fi
  return 0
}
enter()   { distrobox enter "$CONTAINER" -- bash "$HERE/flight_log_inner.sh" "$@"; }

# The container has no restart policy, so it is stopped after every power cycle of
# the Jetson (exit 143 = SIGTERM at shutdown). Start it rather than fail obscurely
# later; never recreate it.
container_state() { docker inspect -f '{{.State.Status}}' "$CONTAINER" 2>/dev/null || echo missing; }
ensure_container() {
  local state policy
  state=$(container_state)
  [ "$state" = running ] && return 0
  if [ "$state" = missing ]; then
    echo "container '$CONTAINER' does not exist (see docker ps -a); not creating it here." >&2
    return 1
  fi
  policy=$(docker inspect -f '{{.HostConfig.RestartPolicy.Name}}' "$CONTAINER" 2>/dev/null || echo '?')
  echo "container '$CONTAINER' was $state (restart policy '$policy'); starting it ..."
  docker start "$CONTAINER" >/dev/null || { echo "docker start $CONTAINER failed" >&2; return 1; }
  for _ in $(seq 1 30); do
    if docker exec "$CONTAINER" true 2>/dev/null; then echo "container '$CONTAINER' running"; return 0; fi
    sleep 1
  done
  echo "container '$CONTAINER' did not come up -- docker logs $CONTAINER" >&2
  return 1
}

# Go / no-go: the preflight probe, plus the live node's checks when one is configured.
# Exit 0 GO, 2 GO with warnings, 1 NO-GO.
run_check() {
  local state extra=()
  state=$(container_state)
  if [ "$state" != running ]; then
    echo "  XX container '$CONTAINER'  NO-GO  $state -- nothing is running (power cycle?). Run start."
    return 1
  fi
  if [ -n "$NODE_CONFIG" ]; then extra=(--node-config "$NODE_CONFIG"); fi
  local rc=0
  enter check --bag-dir "$BAG_DIR" --link "$LINK" "${extra[@]}" "$@" || rc=$?
  return "$rc"
}

# Stop everything start brings up: VO/NGPS first (it publishes /ap/tf), then Ctrl-C the
# launch so rosbag2 writes metadata.yaml, then anything left over.
stop_all() {
  # `docker exec` does not pass the screen's hangup on, so the VO/NGPS node outlives
  # its session: stop it explicitly, now, rather than leave it on /ap/tf.
  if fusion_running; then screen -S "$FUSION_SESSION" -X quit || true; fi
  cleanup_orphans "$1 VO/NGPS" "$FUSION_RE"
  if running; then
    echo "stopping (Ctrl-C to ros2 launch; rosbag2 needs a moment to write metadata)..."
    screen -S "$SESSION" -X stuff $'\003'
    for _ in $(seq 1 30); do running || break; sleep 1; done
    if running; then
      echo "still up after 30 s; sending a second Ctrl-C"
      screen -S "$SESSION" -X stuff $'\003'
      for _ in $(seq 1 15); do running || break; sleep 1; done
    fi
    if running; then
      echo "forcing the session closed"; screen -S "$SESSION" -X quit || true
    fi
  fi
  cleanup_orphans "$1"
}

START_OK=false
start_failed() {
  local rc=$?
  [ "$START_OK" = true ] && return
  set +e   # tear down as much as possible, whatever fails along the way
  trap - EXIT
  echo >&2
  echo "start FAILED (exit $rc): stopping everything it started, so nothing is left on /ap/tf" >&2
  stop_all start-failed
  exit "$rc"
}

cmd="${1:-}"; shift || true
case "$cmd" in
  start)
    LINK=dds; FC=none
    while [ $# -gt 0 ]; do
      # Every option but --archive/--no-check takes a value; `--ngps` with nothing after
      # it (or an empty "$VAR") must fail here, not launch with ngps:= and no node.
      case "$1" in
        --archive|--no-check) ;;
        *) if [ $# -lt 2 ] || [ -z "$2" ]; then echo "start: $1 needs a value" >&2; exit 1; fi ;;
      esac
      case "$1" in
        --link)    LINK="$2"; shift ;;
        --fc)      FC="$2"; shift ;;
        --agent)   LINK=dds; FC="$2"; shift ;;
        --name)    NAME="$2"; shift ;;
        --bag-dir) BAG_DIR="$2"; shift ;;
        --archive) ARCHIVE=true ;;
        --ngps)    NGPS="$2"; shift ;;
        --fusion)  FUSION="$2"; shift ;;
        --no-check) CHECK=false ;;
        --check-timeout) CHECK_TIMEOUT="$2"; shift ;;
        *) echo "unknown start option: $1" >&2; exit 1 ;;
      esac
      shift
    done
    # Everything that can be refused is refused before anything is started.
    if [ -n "$NGPS" ] && [ -n "$FUSION" ]; then
      echo "--ngps and --fusion both publish /ap/tf; pick one." >&2; exit 1
    fi
    case "$LINK" in
      dds|mavlink) ;;
      *) echo "--link must be dds or mavlink, got '$LINK'" >&2; exit 1 ;;
    esac
    case "$CHECK_TIMEOUT" in ''|*[!0-9]*) echo "--check-timeout must be whole seconds" >&2; exit 1 ;; esac
    case "$NAME" in *[!A-Za-z0-9_.-]*) echo "--name: letters, digits, _ . - only" >&2; exit 1 ;; esac
    for f in "$NGPS" "$FUSION"; do
      if [ -n "$f" ] && [ ! -f "$f" ]; then echo "parameter file not found: $f" >&2; exit 1; fi
    done
    if [ -n "$NGPS" ]; then NGPS=$(readlink -f "$NGPS"); fi
    if [ -n "$FUSION" ]; then FUSION=$(readlink -f "$FUSION"); fi
    NODE_CONFIG="${FUSION:-$NGPS}"
    if running || fusion_running; then
      echo "already running (session '$SESSION'). './flight_log.sh stop' first." >&2; exit 1
    fi
    mkdir -p "$BAG_DIR"
    free_gb=$(df -BG --output=avail "$BAG_DIR" | tail -n1 | tr -dc '0-9')
    if [ "${free_gb:-0}" -lt "$MIN_FREE_GB" ]; then
      echo "only ${free_gb} GB free in $BAG_DIR (need ${MIN_FREE_GB})." >&2; exit 1
    fi
    ensure_container
    cleanup_orphans start
    if [ -n "$NODE_CONFIG" ]; then
      echo "checking $NODE_CONFIG ..."
      if ! enter validate "$NODE_CONFIG"; then
        echo "REFUSING to start: fix the errors above in $NODE_CONFIG" >&2; exit 1
      fi
    fi
    if [ "$FC" = none ]; then
      echo "NOTE: --fc not given, so nothing connects to the flight controller here."
      echo "      Flight-controller topics will only appear if a link is running elsewhere."
    fi

    # From here on a failure or Ctrl-C stops whatever was started.
    trap start_failed EXIT
    trap 'exit 130' INT TERM HUP
    LOG="$BAG_DIR/${NAME}_$(date +%Y%m%d_%H%M%S).log"
    launch_args=("link:=$LINK" "fc:=$FC" "bag_dir:=$BAG_DIR" "bag_name:=$NAME" "archive_h265:=$ARCHIVE")
    if [ -n "$NGPS" ]; then launch_args+=("ngps:=$NGPS"); fi
    # argv, not a bash -c string: paths are passed through untouched, however quoted.
    screen -dmS "$SESSION" -L -Logfile "$LOG" \
      distrobox enter "$CONTAINER" -- bash "$HERE/flight_log_inner.sh" launch "${launch_args[@]}"
    sleep 3
    if ! running; then
      echo "failed to start -- see $LOG" >&2; tail -20 "$LOG" >&2 || true; exit 1
    fi
    if [ -n "$FUSION" ]; then
      FLOG="${LOG%.log}_fusion.log"
      screen -dmS "$FUSION_SESSION" -L -Logfile "$FLOG" \
        docker exec -u "$(id -un)" -e HOME="$HOME" "$CONTAINER" \
        bash "$HOME/ngps_ws/src/ngps_flight/ap_ngps_ros2/scripts/run_fusion.sh" "$FUSION"
      echo "          VO+NGPS live: $FUSION   log: $FLOG"
    fi
    printf 'BAG_DIR=%q\nNAME=%q\nLOG=%q\nLINK=%q\nNODE_CONFIG=%q\n' \
      "$BAG_DIR" "$NAME" "$LOG" "$LINK" "$NODE_CONFIG" > "$STATE"
    echo "started   session: $SESSION   link: $LINK ($FC)   log: $LOG"
    [ -n "$NGPS" ] && echo "          NGPS live: $NGPS"
    echo "          free disk: ${free_gb} GB (~$(( free_gb * 10 / 30 / 10 )).$(( free_gb * 10 / 30 % 10 )) h at ~30 GB/h, daylight q85)"
    echo
    if [ "$CHECK" = true ]; then
      # MAVROS downloads every FC parameter on connect; ArduPilot throttles all streams
      # during that burst and takes about a minute to return to full rate.
      settle=15; [ "$LINK" = mavlink ] && settle=60
      echo "waiting ${settle} s for topics to settle, then checking (up to ${CHECK_TIMEOUT} s) ..."
      sleep "$settle"
      deadline=$(( SECONDS + CHECK_TIMEOUT ))
      while true; do
        rc=0; run_check || rc=$?
        [ "$rc" != 1 ] && break
        if [ "$SECONDS" -ge "$deadline" ]; then
          echo "still NO-GO after ${CHECK_TIMEOUT} s -- not leaving this running." >&2
          exit 1   # -> start_failed tears it all down
        fi
        echo "NO-GO; re-checking in 5 s (gives up at ${CHECK_TIMEOUT} s; Ctrl-C stops everything) ..."
        sleep 5
      done
      if [ "$rc" = 2 ]; then echo "started: GO WITH WARNINGS -- read them above before takeoff."; else echo "started: GO"; fi
    else
      echo "WARNING: --no-check: nothing has verified this setup. Run   $0 check   before takeoff."
    fi
    START_OK=true
    trap - EXIT INT TERM HUP
    ;;

  check)
    rc=0; run_check "$@" || rc=$?
    exit "$rc"
    ;;

  stop)
    state=$(container_state)
    if [ "$state" != running ]; then
      echo "container '$CONTAINER' is $state: nothing inside it is running."
      if fusion_running; then screen -S "$FUSION_SESSION" -X quit || true; fi
      if running; then screen -S "$SESSION" -X quit || true; fi
      exit 0
    fi
    if ! running && ! fusion_running; then echo "not running."; cleanup_orphans stop; exit 0; fi
    stop_all stop
    # shellcheck disable=SC2012  # NAME is restricted to [A-Za-z0-9_.-]
    bag=$(ls -dt "$BAG_DIR"/"$NAME"_*/ 2>/dev/null | head -1 || true)
    if [ -z "$bag" ]; then
      echo "could not find a '${NAME}_*' bag under $BAG_DIR"
    else
      echo "bag: $bag  ($(du -sh "$bag" | cut -f1))"
      if [ -f "$bag/metadata.yaml" ]; then
        echo "     metadata.yaml written -- closed cleanly"
      else
        echo "     NO metadata.yaml -- repair with: ros2 bag reindex $bag -s mcap"
      fi
    fi
    ;;

  attach) exec screen -r "$SESSION" ;;
  status)
    echo "container: $CONTAINER $(container_state)"
    if running; then echo "running"; screen -list | grep -E "\.($SESSION|$FUSION_SESSION)[[:space:]]"; else echo "not running"; fi ;;
  view)   exec "$HERE/view_camera.sh" "$@" ;;
  *)      sed -n '2,39p' "$0"; exit 1 ;;
esac
