#!/usr/bin/env bash
# Live preview of the gimbal stream, tuned for the lowest latency the link allows.
# Wave a hand in front of the camera and watch the screen to judge it by eye.
#
#   ./view_camera.sh                 # lowest-latency preview
#   ./view_camera.sh --fps           # + measured render rate in the title bar
#   ./view_camera.sh --tcp           # if UDP is being dropped by the network
#   ./view_camera.sh --buffered      # what it looks like WITHOUT the latency fixes
#
# Run this on the Jetson host, not in the container: the host has the full L4T
# GStreamer stack, and the preview needs no ROS at all.
#
# Over SSH, point it at the physical monitor's X session:
#   DISPLAY=:1 ./view_camera.sh
#
# For a number rather than an impression, use measure_camera_latency.py, which strobes
# the screen at known times and reports the delay.

set -euo pipefail

URL="${NGPS_RTSP_URL:-rtsp://192.168.144.25:8554/video1}"
SINK="${NGPS_SINK:-nv3dsink}"
PROTO=udp
FPS=0
BUFFERED=0

while [ $# -gt 0 ]; do
  case "$1" in
    --tcp)      PROTO=tcp ;;
    --fps)      FPS=1 ;;
    --buffered) BUFFERED=1 ;;
    --url)      URL="$2"; shift ;;
    --sink)     SINK="$2"; shift ;;
    -h|--help)  sed -n '2,20p' "$0"; exit 0 ;;
    *)          echo "unknown option: $1" >&2; exit 1 ;;
  esac
  shift
done

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "No DISPLAY set. If you are on SSH and the Jetson has a monitor attached," >&2
  echo "find the session with 'who' and prefix the command, e.g. DISPLAY=:1 $0" >&2
  exit 1
fi

# Ask the server what it serves rather than trusting the path name: the ArduPilot ZR10
# firmware serves H264 from /video1, while older firmware served H265 from /main.264.
CODEC=$(printf 'DESCRIBE %s RTSP/1.0\r\nCSeq: 1\r\nAccept: application/sdp\r\n\r\n' "$URL" \
  | timeout 4 nc -w 3 "$(echo "$URL" | sed -E 's#rtsp://([^:/]+).*#\1#')" \
      "$(echo "$URL" | sed -E 's#rtsp://[^:]+:([0-9]+).*#\1#')" 2>/dev/null \
  | grep -oiE 'H26[45]' | head -1 | tr '[:upper:]' '[:lower:]')
case "${CODEC:-h264}" in
  h265) DEPAY="rtph265depay ! h265parse" ;;
  *)    DEPAY="rtph264depay ! h264parse" ;;
esac
echo "codec: ${CODEC:-h264 (assumed)}"

if [ "$BUFFERED" = 1 ]; then
  # rtspsrc's default 2000 ms jitter buffer, and a sink that honours timestamps. This is
  # what "RTSP is laggy" normally means -- run it once to see the difference.
  SRC="rtspsrc location=$URL protocols=$PROTO"
  SYNC="sync=true"
  LEAKY=""
  echo "buffered mode: default 2000 ms jitter buffer + clock sync (deliberately slow)"
else
  # latency=0 disables the jitter buffer, drop-on-latency throws away anything late,
  # disable-dpb skips decoder reordering delay (safe: no B-frames in this stream), and
  # sync=false renders each frame on arrival instead of holding it to its timestamp.
  SRC="rtspsrc location=$URL latency=0 drop-on-latency=true protocols=$PROTO do-retransmission=false"
  SYNC="sync=false"
  # The one that actually mattered. nv3dsink holds on to the decoder's output buffers,
  # so a ~13-frame queue builds up inside the decoder while the window initialises --
  # and since the sink then consumes at exactly the incoming rate, it never drains.
  # Measured: 422 ms packet-to-sink without this, 10.7 ms with it. It must sit directly
  # after the decoder: anything with its own buffer pool in between (nvvidconv feeding
  # glimagesink, say) lets the backlog form upstream of the queue regardless.
  LEAKY="queue max-size-buffers=1 leaky=downstream ! "
fi

if [ "$FPS" = 1 ]; then
  # fpsdisplaysink draws its overlay in system memory and cannot negotiate NVMM, so the
  # frame is copied out with nvvidconv first and shown through xvimagesink. Measured at
  # 14.7 ms packet-to-sink, a few ms more than the default path.
  VIDEOSINK="nvvidconv ! video/x-raw,format=I420 ! fpsdisplaysink video-sink=xvimagesink $SYNC text-overlay=true"
else
  VIDEOSINK="$SINK $SYNC"
fi

echo "url:   $URL  ($PROTO)"
echo "sink:  $SINK   display: ${DISPLAY:-$WAYLAND_DISPLAY}"
echo "Ctrl-C to stop."

# shellcheck disable=SC2086
exec gst-launch-1.0 $SRC \
  ! $DEPAY \
  ! nvv4l2decoder disable-dpb=true \
  ! $LEAKY$VIDEOSINK
