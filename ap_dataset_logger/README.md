# ap_dataset_logger

Flight dataset capture: low-latency RTSP camera bringup with jitter-free capture
timestamps, recorded to mcap alongside the ArduPilot topics, so NGPS can be replayed
against the flight afterwards.

## Flight day

All commands run on the Jetson host (SSH is fine) from `scripts/`.

**Before takeoff**

1. Power up with the gimbal and flight controller connected.
2. Set the gimbal to a fixed pitch straight down, in lock mode, at **1× zoom**. Do not
   touch zoom or gimbal during the flight: both change what the camera sees in ways
   neither NGPS nor VIO can undo.
3. Start logging. **This vehicle uses MAVLink**: the CubeOrange+ firmware (ArduPilot
   4.8.0) has no DDS compiled in, and it already serves MAVLink on TCP 5760 over its
   PPP link to the DroneNet (`NET_P2`). No flight-controller changes needed:
   ```bash
   ./flight_log.sh start --link mavlink --fc tcp:192.168.144.15:5760 --name flight1 --archive
   ```
   Wait ~60 s before `check`: MAVROS downloads every parameter on connect and ArduPilot
   throttles all streams to ~55% until that burst clears.
4. Wait for GPS lock, then `./flight_log.sh check` and look for **GO**. It checks the
   camera, clock sync, every ArduPilot topic, GPS fix, EKF origin, that the bag is
   actually growing, and disk space.
5. Sit still on the ground for ~10 s after arming: it gives VIO a clean start and the
   camera clock fit its warm-up.

**After landing**

6. `./flight_log.sh stop`, and confirm it says `metadata.yaml written`.
7. Keep the ArduCopter dataflash log (`.BIN`) from the same flight.

**If DDS won't work: MAVLink fallback**

Add `--link mavlink`; everything else is the same, and `check` knows which topics to expect:

```bash
./flight_log.sh start --link mavlink --fc serial:/dev/ttyTHS1:921600 --name flight1 --archive
```

The flight controller's port must speak MAVLink first: set that port's `SERIALx_PROTOCOL`
from **45 (DDS)** to **2 (MAVLink2)** in your GCS and **reboot the flight controller**, with
`SERIALx_BAUD` matching the baud in `--fc`. Set it back to 45 to return to DDS.

MAVLink mode runs MAVROS, which publishes `/mavros/*` instead of `/ap/*`, and
`mavlink_rates.py`, which asks for these rates on that link (ArduPilot's defaults on a
telemetry port are a few Hz). It uses `MAV_CMD_SET_MESSAGE_INTERVAL`, so nothing in the
flight controller's saved parameters changes:

| MAVROS topic | from | rate |
|---|---|---|
| `/mavros/imu/data`, `/imu/data_raw` | ATTITUDE, SCALED_IMU, RAW_IMU | 50 Hz |
| `/mavros/local_position/pose`, `/velocity_local` | LOCAL_POSITION_NED | 30 Hz |
| `/mavros/global_position/global` (EKF) | GLOBAL_POSITION_INT | 30 Hz |
| `/mavros/global_position/raw/fix` (GPS) | GPS_RAW_INT | 10 Hz |
| `/mavros/time_reference` (aligns with the dataflash log) | SYSTEM_TIME | 10 Hz |
| `/mavros/global_position/gp_origin` (EKF origin) | GPS_GLOBAL_ORIGIN | 1 Hz |

MAVROS stamps these on this machine's clock using MAVLink TIMESYNC, which is what lines
them up with the camera. Both modes were rehearsed against SITL and passed `check` with a
clean GO. Local position and velocity appear ~30 s after GPS lock, once the EKF has a
position relative to its origin; `check` says so if you run it too early.

Logging runs in a detached `screen`, so a dropped SSH or Wi-Fi link does **not** stop it.
`./flight_log.sh attach` shows the live output (Ctrl-A then D to leave it running).

**Things to know**

* **Disk: ~30 GB/hour** in daylight at JPEG q85 (280 KB/frame on a detailed scene; q75 is
  20 GB/h). With ~37 GB free that is about 1.2 h of recording. `check` reports hours left
  from the rate it actually measures.
* **Gimbal attitude** is recorded at 30 Hz (`/mavros/gimbal_control/device/attitude_status`).
  Use it to confirm the camera really points straight down: pitch should read about −90°.

* **IMU rate**: **200 Hz over MAVLink** on this vehicle (measured 192 Hz average over a
  full recording, including start-up), timestamped from the FC's own clock via TIMESYNC —
  the best VIO source available. The dataflash log only has **25 Hz** IMU with the current
  `LOG_BITMASK` (180222); adding the `IMU_FAST` bit (`LOG_BITMASK` = 442366) logs it at the
  400 Hz loop rate. DDS, where available, tops out near 110–120 Hz, not the 200 Hz
  ArduPilot is configured for —
  measured in SITL over local UDP, so it is the publisher's ceiling rather than the link.
  Over serial at 921600 baud expect less. For VIO, take IMU from the dataflash log and use
  the DDS copy for convenience.
* **Lining the bag up with the dataflash log**: `/ap/time` (DDS) or `/mavros/time_reference`
  (MAVLink) carries ArduPilot's GPS-derived UTC stamped with this machine's clock on
  arrival, which maps one timeline onto the other.
* **Camera timestamps are the frame's arrival at the Jetson**, currently with
  `latency_offset_ms: 0`. The camera's own delay (sensor to network, ~120–170 ms
  estimated) is a constant you can subtract afterwards once measured; recording without
  a guessed value keeps the raw data honest.
* **The camera drops to ~25 fps in low light** and runs 30 fps in daylight. The clock fit
  detects a mid-flight change and restarts within about half a second (`sync_rate_breaks`
  in `/diagnostics`); without that, a change produced 3.1 s of timestamp error in
  simulation.
* The launch loads **both** `~/ngps_ws/install` (micro-ROS agent, `ardupilot_msgs`) and
  `~/ngps_ws/install-jazzy` (this logger). With only the latter there is no agent — so no
  `/ap/*` at all — and `/ap/status`, `/ap/rc`, `/ap/airspeed` cannot be recorded.

## Quick start

```bash
ros2 launch ap_dataset_logger dataset.launch.py                    # camera + recording
ros2 launch ap_dataset_logger dataset.launch.py bag_dir:=/media/ssd/bags
ros2 launch ap_dataset_logger dataset.launch.py archive_h265:=true # + lossless archive
ros2 launch ap_dataset_logger dataset.launch.py record:=false      # camera only
```

Afterwards, before you trust the data:

```bash
ros2 run ap_dataset_logger check_dataset_sync.py <bag_dir>
```

## The camera

SIYI ZR10 at `192.168.144.25`, running the ArduPilot firmware (`zr10-web/2.0`, web UI on
port 80 behind Basic auth). The node runs an RTSP DESCRIBE at startup and believes the
SDP rather than the path name, because the path name has lied under both firmwares.

| | ArduPilot firmware (current) | older firmware |
|---|---|---|
| path | `/video1` (main), `/video2` (sub) | `/main.264` — now **404** |
| codec | **H264** | H265 |
| capture rate | 30.02 fps (was 24.95 — see below) | 30.18 fps |
| RTCP sender reports | **none** | on every frame |
| RTP timestamp step | 33.33 ms (as if 30 fps) | 33.33 ms (correct) |

Two things about the current firmware are worth knowing before you trust a dataset:

**It sends no RTCP sender reports.** Confirmed by asking the RTP session directly:
`have-sr=False` after 45 s and 7,387 packets, over UDP *and* TCP. So the `timestamp/x-ntp`
reference meta never appears, and the node falls back to the RTP header clock instead
(see below).

**Its RTP timestamps are a frame counter, not a clock.** The encoder adds a fixed
3000-tick step (33.33 ms) per frame regardless of real elapsed time. On 2026-09-11 the
camera was capturing at 24.95 fps while stamping as if 30 fps — a ratio of 0.832 — and by
2026-09-12 it was capturing a true 30.02 fps (ratio 1.0008), presumably after a settings
change. That is exactly why the fit runs with a free slope: it landed on 1.20 for the
first and ~1.0 for the second, with no change to this package. Read `sync_slope` in
`/diagnostics` to see which one your camera is doing.

## The latency and sync problem

Two different problems hide behind "there is latency":

**Pipeline delay** — how long after the photons a frame is available to publish. Attacked
directly in the pipeline:

| Setting | Why |
|---|---|
| `latency=0 drop-on-latency=true` | The RTP jitter buffer defaults to **2000 ms**. This is usually the entire complaint. |
| `protocols=udp`, `do-retransmission=false` | No TCP retransmit stalls on a direct wired link. |
| `nvv4l2decoder disable-dpb=true` | Skips decoded-picture-buffer reordering delay; safe because this stream has no B-frames. |
| `appsink max-buffers=1 drop=true sync=false` | Always the newest frame, never a queue of stale ones. |

**Display delay** — only relevant to the preview, but it is what you notice first.
`nv3dsink` holds on to the decoder's output buffers, so a ~13-frame queue builds up inside
the decoder while the window initialises; the sink then consumes at exactly the incoming
rate, so it never drains. Measured packet-to-sink:

| decoder feeding | latency | frames stuck in decoder |
|---|---|---|
| `nv3dsink` directly | **422 ms** | 13 |
| `queue max-size-buffers=1 leaky=downstream ! nv3dsink` | **10.3 ms** | 0–1 |
| `jpegenc ! appsink drop=true` (the ROS node) | 9.7 ms | 1 |

The leaky queue has to sit directly after the decoder: with anything that owns a buffer
pool in between (`nvvidconv ! queue ! glimagesink` measured 461 ms), the backlog just
forms upstream of the queue instead. The recorded dataset was never affected — the node's
`appsink drop=true` already discards stale frames — only the preview was.

**Timestamp error** — how wrong the timestamp on a frame is. This is the one that
actually ruins a dataset, and it is *not* fixed by making the pipeline faster. Measured
end to end on a 113-second recording from the current firmware:

```
header stamps (camera clock)  inter-frame  p50 40.09  p95 40.11  max  56.96 ms
bag arrival (wall clock)      inter-frame  p50 40.00  p95 49.81  max 373.16 ms
```

Stamping frames on arrival bakes tens of milliseconds of network and scheduling jitter
into the dataset. At flight speed that is real position error against the IMU and pose
topics, and it is unrecoverable after the fact.

So frames are stamped from whichever metronomic counter the camera offers — RTCP when
it sends sender reports, otherwise the RTP header clock — mapped onto host time.
`sync_source: auto` watches the first 3 seconds and commits to one; mixing them in a
single fit would be meaningless, since they are different clocks. Whichever it picks,
the counter is arbitrary and runs at its own rate, so `camera_clock_sync.py`
continuously fits

```
host_time  ~=  slope * camera_time + intercept
```

using only the **lowest-residual** sample in each one-second bucket. Network delay is
one-sided — it can only make a frame arrive later than the truth, never earlier — so the
lower envelope is the best estimate of the true relationship, and averaging would track
the mean delay instead. Same minimum-filter idea NTP uses. Measured residual is ~1.0 ms
on the RTP clock.

The fit refuses to answer until it has a real slope (`sync_ready`), and frames captured
before that are stamped on arrival and counted as `frames_predecision`. This matters:
slope 1.0 is a fine prior for an RTCP clock but is ~20% wrong for an RTP frame clock, and
using it during warm-up produced half-second timestamp discontinuities in the first
seconds of a recording. Expect roughly the first 6 seconds of any capture to be
arrival-stamped; `frames_arrival_stamped` staying at 0 afterwards is the healthy case.

### What this cannot fix

If the camera stamps a frame at the wrong instant relative to the exposure, no amount of
clock fitting reveals it: that error is a constant. Measure it once with

```bash
ros2 run ap_dataset_logger measure_camera_latency.py
```

which strobes the screen at known times and watches when the change appears, then put
the result in `latency_offset_ms`. Use `--manual` if there is no display.

## Data rates

Measured at 1280x720. The current firmware's 24.95 fps makes these ~17% lower than the
figures below, which were taken at 30 fps:

| Path | Per frame | Rate | 10-min flight |
|---|---|---|---|
| JPEG q85 (`/camera/image_raw/compressed`) | 83 KB | 9.2 GB/h | 1.5 GB |
| JPEG q70 | 15 KB | 1.6 GB/h | 0.3 GB |
| Native stream (`archive_h265:=true`) | 7 KB | 0.7 GB/h | 0.12 GB |

JPEG is the default because NGPS consumes that topic unchanged and a bag replays with no
special tooling. Prefer `archive_h265:=true` over dropping the quality to save space:
q70 visibly softens the texture SuperPoint keys on, while the H265 archive is a
byte-exact copy of what the camera sent, costs no re-encode, and is 13x smaller.

## JPEG encoder

`jpeg_encoder: auto` probes `nvjpegenc` in a **subprocess** at startup and falls back to
CPU `jpegenc`. The subprocess matters: where `nvjpegenc` does not work it fails with
`Nvjpeg failure: code=6` and segfaults the process.

It does not work inside this workspace's container — it needs L4T's `libnvjpeg` from
`/usr/lib/aarch64-linux-gnu/tegra`, which the CUDA base image does not ship (only CUDA's
unrelated `libnvjpeg`). So expect:

* **container** → `jpegenc`, roughly half of one core at 720p30
* **host** → `nvjpegenc`, near zero CPU

Installing `nvidia-l4t-gstreamer` in the container would enable the hardware encoder.
That is an L4T package going into an image with a deliberate TensorRT apt pin, so it is
left as a decision rather than a default.

## Robustness

* Watchdog reconnects if frames stop for `watchdog_s`, with exponential backoff. The
  backoff never sleeps on the executor thread, so diagnostics stay live throughout.
* The clock fit resets on reconnect and on any camera clock jump, since the camera's
  clock restarts with the RTSP session. It also resets when frames keep missing the fitted
  line by more than 50 ms (3 in a row) — the signature of a frame-rate change, which
  network jitter alone cannot produce. Tested against 20–45 ms network bursts with no
  false alarms.
* Each pipeline carries a generation tag; buffers from a superseded pipeline are dropped
  rather than published, because `set_state(NULL)` can return asynchronously.
* GStreamer's streaming thread touches **no rclpy object** — it only appends to a deque,
  and a timer on the executor drains it. Publishing (or even triggering a guard
  condition) from that thread segfaults rclpy. The drain interval delays publication,
  not the timestamp, so it costs no sync accuracy.
* Live health is on `/diagnostics`: fps, dropped frames, reconnects, `sync_skew_ppm`,
  `sync_residual_ms`, and whether any frame arrived without an RTCP timestamp.

If a recording is cut short, the `.mcap` is intact but `metadata.yaml` is missing.
`check_dataset_sync.py` reads it anyway; `ros2 bag reindex <dir> -s mcap` repairs it.

## Recording

`record.launch.py` in this package owns the topic allowlist and the rosbag2 flags, so
the package stands alone on any branch. `--max-cache-size` is cut from rosbag2's 100 MB
default to 16 MB, which is the one setting that matters for RAM on an 8 GB Orin with no
swap; the recorder measured 73 MB RSS during a capture.

Note the `ngps: Add per-frame structured logging` commit on
`feat/tensorrt-lightglue-jetson` adds a second, near-identical `record.launch.py` under
`ap_ngps_ros2` for recording an NGPS *run*. If both ever land on one branch, keep this
one and drop the other.
