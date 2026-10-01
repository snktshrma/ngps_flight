#!/usr/bin/env python3
"""Low-latency RTSP camera bringup with jitter-free capture timestamps.

Publishes ``sensor_msgs/CompressedImage`` (JPEG) so the existing NGPS pipeline can
consume the stream unchanged, and so a recording replays with no special tooling.

Latency is attacked at every stage that adds it:

* ``latency=0 drop-on-latency=true`` -- the RTP jitter buffer is where most "RTSP is
  laggy" complaints actually come from; its default is 2000 ms.
* ``protocols=udp`` -- no TCP retransmit stalls on a direct wired link.
* ``nvv4l2decoder disable-dpb=true`` -- skips decoded-picture-buffer reordering delay,
  valid because this stream has no B-frames.
* ``nvjpegenc`` -- JPEG encoding on the Jetson's hardware block, so frames never leave
  NVMM memory for a CPU colour conversion.
* ``appsink max-buffers=1 drop=true sync=false`` -- always the newest frame, never a
  queue of stale ones.

Timestamps come from the camera's RTCP clock rather than arrival time; see
``camera_clock_sync`` for why that matters and how the mapping is fitted.
"""

import socket
import subprocess
import threading
import time
from collections import deque
from urllib.parse import urlparse

import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstRtp', '1.0')
from gi.repository import Gst, GstRtp  # noqa: E402

import rclpy  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402

from ap_dataset_logger.camera_clock_sync import CameraClockSync  # noqa: E402

#: NTP epoch (1900-01-01) to Unix epoch (1970-01-01), in seconds.
NTP_UNIX_OFFSET = 2208988800.0

#: RTP timestamps are a 32-bit 90 kHz counter, so they wrap every ~13.25 hours.
RTP_CLOCK_HZ = 90000.0
RTP_WRAP = 1 << 32

#: How long to watch the stream before deciding which clock to sync against.
SOURCE_DECISION_S = 3.0


def probe_codec(url: str, timeout: float = 4.0):
    """Ask the RTSP server what it actually serves.

    Worth doing rather than trusting the URL: this gimbal serves H265 from a path
    called ``main.264``.
    """
    parsed = urlparse(url)
    host = parsed.hostname
    port = parsed.port or 554
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.sendall(
                f'DESCRIBE {url} RTSP/1.0\r\nCSeq: 1\r\nAccept: application/sdp\r\n\r\n'.encode()
            )
            sock.settimeout(timeout)
            data = sock.recv(8192).decode(errors='replace')
    except OSError:
        return None
    if '200 OK' not in data.split('\r\n')[0]:
        return None
    upper = data.upper()
    if 'H265' in upper or 'HEVC' in upper:
        return 'h265'
    if 'H264' in upper:
        return 'h264'
    return None


def probe_jpeg_encoder(timeout: float = 10.0) -> str:
    """Pick a JPEG encoder that actually works in this environment.

    ``nvjpegenc`` is the fast path -- it keeps frames in NVMM and costs no CPU -- but it
    needs L4T's libnvjpeg, which lives in /usr/lib/aarch64-linux-gnu/tegra and is absent
    from the CUDA base image this workspace's container is built on. There it fails with
    "Nvjpeg failure: code=6" and takes the process down with it.

    Probing in a subprocess is deliberate: the failure mode is a segfault, so it has to
    happen somewhere that cannot kill the node.
    """
    cmd = [
        'gst-launch-1.0', '-q', 'videotestsrc', 'num-buffers=3', '!',
        'video/x-raw,width=640,height=480', '!', 'nvvidconv', '!', 'nvjpegenc', '!',
        'fakesink',
    ]
    try:
        result = subprocess.run(cmd, timeout=timeout,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return 'nvjpegenc' if result.returncode == 0 else 'jpegenc'
    except (subprocess.TimeoutExpired, OSError):
        return 'jpegenc'


class RtspCameraNode(Node):

    def __init__(self):
        super().__init__('rtsp_camera_node')

        self.declare_parameter('rtsp_url', 'rtsp://192.168.144.25:8554/video1')
        self.declare_parameter('codec', 'auto')            # auto | h265 | h264
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('rtsp_latency_ms', 0)
        self.declare_parameter('protocols', 'udp')         # udp | tcp | udp-mcast
        self.declare_parameter('qos_depth', 30)
        self.declare_parameter('qos_reliable', True)
        # Sync
        self.declare_parameter('use_rtcp_timestamps', True)
        # auto | rtcp | rtp | arrival -- see _decide_sync_source
        self.declare_parameter('sync_source', 'auto')
        self.declare_parameter('sync_window_s', 120.0)
        self.declare_parameter('latency_offset_ms', 0.0)
        # Robustness
        self.declare_parameter('watchdog_s', 5.0)
        self.declare_parameter('reconnect_backoff_s', 2.0)
        self.declare_parameter('max_backoff_s', 20.0)
        # Optional pristine archive of the source stream (no re-encode)
        self.declare_parameter('archive_h265', False)
        self.declare_parameter('archive_dir', '')
        self.declare_parameter('archive_split_bytes', 500000000)
        self.declare_parameter('diagnostics_period_s', 1.0)
        self.declare_parameter('drain_rate_hz', 200.0)
        self.declare_parameter('jpeg_encoder', 'auto')       # auto | nvjpegenc | jpegenc
        self.declare_parameter('decoder_max_performance', False)

        def p(name):
            return self.get_parameter(name).get_parameter_value()

        self.url = p('rtsp_url').string_value
        self.frame_id = p('frame_id').string_value
        self.jpeg_quality = int(p('jpeg_quality').integer_value)
        self.rtsp_latency_ms = int(p('rtsp_latency_ms').integer_value)
        self.protocols = p('protocols').string_value
        self.use_rtcp = p('use_rtcp_timestamps').bool_value
        self.watchdog_s = float(p('watchdog_s').double_value)
        self.backoff_s = float(p('reconnect_backoff_s').double_value)
        self.max_backoff_s = float(p('max_backoff_s').double_value)
        self.archive_h265 = p('archive_h265').bool_value
        self.archive_dir = p('archive_dir').string_value
        self.archive_split_bytes = int(p('archive_split_bytes').integer_value)
        self.decoder_max_perf = p('decoder_max_performance').bool_value

        qos = QoSProfile(
            depth=max(1, int(p('qos_depth').integer_value)),
            # Reliable by default: this is a dataset logger and the recorder is on the
            # same host, so guaranteeing the recorder sees every frame beats shaving a
            # negligible amount of loopback latency.
            reliability=(ReliabilityPolicy.RELIABLE if p('qos_reliable').bool_value
                         else ReliabilityPolicy.BEST_EFFORT),
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(CompressedImage, p('image_topic').string_value, qos)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)

        self._sync_window_s = float(p('sync_window_s').double_value)
        self.latency_offset_s = float(p('latency_offset_ms').double_value) / 1000.0
        self.sync_source_param = p('sync_source').string_value.strip().lower()
        if not self.use_rtcp and self.sync_source_param == 'auto':
            # Legacy switch: use_rtcp_timestamps=false means "do not sync at all".
            self.sync_source_param = 'arrival'
        self.active_sync_source = (
            self.sync_source_param if self.sync_source_param in ('rtcp', 'rtp', 'arrival')
            else 'deciding'
        )
        self.sync = self._make_sync(self.active_sync_source)

        # RTP timestamps are read off the RTP header by a pad probe before the
        # depayloader discards them, then matched to decoded frames through this FIFO.
        # The decoder emits one frame per input frame (no B-frames, disable-dpb), so the
        # pairing is positional; _rtp_desyncs counts the times that assumption broke.
        self._rtp_fifo = deque(maxlen=32)
        self._rtp_last_raw = None
        self._rtp_wraps = 0
        self._rtp_desyncs = 0
        self._decide_deadline = None
        self._decide_rtcp = 0
        self._decide_total = 0

        # Counters (written from the GStreamer streaming thread, read from ROS timers;
        # int increments are atomic under the GIL so no lock is needed).
        self.frames_rx = 0
        self.frames_pub = 0
        self.publish_errors = 0
        self.reconnects = 0
        self.no_meta_frames = 0
        self.frames_synced = 0
        self.frames_arrival_stamped = 0
        # Frames stamped on arrival before a clock source was chosen. Expected, and
        # counted apart from real fallbacks so they do not raise a standing warning.
        self.frames_predecision = 0
        self._pending_source_log = None
        self._last_frame_wall = 0.0
        self._last_diag_frames = 0
        self._last_diag_t = time.time()
        self._recent_delay_ms = None

        self.pipeline = None
        self._lock = threading.Lock()
        self._current_backoff = self.backoff_s
        self._retry_at = 0.0
        # Bumped on every (re)build. set_state(NULL) can return ASYNC, so an old
        # pipeline may still push a few buffers after a new one starts; samples
        # tagged with a stale generation are dropped rather than published twice.
        self._generation = 0
        self._shutdown = False

        # GStreamer delivers frames on its own streaming thread. Touching *any* rclpy
        # object from there while the executor is waiting segfaults the process --
        # publish() and guard_condition.trigger() both do it. So that thread touches
        # nothing but this deque (plain Python, atomic under the GIL) and a timer on
        # the executor drains it. maxlen gives drop-oldest for free: a stale frame is
        # worth less than a fresh one.
        #
        # The drain interval delays *publication*, not the timestamp -- the stamp is
        # already fixed at capture time from the RTCP clock -- so it costs no sync
        # accuracy, only a few ms of liveness.
        self._queue = deque(maxlen=4)
        self.frames_dropped_queue = 0

        Gst.init(None)

        self.codec = p('codec').string_value.strip().lower()
        if self.codec == 'auto':
            detected = probe_codec(self.url)
            if detected is None:
                self.get_logger().warn(
                    f'could not DESCRIBE {self.url}; assuming h265. '
                    'Set the codec parameter explicitly if this is wrong.'
                )
                detected = 'h265'
            else:
                self.get_logger().info(f'detected codec: {detected}')
            self.codec = detected

        self.jpeg_encoder = p('jpeg_encoder').string_value.strip().lower()
        if self.jpeg_encoder == 'auto':
            self.jpeg_encoder = probe_jpeg_encoder()
            self.get_logger().info(
                f'jpeg encoder: {self.jpeg_encoder}'
                + ('' if self.jpeg_encoder == 'nvjpegenc'
                   else ' (nvjpegenc unusable here; encoding on CPU)')
            )

        self._start_pipeline()
        drain_hz = max(30.0, float(p('drain_rate_hz').double_value))
        self.create_timer(1.0 / drain_hz, self._drain_queue)
        self.create_timer(0.5, self._poll_bus)
        self.create_timer(1.0, self._watchdog)
        self.create_timer(max(0.1, float(p('diagnostics_period_s').double_value)),
                          self._publish_diagnostics)
        self.get_logger().info('rtsp_camera_node ready')

    # -- clock source ---------------------------------------------------------

    def _make_sync(self, source: str) -> CameraClockSync:
        # An RTCP clock is a real clock and should sit within a few hundred ppm of ours,
        # so a tight bound rejects nonsense fits. An RTP frame clock's slope is the
        # camera's nominal-versus-actual frame-rate ratio instead -- the ZR10 firmware
        # captures at 24.95 fps while stamping 30 fps, a slope near 1.20 -- so it needs
        # room to move.
        tight = source == 'rtcp'
        return CameraClockSync(
            window_s=self._sync_window_s,
            latency_offset_s=self.latency_offset_s,
            slope_min=0.999 if tight else 0.5,
            slope_max=1.001 if tight else 2.0,
            assume_unit_slope=tight,
        )

    def _decide_sync_source(self, now: float) -> None:
        """Pick the clock to sync against, once, from what the stream actually offers.

        Mixing sources in one fit would be meaningless -- they are different clocks --
        so this commits to one and says so in the log.
        """
        if self._decide_deadline is None:
            self._decide_deadline = now + SOURCE_DECISION_S
            return
        if now < self._decide_deadline:
            return

        if self._decide_total and self._decide_rtcp > self._decide_total * 0.5:
            chosen, why = 'rtcp', f'{self._decide_rtcp}/{self._decide_total} frames carry RTCP time'
        elif self._rtp_fifo or self._rtp_last_raw is not None:
            chosen, why = 'rtp', 'no RTCP sender reports; using the RTP header clock'
        else:
            chosen, why = 'arrival', 'neither RTCP nor RTP timing available'

        self.active_sync_source = chosen
        self.sync = self._make_sync(chosen)
        self._pending_source_log = f'clock source: {chosen} ({why})'

    def _rtp_probe(self, pad, info):
        """Capture the RTP header timestamp of each complete frame (marker bit set)."""
        buf = info.get_buffer()
        ok, rtp = GstRtp.RTPBuffer.map(buf, Gst.MapFlags.READ)
        if not ok:
            return Gst.PadProbeReturn.OK
        try:
            if rtp.get_marker():
                raw = rtp.get_timestamp()
                if self._rtp_last_raw is not None and raw < self._rtp_last_raw - (RTP_WRAP >> 1):
                    self._rtp_wraps += 1          # 32-bit counter rolled over
                self._rtp_last_raw = raw
                stamp = (raw + self._rtp_wraps * RTP_WRAP) / RTP_CLOCK_HZ
                if len(self._rtp_fifo) == self._rtp_fifo.maxlen:
                    # Decoder output fell behind its input: positional pairing is no
                    # longer trustworthy, so start over rather than mislabel frames.
                    self._rtp_fifo.clear()
                    self._rtp_desyncs += 1
                self._rtp_fifo.append((stamp, time.clock_gettime(time.CLOCK_REALTIME)))
        finally:
            rtp.unmap()
        return Gst.PadProbeReturn.OK

    # -- pipeline -------------------------------------------------------------

    def _build_description(self) -> str:
        depay = ('rtph265depay name=depay ! h265parse' if self.codec == 'h265'
                 else 'rtph264depay name=depay ! h264parse')
        src = (
            f'rtspsrc location={self.url} latency={self.rtsp_latency_ms} '
            f'drop-on-latency=true protocols={self.protocols} '
            f'add-reference-timestamp-meta=true do-retransmission=false '
            f'! {depay}'
        )
        # disable-dpb skips decoded-picture-buffer reordering delay, which is safe
        # because this stream carries no B-frames.
        decoder = 'nvv4l2decoder disable-dpb=true'
        if self.decoder_max_perf:
            decoder += ' enable-max-performance=1'
        if self.jpeg_encoder == 'nvjpegenc':
            encode = f'nvjpegenc quality={self.jpeg_quality}'
        else:
            # nvvidconv pulls the frame out of NVMM; jpegenc then costs roughly a third
            # of one core at 720p30, which the Orin's six cores absorb comfortably.
            encode = ('nvvidconv ! video/x-raw,format=I420 '
                      f'! jpegenc quality={self.jpeg_quality}')
        # The leaky 1-deep queue stops nvv4l2decoder holding on to a decoded frame while
        # the encoder is busy; without it glass-to-glass was well above the 0.23-0.25 s
        # measured with it on the ZR10 (2026-09-12).
        decode = (
            f'{decoder} ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream '
            f'! {encode} '
            '! appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false'
        )
        if not self.archive_h265:
            return f'{src} ! {decode}'

        # Archive branch carries the *original* elementary stream, so it costs no
        # encode and no quality: roughly 0.7 GB/hour against 9 GB/hour for the JPEGs.
        import os
        # Same default as the recorder, so the archive lands beside the bag rather than
        # in whatever directory the launch happened to be started from.
        directory = self.archive_dir or os.path.join(
            os.path.expanduser('~'), 'ngps_ws', 'src', 'ngps_flight', 'bags')
        os.makedirs(directory, exist_ok=True)
        pattern = os.path.join(directory, f'camera_{time.strftime("%Y%m%d_%H%M%S")}_%05d.mkv')
        return (
            f'{src} ! tee name=t '
            f't. ! queue max-size-buffers=4 leaky=downstream ! {decode} '
            f't. ! queue max-size-buffers=64 ! splitmuxsink muxer-factory=matroskamux '
            f'max-size-bytes={self.archive_split_bytes} location="{pattern}"'
        )

    def _start_pipeline(self) -> None:
        with self._lock:
            self._teardown_locked()
            desc = self._build_description()
            self.get_logger().info(f'starting pipeline: {desc}')
            try:
                self.pipeline = Gst.parse_launch(desc)
            except Exception as exc:
                self.get_logger().error(f'pipeline build failed: {exc}')
                self.pipeline = None
                return
            self._generation += 1
            generation = self._generation
            depay = self.pipeline.get_by_name('depay')
            if depay is not None:
                depay.get_static_pad('sink').add_probe(
                    Gst.PadProbeType.BUFFER, self._rtp_probe)
            sink = self.pipeline.get_by_name('sink')
            sink.connect('new-sample', lambda s: self._on_sample(s, generation))
            self.pipeline.set_state(Gst.State.PLAYING)
            self._last_frame_wall = time.time()

    def _teardown_locked(self) -> None:
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None

    def _reconnect(self, why: str) -> None:
        """Tear down now, rebuild later. Never sleeps on the executor thread."""
        self.reconnects += 1
        self.get_logger().warn(
            f'reconnecting ({why}); retrying in {self._current_backoff:.1f}s '
            f'[reconnect #{self.reconnects}]'
        )
        with self._lock:
            self._teardown_locked()
        # The camera clock restarts across an RTSP session, so the old fit is invalid.
        self.sync.reset()
        self._rtp_fifo.clear()
        self._rtp_last_raw = None
        self._rtp_wraps = 0
        self._retry_at = time.time() + self._current_backoff
        self._current_backoff = min(self.max_backoff_s, self._current_backoff * 2.0)

    # -- streaming thread -----------------------------------------------------

    def _on_sample(self, sink, generation):
        if generation != self._generation or self._shutdown:
            return Gst.FlowReturn.OK
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        arrival = time.clock_gettime(time.CLOCK_REALTIME)
        buf = sample.get_buffer()
        self.frames_rx += 1
        self._last_frame_wall = arrival
        # A frame arriving means the stream is healthy; stop backing off.
        self._current_backoff = self.backoff_s

        meta = buf.get_reference_timestamp_meta()
        rtcp_t = meta.timestamp / 1e9 if meta is not None else None
        if rtcp_t is None:
            self.no_meta_frames += 1

        # One RTP entry per decoded frame; empty means the pairing slipped, so this
        # frame falls back to arrival rather than borrowing another frame's timestamp.
        rtp_t = rtp_arrival = None
        if self._rtp_fifo:
            rtp_t, rtp_arrival = self._rtp_fifo.popleft()

        if self.active_sync_source == 'deciding':
            self._decide_total += 1
            if rtcp_t is not None:
                self._decide_rtcp += 1
            self._decide_sync_source(arrival)

        camera_t = ref_arrival = None
        if self.active_sync_source == 'rtcp':
            camera_t, ref_arrival = rtcp_t, arrival
        elif self.active_sync_source == 'rtp':
            # Pair with the probe's own arrival time: that is measured at the network
            # edge, before decode and JPEG encode add their variable share.
            camera_t, ref_arrival = rtp_t, rtp_arrival

        if camera_t is not None:
            self.sync.update(camera_t, ref_arrival)
        if camera_t is not None and self.sync.ready:
            stamp_s = self.sync.to_host(camera_t)
            self.frames_synced += 1
        else:
            stamp_s = arrival - self.latency_offset_s
            # Warm-up (choosing a source, or waiting for the first slope fit) is
            # expected; only a fallback after the fit was working counts as a fault.
            if self.active_sync_source == 'deciding' or not self.sync.ready:
                self.frames_predecision += 1
            else:
                self.frames_arrival_stamped += 1
        self._recent_delay_ms = (arrival - stamp_s) * 1e3

        ok, mapped = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        try:
            payload = bytes(mapped.data)
        finally:
            buf.unmap(mapped)

        if len(self._queue) == self._queue.maxlen:
            self.frames_dropped_queue += 1
        self._queue.append((stamp_s, payload))
        return Gst.FlowReturn.OK

    # -- executor thread ------------------------------------------------------

    def _log_pending(self) -> None:
        # The source decision happens on the GStreamer thread, which must not touch
        # rclpy; the message is logged here instead, on the executor.
        if self._pending_source_log is not None:
            self.get_logger().info(self._pending_source_log)
            self._pending_source_log = None

    def _drain_queue(self) -> None:
        self._log_pending()
        """Publish queued frames. Runs on the executor via the guard condition."""
        while True:
            if self._shutdown or not rclpy.ok():
                return
            try:
                stamp_s, payload = self._queue.popleft()
            except IndexError:
                return
            try:
                msg = CompressedImage()
                msg.header.stamp.sec = int(stamp_s)
                msg.header.stamp.nanosec = int(round((stamp_s - int(stamp_s)) * 1e9))
                # Rounding can carry nanosec to exactly 1e9, an invalid Time.
                if msg.header.stamp.nanosec >= 1_000_000_000:
                    msg.header.stamp.sec += 1
                    msg.header.stamp.nanosec -= 1_000_000_000
                msg.header.frame_id = self.frame_id
                msg.format = 'jpeg'
                msg.data = payload
                self.pub.publish(msg)
                self.frames_pub += 1
            except Exception as exc:
                self.publish_errors += 1
                if self.publish_errors <= 3:
                    self.get_logger().warn(f'publish failed: {exc}')

    # -- ROS timers -----------------------------------------------------------

    def _poll_bus(self) -> None:
        with self._lock:
            pipeline = self.pipeline
        if pipeline is None:
            return
        bus = pipeline.get_bus()
        msg = bus.poll(Gst.MessageType.ERROR | Gst.MessageType.EOS, 0)
        if msg is None:
            return
        if msg.type == Gst.MessageType.ERROR:
            err, _ = msg.parse_error()
            self._reconnect(f'gstreamer error: {err}')
        else:
            self._reconnect('end of stream')

    def _watchdog(self) -> None:
        if self.pipeline is None:
            if time.time() >= self._retry_at:
                self._start_pipeline()
            return
        idle = time.time() - self._last_frame_wall
        if self.watchdog_s > 0 and idle > self.watchdog_s:
            self._reconnect(f'no frames for {idle:.1f}s')

    def _publish_diagnostics(self) -> None:
        now = time.time()
        span = max(1e-6, now - self._last_diag_t)
        fps = (self.frames_rx - self._last_diag_frames) / span
        self._last_diag_frames = self.frames_rx
        self._last_diag_t = now
        idle = now - self._last_frame_wall

        status = DiagnosticStatus()
        status.name = 'rtsp_camera: stream'
        status.hardware_id = f'{self.url} ({self.codec})'
        if self.frames_rx == 0:
            status.level = DiagnosticStatus.ERROR
            status.message = 'no frames received'
        elif idle > max(1.0, self.watchdog_s / 2):
            status.level = DiagnosticStatus.ERROR
            status.message = f'stream stalled {idle:.1f}s'
        elif self.active_sync_source != 'arrival' and not self.sync.ready:
            status.level = DiagnosticStatus.WARN
            status.message = 'clock sync converging'
        elif self.active_sync_source == 'arrival' and self.sync_source_param != 'arrival':
            status.level = DiagnosticStatus.WARN
            status.message = ('no camera clock available; frames stamped on arrival '
                              '(timestamps carry network jitter)')
        elif self.frames_arrival_stamped > self.frames_synced * 0.02:
            status.level = DiagnosticStatus.WARN
            status.message = (f'{self.frames_arrival_stamped} frames fell back to '
                              f'arrival stamping')
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'{fps:.1f} fps'

        values = {
            'fps': f'{fps:.2f}',
            'frames_rx': self.frames_rx,
            'frames_published': self.frames_pub,
            'sync_source': self.active_sync_source,
            'frames_synced': self.frames_synced,
            'frames_arrival_stamped': self.frames_arrival_stamped,
            'frames_predecision': self.frames_predecision,
            'frames_without_rtcp': self.no_meta_frames,
            'rtp_desyncs': self._rtp_desyncs,
            'publish_errors': self.publish_errors,
            'frames_dropped_queue': self.frames_dropped_queue,
            'reconnects': self.reconnects,
            'stream_idle_s': f'{idle:.2f}',
            'jpeg_quality': self.jpeg_quality,
            'jpeg_encoder': self.jpeg_encoder,
            'observed_delay_ms': ('-' if self._recent_delay_ms is None
                                  else f'{self._recent_delay_ms:.1f}'),
        }
        values.update(self.sync.stats())
        for key, value in values.items():
            text = f'{value:.3f}' if isinstance(value, float) else str(value)
            status.values.append(KeyValue(key=key, value=text))

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.append(status)
        self.diag_pub.publish(msg)

    def stop(self) -> None:
        """Stop the stream before rclpy tears down, so nothing publishes into a dead
        context. Called from main's finally block."""
        self._shutdown = True
        with self._lock:
            self._teardown_locked()
        self._queue.clear()

    def destroy_node(self):
        self.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RtspCameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C under ros2 launch arrives as ExternalShutdownException, not
        # KeyboardInterrupt; letting it escape would exit non-zero on a normal stop.
        pass
    finally:
        # Order matters: stop GStreamer first so no frame can be published into a
        # context that is about to go away, then tear down ROS.
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
