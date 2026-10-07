"""Direct camera frames for VO: RTSP -> hardware decode -> hardware scale -> numpy, no JPEG, no DDS.

The recorder's camera node stays as it is (it owns the bag's /camera/image_raw/compressed); this is a
second RTSP client on the same ZR10 stream (it serves two at once, tested 2026-10-02). Skipping the
JPEG encode, the ROS/DDS hop and the JPEG decode takes roughly 80-90 ms off each frame.

Timestamps follow rtsp_camera_node exactly -- the camera's RTCP clock if its sender reports carry one,
else the RTP header clock, fitted to host time by CameraClockSync, minus the measured constant
latency_offset -- so a stamp means the same thing on either path.

No rclpy in here: frames are handed to on_frame(stamp_s, arrival_s, bgr) on GStreamer's streaming
thread, and the caller moves them onto its executor. log(level, text) is called from either thread.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable, Optional

import numpy as np
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtp', '1.0')
from gi.repository import Gst, GstRtp  # noqa: E402

from ap_dataset_logger.camera_clock_sync import CameraClockSync  # noqa: E402

RTP_CLOCK_HZ = 90000.0
RTP_WRAP = 1 << 32
SOURCE_DECISION_S = 3.0      # watch the stream this long before choosing the clock (as the camera node)


def build_description(url: str, codec: str = 'h264', protocols: str = 'udp',
                      width: int = 640, height: int = 360) -> str:
    depay = 'rtph265depay name=depay ! h265parse' if codec == 'h265' else 'rtph264depay name=depay ! h264parse'
    return (
        f'rtspsrc location={url} latency=0 drop-on-latency=true protocols={protocols} '
        'add-reference-timestamp-meta=true do-retransmission=false '
        f'! {depay} '
        # no B-frames on the ZR10, so the decoded-picture buffer only adds delay
        '! nvv4l2decoder disable-dpb=true '
        # never let the decoder hold a frame while the scaler or Python is busy
        '! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream '
        # VIC hardware scaling straight to the size VO and NGPS work at
        f'! nvvidconv ! video/x-raw,format=BGRx,width={width},height={height} '
        '! appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false'
    )


class RtspFrameSource:
    def __init__(self, url: str, on_frame: Callable[[float, float, np.ndarray], None],
                 log: Callable[[str, str], None] = lambda level, text: None,
                 codec: str = 'h264', protocols: str = 'udp', latency_offset_s: float = 0.0,
                 width: int = 640, height: int = 360, watchdog_s: float = 3.0, sync_window_s: float = 120.0):
        Gst.init(None)
        self.url, self.codec, self.protocols = url, codec, protocols
        self.width, self.height = width, height
        self.on_frame, self.log = on_frame, log
        self.latency_offset_s = float(latency_offset_s)
        self.watchdog_s = float(watchdog_s)
        self.sync_window_s = float(sync_window_s)
        self.pipeline = None
        self._lock = threading.Lock()
        self._generation = 0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._reset_timing()
        self.frames = 0
        self.frames_synced = 0
        self.reconnects = 0
        self.last_frame_wall = 0.0
        self.last_delay_ms = None

    # -- clock (same rules as rtsp_camera_node) ---------------------------------
    def _reset_timing(self):
        self.source = 'deciding'
        self.sync = self._make_sync('rtcp')
        self._decide_deadline = None
        self._decide_total = self._decide_rtcp = 0
        self._rtp_fifo = deque(maxlen=8)
        self._rtp_last_raw = None
        self._rtp_wraps = 0

    def _make_sync(self, source: str) -> CameraClockSync:
        tight = source == 'rtcp'
        return CameraClockSync(window_s=self.sync_window_s, latency_offset_s=self.latency_offset_s,
                               slope_min=0.999 if tight else 0.5, slope_max=1.001 if tight else 2.0,
                               assume_unit_slope=tight)

    def _decide(self, now: float):
        if self._decide_deadline is None:
            self._decide_deadline = now + SOURCE_DECISION_S
            return
        if now < self._decide_deadline:
            return
        if self._decide_total and self._decide_rtcp > self._decide_total * 0.5:
            self.source = 'rtcp'
        elif self._rtp_fifo or self._rtp_last_raw is not None:
            self.source = 'rtp'
        else:
            self.source = 'arrival'
        self.sync = self._make_sync(self.source)
        self.log('info', f'direct camera clock source: {self.source}')

    def _rtp_probe(self, pad, info):
        buf = info.get_buffer()
        ok, rtp = GstRtp.RTPBuffer.map(buf, Gst.MapFlags.READ)
        if not ok:
            return Gst.PadProbeReturn.OK
        try:
            if rtp.get_marker():
                raw = rtp.get_timestamp()
                if self._rtp_last_raw is not None and raw < self._rtp_last_raw - (RTP_WRAP >> 1):
                    self._rtp_wraps += 1
                self._rtp_last_raw = raw
                if len(self._rtp_fifo) == self._rtp_fifo.maxlen:
                    self._rtp_fifo.clear()          # decoder fell behind: re-pair rather than mislabel
                self._rtp_fifo.append(((raw + self._rtp_wraps * RTP_WRAP) / RTP_CLOCK_HZ,
                                       time.clock_gettime(time.CLOCK_REALTIME)))
        finally:
            rtp.unmap()
        return Gst.PadProbeReturn.OK

    # -- streaming thread ------------------------------------------------------
    def _on_sample(self, sink, generation):
        if generation != self._generation or self._stop.is_set():
            return Gst.FlowReturn.OK
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        arrival = time.clock_gettime(time.CLOCK_REALTIME)
        buf = sample.get_buffer()
        meta = buf.get_reference_timestamp_meta()
        rtcp_t = meta.timestamp / 1e9 if meta is not None else None
        rtp_t = rtp_arrival = None
        if self._rtp_fifo:
            rtp_t, rtp_arrival = self._rtp_fifo.popleft()
        if self.source == 'deciding':
            self._decide_total += 1
            self._decide_rtcp += rtcp_t is not None
            self._decide(arrival)
        cam_t = ref = None
        if self.source == 'rtcp':
            cam_t, ref = rtcp_t, arrival
        elif self.source == 'rtp':
            cam_t, ref = rtp_t, rtp_arrival
        if cam_t is not None:
            self.sync.update(cam_t, ref)
        if cam_t is not None and self.sync.ready:
            stamp = self.sync.to_host(cam_t)
            self.frames_synced += 1
        else:
            stamp = arrival - self.latency_offset_s
        self.last_delay_ms = (arrival - stamp) * 1e3
        ok, mapped = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        try:
            # BGRx, possibly row-padded: take the pitch from the caps-derived size
            data = np.frombuffer(mapped.data, np.uint8)
            pitch = data.size // self.height
            bgr = data.reshape(self.height, pitch)[:, :self.width * 4].reshape(self.height, self.width, 4)[:, :, :3].copy()
        finally:
            buf.unmap(mapped)
        self.frames += 1
        self.last_frame_wall = arrival
        try:
            self.on_frame(stamp, arrival, bgr)
        except Exception as exc:          # never let a consumer error kill the streaming thread
            self.log('warn', f'direct camera consumer: {exc}')
        return Gst.FlowReturn.OK

    # -- lifecycle ---------------------------------------------------------------
    def _start_pipeline(self):
        with self._lock:
            self._teardown_locked()
            desc = build_description(self.url, self.codec, self.protocols, self.width, self.height)
            try:
                self.pipeline = Gst.parse_launch(desc)
            except Exception as exc:
                self.log('error', f'direct camera pipeline failed: {exc}')
                self.pipeline = None
                return
            self._generation += 1
            gen = self._generation
            self.pipeline.get_by_name('depay').get_static_pad('sink').add_probe(
                Gst.PadProbeType.BUFFER, self._rtp_probe)
            self.pipeline.get_by_name('sink').connect('new-sample', lambda s: self._on_sample(s, gen))
            self.pipeline.set_state(Gst.State.PLAYING)
            self.last_frame_wall = time.clock_gettime(time.CLOCK_REALTIME)
            self.log('info', f'direct camera: {self.url} ({self.codec}, {self.width}x{self.height})')

    def _teardown_locked(self):
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None

    def _supervise(self):
        backoff = 1.0
        self._start_pipeline()
        while not self._stop.wait(0.5):
            with self._lock:
                pipe = self.pipeline
            why = None
            if pipe is None:
                why = 'no pipeline'
            else:
                msg = pipe.get_bus().pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
                if msg is not None:
                    why = (f'gstreamer error: {msg.parse_error()[0]}' if msg.type == Gst.MessageType.ERROR
                           else 'end of stream')
                elif time.clock_gettime(time.CLOCK_REALTIME) - self.last_frame_wall > self.watchdog_s:
                    why = f'no frames for {self.watchdog_s:g}s'
            if why is None:
                backoff = 1.0
                continue
            self.reconnects += 1
            self.log('warn', f'direct camera reconnecting ({why}), #{self.reconnects}')
            with self._lock:
                self._teardown_locked()
            self._reset_timing()                 # the camera clock restarts with the session
            if self._stop.wait(backoff):
                break
            backoff = min(8.0, backoff * 2)
            self._start_pipeline()

    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._supervise, name='rtsp_frames', daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        with self._lock:
            self._teardown_locked()

    def stats(self) -> dict:
        return {'source': self.source, 'frames': self.frames, 'synced': self.frames_synced,
                'reconnects': self.reconnects, 'delay_ms': self.last_delay_ms}
