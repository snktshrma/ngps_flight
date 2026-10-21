#!/usr/bin/env python3
"""Map a camera's own frame clock onto host time, without the network jitter.

Frames arrive with jitter; they were captured without it. Stamping on arrival bakes that
jitter into the dataset, which at flight speed is real position error against the IMU and
pose topics, and it cannot be recovered afterwards. So we stamp from whatever metronomic
counter the camera provides, mapped onto host time by

    host_time  ~=  slope * camera_time + intercept

Two counters can feed this, depending on what the camera offers:

* **RTCP** -- a ``timestamp/x-ntp`` reference meta, present when the camera sends sender
  reports. Its slope sits within a few hundred ppm of 1.0.
* **RTP** -- the 90 kHz timestamp in the RTP header, which needs no sender reports at
  all. Its slope is whatever the camera's nominal-versus-actual frame rate happens to
  be: the SIYI ZR10 ArduPilot firmware captures at 24.95 fps while stamping frames
  33.33 ms apart, giving a slope near 1.20.

That second case is why ``slope_min``/``slope_max`` default wide. It also means "delay"
(``host - camera``) is not a meaningful quantity here -- with slope far from 1 it grows
without bound -- so the lower envelope is taken over the *residual* against the current
rate estimate, not over the raw difference.

The fit deliberately uses only the **lowest-residual** sample in each bucket. Network
delay is one-sided: it can only ever make a frame arrive later than the truth, never
earlier, so the lower envelope is the best available estimate of the true relationship
while averaging would instead track the mean delay. Same minimum-filter idea NTP uses.

What no amount of fitting can recover is the absolute glass-to-timestamp latency: if the
camera stamps a frame at the wrong moment relative to the photons, that error is a
constant and invisible here. ``latency_offset_s`` holds a calibrated value for it (see
the sync section of the package README).
"""

import math
import time
from collections import deque

#: Camera clock jumps larger than this (seconds) mean the camera rebooted or the RTSP
#: session restarted, so the accumulated fit is meaningless and must be thrown away.
RESET_JUMP_S = 5.0

#: A fitted line that frames keep missing by this much has stopped describing the camera.
#: Network jitter on this link is under ~20 ms, so 50 ms leaves margin against false alarms
#: while catching a frame-rate change within about half a second.
BREAK_THRESHOLD_S = 0.05

#: Consecutive misses before the fit is abandoned. One bad frame is a network hiccup;
#: several in a row is a change in the camera.
BREAK_SAMPLES = 3


class CameraClockSync:
    """Lower-envelope linear fit from camera clock to host clock.

    Usage is one call per frame::

        sync.update(camera_t, host_arrival_t)
        stamp = sync.to_host(camera_t)

    ``to_host`` degrades gracefully: with no samples it returns the arrival time it was
    last given, and with too few buckets for a slope it falls back to a constant offset.
    """

    def __init__(self, window_s: float = 120.0, bucket_s: float = 1.0,
                 latency_offset_s: float = 0.0, min_buckets: int = 3,
                 slope_min: float = 0.5, slope_max: float = 2.0,
                 assume_unit_slope: bool = False,
                 break_threshold_s: float = BREAK_THRESHOLD_S,
                 break_samples: int = BREAK_SAMPLES):
        self.window_s = float(window_s)
        self.bucket_s = max(1e-3, float(bucket_s))
        self.latency_offset_s = float(latency_offset_s)
        self.min_buckets = max(2, int(min_buckets))
        # Wide by default: an RTP frame clock's slope is the camera's nominal/actual
        # frame-rate ratio, not a crystal offset. Pass a tight range for an RTCP clock.
        self.slope_min = float(slope_min)
        self.slope_max = float(slope_max)
        # Whether slope 1.0 is a usable prior while the fit is still converging. True
        # for a real clock (RTCP), where 1.0 is right to within a few hundred ppm.
        # False for an RTP frame clock, whose slope is a frame-rate ratio -- assuming
        # 1.0 there would be wrong by ~20% and produce timestamps far worse than simply
        # using arrival time, so the caller is told "not ready" until a slope exists.
        self.assume_unit_slope = bool(assume_unit_slope)
        # Break detection. The fit assumes one straight line from camera counter to host
        # time, but an RTP frame counter's slope is the camera's actual frame rate, and
        # this camera drops from 30 to 25 fps in low light. A window spanning both rates
        # fits neither: simulated, that gave 3.1 s of timestamp error with nothing
        # looking wrong. Frames persistently missing the line is the tell.
        self.break_threshold_s = float(break_threshold_s)
        self.break_samples = max(1, int(break_samples))
        self._miss_run = 0
        self.rate_breaks = 0

        # (camera_t, host_t) of the lowest-delay sample seen in each closed bucket.
        self._buckets = deque()
        self._open_key = None
        self._open_best = None          # (camera_t, host_t, delay)

        self.slope = 1.0
        self._slope_fitted = False      # True once a real slope has been estimated
        self.intercept = None           # None until the first bucket closes
        self.residual_rms = float('nan')
        self.samples = 0
        self.resets = 0
        self.outliers = 0
        self._last_camera_t = None
        self._last_host_t = None

    # -- ingest ---------------------------------------------------------------

    def update(self, camera_t: float, host_t: float) -> None:
        """Feed one frame's camera timestamp and its host arrival time."""
        if not (math.isfinite(camera_t) and math.isfinite(host_t)):
            return

        if self._last_camera_t is not None:
            step = camera_t - self._last_camera_t
            # Backwards or a large forward leap: the source clock is no longer the one
            # we fitted against. Keep serving stamps, but from a clean fit.
            if step < -1e-3 or step > RESET_JUMP_S:
                self.reset()

        if self.ready:
            residual = host_t - (self.slope * camera_t + self.intercept)
            if abs(residual) > self.break_threshold_s:
                self._miss_run += 1
                if self._miss_run >= self.break_samples:
                    # Start again on the new rate. Frames until the refit are stamped on
                    # arrival by the caller, which costs jitter but never seconds.
                    self.reset()
                    self.rate_breaks += 1
            else:
                self._miss_run = 0

        self._last_camera_t = camera_t
        self._last_host_t = host_t
        self.samples += 1

        # Score against the current rate estimate rather than the raw difference: with
        # a slope far from 1 the raw difference grows monotonically inside every bucket,
        # so minimising it would just pick the first sample every time.
        rate = self.slope if self.intercept is not None else 1.0
        score = host_t - rate * camera_t
        key = math.floor(camera_t / self.bucket_s)
        if self._open_key is None:
            self._open_key, self._open_best = key, (camera_t, host_t, score)
            return
        if key == self._open_key:
            if score < self._open_best[2]:
                self._open_best = (camera_t, host_t, score)
            return

        # Bucket rolled over: commit its minimum-delay representative and refit.
        self._buckets.append((self._open_best[0], self._open_best[1]))
        self._open_key, self._open_best = key, (camera_t, host_t, score)
        self._trim(camera_t)
        self._fit()

    def _trim(self, now_camera_t: float) -> None:
        horizon = now_camera_t - self.window_s
        while self._buckets and self._buckets[0][0] < horizon:
            self._buckets.popleft()

    def reset(self) -> None:
        self._buckets.clear()
        self._open_key = None
        self._open_best = None
        self.slope = 1.0
        self._slope_fitted = False
        self.intercept = None
        self.residual_rms = float('nan')
        self._miss_run = 0
        self.resets += 1

    # -- fit ------------------------------------------------------------------

    def _fit(self) -> None:
        pts = list(self._buckets)
        if len(pts) < self.min_buckets:
            # Not enough span to separate skew from offset; constant offset only.
            if pts:
                # Not enough span to separate rate from offset. Hold the current rate
                # (1.0 before anything is known) and fit the offset alone.
                self.intercept = min(h - self.slope * c for c, h in pts)
            return

        # Fit against a local origin: camera_t is ~1e5 and host_t ~1.8e9, and a least
        # squares fit on raw magnitudes that different loses precision in float64.
        c0 = pts[0][0]
        h0 = pts[0][1]
        xs = [c - c0 for c, _ in pts]
        ys = [h - h0 for _, h in pts]
        n = len(xs)
        mx = sum(xs) / n
        my = sum(ys) / n
        den = sum((x - mx) ** 2 for x in xs)
        if den <= 0:
            return
        slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den

        # Outside the configured range this is a bad fit rather than a real rate, and
        # applying it would be worse than not fitting at all.
        if not (self.slope_min < slope < self.slope_max):
            slope = 1.0

        intercept = (my - slope * mx) + h0 - slope * c0
        resid = [(h - (slope * c + intercept)) for c, h in pts]
        self.slope = slope
        self._slope_fitted = True
        self.intercept = intercept
        self.residual_rms = math.sqrt(sum(r * r for r in resid) / len(resid))

    # -- query ----------------------------------------------------------------

    def to_host(self, camera_t: float) -> float:
        """Camera timestamp -> host timestamp, minus the calibrated latency offset."""
        if self.intercept is None:
            # Before the first bucket closes, the freshest single observation is the
            # best we have. Never return a nonsense epoch.
            if self._open_best is not None:
                return self._open_best[1] - self.latency_offset_s
            return (self._last_host_t or time.time()) - self.latency_offset_s
        return (self.slope * camera_t + self.intercept) - self.latency_offset_s

    @property
    def ready(self) -> bool:
        if self.intercept is None:
            return False
        # Without a usable prior, a fitted slope is required before the mapping means
        # anything; _fit only sets one once min_buckets have closed.
        return self.assume_unit_slope or self._slope_fitted

    @property
    def skew_ppm(self) -> float:
        """Rate error in ppm. Only meaningful for a real clock (RTCP); for an RTP frame
        clock the slope is a frame-rate ratio, so read ``slope`` instead."""
        return (self.slope - 1.0) * 1e6

    def stats(self) -> dict:
        return {
            'sync_ready': self.ready,
            'sync_slope_fitted': self._slope_fitted,
            'sync_buckets': len(self._buckets),
            'sync_slope': self.slope,
            'sync_skew_ppm': self.skew_ppm,
            'sync_residual_ms': self.residual_rms * 1e3,
            'sync_resets': self.resets,
            'sync_rate_breaks': self.rate_breaks,
            'sync_samples': self.samples,
            'latency_offset_ms': self.latency_offset_s * 1e3,
        }
