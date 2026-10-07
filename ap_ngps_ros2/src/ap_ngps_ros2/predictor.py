"""Forward prediction of the VO+NGPS position to hide camera latency beyond what ArduPilot absorbs.

The position reaches ArduPilot ~420-480 ms after the camera saw the scene; VISO_DELAY_MS can only cover
250 ms. Each published position is pushed forward by horizon_s along the velocity of the last window_s
of real VO outputs (least squares, so single-frame jitter is not amplified much). Choose
horizon_s = measured lag - VISO_DELAY_MS (~0.2 s today; fc_log_check.py reports the lag).

Tested before flight on SITL (450 ms delay: overshoot 1.63 -> 0.15 m) and on flights 5/6 logs
(lag 420-460 -> 240-270 ms, jitter +0.01 m): bags/flight6_analysis/.

The node feeds update() with the raw VO track (before NGPS corrections) and adds shift() to the
corrected output, so NGPS corrections never look like motion (a 0.6 m correction read as motion was
extrapolated into a 1.1 m output step in the flight 6 replay).

Guards:
  * step: a jump no real motion could make (an NGPS correction, an EKF origin shift) restarts the
    velocity window, so the jump itself is never extrapolated (flight 6: a 2.3 m NGPS step became a
    4 m spike without this);
  * staleness: no new VO sample for stale_s -> no prediction (never extrapolate a dead VO);
  * caps on speed and on the total forward shift.
"""
from __future__ import annotations

from collections import deque
from typing import Sequence, Tuple

import numpy as np


class PositionPredictor:
    def __init__(self, horizon_s: float = 0.0, window_s: float = 0.3, step_m: float = 0.4,
                 max_speed: float = 15.0, max_shift_m: float = 2.0, stale_s: float = 0.2, min_samples: int = 4):
        self.horizon_s = float(horizon_s)
        self.window_s = float(window_s)
        self.step_m = float(step_m)
        self.max_speed = float(max_speed)
        self.max_shift_m = float(max_shift_m)
        self.stale_s = float(stale_s)
        self.min_samples = int(min_samples)
        self.hist: deque = deque()
        self.resets = 0
        self.last_shift = 0.0

    def reset(self):
        self.hist.clear()

    def update(self, t: float, en: Sequence[float]):
        """A fresh VO output (time in seconds, any monotonic clock; east/north in metres)."""
        p = np.asarray(en, float)
        if self.hist:
            t0, p0 = self.hist[-1]
            dt = max(t - t0, 1e-3)
            if np.linalg.norm(p - p0) > self.step_m + self.max_speed * dt:
                self.hist.clear()
                self.resets += 1
        self.hist.append((float(t), p))
        while self.hist and self.hist[0][0] < t - self.window_s:
            self.hist.popleft()

    def velocity(self, t_now: float):
        if len(self.hist) < self.min_samples or t_now - self.hist[-1][0] > self.stale_s:
            return None
        ts = np.array([h[0] for h in self.hist])
        ps = np.array([h[1] for h in self.hist])
        tc = ts - ts.mean()
        den = float((tc ** 2).sum())
        if den <= 0:
            return None
        v = (tc[:, None] * (ps - ps.mean(0))).sum(0) / den
        sp = float(np.linalg.norm(v))
        if sp > self.max_speed:
            v = v * (self.max_speed / sp)
        return v

    def shift(self, t_now: float) -> np.ndarray:
        """Forward shift to add now: velocity x (horizon_s + age of the newest sample, when a keep-alive
        republishes between frames). Zero when prediction is off, not yet possible, or stale."""
        self.last_shift = 0.0
        if self.horizon_s <= 0:
            return np.zeros(2)
        v = self.velocity(t_now)
        if v is None:
            return np.zeros(2)
        age = max(0.0, t_now - self.hist[-1][0])
        s = v * (self.horizon_s + age)
        n = float(np.linalg.norm(s))
        if n > self.max_shift_m:
            s = s * (self.max_shift_m / n)
            n = self.max_shift_m
        self.last_shift = n
        return s

    def predict(self, t_now: float, en: Sequence[float]) -> Tuple[float, float]:
        """en pushed forward by shift(t_now). Use this when the track fed to update() is the one published."""
        p = np.asarray(en, float) + self.shift(t_now)
        return float(p[0]), float(p[1])


class RateLimitedOffset:
    """The NGPS map->odom offset as published: follows the filter's offset at no more than rate_mps.

    The NGPS filter steps its offset when a fix is accepted (up to ~2 m). Published as a step, EKF3 pulls
    it in within ~2 s and its velocity is off by ~0.9 m/s meanwhile (EKF3 Replay of flight 7, log 19).
    Moving the published offset at <= rate_mps bounds that disturbance. rate_mps <= 0: no limit.
    """

    def __init__(self, rate_mps: float = 0.4):
        self.rate = float(rate_mps)
        self.value = np.zeros(2)
        self.t = None

    def update(self, t: float, target) -> np.ndarray:
        target = np.asarray(target, float)
        if self.rate <= 0 or self.t is None:
            self.value = target.copy()
        else:
            d = target - self.value
            n = float(np.linalg.norm(d))
            step = self.rate * max(0.0, t - self.t)
            self.value = target.copy() if n <= step else self.value + d * (step / n)
        self.t = t
        return self.value

    def shift(self, delta) -> None:
        """A frame change (EKF origin move) applies to both, so it is never blended."""
        self.value = self.value + np.asarray(delta, float)
