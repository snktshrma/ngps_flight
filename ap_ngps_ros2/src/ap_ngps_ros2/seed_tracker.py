"""When to seed live VO, in the frame ArduPilot expects, and how to keep it there.

Kept free of ROS so the edge cases can be unit tested.

Everything is ENU metres from the EKF ORIGIN (/ap/gps_global_origin/filtered), with the
vehicle's position taken from /ap/geopose/filtered -- never from /ap/pose/filtered,
which is relative to HOME. Home moves when ArduPilot arms (15 m on flight 3), the
origin does not, so a home move changes nothing here. See ap_frame.py.

  * Seed once armed for ``settle_s``, from the EKF position in that window, and record
    the height of the takeoff point so VO can scale by height above it.
  * If the origin itself changes (an EKF reset), ``on_origin`` returns the shift for
    the caller to apply to its state; a change in the seeding window restarts it.
  * Disarming before seeding throws the half-collected samples away.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

from ap_ngps_ros2 import ap_frame


class SeedTracker:
    def __init__(self, seed_samples: int = 5, settle_s: float = 1.0, origin_move_m: float = 0.5):
        self.n = max(1, int(seed_samples))
        self.settle_s = float(settle_s)
        self.origin_move_m = float(origin_move_m)
        self.armed = False
        self.armed_since: Optional[float] = None
        self.origin: Optional[Tuple[float, float, float]] = None   # EKF origin lat, lon, alt (AMSL)
        self.pos_xy: Optional[np.ndarray] = None   # EKF position, ENU m from the origin
        self.up: Optional[float] = None            # EKF height above the origin
        self.seed: List[Tuple[float, float, float]] = []
        self.seeded = False
        self.seed_xy: Optional[np.ndarray] = None
        self.ground_up: Optional[float] = None     # origin-relative height of the takeoff point
        self.status_msgs = 0
        self.geo_msgs = 0
        self.origin_moves = 0

    # ---- inputs ----
    def on_status(self, armed: bool, now: float) -> None:
        self.status_msgs += 1
        self.armed = bool(armed)
        if armed and self.armed_since is None:
            self.armed_since = now
        elif not armed and self.armed_since is not None and not self.seeded:
            self.armed_since = None
            self.seed.clear()

    def seeding_window(self, now: float) -> bool:
        return (not self.seeded and self.armed_since is not None
                and now - self.armed_since > self.settle_s)

    def on_origin(self, lat: float, lon: float, alt: float) -> Optional[Tuple[np.ndarray, float]]:
        """Record the EKF origin; returns (dEN, dUp) to shift by if it moved."""
        if not ap_frame.valid_ll(lat, lon):
            return None
        new = (float(lat), float(lon), float(alt))
        if self.origin is None:
            self.origin = new
            return None
        # A fixed ground point's coordinates change by where the old origin sits in the new one.
        d = np.array(ap_frame.ll_to_en(new, self.origin[0], self.origin[1]))
        dz = self.origin[2] - new[2]
        if float(np.linalg.norm(d)) < self.origin_move_m and abs(dz) < self.origin_move_m:
            return None
        self.origin = new
        self.origin_moves += 1
        if not self.seeded:
            self.seed.clear()
            return None
        self.seed_xy = self.seed_xy + d
        self.ground_up = self.ground_up + dz
        if self.pos_xy is not None:
            self.pos_xy = self.pos_xy + d
            self.up = self.up + dz
        return d, dz

    def on_geopose(self, lat: float, lon: float, alt: float, now: float) -> None:
        self.geo_msgs += 1
        p = ap_frame.ekf_en_up(self.origin, lat, lon, alt)
        if p is None or not np.isfinite(p).all():
            return
        self.pos_xy = np.array(p[:2])
        self.up = float(p[2])
        if self.seeding_window(now):
            self.seed.append(p)

    # ---- seeding ----
    def try_seed(self) -> bool:
        """Seed once there are enough samples; True only on the call that seeds."""
        if self.seeded or self.origin is None or len(self.seed) < self.n:
            return False
        s = np.array(self.seed)
        self.seeded = True
        self.seed_xy = s[:, :2].mean(axis=0)
        self.ground_up = float(s[:, 2].mean())
        return True

    def height(self) -> float:
        """Height above the takeoff point, for VO scale and the map crop (0 before seeding)."""
        if not self.seeded or self.up is None:
            return 0.0
        return self.up - self.ground_up

    def output_ready(self) -> bool:
        """Is there an origin-relative position to send? Never send one we do not have."""
        return self.origin is not None and self.pos_xy is not None

    def output_en(self, vo_en=None):
        """The position to send: VO's (already corrected) once running, else the seed point,
        else the EKF's own; always ENU from the EKF origin. None when there is none."""
        if not self.output_ready():
            return None
        if vo_en is not None:
            return np.asarray(vo_en, float)
        if self.seeded:
            return self.seed_xy
        return self.pos_xy

    def armed_for(self, now: float) -> float:
        return 0.0 if self.armed_since is None else now - self.armed_since

    def waiting_for(self, now: float, frames: int = 1) -> Optional[str]:
        """Why seeding has not happened yet, or None once seeded."""
        if self.seeded:
            return None
        if self.origin is None:
            return 'no EKF origin (/ap/gps_global_origin/filtered)'
        if self.geo_msgs == 0 or self.pos_xy is None:
            return 'no EKF position (/ap/geopose/filtered)'
        if self.status_msgs == 0:
            return 'no /ap/status (cannot see arming)'
        if self.armed_since is None:
            return 'not armed'
        if frames == 0:
            return 'no camera frames'
        if len(self.seed) < self.n:
            return 'collecting samples'
        return 'waiting for a camera frame'
