#!/usr/bin/env python3
"""Live VO seeding and output frame (no ROS, no hardware).

/ap/tf must be ENU from the EKF ORIGIN. Flight 3: home moved 15 m from the origin when
ArduPilot armed (5.24 m in height), and the old home-relative output went with it.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from ap_ngps_ros2 import ap_frame  # noqa: E402
from ap_ngps_ros2.seed_tracker import SeedTracker  # noqa: E402

ORIGIN = (36.3242, 138.6393, 940.0)   # EKF origin, AMSL
DT = 0.05                               # /ap/geopose/filtered at 20 Hz


def ll(east, north):
    return ap_frame.en_to_ll(ORIGIN, east, north)


def feed(tr, t0, seconds, en, up, armed=None, vel=(0.0, 0.0)):
    """Drive the tracker with origin + geopose (+ status) for a while; returns the end time."""
    t, en = t0, np.array(en, float)
    while t < t0 + seconds - 1e-9:
        tr.on_origin(*ORIGIN)
        if armed is not None:
            tr.on_status(armed, t)
        lat, lon = ll(*en)
        tr.on_geopose(lat, lon, ORIGIN[2] + up, t)
        en = en + np.array(vel) * DT
        t += DT
    return t


def test_matches_ardupilot_location_scaling():
    # AP_Common/Location: 1e-7 deg of latitude is LATLON_TO_M metres
    e, n = ap_frame.ll_to_en(ORIGIN, ORIGIN[0] + 1e-7, ORIGIN[1])
    assert math.isclose(n, 0.011131884502145034, rel_tol=1e-6) and e == 0.0
    e, n = ap_frame.ll_to_en(ORIGIN, ORIGIN[0], ORIGIN[1] + 1e-7)
    assert math.isclose(e, 0.011131884502145034 * math.cos(math.radians(ORIGIN[0])), rel_tol=1e-6)
    for en in [(0.0, 0.0), (250.0, -480.0), (-1200.0, 900.0)]:
        assert np.allclose(ap_frame.ll_to_en(ORIGIN, *ll(*en)), en, atol=1e-6)


def test_nothing_to_send_without_an_origin():
    tr = SeedTracker()
    lat, lon = ll(3.0, 4.0)
    tr.on_geopose(lat, lon, 945.0, 0.0)
    assert tr.output_en() is None           # no (0,0), no guess
    tr.on_origin(0.0, 0.0, 0.0)              # AP_DDS sends zeros before the EKF has an origin
    assert tr.output_en() is None
    assert 'EKF origin' in tr.waiting_for(0.0)


def test_home_move_at_arming_does_not_move_the_output():
    """Origin fixed, home moves 15 m at arming: output stays continuous and origin-relative."""
    tr = SeedTracker()
    takeoff = (15.0, -2.0)   # where the aircraft sits, from the origin (home at boot was elsewhere)
    t = feed(tr, 0.0, 3.0, takeoff, 5.24, armed=False)
    before = tr.output_en().copy()
    # Arming: ArduPilot moves home to here, so /ap/pose/filtered jumps by 15 m and 5.24 m.
    # The origin and the EKF's location do not move, and neither may the output.
    t = feed(tr, t, 3.0, takeoff, 5.24, armed=True)
    assert tr.try_seed()
    assert np.allclose(before, takeoff, atol=1e-6)
    assert np.allclose(tr.output_en(), takeoff, atol=1e-6)
    assert math.isclose(tr.up, 5.24, abs_tol=1e-6)   # z from the origin too, not from home
    assert math.isclose(tr.height(), 0.0, abs_tol=1e-6)   # ...while VO scales by height above takeoff
    feed(tr, t, 1.0, takeoff, 25.24, armed=True)
    assert math.isclose(tr.height(), 20.0, abs_tol=1e-6)


def test_output_before_seed_is_the_ekf_and_after_is_seed_then_vo():
    tr = SeedTracker()
    t = feed(tr, 0.0, 1.0, (7.0, 8.0), 0.0, armed=False)
    assert np.allclose(tr.output_en(), (7.0, 8.0))
    feed(tr, t, 2.0, (7.0, 8.0), 0.0, armed=True)
    assert tr.try_seed() and not tr.try_seed()
    assert np.allclose(tr.output_en(), (7.0, 8.0))
    assert np.allclose(tr.output_en((9.0, 8.5)), (9.0, 8.5))


def test_does_not_seed_before_arming():
    tr = SeedTracker()
    feed(tr, 0.0, 10.0, (5.0, 5.0), 0.0, armed=False)
    assert not tr.try_seed()
    assert tr.waiting_for(10.0) == 'not armed'


def test_disarm_before_seeding_restarts_collection():
    tr = SeedTracker(seed_samples=50)
    t = feed(tr, 0.0, 1.5, (0.0, 0.0), 0.0, armed=True)
    assert tr.seed
    feed(tr, t, 0.5, (0.0, 0.0), 0.0, armed=False)
    assert not tr.seed and tr.armed_since is None


def test_origin_move_after_seeding_shifts_state_with_the_frame():
    tr = SeedTracker()
    t = feed(tr, 0.0, 1.0, (0.0, 0.0), 0.0, armed=False)
    t = feed(tr, t, 2.0, (0.0, 0.0), 0.0, armed=True)
    assert tr.try_seed()
    # EKF reset: origin re-set 10 m north and 1 m lower; the aircraft has not moved
    new_lat, new_lon = ll(0.0, 10.0)
    d, dz = tr.on_origin(new_lat, new_lon, ORIGIN[2] - 1.0)
    assert np.allclose(d, (0.0, -10.0), atol=1e-3) and math.isclose(dz, 1.0)
    assert np.allclose(tr.seed_xy, (0.0, -10.0), atol=1e-3)
    assert math.isclose(tr.height(), 0.0, abs_tol=1e-6)


def test_origin_jitter_is_not_a_move():
    tr = SeedTracker()
    tr.on_origin(*ORIGIN)
    assert tr.on_origin(ORIGIN[0] + 1e-7, ORIGIN[1], ORIGIN[2] + 0.01) is None
    assert tr.origin_moves == 0


@pytest.mark.parametrize('missing,expect', [
    ('origin', 'EKF origin'), ('geo', 'EKF position'), ('status', '/ap/status')])
def test_reports_why_it_cannot_seed(missing, expect):
    tr = SeedTracker()
    if missing != 'origin':
        tr.on_origin(*ORIGIN)
    if missing not in ('origin', 'geo'):
        tr.on_geopose(*ll(0.0, 0.0), ORIGIN[2], 0.0)
    assert expect in tr.waiting_for(0.0)
