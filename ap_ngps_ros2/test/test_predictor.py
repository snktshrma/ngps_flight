"""Forward prediction: pinned to the known answer on synthetic tracks (exact motion, noise, steps, stalls)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from ap_ngps_ros2.predictor import PositionPredictor  # noqa: E402

RATE = 25.0


def feed(pred, f, t_end, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for i in range(int(t_end * RATE)):
        t = i / RATE
        p = np.asarray(f(t)) + rng.normal(0, noise, 2)
        pred.update(t, p)
        out.append((t, pred.predict(t, p)))
    return out


def test_off_passes_through():
    p = PositionPredictor(horizon_s=0.0)
    for t, q in feed(p, lambda t: (2.0 * t, -t), 2.0):
        assert np.allclose(q, (2.0 * t, -t))


def test_constant_velocity_lands_on_the_future_position():
    p = PositionPredictor(horizon_s=0.2)
    out = feed(p, lambda t: (3.0 * t, 1.0 * t), 3.0)
    for t, q in out[20:]:
        assert np.allclose(q, (3.0 * (t + 0.2), 1.0 * (t + 0.2)), atol=1e-6)


def test_noise_is_not_blown_up():
    p = PositionPredictor(horizon_s=0.2)
    out = feed(p, lambda t: (0.0, 0.0), 10.0, noise=0.08)     # hovering, VO jitter 0.08 m
    err = np.array([np.linalg.norm(q) for _, q in out[20:]])
    assert err.std() < 0.2                                    # ~0.08 m in -> well under 0.2 m out


def test_step_is_not_extrapolated():
    # an NGPS correction: 2.3 m jump while hovering
    p = PositionPredictor(horizon_s=0.2)
    out = feed(p, lambda t: (0.0, 0.0) if t < 2.0 else (2.3, 0.0), 4.0)
    peak = max(abs(q[0]) for _, q in out)
    assert peak <= 2.3 + 1e-6 and p.resets == 1


def test_stale_vo_stops_predicting():
    p = PositionPredictor(horizon_s=0.2, stale_s=0.2)
    feed(p, lambda t: (3.0 * t, 0.0), 2.0)
    last_t = (int(2.0 * RATE) - 1) / RATE
    q = p.predict(last_t + 0.5, (3.0 * last_t, 0.0))          # keep-alive 0.5 s after the last frame
    assert np.allclose(q, (3.0 * last_t, 0.0))


def test_shift_is_capped():
    p = PositionPredictor(horizon_s=0.2, max_shift_m=0.5, max_speed=15.0)
    out = feed(p, lambda t: (10.0 * t, 0.0), 2.0)              # 10 m/s -> 2 m shift wanted
    t, q = out[-1]
    assert abs(q[0] - 10.0 * t - 0.5) < 1e-6


def test_ngps_corrections_are_not_extrapolated_when_fed_raw_vo():
    # node wiring: velocity from the raw VO track, shift added to the NGPS-corrected output
    p = PositionPredictor(horizon_s=0.2)
    out = []
    for i in range(int(4.0 * RATE)):
        t = i / RATE
        raw = np.array([2.0 * t, 0.0])                       # moving east at 2 m/s
        offset = np.array([0.0, 0.6]) if t >= 2.0 else np.zeros(2)   # a 0.6 m NGPS correction north
        p.update(t, raw)
        out.append(raw + offset + p.shift(t))
    out = np.array(out)
    steps = np.linalg.norm(np.diff(out, axis=0), axis=1)
    assert abs(steps.max() - np.hypot(0.08, 0.6)) < 1e-6      # one frame of motion + exactly the correction
    assert np.allclose(out[-1], [2.0 * (3.96 + 0.2), 0.6])


def test_offset_blend_is_rate_limited_and_lands_exactly():
    from ap_ngps_ros2.predictor import RateLimitedOffset
    b = RateLimitedOffset(rate_mps=0.4)
    b.update(0.0, (0.0, 0.0))
    out = [b.update(i / RATE, (2.2, 0.0)).copy() for i in range(1, int(8 * RATE))]   # a 2.2 m fix at t=0
    steps = np.linalg.norm(np.diff(np.r_[[[0.0, 0.0]], out], axis=0), axis=1)
    assert steps.max() <= 0.4 / RATE + 1e-9                        # never faster than 0.4 m/s
    assert np.allclose(out[-1], (2.2, 0.0))                         # and it gets there (5.5 s)
    assert np.linalg.norm(out[int(2 * RATE)]) < 1.0                 # still blending after 2 s


def test_offset_blend_zero_rate_is_a_step():
    from ap_ngps_ros2.predictor import RateLimitedOffset
    b = RateLimitedOffset(rate_mps=0.0)
    b.update(0.0, (0.0, 0.0))
    assert np.allclose(b.update(0.04, (2.2, 0.0)), (2.2, 0.0))
