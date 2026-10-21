"""The VPI-CUDA ORB backend must measure the same motion as the OpenCV one (pinned to a known shift)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
cv = pytest.importorskip('cv2')
fusion = pytest.importorskip('ap_ngps_ros2.vo_ngps_fusion_node')

W, H = 640, 360
F = (W / 2) / math.tan(math.radians(49.2 / 2))
P = np.array([W / 2, H / 2])


def textured(seed=0):
    rng = np.random.default_rng(seed)
    img = cv.GaussianBlur(rng.integers(0, 255, (H + 80, W + 80), np.uint8), (0, 0), 1.2)
    for _ in range(300):        # add corner-rich structure so ORB has something to hold on to
        x, y = rng.integers(0, W + 60, 2)
        cv.rectangle(img, (int(x), int(y)), (int(x + rng.integers(5, 30)), int(y + rng.integers(5, 30))),
                     int(rng.integers(0, 255)), -1)
    return img


def run(backend, dx, dy):
    big = textured()
    a = big[40:40 + H, 40:40 + W].copy()
    b = big[40 + dy:40 + dy + H, 40 + dx:40 + dx + W].copy()   # camera moved by (+dx, +dy) px
    vo = fusion.KeyframeVO(F, P, vmax=100.0, backend=backend)
    vo.start(0.0, a, (0.0, 0.0), 50.0, 0.0)
    assert vo.step(0.1, b, 50.0, 0.0)
    return vo.pose.copy()


def _vpi_available():
    try:
        import vpi  # noqa: F401
        return True
    except Exception:
        return False


def test_opencv_backend_recovers_known_shift():
    pose = run('opencv', 12, -7)
    sc = 50.0 / F                                  # metres per pixel at 50 m, heading 0 (north up)
    assert np.allclose(pose, [12 * sc, 7 * sc], atol=0.05)


@pytest.mark.skipif(not _vpi_available(), reason='NVIDIA VPI not installed')
def test_vpi_backend_matches_opencv():
    for dx, dy in ((12, -7), (-25, 18), (40, 0)):
        assert np.allclose(run('vpi_cuda', dx, dy), run('opencv', dx, dy), atol=0.05)


@pytest.mark.skipif(not _vpi_available(), reason='NVIDIA VPI not installed')
def test_vpi_failure_mid_run_falls_back_to_opencv():
    big = textured()
    a, b = big[40:40 + H, 40:40 + W].copy(), big[33:33 + H, 52:52 + W].copy()
    vo = fusion.KeyframeVO(F, P, vmax=100.0, backend='vpi_cuda')
    vo.start(0.0, a, (0.0, 0.0), 50.0, 0.0)

    def boom(*_):
        raise ValueError('VPI_ERROR_INVALID_ARGUMENT')
    vo.matcher.feat = boom                       # the GPU starts failing after the keyframe
    assert vo.step(0.1, b, 50.0, 0.0)
    assert vo.backend == 'opencv' and vo.fallback_reason
    sc = 50.0 / F
    assert np.allclose(vo.pose, [12 * sc, 7 * sc], atol=0.05)


def test_trt_matcher_does_not_create_its_own_cuda_context():
    # pycuda.autoinit makes a private context that breaks VPI in the same process
    src = (Path(__file__).resolve().parents[1] / 'src' / 'ap_ngps_ros2' / 'trt_matcher.py').read_text()
    assert 'import pycuda.autoinit' not in src and 'retain_primary_context' in src


def test_scale_change_waits_for_the_next_keyframe():
    big = textured()
    a = big[40:40 + H, 40:40 + W].copy()
    b = big[33:33 + H, 52:52 + W].copy()            # +12 px east, +7 px north of a
    vo = fusion.KeyframeVO(F, P, vmax=100.0, backend='opencv')
    vo.start(0.0, a, (0.0, 0.0), 50.0, 0.0)
    vo.scale = 1.10                                  # correction arrives mid-keyframe
    assert vo.step(0.1, b, 50.0, 0.0)
    sc = 50.0 / F
    assert np.allclose(vo.pose, [12 * sc, 7 * sc], atol=0.05)     # still the old scale: no jump
    vo.start(0.2, a, (0.0, 0.0), 50.0, 0.0)          # next keyframe latches the new scale
    assert vo.step(0.3, b, 50.0, 0.0)
    assert np.allclose(vo.pose, [12 * sc * 1.10, 7 * sc * 1.10], atol=0.05)
