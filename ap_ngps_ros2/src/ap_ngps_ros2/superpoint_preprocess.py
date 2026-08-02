"""SuperPoint input preprocessing (matches LightGlue-ONNX superpoint.py)."""

from __future__ import annotations

import numpy as np

EXPECTED_HW = (216, 384)  # default NGPS resolution after 0.6x camera resize
_LUMA_WEIGHTS = np.array([0.299, 0.587, 0.114], dtype=np.float32)


def preprocess_superpoint_bgr(image_bgr: np.ndarray) -> np.ndarray:
    """Convert BGR uint8 (H,W,3) to SuperPoint luma float32 (1,H,W)."""
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError(f"Expected BGR image (H,W,3), got shape {image_bgr.shape}")
    rgb = image_bgr[..., ::-1].astype(np.float32) / 255.0
    luma = (rgb * _LUMA_WEIGHTS).sum(axis=-1, keepdims=True)  # (H,W,1)
    return luma.transpose(2, 0, 1)  # (1,H,W)


def _assert_hw(image_bgr: np.ndarray, label: str, expected_hw: tuple[int, int]) -> None:
    h, w = image_bgr.shape[:2]
    expected_h, expected_w = expected_hw
    if (h, w) != (expected_h, expected_w):
        raise ValueError(
            f"{label} must be {expected_h}x{expected_w} (HxW), got {h}x{w}"
        )


def preprocess_pair_bgr(
    kernel_bgr: np.ndarray,
    camera_bgr: np.ndarray,
    expected_hw: tuple[int, int] | None = None,
) -> np.ndarray:
    """Stack kernel and camera into batched SuperPoint input (2,1,H,W)."""
    hw = expected_hw if expected_hw is not None else EXPECTED_HW
    _assert_hw(kernel_bgr, "kernel", hw)
    _assert_hw(camera_bgr, "camera", hw)
    kernel = preprocess_superpoint_bgr(kernel_bgr)
    camera = preprocess_superpoint_bgr(camera_bgr)
    return np.stack([kernel, camera], axis=0).astype(np.float32, copy=False)
