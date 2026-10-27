#!/usr/bin/env python3

import numpy as np
import pytest

from ap_ngps_ros2.superpoint_preprocess import (
    EXPECTED_HW,
    preprocess_pair_bgr,
    preprocess_superpoint_bgr,
)
from ap_ngps_ros2.trt_matcher import parse_trt_matches

H, W = EXPECTED_HW


def test_preprocess_superpoint_bgr_shape_and_range():
    bgr = np.zeros((H, W, 3), dtype=np.uint8)
    bgr[..., 0] = 255
    out = preprocess_superpoint_bgr(bgr)
    assert out.shape == (1, H, W)
    assert out.dtype == np.float32
    assert 0.0 <= out.min() <= out.max() <= 1.0


def test_preprocess_superpoint_bgr_grayscale_weights():
    white_bgr = np.full((H, W, 3), 255, dtype=np.uint8)
    out = preprocess_superpoint_bgr(white_bgr)
    assert np.allclose(out, 1.0, atol=1e-5)


def test_preprocess_pair_bgr_stacks_batch_dim():
    kernel = np.zeros((H, W, 3), dtype=np.uint8)
    camera = np.full((H, W, 3), 128, dtype=np.uint8)
    batch = preprocess_pair_bgr(kernel, camera)
    assert batch.shape == (2, 1, H, W)
    assert batch.dtype == np.float32
    assert not np.allclose(batch[0], batch[1])


def test_preprocess_pair_bgr_rejects_wrong_shape():
    kernel = np.zeros((H, W, 3), dtype=np.uint8)
    camera = np.zeros((H, W - 1, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match="216x384"):
        preprocess_pair_bgr(kernel, camera)


def test_parse_trt_matches_filters_by_threshold():
    keypoints = np.array(
        [[[10.0, 20.0], [30.0, 40.0]], [[11.0, 21.0], [31.0, 41.0]]],
        dtype=np.float32,
    )
    matches = np.array([[0, 0, 0], [0, 1, 1]], dtype=np.int64)
    mscores = np.array([0.8, 0.3], dtype=np.float32)
    k0, k1 = parse_trt_matches(keypoints, matches, mscores, match_threshold=0.5)
    assert k0.shape == (1, 2)
    np.testing.assert_allclose(k0[0], [10.0, 20.0])
    np.testing.assert_allclose(k1[0], [11.0, 21.0])


def test_parse_trt_matches_empty_when_none_pass_threshold():
    keypoints = np.zeros((2, 2, 2), dtype=np.float32)
    matches = np.array([[0, 0, 0]], dtype=np.int64)
    mscores = np.array([0.1], dtype=np.float32)
    k0, k1 = parse_trt_matches(keypoints, matches, mscores, match_threshold=0.5)
    assert k0.shape == (0, 2)
    assert k1.shape == (0, 2)
