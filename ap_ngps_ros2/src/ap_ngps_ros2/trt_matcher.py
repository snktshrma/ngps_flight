"""Pure TensorRT inference for fused SuperPoint+LightGlue."""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pycuda.autoinit  # noqa: F401
import pycuda.driver as cuda
import tensorrt as trt

from ap_ngps_ros2.superpoint_preprocess import preprocess_pair_bgr


def parse_trt_matches(
    keypoints: np.ndarray,
    matches: np.ndarray,
    mscores: np.ndarray,
    match_threshold: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Filter TRT matches and return (m_kpts0, m_kpts1) as float32 (N,2)."""
    if matches.size == 0:
        empty = np.empty((0, 2), dtype=np.float32)
        return empty, empty
    valid = mscores > match_threshold
    if not np.any(valid):
        empty = np.empty((0, 2), dtype=np.float32)
        return empty, empty
    sel = matches[valid]
    idx0 = sel[:, 1]
    idx1 = sel[:, 2]
    kpts0 = keypoints[0, idx0].astype(np.float32, copy=False)
    kpts1 = keypoints[1, idx1].astype(np.float32, copy=False)
    return kpts0, kpts1


def _resolve_tensor_shape(
    context: trt.IExecutionContext,
    engine: trt.ICudaEngine,
    name: str,
    shape: tuple[int, ...],
) -> tuple[int, ...]:
    if not any(d < 0 for d in shape):
        return shape
    max_bytes = context.get_max_output_size(name)
    if max_bytes <= 0:
        raise RuntimeError(f"Cannot resolve dynamic shape for tensor {name!r}")
    dtype = trt.nptype(engine.get_tensor_dtype(name))
    itemsize = np.dtype(dtype).itemsize
    if name == "matches":
        return (max_bytes // (3 * itemsize), 3)
    if name == "mscores":
        return (max_bytes // itemsize,)
    return tuple(max_bytes // itemsize if d < 0 else d for d in shape)


class TrtMatcher:
    def __init__(
        self,
        engine_path: str,
        match_threshold: float = 0.5,
        warmup: bool = True,
    ) -> None:
        path = Path(engine_path)
        if not path.is_file():
            raise FileNotFoundError(f"TensorRT engine not found: {engine_path}")

        self.match_threshold = match_threshold
        self.logger = trt.Logger(trt.Logger.WARNING)
        runtime = trt.Runtime(self.logger)

        with path.open("rb") as f:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(
                f"Failed to deserialize TensorRT engine: {engine_path}. "
                "Check JetPack TensorRT version matches build environment."
            )

        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()
        self._input_shape = tuple(self.engine.get_tensor_shape(self.engine.get_tensor_name(0)))

        self._host: dict[str, np.ndarray] = {}
        self._device: dict[str, cuda.DeviceAllocation] = {}
        self._bindings: list[int] = []

        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = _resolve_tensor_shape(
                self.context, self.engine, name, tuple(self.engine.get_tensor_shape(name))
            )
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            size = int(np.prod(shape))
            host = cuda.pagelocked_empty(size, dtype)
            device = cuda.mem_alloc(host.nbytes)
            self._host[name] = host
            self._device[name] = device
            self.context.set_tensor_address(name, int(device))
            self._bindings.append(int(device))

        if warmup:
            self._warmup()

    def _warmup(self) -> None:
        _, _, h, w = self._input_shape
        dummy = np.zeros((2, 1, h, w), dtype=np.float32)
        self._execute(dummy)

    def _execute(self, images: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        input_name = self.engine.get_tensor_name(0)
        host_in = self._host[input_name]
        np.copyto(host_in, images.ravel())
        cuda.memcpy_htod_async(self._device[input_name], host_in, self.stream)

        self.context.execute_async_v3(stream_handle=self.stream.handle)

        outputs: dict[str, np.ndarray] = {}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                continue
            cuda.memcpy_dtoh_async(self._host[name], self._device[name], self.stream)
            shape = tuple(self.context.get_tensor_shape(name))
            outputs[name] = self._host[name].reshape(shape).copy()

        self.stream.synchronize()
        keypoints = outputs["keypoints"]
        matches = outputs["matches"]
        mscores = outputs["mscores"]
        count = int(np.count_nonzero(mscores))
        if count < mscores.size:
            mscores = mscores[:count]
            matches = matches[:count]
        return keypoints, matches, mscores

    def match(
        self,
        kernel_bgr: np.ndarray,
        camera_bgr: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
        t0 = time.perf_counter()
        _, _, h, w = self._input_shape
        images = preprocess_pair_bgr(kernel_bgr, camera_bgr, expected_hw=(h, w))
        keypoints, matches, mscores = self._execute(images)
        m_kpts0, m_kpts1 = parse_trt_matches(
            keypoints, matches, mscores, self.match_threshold
        )
        latency_ms = (time.perf_counter() - t0) * 1000.0
        mean_score = float(mscores.mean()) if mscores.size else 0.0
        stats = {
            "num_matches": float(len(m_kpts0)),
            "mean_score": mean_score,
            "latency_ms": latency_ms,
        }
        return m_kpts0, m_kpts1, stats
