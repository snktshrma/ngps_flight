#!/usr/bin/env bash
# Build FP16 TensorRT engine from a LightGlue-ONNX export.
# Run inside the container: its TensorRT matches the host, and engines are locked to the
# TensorRT version that built them.
#
# Env overrides:
#   LIGHTGLUE_ONNX  — path to LightGlue-ONNX repo (for _prepare_tensorrt_onnx)
#   ONNX            — input .onnx from `lightglue-onnx export`
#   ENGINE          — output .engine path
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WEIGHTS_DIR="${PKG_DIR}/weights"
mkdir -p "${WEIGHTS_DIR}"

LIGHTGLUE_ONNX="${LIGHTGLUE_ONNX:-${HOME}/LightGlue-ONNX}"
ONNX="${ONNX:-${LIGHTGLUE_ONNX}/weights/superpoint_lightglue_k1024_384x216.onnx}"
ENGINE="${ENGINE:-${WEIGHTS_DIR}/superpoint_lightglue_fp16.engine}"
TRT_ONNX="${WEIGHTS_DIR}/superpoint_lightglue.trt-ready.onnx"

if [[ ! -f "${ONNX}" ]]; then
  echo "ONNX not found: ${ONNX}" >&2
  exit 1
fi

if ! command -v trtexec >/dev/null 2>&1; then
  echo "trtexec not found. Install TensorRT, or run inside the vps-jp7 container." >&2
  exit 1
fi

# Optional TRT parser fix (Reduce-axis initializers) from LightGlue-ONNX.
# Passed as argv so the heredoc can stay quoted.
if [[ -f "${LIGHTGLUE_ONNX}/lightglue_dynamo/scripts/benchmark.py" ]]; then
  python3 - <<'PY' "${ONNX}" "${TRT_ONNX}" "${LIGHTGLUE_ONNX}"
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[3])
from lightglue_dynamo.scripts.benchmark import _prepare_tensorrt_onnx
_prepare_tensorrt_onnx(Path(sys.argv[1]), Path(sys.argv[2]))
print(f"Prepared TRT ONNX: {sys.argv[2]}")
PY
  BUILD_ONNX="${TRT_ONNX}"
else
  echo "LightGlue-ONNX not found at ${LIGHTGLUE_ONNX}; skipping the TRT parser fix." >&2
  echo "Set LIGHTGLUE_ONNX=/path/to/LightGlue-ONNX if the engine build fails." >&2
  BUILD_ONNX="${ONNX}"
fi

echo "Building FP16 engine from ${BUILD_ONNX}"
trtexec \
  --onnx="${BUILD_ONNX}" \
  --saveEngine="${ENGINE}" \
  --fp16 \
  --memPoolSize=workspace:2048

ls -lh "${ENGINE}"
echo "Engine ready: ${ENGINE}"
