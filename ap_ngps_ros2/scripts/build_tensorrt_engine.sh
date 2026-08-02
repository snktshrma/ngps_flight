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

ONNX="${ONNX:-${LIGHTGLUE_ONNX:-/home/rmackay9/LightGlue-ONNX}/weights/superpoint_lightglue_k1024_384x216.onnx}"
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

# Optional TRT parser fix (Reduce-axis initializers) from LightGlue-ONNX
if [[ -f "/home/rmackay9/LightGlue-ONNX/lightglue_dynamo/scripts/benchmark.py" ]]; then
  python3 - <<'PY' "${ONNX}" "${TRT_ONNX}"
import sys
from pathlib import Path
sys.path.insert(0, "/home/rmackay9/LightGlue-ONNX")
from lightglue_dynamo.scripts.benchmark import _prepare_tensorrt_onnx
_prepare_tensorrt_onnx(Path(sys.argv[1]), Path(sys.argv[2]))
print(f"Prepared TRT ONNX: {sys.argv[2]}")
PY
  BUILD_ONNX="${TRT_ONNX}"
else
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
