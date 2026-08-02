#!/usr/bin/env bash
# Source inside distrobox before running TRT inference or ROS node with tensorrt backend.

TRT_LIB_PATHS=(
  "/usr/lib/aarch64-linux-gnu"
  "/usr/local/cuda/lib64"
  "/usr/lib/aarch64-linux-gnu/nvidia"
)

# JetPack TensorRT install locations (first existing wins)
for candidate in \
  "/usr/lib/python3.10/dist-packages/tensorrt" \
  "/usr/lib/python3.8/dist-packages/tensorrt" \
  "/usr/src/tensorrt/lib"; do
  if [[ -d "${candidate}" ]]; then
    TRT_LIB_PATHS+=("${candidate}")
  fi
done

export LD_LIBRARY_PATH="$(IFS=:; echo "${TRT_LIB_PATHS[*]}"):${LD_LIBRARY_PATH:-}"

if ! python3 -c "import tensorrt" 2>/dev/null; then
  echo "WARNING: import tensorrt failed after setting LD_LIBRARY_PATH" >&2
  echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH}" >&2
else
  python3 -c "import tensorrt as trt; print('tensorrt', trt.__version__)"
fi
