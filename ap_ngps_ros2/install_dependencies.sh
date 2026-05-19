#!/bin/bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REQ_FILE="${NGPS_REQUIREMENTS_FILE:-${SCRIPT_DIR}/requirements.txt}"

if command -v ros2 >/dev/null 2>&1; then
    :
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    set +u
    source /opt/ros/humble/setup.bash
    set -u
else
    echo "Error: ROS 2 Humble is not available. Source /opt/ros/humble/setup.bash first."
    exit 1
fi

if [[ "$(id -u)" -eq 0 ]]; then
    APT=(apt-get)
    PIP=(pip3)
    PIP_FLAGS=(--no-cache-dir)
else
    APT=(sudo apt-get)
    PIP=(pip3)
    PIP_FLAGS=(--user)
fi

TORCH_INDEX_URL="${NGPS_TORCH_INDEX_URL:-https://download.pytorch.org/whl/cu126}"
INSTALL_TORCH="${NGPS_INSTALL_TORCH:-auto}"

echo "Installing NGPS ROS 2 apt dependencies..."
"${APT[@]}" update
"${APT[@]}" install -y \
    git \
    python3-pip \
    python3-dev \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-std-msgs \
    ros-humble-geographic-msgs

if [[ ! -f "${REQ_FILE}" ]]; then
    echo "Error: requirements file not found: ${REQ_FILE}"
    exit 1
fi

if [[ "${INSTALL_TORCH}" == "auto" ]]; then
  if ! python3 -c "import torch" 2>/dev/null; then
    INSTALL_TORCH=yes
  else
    INSTALL_TORCH=no
  fi
fi

if [[ "${INSTALL_TORCH}" == "yes" ]]; then
    echo "Installing PyTorch and Torchvision from Jetson AI Lab wheels..."
    
    # We use --extra-index-url so pip fallback-checks standard PyPI for basic dependencies 
    # but grabs the heavy torch/torchvision binaries directly from your custom URL.
    "${PIP[@]}" install "${PIP_FLAGS[@]}" \
    	--ignore-installed \
    	--no-build-isolation \
        "torch==2.11.0" \
        "torchvision==0.26.0" \
        --extra-index-url "${TORCH_INDEX_URL}"

elif [[ "${INSTALL_TORCH}" == "cpu" ]]; then
    echo "Installing CPU-only PyTorch wheels..."
    "${PIP[@]}" install "${PIP_FLAGS[@]}" torch torchvision --index-url https://download.pytorch.org/whl/cpu
fi
echo "Installing NGPS Python requirements from ${REQ_FILE}..."
"${PIP[@]}" install "${PIP_FLAGS[@]}" \
    --no-build-isolation \
    -r "${REQ_FILE}" \
    --index-url "${TORCH_INDEX_URL}" \
    --extra-index-url https://pypi.org/simple

echo "NGPS Python stack installed."
python3 - <<'PY'
import importlib
for name in ("torch", "lightglue", "pyproj", "PIL", "cv2"):
    importlib.import_module(name)
print("import smoke check ok")
PY
