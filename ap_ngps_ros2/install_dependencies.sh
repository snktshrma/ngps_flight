#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REQ_FILE="${NGPS_REQUIREMENTS_FILE:-${SCRIPT_DIR}/requirements.txt}"

if command -v ros2 >/dev/null 2>&1; then
    :
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
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

TORCH_INDEX_URL="${NGPS_TORCH_INDEX_URL:-https://download.pytorch.org/whl/cu121}"
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
    ros-humble-geographic-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport

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
    echo "Installing PyTorch wheels from ${TORCH_INDEX_URL}..."
    "${PIP[@]}" install "${PIP_FLAGS[@]}" torch torchvision --index-url "${TORCH_INDEX_URL}"
elif [[ "${INSTALL_TORCH}" == "cpu" ]]; then
    echo "Installing CPU-only PyTorch wheels..."
    "${PIP[@]}" install "${PIP_FLAGS[@]}" torch torchvision --index-url https://download.pytorch.org/whl/cpu
fi

echo "Installing NGPS Python requirements from ${REQ_FILE}..."
"${PIP[@]}" install "${PIP_FLAGS[@]}" -r "${REQ_FILE}"

echo "NGPS Python stack installed."
python3 - <<'PY'
import importlib
for name in ("torch", "lightglue", "pyproj", "PIL", "cv2"):
    importlib.import_module(name)
print("import smoke check ok")
PY
