# NGPS ROS2 Localization Package

NGPS localization using LightGlue for ROS2.

**Jetson + TensorRT:** [Run](#tensorrt-on-jetson-recommended) · [Build engine](#build-tensorrt-engine) · [Troubleshooting](#troubleshooting)

## Features

- Real-time camera image processing
- LightGlue-based feature matching
- Rotation detection with multiple methods
- Pose estimation and tracking
- Global coordinate extraction (WGS84 and ECEF)
- Debug visualization
- Configurable parameters

## Dependencies

```bash
sudo apt update
sudo apt install ros-humble-rclpy ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-std-msgs ros-humble-cv-bridge ros-humble-image-transport
```

```bash
pip install -r requirements.txt
```

## Installation

1. Clone the repository:
```bash
cd /path/to/workspace/src
git clone <repository-url>
cd ap_ngps_ros2
```

2. Install Python dependencies:
```bash
pip install -r requirements.txt
```

3. Build the package:
```bash
cd /path/to/workspace
colcon build --packages-select ap_ngps_ros2
source install/setup.bash
```

## Usage

### Basic Usage

1. Launch the NGPS localization node:
```bash
ros2 launch ap_ngps_ros2 ngps_localization.launch.py
```

2. With a reference image:
```bash
ros2 launch ap_ngps_ros2 ngps_localization.launch.py reference_image_path:=/path/to/reference/image.tif
```

3. With custom camera topic:
```bash
ros2 launch ap_ngps_ros2 ngps_localization.launch.py camera_topic:=/camera/image_raw
```

### Parameters

**Launch Arguments:**
- `reference_image_path`: Path to the reference image for localization
- `camera_topic`: Camera topic to subscribe to (default: `/camera/image_raw`)
- `config_file`: Path to the YAML configuration file (default: `config/ngps_config.yaml`)

**YAML Configuration Parameters:**
- `kernel_size`: Size of the kernel for feature extraction (default: 300)
- `match_threshold`: Threshold for feature matching (default: 0.5)
- `min_matches`: Minimum number of matches required (default: 20)
- `max_rotation_change`: Maximum allowed rotation change per frame (default: 30.0 degrees)
- `rotation_std_threshold`: Maximum standard deviation for recent rotations (default: 15.0 degrees)
- `enable_rotation_smoothing`: Enable rotation smoothing (default: true)
- `enable_rotation_validation`: Enable rotation validation (default: true)
- `frame_id`: Frame ID for published messages (default: "map")

**Georeferencing Parameters (for global coordinates):**
- `reference_min_lon`: Minimum longitude (west edge) of reference image in decimal degrees (default: 0.0)
- `reference_min_lat`: Minimum latitude (south edge) of reference image in decimal degrees (default: 0.0)
- `reference_max_lon`: Maximum longitude (east edge) of reference image in decimal degrees (default: 0.0)
- `reference_max_lat`: Maximum latitude (north edge) of reference image in decimal degrees (default: 0.0)
- `reference_altitude`: Reference altitude in meters (AGL or MSL) (default: 0.0)
- `enable_global_coordinates`: Enable publishing of global coordinates (WGS84 and ECEF) (default: false)

### Published Topics

- `/ngps/pose` (geometry_msgs/msg/PoseStamped): Current pose with timestamp (local coordinates)
- `/ngps/position` (geometry_msgs/msg/PointStamped): Current position with timestamp (local coordinates)
- `/ngps/rotation` (std_msgs/msg/Float64): Current rotation angle
- `/ngps/debug_image` (sensor_msgs/msg/Image): Debug visualization with timestamp
- `/ngps/global_position` (sensor_msgs/msg/NavSatFix): Global position in WGS84 coordinates (lat/lon/alt) - *only published if `enable_global_coordinates` is true*
- `/ngps/ecef_position` (geometry_msgs/msg/PointStamped): Position in ECEF (Earth-Centered, Earth-Fixed) coordinates - *only published if `enable_global_coordinates` is true*

### Subscribed Topics

- `/camera/image_raw` (sensor_msgs/msg/Image): Input camera images

## Configuration

Edit `config/ngps_config.yaml` to modify default parameters.

### Enabling Global Coordinates

To enable global coordinate extraction and publishing:

1. **Configure georeferencing parameters** in `config/ngps_config.yaml`:
   ```yaml
   reference_min_lon: -122.4194  # West edge longitude
   reference_min_lat: 37.7749     # South edge latitude
   reference_max_lon: -122.4094   # East edge longitude
   reference_max_lat: 37.7849      # North edge latitude
   reference_altitude: 100.0       # Altitude in meters
   enable_global_coordinates: true
   ```

2. **Determine reference image bounding box**:
   - If reference image is a GeoTIFF, can extract the bounding box using tools like `gdalinfo`
   - For satellite imagery, use the coordinates from the imagery provider
   - The bounding box should be in WGS84 (EPSG:4326) decimal degrees

3. **Global coordinates will be published** to:
   - `/ngps/global_position` (NavSatFix): WGS84 latitude, longitude, altitude
   - `/ngps/ecef_position` (PointStamped): ECEF coordinates in meters

## TensorRT on Jetson (recommended)

The node loads the TensorRT engine in-process; everything runs inside the container. One-time
setup: build the `.engine` (below), then run the three launcher steps.

### Run (after engine is built)

| # | Command |
|---|---------|
| 1 | `~/ngps_ws/src/ngps_flight/scripts/run_sitl_stack.sh` |
| 2 | `~/ngps_ws/src/ngps_flight/scripts/run_sat_cam.sh` *(wait for GPS in SITL)* |
| 3 | `~/ngps_ws/src/ngps_flight/scripts/run_ngps.sh` |

**Check:** `ros2 topic hz /odometry/vps` and `ros2 topic echo /ngps/pose --once`.

**Config** (`config/ngps_config.yaml` — defaults for 640×360 sat cam):

```yaml
inference_backend: "tensorrt"
camera_resize_scale: 1.0
max_keypoints: 1024
tensorrt_engine_path: "/path/to/superpoint_lightglue_k1024_640x360_fp16.engine"
reference_image_path: "/path/to/your/reference.tif"
```

**PyTorch fallback:** `inference_backend: pytorch` (slower; optional `camera_resize_scale: 0.6`).

Launcher aliases and full stack options: [ngps_flight README](../README.md#quick-start-launcher-scripts).

---

## Build TensorRT engine

Export with [LightGlue-ONNX](https://github.com/fabio-sim/LightGlue-ONNX) and compile **inside the
container** — its TensorRT matches the host, and engines only load on the version that built them.

**Prerequisites:** clone `~/LightGlue-ONNX` (the home directory is shared with the container).

**1. Export ONNX** (match camera size; 640×360 for sat cam):

```bash
cd ~/LightGlue-ONNX
uv sync --group export --extra torch-cpu

uv run lightglue-onnx export superpoint \
  --num-keypoints 1024 -b 2 -h 360 -w 640 \
  -o weights/superpoint_lightglue_k1024_640x360.onnx
```

**2. Build `.engine` on Jetson host:**

```bash
cd ~/ngps_ws/src/ngps_flight/ap_ngps_ros2
export LIGHTGLUE_ONNX=~/LightGlue-ONNX
export ONNX=$LIGHTGLUE_ONNX/weights/superpoint_lightglue_k1024_640x360.onnx
export ENGINE=$PWD/weights/superpoint_lightglue_k1024_640x360_fp16.engine
./scripts/build_tensorrt_engine.sh
```

**3. Smoke test (no ROS):**

```bash
source scripts/trt_env.sh
python3 scripts/test_trt_matcher.py \
  --engine weights/superpoint_lightglue_k1024_640x360_fp16.engine \
  --width 640 --height 360
```

Expect `matches > 0`, `latency_ms` ~80–100.

**4. Set paths** in `config/ngps_config.yaml` (`tensorrt_engine_path`, `reference_image_path`).

| Resolution | Export `-h` / `-w` | `camera_resize_scale` |
|------------|-------------------|------------------------|
| 384 × 216 | `-h 216 -w 384` | `0.6` (legacy) |
| **640 × 360** | **`-h 360 -w 640`** | **`1.0` (sat cam)** |
| 1280 × 720 | `-h 720 -w 1280` | `1.0` |

`--num-keypoints` must match `max_keypoints` in config. Use FP32 ONNX + `trtexec --fp16` (what the build script does); do not use LightGlue-ONNX’s separate `.fp16.onnx` for TRT.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| No `/odometry/vps` | TRT server running on **host**? `reference_image_path` valid? GPS in SITL before sat cam? |
| Engine fails to deserialize | Engine was built with a different TensorRT version — rebuild it in the container |
| `cuInit: operation not supported` | Container missing GPU groups — recreate it with the `--init-hooks` line from the [main README](../README.md) |
| Wrong resolution / bad matches | Engine `-h`/`-w` must match effective camera size (`camera_resize_scale`) |
| Insufficient matches | Lower `match_threshold`, check reference `.tif` covers the flight area |
| CUDA not available (PyTorch path) | Node falls back to CPU |
| GPU OOM | Reduce `max_keypoints` or use smaller export resolution |

## License

MIT License
