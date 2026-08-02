# TensorRT + LightGlue on Jetson — Internal Reference

> **Maintainer notes only.** End users should follow **[ap_ngps_ros2/README.md](../ap_ngps_ros2/README.md)** (build + run commands). This file is background reading: glossary, architecture, and why things are wired the way they are.

**Audience:** Maintainers who want the full PyTorch → ONNX → TRT story explained in depth.

**Related repos:**
- NGPS stack: `ngps_ws/src/ngps_flight/ap_ngps_ros2`
- Model export: `LightGlue-ONNX` (fabio-sim/LightGlue-ONNX fork or clone)

---

## Table of contents

1. [What problem are we solving?](#1-what-problem-are-we-solving)
2. [Glossary (important terms)](#2-glossary-important-terms)
3. [Original pipeline (PyTorch)](#3-original-pipeline-pytorch)
4. [Why we moved to TensorRT](#4-why-we-moved-to-tensorrt)
5. [The conversion chain: PyTorch → ONNX → TensorRT](#5-the-conversion-chain-pytorch--onnx--tensorrt)
6. [Libraries and tools used](#6-libraries-and-tools-used)
7. [What changed in NGPS (integration)](#7-what-changed-in-ngps-integration)
8. [Runtime architecture](#8-runtime-architecture)
9. [Build from scratch: SuperPoint → LightGlue → ONNX → TRT](#9-build-from-scratch-superpoint--lightglue--onnx--trt)
10. [Running the full stack](#10-running-the-full-stack)
11. [Configuration reference](#11-configuration-reference)
12. [Performance tuning](#12-performance-tuning)
13. [Troubleshooting](#13-troubleshooting)
14. [File map](#14-file-map)

---

## 1. What problem are we solving?

**NGPS** (Non-GPS navigation) localizes a drone/aircraft by matching a **live camera view** against a **reference satellite map** (GeoTIFF mosaic).

High-level loop each frame:

```
Camera image  +  Map kernel (crop from mosaic)
        ↓
   Feature matching (find same points in both images)
        ↓
   Homography (where is the camera patch on the map?)
        ↓
   Pose / GPS-like position published to ROS + ArduPilot
```

The heavy part is **feature matching**: finding corresponding keypoints between the map patch and the camera image. That used to run in **PyTorch** (SuperPoint + LightGlue). On Jetson Orin it was too slow for real-time use. We replaced it with a **TensorRT engine** — a GPU-optimized binary that runs the same math much faster.

---

## 2. Glossary (important terms)

| Term | Plain English |
|------|----------------|
| **SuperPoint** | A neural network that finds “interesting points” (corners, edges) in an image and describes each with a vector (descriptor). |
| **LightGlue** | A neural network that takes keypoints from two images and decides which points correspond (matches). |
| **Keypoint** | A pixel location `(x, y)` the model thinks is distinctive. |
| **Match** | A pair of keypoints — one in the map patch, one in the camera — that the model believes show the same physical feature. |
| **Kernel / map patch** | A crop from the reference mosaic centered where we think the vehicle is; same size as the camera frame. |
| **Homography** | A 3×3 matrix mapping camera pixels to map pixels; used after matching to update position. |
| **PyTorch** | Python deep-learning framework; easy to develop in, slower on embedded GPUs for this workload. |
| **ONNX** | Open Neural Network Exchange — a **file format** (`.onnx`) describing a model graph. Hardware-agnostic. |
| **TensorRT (TRT)** | NVIDIA’s inference compiler. Takes ONNX (or other formats) and builds a **`.engine`** file optimized for a specific GPU (Jetson Orin). |
| **`trtexec`** | Command-line tool (part of TensorRT) that builds engines from ONNX. |
| **FP16** | 16-bit floating point; half the memory/bandwidth of FP32; standard on Jetson for speed. |
| **Engine (`.engine`)** | Serialized TensorRT plan — **not portable** across GPU types or TRT versions; rebuild when you change resolution or JetPack. |
| **Distrobox** | Linux container that shares your home folder but has its own packages. We run **ROS 2 Humble** inside `vps-dev`. |
| **VISP / visual odometry** | ArduPilot log of external vision position fed via DDS/MAVLink from our `/ap/tf` or fusion stack. |
| **VPS** | Visual Positioning System — our `/odometry/vps` topic with local x/y from NGPS. |

---

## 3. Original pipeline (PyTorch)

Before TensorRT, `ngps_localization_node.py` did **three separate GPU calls** per frame:

```python
# 1. Extract features from map kernel
feats0 = SuperPoint.extract(kernel_image)

# 2. Extract features from camera
feats1 = SuperPoint.extract(camera_image)

# 3. Match with LightGlue
matches = LightGlue({"image0": feats0, "image1": feats1})
```

**Libraries:** `torch`, `lightglue` (Python package), CUDA inside distrobox.

**Problems on Jetson:**
- Three launches + Python overhead
- LightGlue + SuperPoint not tuned for Orin
- Total latency often **hundreds of ms to seconds** per frame
- `import tensorrt` inside distrobox was broken (stub NVIDIA libs)

---

## 4. Why we moved to TensorRT

| Goal | How TRT helps |
|------|----------------|
| **Speed** | Single fused GPU kernel; ~**90–100 ms** inference at 640×360 vs much slower PyTorch |
| **Deterministic latency** | Fixed input shape, no dynamic Python graph |
| **Deploy without PyTorch** | Phase C can drop `torch`/`lightglue` from the hot path (optional) |

We kept **`inference_backend: pytorch`** as a fallback until TRT is validated.

---

## 5. The conversion chain: PyTorch → ONNX → TensorRT

Think of three **representations** of the same math:

```
┌─────────────────┐     export      ┌─────────────────┐     trtexec      ┌─────────────────┐
│ PyTorch models  │  ────────────►  │  ONNX graph     │  ────────────►  │  TRT .engine    │
│ (train/dev)     │   lightglue-onnx │  (.onnx file)   │   FP16 build    │  (Jetson only)  │
└─────────────────┘                 └─────────────────┘                 └─────────────────┘
```

### Step A — Fuse SuperPoint + LightGlue in PyTorch

The **LightGlue-ONNX** project wraps both into one module called `Pipeline`:

```python
extractor = SuperPoint(num_keypoints=1024)
matcher = LightGlue(...)
pipeline = Pipeline(extractor, matcher)
```

One forward pass: **two images in → keypoints + matches out**.

### Step B — Export to ONNX

Command (from `LightGlue-ONNX` repo):

```bash
cd ~/LightGlue-ONNX
uv sync --group export --extra torch-cpu   # or CUDA group on x86

uv run lightglue-onnx export superpoint \
  --num-keypoints 1024 \
  -b 2 \
  -h 360 -w 640 \
  -o weights/superpoint_lightglue_k1024_640x360.onnx
```

**Important flags:**
- `-b 2` — batch dimension is **2 images** (kernel + camera) in one tensor
- `-h / -w` — **height × width** of each image (must match NGPS camera + engine)
- `--num-keypoints 1024` — max keypoints SuperPoint returns (must match config `max_keypoints`)

**ONNX I/O contract:**

| Tensor | Shape | Type | Meaning |
|--------|-------|------|---------|
| `images` (input) | `(2, 1, H, W)` | float32 | Stacked SuperPoint luma for kernel + camera |
| `keypoints` | `(2, 1024, 2)` | int64 | Keypoint coords per image |
| `matches` | `(M, 3)` | int64 | Columns: `[batch_idx, idx0, idx1]` |
| `mscores` | `(M,)` | float32 | Match confidence scores |

### Step C — Prepare ONNX for TensorRT (optional fix)

Some ONNX `Reduce` ops need axis constants for TensorRT’s parser. LightGlue-ONNX provides:

```python
from lightglue_dynamo.scripts.benchmark import _prepare_tensorrt_onnx
_prepare_tensorrt_onnx(source.onnx, trt_ready.onnx)
```

Our `build_tensorrt_engine.sh` runs this automatically when `LightGlue-ONNX` is present.

### Step D — Build TensorRT engine with `trtexec`

On **Jetson host** (not distrobox):

```bash
trtexec \
  --onnx=weights/superpoint_lightglue_640x360.trt-ready.onnx \
  --saveEngine=superpoint_lightglue_k1024_640x360_fp16.engine \
  --fp16 \
  --memPoolSize=workspace:2048
```

Or use the wrapper script:

```bash
cd ~/ngps_ws/src/ngps_flight/ap_ngps_ros2
# Default ONNX path is 384×216; override for 640×360:
export LIGHTGLUE_ONNX=~/LightGlue-ONNX
# Copy or symlink your 640×360 ONNX, then:
./scripts/build_tensorrt_engine.sh
```

**Do not** use LightGlue-ONNX’s separate `.fp16.onnx` file for TRT — build FP16 with `trtexec --fp16` instead (documented in their CLI).

---

## 6. Libraries and tools used

### Model export (PC or Jetson host)

| Library | Role |
|---------|------|
| **PyTorch** | Source models, `torch.onnx.export` |
| **ONNX / onnxscript** | Graph format, opset 20 |
| **LightGlue-ONNX (`lightglue_dynamo`)** | Fused pipeline, CLI `lightglue-onnx export` |
| **uv** | Python env manager for LightGlue-ONNX |

### TensorRT inference (Jetson host)

| Library | Role |
|---------|------|
| **TensorRT** (`tensorrt` Python + `libnvinfer`) | Load `.engine`, run inference |
| **PyCUDA** | CUDA memory alloc, async memcpy |
| **NumPy** | Pre/post processing arrays |

### NGPS ROS node (distrobox)

| Library | Role |
|---------|------|
| **ROS 2 Humble** | Topics, launch, params |
| **OpenCV (`cv2`)** | Image resize, homography, mosaic warp |
| **NumPy** | Geometry |
| **PyTorch + lightglue** | Only if `inference_backend: pytorch` |

---

## 7. What changed in NGPS (integration)

### Preprocessing (must match export)

Both kernel and camera BGR images are converted the same way before TRT (see `superpoint_preprocess.py`):

```
BGR uint8
  → RGB float / 255
  → grayscale luma: 0.299·R + 0.587·G + 0.114·B
  → shape (1, H, W) float32
  → stack to (2, 1, H, W)
```

This matches LightGlue-ONNX’s SuperPoint preprocessor.

### Inference backends

| `inference_backend` | Where TRT runs | Use case |
|---------------------|----------------|----------|
| `tensorrt` | In-process | **Production** |
| `pytorch` | N/A (LightGlue in Python) | Legacy / debug |

### PyTorch vs TRT output mapping

**PyTorch (old):**
```python
valid = scores0 > threshold
m_kpts0 = kpts0[valid]
m_kpts1 = kpts1[matches0[valid]]
```

**TRT (new)** — `trt_matcher.parse_trt_matches()`:
```python
valid = mscores > threshold
m_kpts0 = keypoints[0, matches[valid, 1]]
m_kpts1 = keypoints[1, matches[valid, 2]]
```

After matching, **everything else is unchanged**: homography, rotation gates, georef, `/ngps/pose`, `/odometry/vps`, `/ap/tf`.

### Resolution change (0.6× resize removed)

Originally NGPS resized camera to **0.6×** (1280×720 → 768×432, or 640×360 → 384×216). For sat cam at native **640×360**, we set:

```yaml
camera_resize_scale: 1.0
```

and built a **640×360 engine**. Engine input shape **must equal** kernel + camera shape.

---

## 8. Runtime architecture

The TensorRT engine is loaded in-process by the ROS node. The container ships the same
TensorRT build as the JetPack host, so no host-side server or IPC bridge is involved.

```
┌──────────────────── container (ROS 2 Jazzy, vps-jp7) ────────────────────┐
│  sat_cam → /camera/image_raw/compressed                                  │
│  ngps_localization_node (inference_backend: tensorrt)                    │
│    → kernel_show + preprocess                                            │
│    → TrtMatcher.match() → TensorRT GPU                                   │
│    → homography → /odometry/vps, /ap/tf                                  │
└──────────────────────────────────────────────────────────────────────────┘
```

Earlier revisions ran TensorRT on the host and reached it over ZMQ, because the JetPack 6
container could not use the GPU. The JetPack 7 image (CUDA 13.2 + TensorRT 10.16.2, matching
the host) removes that constraint, so the split and its `zmq_trt` backend were dropped.

**Scripts:** `ngps_flight/scripts/run_*.sh` (see main README).

---

## 9. Build from scratch: SuperPoint → LightGlue → ONNX → TRT

### Prerequisites

- Jetson Orin with JetPack (TensorRT + CUDA)
- `LightGlue-ONNX` cloned (e.g. `~/LightGlue-ONNX`)
- `uv` installed for export ([LightGlue-ONNX README](https://github.com/fabio-sim/LightGlue-ONNX))
- NGPS workspace built (`colcon build --packages-select ap_ngps_ros2`)

### 9.1 Export ONNX (any machine with PyTorch; can be Jetson or PC)

```bash
cd ~/LightGlue-ONNX
uv sync --group export --extra torch-cpu

# Pick resolution to match your camera (HxW):
#   384×216  — old 0.6× pipeline
#   640×360  — sat cam native (recommended)
#   1280×720 — highest quality, ~220ms TRT

uv run lightglue-onnx export superpoint \
  --num-keypoints 1024 \
  -b 2 \
  -h 360 -w 640 \
  -o weights/superpoint_lightglue_k1024_640x360.onnx
```

Verify with ONNX Runtime (optional):

```bash
uv run lightglue-onnx infer \
  weights/superpoint_lightglue_k1024_640x360.onnx \
  assets/sacre_coeur1.jpg assets/sacre_coeur2.jpg \
  superpoint -h 360 -w 640 -d cpu
```

### 9.2 Build TensorRT engine (Jetson host only)

```bash
cd ~/ngps_ws/src/ngps_flight/ap_ngps_ros2
source scripts/trt_env.sh

# Point build script at your ONNX (edit paths or symlink):
cp ~/LightGlue-ONNX/weights/superpoint_lightglue_k1024_640x360.onnx \
   ~/LightGlue-ONNX/weights/superpoint_lightglue_k1024_384x216.onnx
# OR edit ONNX= line in scripts/build_tensorrt_engine.sh

./scripts/build_tensorrt_engine.sh
# Output: weights/superpoint_lightglue_fp16.engine (rename/copy for clarity)
cp weights/superpoint_lightglue_fp16.engine \
   weights/superpoint_lightglue_k1024_640x360_fp16.engine
```

Manual `trtexec` (equivalent):

```bash
python3 - <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, "/home/rmackay9/LightGlue-ONNX")
from lightglue_dynamo.scripts.benchmark import _prepare_tensorrt_onnx
src = Path("~/LightGlue-ONNX/weights/superpoint_lightglue_k1024_640x360.onnx").expanduser()
dst = Path("weights/superpoint_lightglue_640x360.trt-ready.onnx")
_prepare_tensorrt_onnx(src, dst)
PY

trtexec --onnx=weights/superpoint_lightglue_640x360.trt-ready.onnx \
        --saveEngine=weights/superpoint_lightglue_k1024_640x360_fp16.engine \
        --fp16 --memPoolSize=workspace:2048
```

### 9.3 Smoke test (no ROS)

```bash
source scripts/trt_env.sh
python3 scripts/test_trt_matcher.py \
  --engine weights/superpoint_lightglue_k1024_640x360_fp16.engine \
  --width 640 --height 360 \
  --kernel /path/to/kernel.jpg \
  --camera /path/to/camera.jpg
```

Expect: `matches > 0`, `latency_ms ~ 80–100` at 640×360.

### 9.4 Configure NGPS

`config/ngps_config.yaml`:

```yaml
inference_backend: "tensorrt"
camera_resize_scale: 1.0
max_keypoints: 1024
tensorrt_engine_path: ".../superpoint_lightglue_k1024_640x360_fp16.engine"
reference_image_path: ".../ngps_config/tiff/osrf0/osrf.tif"
publish_match_pair: false
dds_tf_median_window: 1
```

### 9.5 Run NGPS

```bash
# inside the container (or use the ngps-run helper):
ros2 launch ap_ngps_ros2 ngps_localization.launch.py
```

---

## 10. Running the full stack

See `ngps_flight/README.md` **Quick start** section. Summary:

| Step | Script |
|------|--------|
| SITL + micro-ROS | `~/ngps_ws/src/ngps_flight/scripts/run_sitl_stack.sh` |
| Sat cam (after GPS) | `~/ngps_ws/src/ngps_flight/scripts/run_sat_cam.sh` |
| NGPS | `~/ngps_ws/src/ngps_flight/scripts/run_ngps.sh` |

Verify:

```bash
ros2 topic hz /camera/image_raw/compressed   # ~20 Hz
ros2 topic hz /odometry/vps                  # target 3–5 Hz after optimizations
ros2 topic echo /ngps/pose --once
```

---

## 11. Configuration reference

| Parameter | Typical value | Meaning |
|-----------|---------------|---------|
| `inference_backend` | `tensorrt` | `tensorrt` or `pytorch` |
| `camera_resize_scale` | `1.0` | Multiply camera size before matching (`0.6` = legacy) |
| `max_keypoints` | `1024` | Must match ONNX export `--num-keypoints` |
| `match_threshold` | `0.5` | Min LightGlue score to keep a match |
| `min_matches` | `20` | Skip frame if fewer matches |
| `tensorrt_engine_path` | path to `.engine` | Used by host server / local TRT backend |
| `publish_ap_dds_tf` | `true` | Publish vision pose to ArduPilot via `/ap/tf` |
| `dds_tf_median_window` | `1` | Smoothing window (was 5; higher = more lag) |
| `publish_match_pair` | `false` | Debug image topic (costly if true) |
| `max_kernel_translation_step_px` | `120` | Limit map jump per frame (raise if lag on fast motion) |

---

## 12. Performance tuning

**TRT `latency_ms` is only GPU time.** Full loop includes mosaic warp and homography.

Optimizations applied in NGPS:

1. **Patch-based mosaic rotation** — rotate ~900×500 patch instead of full 4096×4096 mosaic
2. **Skip frame if still processing** — drop camera frames instead of queueing
3. **No per-frame `gc.collect()`** on TRT path
4. **`publish_match_pair: false`**
5. **`dds_tf_median_window: 1`** — reduces VISP smoothing lag

If `/odometry/vps` is still slow, check CPU load and consider `max_kernel_translation_step_px: 200`.

---

## 13. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `cuInit: operation not supported` | Container missing GPU groups | Recreate with the `--init-hooks` line in the main README |
| Engine fails to deserialize | Built with a different TensorRT version | Rebuild the engine in the container |
| `matches=0` | Wrong resolution / blank mosaic | Match engine H×W to camera; fix `reference_image_path` |
| Engine build fails on Reduce ops | ONNX not TRT-ready | Run `_prepare_tensorrt_onnx` |
| TRT fast but VISP ~1 Hz | Python pipeline bottleneck | See §12; rebuild after pull |
| Shape mismatch error | Engine 384×216 but camera 640×360 | Re-export ONNX + rebuild engine |
| Position lags aircraft | Median filter + step limit | `dds_tf_median_window: 1`, raise `max_kernel_translation_step_px` |

---

## 14. File map

### NGPS (`ap_ngps_ros2`)

| File | Purpose |
|------|---------|
| `src/ap_ngps_ros2/ngps_localization_node.py` | ROS node; backend switch; homography; publish |
| `src/ap_ngps_ros2/superpoint_preprocess.py` | BGR → `(2,1,H,W)` luma |
| `src/ap_ngps_ros2/trt_matcher.py` | Host-side TensorRT wrapper + match parsing |
| `src/ap_ngps_ros2/superpoint_preprocess.py` | BGR → `(2,1,H,W)` luma |
| `scripts/build_tensorrt_engine.sh` | ONNX → `.engine` via `trtexec` |
| `scripts/trt_env.sh` | `LD_LIBRARY_PATH` for JetPack TensorRT |
| `scripts/test_trt_matcher.py` | CLI smoke test |
| `test/test_trt_unit.py` | Unit tests (preprocess, parse) |
| `config/ngps_config.yaml` | All ROS parameters |
| `weights/*.engine` | Built engines (gitignored) |

### LightGlue-ONNX

| Path | Purpose |
|------|---------|
| `lightglue_dynamo/cli.py` | `lightglue-onnx export` / `infer` |
| `lightglue_dynamo/models/pipeline.py` | Fused SuperPoint+LightGlue |
| `lightglue_dynamo/preprocessors/superpoint.py` | Reference preprocessing |
| `lightglue_dynamo/scripts/benchmark.py` | `_prepare_tensorrt_onnx` |
| `weights/*.onnx` | Exported models |

### Design history

| Doc | Content |
|-----|---------|
| `docs/superpowers/specs/2026-07-23-tensorrt-lightglue-jetson-design.md` | Original design decisions |
| `docs/superpowers/plans/2026-07-23-tensorrt-lightglue-jetson.md` | Implementation task plan |

---

## Quick mental model

1. **Train/dev world:** PyTorch + LightGlue Python package  
2. **Portable model:** ONNX file (fused SuperPoint+LightGlue)  
3. **Jetson-fast model:** TensorRT `.engine` (FP16, fixed resolution)  
4. **ROS integration:** Same homography code; swap only the matcher call  
5. **Our Jetson setup:** TensorRT runs in-process inside the container  

If you understand that chain, you can re-export for a new resolution, rebuild the engine, update `ngps_config.yaml`, and run.
