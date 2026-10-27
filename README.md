# NGPS: Next-Generation Positioning System

<div align="center">

[![arXiv](https://img.shields.io/badge/arXiv-2607.18936-b31b1b?logo=arxiv&logoColor=white)](https://arxiv.org/abs/2607.18936)
[![autotest](https://github.com/snktshrma/ngps_flight/actions/workflows/autotest.yml/badge.svg)](https://github.com/snktshrma/ngps_flight/actions/workflows/autotest.yml)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-Compatible-orange)](https://ardupilot.org/)
<br />
[![Python](https://img.shields.io/badge/Python-3.12-blue?logo=python&logoColor=white)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C++-14-blue?logo=c%2B%2B&logoColor=white)](https://isocpp.org/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.11-orange?logo=pytorch&logoColor=white)](https://pytorch.org/)
<br />
[![OpenCV](https://img.shields.io/badge/OpenCV-4.11-green?logo=opencv&logoColor=white)](https://opencv.org/)
[![LightGlue](https://img.shields.io/badge/LightGlue-SuperPoint-yellow)](https://github.com/cvg/LightGlue)
[![TensorRT](https://img.shields.io/badge/TensorRT-10.16-76B900?logo=nvidia&logoColor=white)](https://developer.nvidia.com/tensorrt)
[![JetPack](https://img.shields.io/badge/JetPack-7.2-76B900?logo=nvidia&logoColor=white)](https://developer.nvidia.com/embedded/jetpack)

</div>

Next-Generation Positioning System (NGPS) for high-altitude drone navigation without GPS.

📄 **Paper:** [NGPS: GPS-Denied Aerial Geo-Localization and 2.5D Reconstruction via Deep Satellite Image Matching and Multi-Rate Sensor Fusion](https://arxiv.org/abs/2607.18936) (arXiv:2607.18936)

## Description

This codebase implements a visual geo-localization system for drones that matches down-facing camera images against satellite reference images using deep learning-based feature matching. The system provides absolute position estimates to correct drift in Visual-Inertial Odometry (VIO) systems.

### Main Components

- **ap_ngps_ros2**: ROS2 node that performs visual geo-localization by matching camera images to satellite reference images using LightGlue/SuperPoint deep learning features. Runs at 1-2 Hz. Also runs ORB keyframe VO (CPU or VPI GPU), corrected by NGPS.

- **ap_ukf** (optional): Unscented Kalman Filter that fuses multiple sensor inputs:
  - NGPS absolute position (1-2 Hz)
  - VIO relative pose (10-20 Hz)
  - IMU data (high frequency)

  Outputs fused odometry at 10-20 Hz for flight control.

- **ap_vo** (optional): Standalone monocular visual odometry (SIFT + metric `solvePnPRansac` on a plane at
  configurable depth). Publishes `nav_msgs/Odometry` compatible with `vio_origin_relay` / `ap_ukf`.

- **ap_vo2** (optional): Standalone map-matching VPS node using classical features (AKAZE + MAGSAC
  homography against pre-computed reference tiles). A lighter-weight alternative to the
  LightGlue pipeline, with no dependency on other NGPS packages.

> Full VIO is expected as an external package publishing relative pose; it is not bundled here.

### How It Works

1. NGPS module matches real-time camera frames to a georeferenced satellite reference image
2. Provides absolute position estimates at low frequency (1-2 Hz)
3. UKF fuses NGPS absolute positions with high-frequency VIO estimates
4. Fused output sent to ArduPilot's EKF for final state estimation

## Demo

[Algorithm Demonstration Video](https://youtu.be/iHE6cFCccTA)

## Related Articles & Blogs

- [GSoC 2024: High Altitude Non-GPS Navigation](https://discuss.ardupilot.org/t/gsoc-2024-wrapping-up-high-altitude-non-gps-navigation/122905/1) - Initial GSoC project summary
- [Transformer & Optimization Based High Altitude GPS-Denied Fusion](https://discuss.ardupilot.org/t/transformer-optimization-based-high-altitude-gps-denied-fusion/134181/1) - Updated implementation and architecture details

## Related Projects

- [ap_nongps](https://github.com/snktshrma/ap_nongps) - Earlier prototype implementation with SIFT-based feature matching and optical flow methods

## TODO

- [ ] Add an intereactive initial guess interface
- [x] Add a fallback VO pipeline
- [ ] Add global optimisation for fusion
- [ ] Update AP to accept position and odometry as separate sources to be fused internally
- [ ] Add support for multiple reference images
- [ ] Optimize feature matching for faster performance
- [x] Add calibration tools and documentation
- [ ] Improve error handling and recovery
- [ ] Add more unit tests
- [x] Document configuration parameters
- [x] Add example launch files for different scenarios
- [ ] Performance profiling and optimization
- [x] Convert lightglue model to jetson friendly compute capable
- [ ] Support for different camera models
- [ ] GTSAM and SFM support

## Installation

See **[INSTALLATION.md](INSTALLATION.md)** for Docker, container, and workspace setup on the Jetson.

## Setting up environment variables
```bash
export MAPBOX_API_KEY=''
```

## Quick start (launcher scripts)

From the host (no need to `distrobox enter` or `source` ROS manually, scripts handle that):

1. `~/ngps_ws/src/ngps_flight/scripts/run_sitl_stack.sh`
2. `~/ngps_ws/src/ngps_flight/scripts/run_sat_cam.sh` *(after GPS in SITL)*
3. `~/ngps_ws/src/ngps_flight/scripts/run_ngps.sh`

Optional aliases (add to `~/.bashrc`):

```bash
alias ngps-sitl='~/ngps_ws/src/ngps_flight/scripts/run_sitl_stack.sh'
alias ngps-cam='~/ngps_ws/src/ngps_flight/scripts/run_sat_cam.sh'
alias ngps-run='~/ngps_ws/src/ngps_flight/scripts/run_ngps.sh'
```


Full fusion stack: `LAUNCH=unified_localization_simple.launch.py ~/ngps_ws/src/ngps_flight/scripts/run_ngps.sh`

## Build TensorRT engine (one-time)

Run this **inside the container** - its TensorRT matches the host exactly, and engines are
locked to the TensorRT version that built them.

Clone [LightGlue-ONNX](https://github.com/fabio-sim/LightGlue-ONNX) (not bundled; the home
directory is shared with the container):

```bash
git clone https://github.com/fabio-sim/LightGlue-ONNX.git ~/LightGlue-ONNX
```

Then, inside `distrobox enter vps-jp7`:

```bash
cd ~/LightGlue-ONNX

# 1. Export ONNX (uses the container's torch; no uv/venv needed)
python3 -m lightglue_dynamo.cli export superpoint --num-keypoints 1024 -b 2 -h 360 -w 640 \
  -o weights/superpoint_lightglue_k1024_640x360.onnx

# 2. Build the FP16 engine
~/ngps_ws/src/ngps_flight/ap_ngps_ros2/scripts/build_tensorrt_engine.sh
```

Set `tensorrt_engine_path` and `reference_image_path` in `ap_ngps_ros2/config/ngps_config.yaml`.

> Full copy-paste steps and options: **[ap_ngps_ros2/README.md](ap_ngps_ros2/README.md#build-tensorrt-engine)**.
>
> To rebuild for a different resolution, change `-h/-w` in the export and rerun both steps.

## Running the package (manual steps)

### Run SITL with DDS:
#### Terminal 1- Run micro ros agent:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

#### Terminal 2- Run SITL with DDS:
```bash
./Tools/autotest/sim_vehicle.py -v ArduCopter --enable-DDS --location OSRF0
```

> Add `-DG` to build with debug symbols and run under gdb. It triggers a full rebuild the
> first time and slows the vehicle loop, so keep it off for timing-sensitive runs.
> Via the launcher: `SITL_EXTRA_ARGS=-DG ~/ngps_ws/src/ngps_flight/scripts/run_sitl_stack.sh`


Then in another terminal, **after GPS is detected in sitl**, run:

```bash
python3 ./Tools/autotest/sat_cam_emulator.py --port 14550 --airfield-radius-m 1500 --airfield-zoom 20  --http-mjpeg-port 8090 --no-hud --ros --no-display --ros-compressed --ros-size 640x360 --pose-source sim
```

> If want to record bag file:
> 
> ```bash
> ros2 bag record -o <location> /camera/image_raw/compressed   /ap/imu/experimental/data   /ap/clock   /ap/tf_static /ap/navsat /ap/gps_global_origin/filtered /ap/geopose/filtered /ap/time /ap/tf /ap/pose/filtered
> ```
> 
> To replay:
> 
> ```bash
> ros2 bag play <bag location>
> ```

#### Now to run our VPS (with bag for debugging or with SITL (with non-GPS EKF params for realtime test),

Set the .tif file location in [ap_ngps_ros2/config/ngps_config.yaml](ap_ngps_ros2/config/ngps_config.yaml).

```bash
ros2 launch ap_ngps_ros2 ngps_localization.launch.py
```

## For changing location
To change location and get new .tif for that location, please follow steps in the gazebo_terrain_generator fork specifically for this: [https://github.com/snktshrma/gazebo_terrain_generator/tree/dev/geotiff](https://github.com/snktshrma/gazebo_terrain_generator/tree/dev/geotiff)

After changing, please change the location for sitl launch as well.

> ## NOTE
> For now the steps to generate a .tif are very manual but addition to sat_camemulator.py already sets a base to automaticallyt manage and autogenerate .TIF using MAPBOX. So in next updates, I'll add that feature as well and that will also help wiith setting initial guess.

## Documentation

- **[ap_ngps_ros2/README.md](ap_ngps_ros2/README.md)** - TensorRT build, config, troubleshooting
- [Changelog](CHANGELOG.md) - Project history and version timeline
- [Camera-IMU Calibration](docs/CAMERA_IMU_CALIBRATION.md) - [Google Docs](https://docs.google.com/document/d/13JY4MAfdqjsa-Oa39xT4HW6WFFT-KQloymyGb0LvuGo/edit?tab=t.0#heading=h.43j03vqklwxn)
- [Non-GPS Navigation Setup](docs/NON_GPS_NAVIGATION.md) - [Google Docs](https://docs.google.com/document/d/1Opsji8ZT2YeRjR8lPMAb49Ai44oMDmwoFYnSJPwfXz0/edit?tab=t.0#heading=h.d9zawrpqff1s)

## Safety & Ethical Considerations

**IMPORTANT DISCLAIMER:** This software is provided for research and educational applications only. The developers and contributors of this project doesn't promote and are not responsible for any misuse.

## Citation

If you use NGPS in your research, please cite:

```bibtex
@misc{sharma2026ngps,
  title         = {NGPS: GPS-Denied Aerial Geo-Localization and 2.5D Reconstruction via Deep Satellite Image Matching and Multi-Rate Sensor Fusion},
  author        = {Sharma, Sanket},
  year          = {2026},
  eprint        = {2607.18936},
  archivePrefix = {arXiv},
  url           = {https://arxiv.org/abs/2607.18936}
}
```

## License

See LICENSE file.
