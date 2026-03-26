# NGPS: Next-Generation Positioning System

<div align="center">

[![WIP](https://img.shields.io/badge/status-WIP-yellow)](https://github.com/snktshrma/ngps_flight)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-Compatible-orange)](https://ardupilot.org/)
<br />
[![Python](https://img.shields.io/badge/Python-3.10+-blue?logo=python&logoColor=white)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C++-17-blue?logo=c%2B%2B&logoColor=white)](https://isocpp.org/)
[![PyTorch](https://img.shields.io/badge/PyTorch-1.9+-orange?logo=pytorch&logoColor=white)](https://pytorch.org/)
<br />
[![OpenCV](https://img.shields.io/badge/OpenCV-4.5+-green?logo=opencv&logoColor=white)](https://opencv.org/)
[![LightGlue](https://img.shields.io/badge/LightGlue-SuperPoint-yellow)](https://github.com/cvg/LightGlue)

</div>

Next-Generation Positioning System (NGPS) for high-altitude drone navigation without GPS.

## Description

This codebase implements a visual geo-localization system for drones that matches down-facing camera images against satellite reference images using deep learning-based feature matching. The system provides absolute position estimates to correct drift in Visual-Inertial Odometry (VIO) systems.

### Main Components

- **ap_ngps_ros2**: ROS2 node that performs visual geo-localization by matching camera images to satellite reference images using LightGlue/SuperPoint deep learning features. Runs at 1-2 Hz.

- **ap_ukf**: Unscented Kalman Filter that fuses multiple sensor inputs:
  - NGPS absolute position (1-2 Hz)
  - VIO relative pose (10-20 Hz)
  - IMU data (high frequency)

  Outputs fused odometry at 10-20 Hz for flight control.

- **ap_vips**: Visual-Inertial Odometry system that provides high-frequency relative pose estimates optimized for high-altitude flight.

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

- [ ] Add a fallback VO pipeline
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
- [ ] Support for different camera models

## Installation

See individual package READMEs:
- [ap_ngps_ros2/README.md](ap_ngps_ros2/README.md)
- [ap_ukf/README.md](ap_ukf/README.md)
- [ap_vips/README.md](ap_vips/README.md)

## Documentation

- [Changelog](CHANGELOG.md) - Project history and version timeline
- [Camera-IMU Calibration](CAMERA_IMU_CALIBRATION.md) - [Google Docs](https://docs.google.com/document/d/13JY4MAfdqjsa-Oa39xT4HW6WFFT-KQloymyGb0LvuGo/edit?tab=t.0#heading=h.43j03vqklwxn)
- [Non-GPS Navigation Setup](NON_GPS_NAVIGATION.md) - [Google Docs](https://docs.google.com/document/d/1Opsji8ZT2YeRjR8lPMAb49Ai44oMDmwoFYnSJPwfXz0/edit?tab=t.0#heading=h.d9zawrpqff1s)

## Acknowledgements

A huge shoutout to my partner in disguise, ChatGPT, for generating tests and helping with packaging this codebase.

## Safety & Ethical Considerations

**IMPORTANT DISCLAIMER:** This software is provided for research and educational applications only. The developers and contributors of this project doesn't promote and are not responsible for any misuse.

## License

See LICENSE file.
