# Camera-IMU Calibration

Calibration guide for Camera-IMU systems used with ArduPilot and VIO/SLAM systems.

**Original Documentation:** [Google Docs](https://docs.google.com/document/d/13JY4MAfdqjsa-Oa39xT4HW6WFFT-KQloymyGb0LvuGo/edit?tab=t.0#heading=h.43j03vqklwxn)

## Introduction

Proper calibration of camera intrinsics, IMU noise parameters, and camera-IMU extrinsics is essential for accurate visual-inertial navigation. This guide covers both manual calibration workflows and automated pipelines.

## Theoretical Background

### Camera Intrinsics

Camera intrinsics describe the internal geometry of the camera:
- Focal length (fx, fy)
- Principal point (cx, cy)
- Distortion parameters (radial and tangential)

These parameters are required for accurate feature matching and pose estimation.

### IMU Noise Modeling

IMU sensors have noise characteristics that must be characterized:
- Accelerometer noise (white noise and bias instability)
- Gyroscope noise (white noise and bias instability)
- Allan variance analysis is used to estimate these parameters

### Camera-IMU Extrinsics

The rigid transformation between camera and IMU frames:
- Translation vector (3D position offset)
- Rotation matrix (3D orientation offset)

### Time Offset & Synchronization

Time synchronization between camera and IMU is critical:
- Hardware synchronization (preferred): Using trigger signals
- Software synchronization: Clock alignment and time offset estimation

## Setup & Prerequisites

### Hardware

- Camera (mono or stereo)
- IMU sensor
- Calibration target (checkerboard or AprilTag)
- Computer for data collection and processing

### Software

- ROS2 (Humble or later)
- Kalibr calibration toolbox
- Python packages: numpy, scipy, matplotlib
- MAVROS or DDS for data collection

### Input Pathways

Data can be collected via:
- MAVROS: ROS2 bridge to ArduPilot
- DDS: Direct Data Distribution Service

### Data Collection

Collect three ROS2 bag files:
1. Static bag: IMU data while stationary (for Allan variance)
2. Camera-only bag: Camera images of calibration target
3. Camera-IMU bag: Synchronized camera and IMU data with motion

## Time Synchronization

### Hard Sync (Recommended)

Hardware synchronization using trigger signals:
- Camera triggered by IMU or external clock
- Ensures precise timestamp alignment
- Requires compatible hardware

### Soft Sync

Software-based clock synchronization:
- Clock disciplining to align system clocks
- Time offset estimation during calibration
- Less accurate but more flexible

## Manual Calibration Workflow

### Step 1: IMU Noise Estimation (Allan Variance)

1. Collect static IMU data for 2-3 hours
2. Run Allan variance analysis:
   ```bash
   python allan_variance.py <imu_bag_file>
   ```
3. Extract noise parameters:
   - Accelerometer white noise
   - Accelerometer bias instability
   - Gyroscope white noise
   - Gyroscope bias instability

### Step 2: Camera Intrinsics (Kalibr camera-only)

1. Record camera bag with calibration target:
   ```bash
   ros2 bag record /camera/image_raw
   ```
2. Run Kalibr camera calibration:
   ```bash
   kalibr_calibrate_cameras \
     --target <target.yaml> \
     --bag <camera_bag> \
     --models <camera_model> \
     --topics /camera/image_raw
   ```
3. Save camera intrinsics YAML file

### Step 3: Camera-IMU Extrinsics (+ time offset)

1. Record synchronized camera-IMU bag with motion
2. Run Kalibr camera-IMU calibration:
   ```bash
   kalibr_calibrate_imu_camera \
     --target <target.yaml> \
     --cam <camera_calib.yaml> \
     --imu <imu_calib.yaml> \
     --bag <camera_imu_bag>
   ```
3. Output includes:
   - Camera-IMU transformation (translation and rotation)
   - Time offset between camera and IMU

## Automation: One-Command Pipeline

For automated calibration, use the provided script:

```bash
./calibrate_camera_imu.sh \
  --imu_bag <static_imu.bag> \
  --camera_bag <camera.bag> \
  --camera_imu_bag <motion.bag> \
  --output_dir <calibration_results>
```

This script runs all three calibration steps and outputs final calibration files.

## Using Your Calibration Results

### Drop-in to VIO/SLAM

Most VIO/SLAM systems accept Kalibr format:
- Camera intrinsics: `camchain.yaml`
- IMU parameters: `imu.yaml`
- Camera-IMU extrinsics: included in `camchain.yaml`

### Feeding ArduPilot

ArduPilot requires specific parameter format:
1. Convert camera intrinsics to ArduPilot camera parameters
2. Set IMU noise parameters in ArduPilot EKF
3. Configure camera-IMU transform in ArduPilot

Example ArduPilot parameters:
```
CAM1_FOCAL_LENGTH = <fx>
CAM1_OPTICAL_CENTER_X = <cx>
CAM1_OPTICAL_CENTER_Y = <cy>
EKF2_ACC_NOISE = <accel_noise>
EKF2_GYR_NOISE = <gyro_noise>
```

## Validation

### Flight/Field Tests

1. Perform test flights with calibrated system
2. Compare estimated trajectory with ground truth (GPS or motion capture)
3. Check for drift and accuracy
4. Verify time synchronization quality

### Troubleshooting

Common issues:
- **Poor matching**: Check camera intrinsics calibration
- **Drift**: Verify IMU noise parameters
- **Jitter**: Check time synchronization
- **Scale errors**: Verify camera-IMU extrinsics

## Resources

- [Original Google Docs Documentation](https://docs.google.com/document/d/13JY4MAfdqjsa-Oa39xT4HW6WFFT-KQloymyGb0LvuGo/edit?tab=t.0#heading=h.43j03vqklwxn)
- Kalibr documentation: https://github.com/ethz-asl/kalibr
- ArduPilot documentation: https://ardupilot.org/
- Allan variance tools: https://github.com/rpng/kalibr_allan

## Notes & Assumptions

- Assumes ROS2 Humble or later
- Camera and IMU must be rigidly mounted
- Calibration target should fill camera FOV
- Motion during camera-IMU calibration should be diverse (rotation and translation)
- Static IMU data collection should be in stable temperature environment
