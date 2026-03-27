# Non-GPS Navigation Documentation

Setup and configuration guide for GPS-free navigation using visual-inertial systems with ArduPilot.

**Original Documentation:** [Google Docs](https://docs.google.com/document/d/1Opsji8ZT2YeRjR8lPMAb49Ai44oMDmwoFYnSJPwfXz0/edit?tab=t.0#heading=h.d9zawrpqff1s)

## Hardware Requirements

- Flight controller running ArduPilot (Pixhawk, Cube, etc.)
- Down-facing camera (mono or stereo)
- IMU sensor (integrated in flight controller or external)
- Companion computer (for running ROS2 nodes)
- Calibrated camera-IMU system (see CAMERA_IMU_CALIBRATION.md)

## Software Requirements

- ArduPilot firmware (Copter, Plane, or Rover)
- ROS2 (Humble or later)
- MAVROS2 or MAVLink
- NGPS, UKF, and VIO packages from this repository

## Wiring & Connection

### Camera Connection

- Connect camera to companion computer via USB, Ethernet, or MIPI CSI
- Ensure camera provides timestamped images
- Configure camera frame rate (typically 10-30 Hz)

### IMU Connection

- Use flight controller's onboard IMU (recommended)
- Or connect external IMU via I2C/SPI to companion computer
- Ensure IMU data is accessible via ROS2 topics

### Companion Computer to Flight Controller

- Connect via serial (USB, UART) or Ethernet
- Configure MAVLink connection
- Set appropriate baud rate (921600 or higher recommended)

## ArduPilot Configuration

### Enable External Navigation

1. Set `EK3_ENABLE = 1` (enable EKF3)
2. Set `EK3_SRC1_POSXY = 3` (external position source)
3. Set `EK3_SRC1_VELXY = 3` (external velocity source)
4. Set `EK3_SRC1_POSZ = 1` (barometric altimeter for altitude)
5. Set `EK3_SRC1_VELZ = 1` (barometric altimeter for vertical velocity)

### Configure Position Source

Set `EK3_SRC1_POSXY = 3` to use external position from MAVLink VISION_POSITION_ESTIMATE message.

### Configure Velocity Source

Set `EK3_SRC1_VELXY = 3` to use external velocity from MAVLink VISION_SPEED_ESTIMATE message.

### Other Important Parameters

```
EK3_GLITCH_RAD = 5.0          # Position glitch protection radius (meters)
EK3_REJECT_GPS = 0            # Don't reject GPS (if available)
AHRS_EKF_TYPE = 3           # Use EKF3
```

## MAVROS Configuration

### Basic Setup

1. Install MAVROS2:
   ```bash
   sudo apt install ros-humble-mavros ros-humble-mavros-extras
   ```

2. Configure connection:
   ```bash
   ros2 run mavros mavros_node \
     --ros-args \
     -p fcu_url:=/dev/ttyUSB0:921600 \
     -p system_id:=1 \
     -p component_id:=1
   ```

### Topic Mapping

MAVROS publishes/subscribes to:
- `/mavros/odometry/out`: Fused odometry output (to ArduPilot)
- `/mavros/imu/data`: IMU data (from ArduPilot, optional)
- `/mavros/vision_pose/pose`: Vision position estimate (to ArduPilot)

## Enabling and Configuring IMU & Camera Data Streams

### Methods to Enable High-Resolution IMU

#### Method 1: Via MAVLink Streams

In ArduPilot, configure high-rate IMU streams:
```
SR0_ADSB = 0
SR0_EXT_STAT = 2
SR0_EXTRA1 = 10
SR0_EXTRA2 = 10
SR0_EXTRA3 = 2
SR0_PARAMS = 10
SR0_POSITION = 3
SR0_RAW_SENS = 2
SR0_RC_CHAN = 0
```

#### Method 2: Via Companion Computer

Read IMU directly from flight controller via I2C/SPI if supported.

### Full Setup Steps

1. **Start MAVROS:**
   ```bash
   ros2 launch mavros apm.launch.py fcu_url:=/dev/ttyUSB0:921600
   ```

2. **Start VIO system:**
   ```bash
   ros2 launch ap_vips vio.launch.py
   ```

3. **Start NGPS localization:**
   ```bash
   ros2 launch ap_ngps_ros2 ngps_localization.launch.py \
     reference_image_path:=/path/to/satellite_image.tif
   ```

4. **Start UKF fusion:**
   ```bash
   ros2 launch ap_ukf estimator.launch.py
   ```

5. **Verify topics are publishing:**
   ```bash
   ros2 topic list
   ros2 topic echo /ukf/odometry
   ```

## Calibration Procedure

### Steps

1. **Calibrate Camera-IMU system** (see CAMERA_IMU_CALIBRATION.md)
   - Camera intrinsics
   - IMU noise parameters
   - Camera-IMU extrinsics
   - Time synchronization

2. **Configure ArduPilot parameters:**
   - Set EKF3 as primary estimator
   - Configure external navigation sources
   - Set appropriate noise parameters

3. **Test on ground:**
   - Power on system
   - Verify all ROS2 nodes are running
   - Check that odometry is being published
   - Verify ArduPilot is receiving external navigation

4. **Initial flight test:**
   - Start in manual/stabilize mode
   - Verify position estimates are reasonable
   - Check for drift and accuracy
   - Monitor EKF health in Mission Planner/QGroundControl

5. **Tune parameters:**
   - Adjust EKF noise parameters if needed
   - Tune UKF fusion weights
   - Optimize NGPS matching parameters

## Notes

- Always test in a safe, open area first
- Keep GPS enabled initially for comparison and fallback
- Monitor EKF health and innovation values
- External navigation requires good camera visibility and lighting
- System performance depends on altitude (higher is better for NGPS)
- Ensure adequate computational resources on companion computer
- Regular calibration recommended (every few months or after hardware changes)

## Troubleshooting

### ArduPilot Not Receiving Position

- Check MAVROS connection: `ros2 topic echo /mavros/state`
- Verify odometry topic is publishing: `ros2 topic echo /ukf/odometry`
- Check EKF3 source parameters in ArduPilot

### Poor Position Accuracy

- Verify camera-IMU calibration
- Check camera focus and image quality
- Ensure adequate lighting conditions
- Verify reference image quality and georeferencing

### High Drift

- Check IMU noise parameters
- Verify time synchronization
- Ensure sufficient visual features in camera view
- Check for camera vibration or motion blur

### System Crashes

- Monitor companion computer CPU/memory usage
- Reduce camera frame rate if needed
- Optimize feature matching parameters
- Check for overheating

## Additional Resources

- [Original Google Docs Documentation](https://docs.google.com/document/d/1Opsji8ZT2YeRjR8lPMAb49Ai44oMDmwoFYnSJPwfXz0/edit?tab=t.0#heading=h.d9zawrpqff1s)
