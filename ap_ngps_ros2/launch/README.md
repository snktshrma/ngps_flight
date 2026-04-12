# Unified Localization System

Launch files for running the complete localization system.

## Quick Start

```bash
ros2 launch ap_ngps_ros2 unified_localization_simple.launch.py
```

```bash
ros2 launch ap_ngps_ros2 unified_localization_simple.launch.py reference_image_path:=/path/to/reference.jpg
```

```bash
ros2 launch ap_ngps_ros2 unified_localization_simple.launch.py camera_topic:=/my_camera/image_raw
```

## System Architecture

```
Camera + IMU → VIPS → /odometry/vio_raw ─┐
     ↓                                   ├→ vio_origin_relay → /odometry/vio ─┐
   NGPS → /odometry/vps ─────────────────┘                                ├→ fusion → /fused/odometry
                                                                            │
                                                              ap_ukf (fusion_ros) OR
                                                              robot_localization (ekf_node)
```

`vio_origin_relay` aligns VIO XY to the first VPS fix (`SYSTEM_PLAN.md` 4B). Choose **`fusion_backend`**: **`ap_ukf`** (custom 4D UKF + soft VPS) or **`robot_localization`** (`ekf_node`; tune `rl_ekf.yaml`). Install RL: `sudo apt install ros-${ROS_DISTRO}-robot-localization`.

## Topic Flow

### Input Topics
- `/camera/image_raw` - Camera images
- `/imu/data` - IMU data (for VINS; not fused in `ap_ukf`)

### Output Topics
- `/fused/odometry` - Final fused odometry
- `/odometry/vio_raw` - Raw VINS odometry (launch remap)
- `/odometry/vio` - Origin-aligned VIO for the UKF (after relay)
- `/odometry/vps` - VPS position from NGPS (local meters)
- `/vips/path` - VIPS trajectory path
- `/vips/pose` - VIPS pose estimates

## Configuration Files

- **NGPS**: `ap_ngps_ros2/config/ngps_config.yaml`
- **VIPS**: `vins/config/high_alt/high_alt_mono_imu_config.yaml`
- **UKF (`ap_ukf`)**: `ap_ukf/params/estimator_config.yaml`
- **EKF (`robot_localization`)**: `ap_ngps_ros2/config/rl_ekf.yaml`

### Fusion backend

```bash
# default: ap_ukf
ros2 launch ap_ngps_ros2 unified_localization_simple.launch.py fusion_backend:=robot_localization
```

Optional: `robot_localization_config_file:=/path/to/custom_rl.yaml`

## Monitoring

```bash
ros2 node list | grep -E '(ngps_localization_node|vips_node|vio_origin_relay|fusion_ros|ekf_filter_node)'
```

```bash
ros2 topic hz /fused/odometry /odometry/vio /odometry/vio_raw /odometry/vps /imu/data
```

```bash
ros2 topic echo /fused/odometry
```

## Troubleshooting

1. **No camera data**
   ```bash
   ros2 topic list | grep camera
   ```

2. **No IMU data**
   ```bash
   ros2 topic list | grep imu
   ```

3. **VIPS not publishing VIO**
   ```bash
   ros2 topic echo /odometry/vio_raw
   ros2 topic echo /odometry/vio
   ```

4. **NGPS not publishing position**
   ```bash
   ros2 topic echo /odometry/vps
   ```

5. **Fusion not publishing `/fused/odometry`**
   ```bash
   ros2 topic echo /fused/odometry
   ros2 node list | grep -E fusion_ros\|ekf_filter_node
   ```

## Launch Arguments

- `reference_image_path` - Path to reference image for NGPS
- `camera_topic` - Camera topic to subscribe to
- `ngps_config_file` - NGPS configuration file path
- `vips_config_file` - VIPS configuration file path  
- `ukf_config_file` - `ap_ukf` / `fusion_ros` configuration file path
- `fusion_backend` - `ap_ukf` (default) or `robot_localization`
- `robot_localization_config_file` - Path to `ekf_filter_node` YAML (default: `rl_ekf.yaml`)
- `use_sim_time` - Use simulation time

## Dependencies

```bash
colcon build --packages-select ap_ngps_ros2 ap_ukf vins
sudo apt install ros-${ROS_DISTRO}-robot-localization   # only if using fusion_backend:=robot_localization
```
