# ap_vo

Standalone **monocular visual odometry** for the NGPS stack: SIFT features, Lowe ratio matching, **metric** `solvePnPRansac` on a **plane at configurable depth**, integrated pose in `odom`, and **`nav_msgs/Odometry`** output compatible with `vio_origin_relay` / `ap_ukf` (remap topics as needed).

## Requirements

- ROS 2 (tested with Humble)
- Python: `opencv-python` (SIFT is in the main wheels since OpenCV 4.4+), `numpy`, `tf-transformations` (ROS package or pip)
- `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `cv_bridge`, `tf2_ros`

```bash
sudo apt install ros-${ROS_DISTRO}-nav-msgs ros-${ROS_DISTRO}-tf-transformations \
  python3-opencv python3-numpy
```

## Build

```bash
cd ~/ngps_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select ap_vo --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch ap_vo vo.launch.py
```

With bag / sim time:

```bash
ros2 launch ap_vo vo.launch.py use_sim_time:=true
```

### Inputs

| Topic | Message |
|-------|---------|
| `camera_topic` (default `/camera/image_raw`) | `sensor_msgs/Image` mono |
| `camera_info_topic` (default `/camera/camera_info`) | `sensor_msgs/CameraInfo` (K, D required) |

### Outputs

| Topic | Message |
|-------|---------|
| `/vo/pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| `odometry_topic` (default `/odometry/vo`) | `nav_msgs/Odometry` pose + twist (twist from pose deltas) |

### Standalone behavior (no robot TF)

- **`use_tf_for_scale: false`** (default): uses **`plane_distance_m`** (meters) as the distance to the dominant ground / scene plane along the optical axis for unprojecting reference keypoints before PnP. Tune for your flight altitude or scene scale.
- **`use_tf_for_scale: true`**: uses **`|map → base_link| z`** as plane distance (falls back to `plane_distance_m` if TF is missing).

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `confidence_threshold` | 0.7 | Lowe ratio test |
| `min_matches` | 30 | Minimum good matches |
| `plane_distance_m` | 50.0 | Scene / ground plane depth for metric PnP (standalone) |
| `use_tf_for_scale` | false | Use TF Z for plane distance |
| `pnp_reprojection_error` | 8.0 | RANSAC reprojection threshold (px) |
| `publish_odometry` | true | Publish `Odometry` |
| `odometry_topic` | `/odometry/vo` | Output odometry topic |
| `output_frame` | `odom` | Parent frame |
| `child_frame_id` | `camera_optical` | Child frame in `Odometry` |

### Fusion stack

To drive `ap_ukf` like VINS, remap **`/odometry/vo` → `/odometry/vio`** (or `/odometry/vio_raw` if using `vio_origin_relay`). This VO is **monocular** and **planar-scene** biased; tune `plane_distance_m` and match thresholds for your environment.

### Limitations

- Scale comes from **plane distance** (or TF Z), not from stereo / IMU.
- No loop closure; drift accumulates.
- Best with **nadir / near-planar** scenes; oblique views need careful tuning.
