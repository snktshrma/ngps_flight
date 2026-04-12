# Changelog

All notable changes to this project are documented in this file.

## [2026-04-05] - 4D `ap_ukf` + NGPS VPS odometry

### Changed

- **`ap_ukf`:** 4D local fusion (x, y, vx, vy); IMU removed; first VPS bootstraps the filter; VPS soft correction over N frames; 4×4 YAML covariances.
- **`ap_ngps_ros2`:** Publishes `nav_msgs/Odometry` on `odometry/vps` (local xy after first-fix origin).
- **Launches:** `unified_localization*.launch.py` no longer remap IMU into `fusion_ros`.

## [2026-04-03] - ardupilot_gazebo iris SDF aligned with `ros2` branch

### Fixed

- `iris_with_standoffs/model.sdf`: collision mesh URI changed from `package://ardupilot_gazebo/.../iris_collision.stl` to `model://iris_with_standoffs/meshes/iris_collision.stl` so plain `gz sim` (no ROS/ament) resolves assets via `GZ_SIM_RESOURCE_PATH` instead of failing with “uri could not be resolved”.
- Restored `ardupilot_gazebo/models/iris_with_gimbal/model.sdf` and `models/gimbal_small_3d/model.sdf` from upstream `ros2`. Our branch had diverged: non-merged `model://` includes and scoped link names (`iris_with_standoffs::base_link`) caused `sdformat_urdf` to fail with “Failed to find sdf canonical link [base_link]” when launching `iris_runway.launch.py`. The `ros2` iris uses `<include merge="true">`, `package://` URIs, and flat `base_link` / `gimbal_link` joint names; the gimbal model on `ros2` names the root link `gimbal_link` (not `base_link`) to match that joint.

## [2026-03-27] - Docker + Distrobox setup

### Added

- Added setup for docker + distrobox for easy replication of sim and hardware tests

## [2025-12-20] - V2: Squashed commits and added docs

### Added
- Documentation files: CAMERA_IMU_CALIBRATION.md and NON_GPS_NAVIGATION.md
- Links to original Google Docs documentation

## [2025-10-26] - README Update

### Changed
- Updated main README.md with improved documentation structure

## [2025-10-11] - Completed Integration

### Added
- Complete NGPS ROS2 package (`ap_ngps_ros2`)
  - ROS2 node for visual geo-localization using LightGlue/SuperPoint
  - Launch files for NGPS localization and unified system
  - Configuration files (ngps_config.yaml, unified_system_config.yaml)
  - Installation scripts and dependencies
  - Package metadata (CMakeLists.txt, package.xml)
  - Test files for NGPS node and rotation detection
  - Comprehensive README with usage instructions

### Changed
- Integrated NGPS localization node with ROS2 ecosystem
- Added unified launch files for complete system integration

## [2025-06-09] - Transformer Based Matcher

### Added
- Initial transformer-based feature matching implementation
- Jupyter notebook (`ap_ngps_2(3).ipynb`) for experimentation and development

### Changed
- Updated UKF README.md with additional documentation

## [2025-06-05] - UKF and VIO Integration

### Added
- **UKF Package (`ap_ukf`)**
  - Unscented Kalman Filter implementation for sensor fusion
  - Core state fusion logic (stateFuser.h, stateMath.h, stateTypes.h)
  - ROS2 integration (fusionRos.h, fusionRos.cpp)
  - Launch files and configuration
  - Comprehensive test suite (localization_tests, ukf_tests, ukf_helpers_tests)
  - README with state estimation documentation

- **VIO Package (`ap_vips`)**
  - Visual-Inertial Odometry system
  - Camera models support
  - Package structure and documentation

### Changed
- Project structure expanded to support multi-sensor fusion architecture

## [2025-06-04] - Initial Commit

### Added
- Initial project structure
- Basic README.md
- LICENSE file
- Foundation for NGPS flight system
