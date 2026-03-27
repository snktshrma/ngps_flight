# Changelog

All notable changes to this project are documented in this file.

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
