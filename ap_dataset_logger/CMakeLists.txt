cmake_minimum_required(VERSION 3.8)
project(ap_dataset_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Sources live under src/<pkg>/ to match the sibling packages in this workspace.
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

install(PROGRAMS
  src/ap_dataset_logger/rtsp_camera_node.py
  scripts/check_dataset_sync.py
  scripts/measure_camera_latency.py
  scripts/view_camera.sh
  scripts/preflight_check.py
  scripts/flight_log.sh
  scripts/flight_log_inner.sh
  scripts/mavlink_rates.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
