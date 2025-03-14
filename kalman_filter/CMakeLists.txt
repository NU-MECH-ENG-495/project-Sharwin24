cmake_minimum_required(VERSION 3.8)
project(kalman_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Doxygen)

include_directories(
  include/kalman_filter
)

# BN0055 Node
add_executable(BNO055_Sensor src/bno055_node.cpp src/bno055.cpp)
ament_target_dependencies(BNO055_Sensor rclcpp sensor_msgs)

# VL53L1X Node
add_executable(VL53L1X_Sensor src/vl53l1x_node.cpp src/vl53l1x.cpp)
ament_target_dependencies(VL53L1X_Sensor rclcpp sensor_msgs)

# Kalman Filter Node
add_executable(Kalman_Filter src/kalman.cpp)
ament_target_dependencies(Kalman_Filter rclcpp sensor_msgs)

install(TARGETS
  VL53L1X_Sensor
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  BNO055_Sensor
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  Kalman_Filter
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
