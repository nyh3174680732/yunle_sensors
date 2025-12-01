cmake_minimum_required(VERSION 3.8)
project(vehicle_autoware_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)

# Find yunle_msgs (custom chassis messages)
# If yunle_msgs is not available, the nodes will fail to compile
# Make sure yunle_msgs package is built first
find_package(yunle_msgs QUIET)
if(NOT yunle_msgs_FOUND)
  message(WARNING "yunle_msgs not found. Please build yunle_msgs package first.")
  message(WARNING "Skipping vehicle_autoware_bridge compilation.")
  return()
endif()

# Include directories
include_directories(include)

# ========== 1. Autoware to Chassis Control Converter ==========
add_executable(autoware_to_chassis_node
  src/autoware_to_chassis_converter.cpp
)
ament_target_dependencies(autoware_to_chassis_node
  rclcpp
  std_msgs
  autoware_control_msgs
  yunle_msgs
)
target_include_directories(autoware_to_chassis_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ========== 2. Chassis to Autoware Status Converter ==========
add_executable(chassis_to_autoware_node
  src/chassis_to_autoware_converter.cpp
)
ament_target_dependencies(chassis_to_autoware_node
  rclcpp
  std_msgs
  autoware_vehicle_msgs
  yunle_msgs
)
target_include_directories(chassis_to_autoware_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ========== 3. IMU Topic Remapper ==========
add_executable(imu_remapper_node
  src/imu_remapper.cpp
)
ament_target_dependencies(imu_remapper_node
  rclcpp
  sensor_msgs
)

# ========== 4. GNSS Converter ==========
add_executable(gnss_converter_node
  src/gnss_to_autoware_converter.cpp
)
ament_target_dependencies(gnss_converter_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Install executables
install(TARGETS
  autoware_to_chassis_node
  chassis_to_autoware_node
  imu_remapper_node
  gnss_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
