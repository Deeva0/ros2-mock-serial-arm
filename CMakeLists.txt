cmake_minimum_required(VERSION 3.8)
project(ros2_mock_serial_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(mock_serial_arm SHARED
  src/mock_hardware.cpp
)

target_include_directories(mock_serial_arm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mock_serial_arm
  rclcpp
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(
  hardware_interface
  mock_serial_arm.xml
)

install(
  TARGETS mock_serial_arm
  EXPORT export_mock_serial_arm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES mock_serial_arm.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
