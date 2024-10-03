cmake_minimum_required(VERSION 3.8)
project(autoware_to_satellite)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclpy REQUIRED)
find_package(satellite_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(swri_transform_util REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(tier4_map_msgs REQUIRED)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/convertor.py
  DESTINATION lib/${PROJECT_NAME}/
)

ament_python_install_module(${PROJECT_NAME})
ament_package()
