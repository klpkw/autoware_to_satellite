cmake_minimum_required(VERSION 3.8)
project(autoware_to_satellite)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(satellite_msgs REQUIRED)
find_package(tf2 REQUIRED)
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

include_directories(
  include
)

add_executable(convertor src/convertor.cpp)

ament_target_dependencies(convertor
  rclcpp
  autoware_auto_vehicle_msgs
  autoware_auto_perception_msgs
  tier4_map_msgs
  geometry_msgs
  swri_transform_util
  satellite_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS convertor
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
