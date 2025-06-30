cmake_minimum_required(VERSION 3.5)
project(oakd_isaac_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(DetectionMaskNode src/oakd_isaac_ros/publish_mask.cpp)
ament_target_dependencies(DetectionMaskNode 
  rclcpp 
  std_msgs 
  sensor_msgs 
  OpenCV 
  vision_msgs 
  cv_bridge)

install(TARGETS
  DetectionMaskNode
  DESTINATION lib/${PROJECT_NAME})

ament_package()