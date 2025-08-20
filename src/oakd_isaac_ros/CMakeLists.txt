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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(message_filters REQUIRED)


add_library(publish_mask_component SHARED src/publish_mask.cpp)
ament_target_dependencies(publish_mask_component 
  rclcpp_components
  rclcpp 
  std_msgs 
  sensor_msgs 
  OpenCV 
  vision_msgs 
  cv_bridge
  message_filters)


rclcpp_components_register_node(
    publish_mask_component
    PLUGIN "publish_mask"
    EXECUTABLE publish_mask
)

ament_export_targets(export_publish_mask_component)
install(TARGETS publish_mask_component
        EXPORT export_publish_mask_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})


# Install Python executables
install(PROGRAMS
  scripts/image_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch assets
  DESTINATION share/${PROJECT_NAME})

ament_package()