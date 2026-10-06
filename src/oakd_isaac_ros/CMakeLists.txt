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
find_package(foxglove_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(isaac_ros_gxf REQUIRED)
find_package(isaac_ros_managed_nitros REQUIRED)
find_package(isaac_ros_nitros_image_type REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)


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

add_library(depth_colorizer_component SHARED src/depth_colorizer.cpp)
ament_target_dependencies(depth_colorizer_component
  rclcpp_components
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge)

rclcpp_components_register_node(
    depth_colorizer_component
    PLUGIN "depth_colorizer"
    EXECUTABLE depth_colorizer
)

ament_export_targets(export_depth_colorizer_component)
install(TARGETS depth_colorizer_component
        EXPORT export_depth_colorizer_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# NITROS variant of publish_mask -- see src/publish_mask_nitros.cpp header
# comment for what's converted and what's still unverified against the
# real installed NITROS headers.
add_library(publish_mask_nitros_component SHARED src/publish_mask_nitros.cpp)
ament_target_dependencies(publish_mask_nitros_component
  rclcpp_components
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  vision_msgs
  cv_bridge
  message_filters
  isaac_ros_gxf
  isaac_ros_managed_nitros
  isaac_ros_nitros_image_type)
target_link_libraries(publish_mask_nitros_component CUDA::cudart)
# Same RPATH workaround isaac_ros_nvblox/isaac_ros_dnn_inference apply to
# any target linking against nitros/gxf shared libs.
set_target_properties(publish_mask_nitros_component PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

rclcpp_components_register_node(
    publish_mask_nitros_component
    PLUGIN "publish_mask_nitros"
    EXECUTABLE publish_mask_nitros
)

ament_export_targets(export_publish_mask_nitros_component)
install(TARGETS publish_mask_nitros_component
        EXPORT export_publish_mask_nitros_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_library(publish_mask_dual_component SHARED 
  src/publish_mask_dual.cpp
)

ament_target_dependencies(publish_mask_dual_component 
  rclcpp_components
  rclcpp 
  std_msgs 
  sensor_msgs 
  OpenCV 
  vision_msgs 
  cv_bridge
  message_filters)
  
rclcpp_components_register_node(
    publish_mask_dual_component
    PLUGIN "publish_mask_dual"
    EXECUTABLE publish_mask_dual
)

ament_export_targets(export_publish_mask_dual_component)
install(TARGETS publish_mask_dual_component
        EXPORT export_publish_mask_dual_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_library(h264_bridge_component SHARED
  src/h264_to_foxglove_bridge.cpp
)

target_compile_definitions(h264_bridge_component PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(h264_bridge_component
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "foxglove_msgs"
)

# This command registers the plugin
rclcpp_components_register_node(h264_bridge_component
  PLUGIN "oakd_isaac_ros::H264ToFoxgloveBridge"
  EXECUTABLE h264_bridge_node
)

install(TARGETS h264_bridge_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install visualizer
add_library(dnn_depth_visualizer_component SHARED src/dnn_depth_visualizer.cpp)

target_compile_definitions(dnn_depth_visualizer_component PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(dnn_depth_visualizer_component
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "foxglove_msgs"
  "stereo_msgs"
  "cv_bridge"
  "OpenCV"
)

rclcpp_components_register_node(dnn_depth_visualizer_component 
  PLUGIN "oakd_isaac_ros::DNNDepthVisualizer" 
  EXECUTABLE dnn_depth_visualizer_node
)

install(TARGETS dnn_depth_visualizer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(map_frame_anchor_component SHARED src/map_frame_anchor.cpp)
ament_target_dependencies(map_frame_anchor_component
  rclcpp_components
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros)

rclcpp_components_register_node(
    map_frame_anchor_component
    PLUGIN "map_frame_anchor"
    EXECUTABLE map_frame_anchor
)

ament_export_targets(export_map_frame_anchor_component)
install(TARGETS map_frame_anchor_component
        EXPORT export_map_frame_anchor_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# component_container_mt variant whose nodes don't publish /parameter_events
# (see the comment in the source for the heartbeat-storm background).
add_executable(component_container_no_param_events src/component_container_no_param_events.cpp)
ament_target_dependencies(component_container_no_param_events rclcpp rclcpp_components)
install(TARGETS component_container_no_param_events
        DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})


# Install Python executables
install(PROGRAMS
  scripts/image_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/image_resizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/foxglove_compressed_video.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch assets
  DESTINATION share/${PROJECT_NAME})

ament_package()