cmake_minimum_required(VERSION 3.5)
project(ros_deep_learning)

# locate dependencies
find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)
find_package(CUDA REQUIRED)

# detect ROS1 vs ROS2
find_package(catkin QUIET)

# detect ROS version
string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
set(ROS_DISTRO "ROS_${ROS_DISTRO}")
message("detected ROS_DISTRO=${ROS_DISTRO}")

find_package(VPI 2.0)
link_directories(/usr/lib/aarch64-linux-gnu/tegra)

if( catkin_FOUND )
    message("detected ROS1 (catkin_make)")
    
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package.ros1.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)

    find_package(catkin REQUIRED COMPONENTS
        image_transport
        roscpp
        sensor_msgs
        vision_msgs
        std_msgs
    )
     
    catkin_package(
        LIBRARIES ros_deep_learning_nodelets
        CATKIN_DEPENDS nodelet roscpp image_transport sensor_msgs
    )

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")    # enable c++11 (TensorRT requirement)
    include_directories(${catkin_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
    add_definitions(-DROS1 -DROS_DISTRO=${ROS_DISTRO})

else()
    find_package(ament_cmake QUIET)

    if( ament_cmake_FOUND )
        message("detected ROS2 (ament_cmake)")

        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package.ros2.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)

        find_package(rclcpp REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(vision_msgs REQUIRED)
        find_package(rclcpp_components REQUIRED) # <--- Added for composable nodes
		find_package(cv_bridge REQUIRED)


        set(CMAKE_CXX_STANDARD 14)
        include_directories(${CUDA_INCLUDE_DIRS})
        add_definitions(-DROS2 -DROS_DISTRO=${ROS_DISTRO})

    else()
        message(FATAL_ERROR "could not find either ROS1 (catkin_make) or ROS2 (ament_cmake) packages")
    endif()
endif()

# sources shared across nodes
set(common_src src/image_converter.cpp src/ros_compat.cpp)

# --- Standard Executables ---
add_executable(imagenet src/node_imagenet.cpp ${common_src})
target_link_libraries(imagenet ${catkin_LIBRARIES} jetson-inference)

add_executable(detectnet src/node_detectnet.cpp ${common_src})
target_link_libraries(detectnet ${catkin_LIBRARIES} jetson-inference)

add_executable(segnet src/node_segnet.cpp ${common_src})
target_link_libraries(segnet ${catkin_LIBRARIES} jetson-inference)

add_executable(video_source src/node_video_source.cpp ${common_src})
target_link_libraries(video_source ${catkin_LIBRARIES} jetson-inference)

add_executable(video_output src/node_video_output.cpp ${common_src})
target_link_libraries(video_output ${catkin_LIBRARIES} jetson-inference)

# --- ROS 2 Composable Node Library ---
if(NOT catkin_FOUND)
    # This builds the shared library that the component container loads
    add_library(video_output_component SHARED 
        src/composable_node_video_output.cpp 
        ${common_src}
    )
    target_link_libraries(video_output_component jetson-inference jetson-utils)
    ament_target_dependencies(video_output_component 
        rclcpp 
        rclcpp_components 
        sensor_msgs 
		vision_msgs
        std_msgs
		cv_bridge
    )

    # Register the component with the ament index
    rclcpp_components_register_nodes(video_output_component 
        "jetson_ros::VideoOutputNode"
    )
endif()


if( catkin_FOUND )
    add_library(ros_deep_learning_nodelets src/nodelet_imagenet.cpp src/image_converter.cpp)
    target_link_libraries(ros_deep_learning_nodelets ${catkin_LIBRARIES} jetson-inference)

    if(catkin_EXPORTED_LIBRARIES)
        add_dependencies(ros_deep_learning_nodelets ${catkin_EXPORTED_LIBRARIES})
    endif()
else()
    # Install executables
    foreach(node imagenet detectnet segnet video_source video_output)
        ament_target_dependencies(${node} rclcpp std_msgs sensor_msgs vision_msgs cv_bridge)
        install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
    endforeach()

    # Install the component library
    install(TARGETS video_output_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

    ament_package()
endif()