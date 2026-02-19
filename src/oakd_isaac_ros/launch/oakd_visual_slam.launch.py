#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (ROS bag) clock if true",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    default_oakd_params_file = os.path.join(
        FindPackageShare("oakd_isaac_ros").find("oakd_isaac_ros"),
        "config",
        "oak_d",
        "oakd_stereo_only_30hz_400p.yaml",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_oakd_params_file,
        description="Path to depthai_ros_driver camera yaml",
    )
    params_file = LaunchConfiguration("params_file")

    declare_enable_vslam = DeclareLaunchArgument(
        "enable_vslam",
        default_value="true",
        description="Enable VisualSlamNode and image republish nodes",
    )
    enable_vslam = LaunchConfiguration("enable_vslam")

    # --- Static TFs (minimal to satisfy cuVSLAM default base_link + stereo) ---
    static_base_to_left_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_base_to_left_optical",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0",
                   "base_link", "oakd_camera_left_camera_optical_frame"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    static_left_to_right_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_left_to_right_optical",
        output="screen",
        arguments=["-0.075", "0", "0", "0", "0", "0",
                   "oakd_camera_left_camera_optical_frame",
                   "oakd_camera_right_camera_optical_frame"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # --- CPU normalize images (prevents Nitros cudaMemcpy2D InvalidPitch) ---
    # Requires image_transport to be installed.
    left_republish = Node(
        package="image_transport",
        executable="republish",
        name="left_republish_raw",
        condition=IfCondition(enable_vslam),
        output="screen",
        arguments=["raw"],
        remappings=[
            ("in", "/oakd_camera/left/image_raw"),
            ("out", "/oakd_camera/left/image_raw_norm"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    right_republish = Node(
        package="image_transport",
        executable="republish",
        name="right_republish_raw",
        condition=IfCondition(enable_vslam),
        output="screen",
        arguments=["raw"],
        remappings=[
            ("in", "/oakd_camera/right/image_raw"),
            ("out", "/oakd_camera/right/image_raw_norm"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # --- Composable container (camera + vslam) ---
    container = ComposableNodeContainer(
        name="oak_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container", # executable="component_container_mt",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        composable_node_descriptions=[
            ComposableNode(
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Camera",
                name="oakd_camera",
                parameters=[params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
    )

    load_vslam = LoadComposableNodes(
        condition=IfCondition(enable_vslam),
        target_container="/oak_container",
        composable_node_descriptions=[
            ComposableNode(
                package="isaac_ros_visual_slam",
                plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
                name="visual_slam_node",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "enable_debug_mode": False,
                }],
                remappings=[
                    ("/visual_slam/image_0", "/oakd_camera/left/image_raw_norm"),
                    ("/visual_slam/camera_info_0", "/oakd_camera/left/camera_info"),
                    ("/visual_slam/image_1", "/oakd_camera/right/image_raw_norm"),
                    ("/visual_slam/camera_info_1", "/oakd_camera/right/camera_info"),
                ],
                extra_arguments=[{"use_intra_process_comms": False}],
            ),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_enable_vslam,
        static_base_to_left_optical,
        static_left_to_right_optical,
        left_republish,
        right_republish,
        container,
        load_vslam,
    ])
