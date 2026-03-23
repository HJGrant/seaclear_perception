from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to the other launch file
    oakd_launch_file = os.path.join(
        get_package_share_directory('oakd_isaac_ros'),
        'config', 'oak_d', 'oakd_dnn_depth.yaml'
    )

    #

    # 2. Define the Camera Node
    camera_node = ComposableNode(
        package="depthai_ros_driver",
        plugin="depthai_ros_driver::Camera",
        name="oak",
        parameters=[oak_params_file],
        remappings=[
            ("/oak/imu/data", "/sc2/imu/imu"), # Match your remapping from camera.launch.py
        ],
    )

    # Include another launch file
    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oakd_launch_file),
        launch_arguments={
            'params_file': os.path.join(
                get_package_share_directory('oakd_isaac_ros'),
                'config', 'oak_d', 'oakd_rgbd.yaml'
            )
        }.items()
    )

    image_converter = Node(
            package='oakd_isaac_ros',
            executable='image_converter.py',
            name='image_converter_node',
            output='screen',
            remappings=[
                ('/oak/rgb/image_raw', '/oak/rgb/image_raw'),
                ('/oak/rgb/image_raw_rgb8', '/oak/rgb/image_raw_rgb8')
            ]
        )

    detection_container = ComposableNodeContainer(
        name='detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[],
        output='screen'
    )

    # Include the detection pipeline, passing arguments and attaching encoder to detection_container
    detection_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('oakd_isaac_ros'), 'launch', 'tools', 'detection.launch.py')
        ),
        launch_arguments={
            'num_cameras': '1',
            'input_topic_list': '["/oak/rgb/image_raw_rgb8"]',
            'one_container_per_camera': 'False',
            'container_name': 'detection_container',
            'verbose' : 'True',
            'target_class_id' : '3'
        }.items()
    )

    rtp_launch_rgb = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(       
                get_package_share_directory('ros_deep_learning'),
                'launch',
                'video_output.ros2.launch'
            )
        ),
        launch_arguments={
            'topic': '/oak/rgb/image_raw',  # <-- stream resized topic
            'output': 'rtp://192.168.2.94:8550',
            'output_codec': 'h264',
            'output_bitrate': '4000000'
        }.items()
    )

    rtp_launch_yolo = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(       
                get_package_share_directory('ros_deep_learning'),
                'launch',
                'video_output.ros2.launch'
            )
        ),
        launch_arguments={
            'topic': '/mask_overlay',  # <-- stream resized topic
            'output': 'rtp://192.168.2.94:8551',
            'output_codec': 'h264',
            'output_bitrate': '4000000'
        }.items()
    )

    #foxglove_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
     #       os.path.join(get_package_share_directory('oakd_isaac_ros'), 'launch', 'tools', 'visualization.launch.py')
      #  ),
    #    launch_arguments={
    #        'use_foxglove_whitelist' : 'True',
    #        'run_foxglove' : 'True'
    #    }.items()
    #)

    return LaunchDescription([
        oakd_launch,
        image_converter,
        detection_container,
        detection_include,
        rtp_launch_rgb,
        rtp_launch_yolo
    ])
