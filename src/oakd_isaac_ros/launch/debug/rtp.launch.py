from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node, LoadComposableNodes
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Path to the other launch file
    oakd_launch_file = os.path.join(
        get_package_share_directory('oakd_isaac_ros'),
        'launch', 'tools', 'camera.launch.py'
    )

    # Include OAK-D launch
    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oakd_launch_file),
        launch_arguments={
            'params_file': os.path.join(
                get_package_share_directory('oakd_isaac_ros'),
                'config', 'oak_d', 'oakd_dnn_depth.yaml'
            )
        }.items()
    )

    OAK_CONTAINER_NAME = 'oak_container' 


    cuda_resize_node_description = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode', 
        name='resize_node',
        parameters=[{
            'input_width': 1280,
            'input_height': 720,
            'output_width': 640,
            'output_height': 360,
            'encoding_desired': ''
        }],
        remappings=[
            ('image', '/oak/rgb/image_raw'),
            ('camera_info', '/oak/rgb/camera_info'),
            ('resize/image', '/oak/rgb/image_resized'),
            ('resize/camera_info', '/oak/rgb/camera_info_resized')
        ],
    )

    load_cuda_resize_node = LoadComposableNodes(
        target_container=OAK_CONTAINER_NAME,
        composable_node_descriptions=[
            cuda_resize_node_description,
        ]
    )

    # WebRTC video output
    webrtc_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(       
                get_package_share_directory('ros_deep_learning'),
                'launch',
                'video_output.ros2.launch'
            )
        ),
        launch_arguments={
            'topic': '/oak/rgb/image_raw',  
            'output': 'rtp://192.168.2.94:8550',
            'output_codec': 'h264',
            'output_bitrate': '4000000'
        }.items(),
    )

    return LaunchDescription([
        oakd_launch,
        webrtc_launch
    ])
