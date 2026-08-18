from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the OAK-D launch file
    oakd_launch_file = os.path.join(
        get_package_share_directory('oakd_isaac_ros'),
        'launch', 'tools', 'camera.launch.py'
    )

    # 1. Include OAK-D launch (This starts the 'component_container')
    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oakd_launch_file),
        launch_arguments={
            'params_file': os.path.join(
                get_package_share_directory('oakd_isaac_ros'),
                'config', 'oak_d', 'oakd_dnn_depth-bremen.yaml'
            )
        }.items()
    )

    # 2. Load the video_output ComposableNode into the existing container
    load_video_output = LoadComposableNodes(
        target_container='oak_container', # Must match the name in camera.launch.py
        composable_node_descriptions=[
            ComposableNode(
                package='ros_deep_learning',
                plugin='jetson_ros::VideoOutputNode',
                name='video_output',
                parameters=[{
                    'resource': 'webrtc://@:8557/rgb_1',
                    'codec': 'h264',
                    'bitrate': 4000000
                }],
                remappings=[
                    ('image_in', '/oak/rgb/image_raw')
                ]
            )
        ]
    )

    webrtc_launch = IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                        os.path.join(
                        get_package_share_directory('ros_deep_learning'),
                        'launch',
                        'video_output.ros2.launch'
                        )),
                        launch_arguments={
                            'topic': '/oak/rgb/image_raw', # <-- stream resized topic
                            'output': 'webrtc://@:8556/rgb_1',
                            'output_codec': 'h264',
                            'output_bitrate': '3000000'
                            }.items()
                        ) 

    # 3. Your custom resizer node (Standard Node)
    resize = Node(
        package="oakd_isaac_ros",
        executable="image_resizer.py",
        name="oak_resizer",
        parameters=[{
            "resize_width": 640,
            "resize_height": 360,
            "input_image_topic": "/oak/rgb/image_raw",
            "input_camera_info_topic": "/oak/rgb/camera_info",
            "output_image_topic": "/oak/rgb/image_resized",
            "output_camera_info_topic": "/oak/rgb/camera_resized_info"
        }]
    )

    return LaunchDescription([
        oakd_launch,
        #webrtc_launch,
        #resize,
        load_video_output
    ])