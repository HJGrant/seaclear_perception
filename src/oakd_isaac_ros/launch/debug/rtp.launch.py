from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


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
                'config', 'oak_d', 'oakd_rgbd.yaml'
            )
        }.items()
    )

    # Our custom resizer node
    resize = Node(
        package="oakd_isaac_ros",          # <-- replace with your package name
        executable="image_resizer.py",    # <-- must match entry point / script name
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
            'topic': '/oak/rgb/image_raw',  # <-- stream resized topic
            'output': 'rtp://192.168.2.94:8550',
            'output_codec': 'h264',
            'output_bitrate': '4000000'
        }.items()
    )

    return LaunchDescription([
        oakd_launch,
        webrtc_launch
    ])
