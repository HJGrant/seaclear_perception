from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Path to the other launch file
    oakd_launch = os.path.join(
        get_package_share_directory('oakd_isaac_ros'),
        'launch', 'tools', 'camera.launch.py'
    )

    # Include another launch file
    include_other_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oakd_launch),
        launch_arguments={
            'params_file': os.path.join(
                get_package_share_directory('oakd_isaac_ros'),
                'config', 'oak_d', 'oakd_rgbd.yaml'
            )
        }.items()
    )

    # Launch a node
    video_stream_node = Node(
        package="oakd_isaac_ros",
        executable="oakd_video_stream_node",
        name="oakd_video_stream",
        output="screen",
        remappings=[
            ("/image_raw", "/oak/rgb/image_raw"),
            ("/compressed_video", "/oak/compressed_video")
        ]
    )

    return LaunchDescription([
        include_other_launch,
        video_stream_node
    ])
