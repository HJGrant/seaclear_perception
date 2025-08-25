from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Path to the other launch file
    oakd_launch_file = os.path.join(
        get_package_share_directory('oakd_isaac_ros'),
        'launch', 'tools', 'camera.launch.py'
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

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge_node'
    )

    # Launch a node
    video_stream_node = Node(
        package="foxglove_compressed_video",
        executable="foxglove_compressed_video_stream",
        name="foxglove_compressed_video_node",
        output="screen",
        remappings=[
        ("/image_raw", "/oak/rgb/image_raw"),
        ("/compressed_video", "/oak/compressed_video")
        ]
    )

    return LaunchDescription([
        oakd_launch,
        foxglove_bridge
    ])
