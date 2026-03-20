import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('oakd_isaac_ros')
    oakd_launch_file = os.path.join(package_share, 'launch', 'tools', 'camera.launch.py')

    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oakd_launch_file),
        launch_arguments={
            'params_file': os.path.join(package_share, 'config', 'oak_d', 'oakd_dnn_depth.yaml')
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

    detection_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'tools', 'detection.launch.py')
        ),
        launch_arguments={
            'num_cameras': '1',
            'input_topic_list': '["/oak/rgb/image_raw_rgb8"]',
            'one_container_per_camera': 'False',
            'container_name': 'detection_container',
            'verbose': 'True',
            'target_class_id': '3'
        }.items()
    )

    return LaunchDescription([
        oakd_launch,
        image_converter,
        detection_container,
        detection_include
    ])
