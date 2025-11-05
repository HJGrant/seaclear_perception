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
            'input_topic_list': '["/rgb"]',
            'input_info_topic_list': '["/camera_info"]',
            'one_container_per_camera': 'False',
            'container_name': 'detection_container',
            'verbose' : 'True',
            'target_class_id' : '3'
        }.items()
    )

    '''
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
            'output': 'rtp://@:8550',
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
            'output': 'rtp://@:8551',
            'output_codec': 'h264',
            'output_bitrate': '4000000'
        }.items()
    )
    '''
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
        detection_container,
        detection_include
    ])
