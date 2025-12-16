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
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


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
                'config', 'oak_d', 'oakd_dnn_depth.yaml'
            )
        }.items()
    )

    #decoder_node = ComposableNode(
    #    name='oak_rgb_decoder_node',
    #    package='isaac_ros_h264_decoder',
    #    plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
    #    remappings=[
    #        ('image_compressed', '/oak/rgb/image_raw/compressed'),
    #        ('image_uncompressed', '/oak/rgb/image_raw/uncompressed')
    #    ]
    #    )

    image_converter = ComposableNode(
        name='oak_format_conv_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        parameters=[{
            'image_width': 1280,
            'image_height': 720,
            'encoding_desired': 'rgb8'
        }],
        remappings=[
            ('image_raw', '/oak/rgb/image_raw'),
            ('image', '/oak/rgb/image_raw_rgb8')
        ]
    )

    detection_container = ComposableNodeContainer(
        name='detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            image_converter
        ],
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
            'topic': '/oak/rgb/image_raw_rgb8',  # <-- stream resized topic
            'output': 'rtp://192.168.2.94:8551',
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
            'topic': '/mask_overlay',
            'output': 'rtp://192.168.2.94:8550',
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
        detection_container,
        detection_include,
        rtp_launch_yolo
    ])
