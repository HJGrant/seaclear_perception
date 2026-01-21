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
    # Include another launch file
    oak_params_file = os.path.join(
        get_package_share_directory('oakd_isaac_ros'),
        'config', 'oak_d', 'oakd_dnn_depth.yaml'
)
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

    # 3. Define the Rectify Node (Optional but recommended if you need it)
    rectify_node = ComposableNode(
        package="image_proc",
        plugin="image_proc::RectifyNode",
        name="rectify_color_node",
        remappings=[
            ("image", "oak/rgb/image_raw"),
            ("camera_info", "oak/rgb/camera_info"),
            ("image_rect", "oak/rgb/image_rect"),
        ],
    )
    
    encoder_node = ComposableNode(
        name='oak_rgb_encoder_node',
        package='isaac_ros_h264_encoder',
        plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
        parameters=[{
            'input_height': 360, 
            'input_width': 640
        }],
        remappings=[
            ('image_raw', '/oak/rgb/image_raw_rgb8/resize'),
            ('image_compressed', '/oak/rgb/image_raw/h264')
        ]
        )

    encoder_node_mask_overlay = ComposableNode(
        name='oak_mask_encoder_node',
        package='isaac_ros_h264_encoder',
        plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
        parameters=[{
            'input_height': 640, 
            'input_width': 640
        }],
        remappings=[
            ('image_raw', '/mask_overlay'),
            ('image_compressed', '/mask_overlay/h264')
        ]
        )

    image_resize_node = ComposableNode(
        name='oak_image_resize_node_for_rtp',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'input_width': 1280,
            'input_height': 720,
            'output_width': 640,
            'output_height': 360,
            'encoding_desired': 'rgb8'
        }],
        remappings=[
            ('image', '/oak/rgb/image_raw'),
            ('camera_info', '/oak/rgb/camera_info'),
            ('resize/image', '/oak/rgb/image_raw_rgb8/resize'),
            ('resize/camera_info', '/oak/rgb/camera_info/resize')
        ]
    )

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

    rgb_video_node = ComposableNode(
        package='oakd_isaac_ros',
        plugin='oakd_isaac_ros::H264ToFoxgloveBridge',
        name='foxglove_bridge_rgb',
        remappings=[
            ('input/h264', '/oak/rgb/image_raw/h264'),
            ('output/foxglove', '/oak/rgb/video')
        ]
    )

    yolo_video_node = ComposableNode(
        package='oakd_isaac_ros',
        plugin='oakd_isaac_ros::H264ToFoxgloveBridge',
        name='foxglove_bridge_yolo',
        remappings=[
            ('input/h264', '/mask_overlay/h264'),
            ('output/foxglove', '/mask_overlay/video')
        ]
    )

    detection_container = ComposableNodeContainer(
        name='detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            camera_node,
            #rectify_node,
            image_converter,
            image_resize_node,
            encoder_node,
            encoder_node_mask_overlay,
            rgb_video_node,
            yolo_video_node
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

    return LaunchDescription([
        detection_container,
        detection_include
    ])
