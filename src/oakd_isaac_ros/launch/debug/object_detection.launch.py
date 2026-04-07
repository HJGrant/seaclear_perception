from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
import os


def generate_launch_description():
    # delcare the launch arguments
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
    ]

    # 2. Define LaunchConfigurations to capture the values
    name = LaunchConfiguration("name")
    camera_model = LaunchConfiguration("camera_model")
    parent_frame = LaunchConfiguration("parent_frame")
    cam_pos_x = LaunchConfiguration("cam_pos_x")
    cam_pos_y = LaunchConfiguration("cam_pos_y")
    cam_pos_z = LaunchConfiguration("cam_pos_z")
    cam_roll = LaunchConfiguration("cam_roll")
    cam_pitch = LaunchConfiguration("cam_pitch")
    cam_yaw = LaunchConfiguration("cam_yaw")
    
    
    # Path to the other launch file
    # Include another launch file
    oak_params_file = os.path.join(
        get_package_share_directory('oakd_isaac_ros'),
        'config', 'oak_d', 'oakd_bluerov_vs.yaml'
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

    # launching the camera urdf
    urdf_launch_dir = os.path.join(
        get_package_share_directory("depthai_descriptions"), "launch"
    )
    
    urdf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urdf_launch_dir, 'urdf_launch.py')
            ),
            launch_arguments={
                'name': name,
                'camera_model': camera_model,
                'parent_frame': parent_frame, # This links camera to your world/robot
                'cam_pos_x': cam_pos_x,
                'cam_pos_y': cam_pos_y,
                'cam_pos_z': cam_pos_z,
                'cam_roll': cam_roll,
                'cam_pitch': cam_pitch,
                'cam_yaw': cam_yaw,
        }.items()
    )

    #publish a transform from the robot base_frame to the oak-d-base-frame
    static_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_base_link_broadcaster',
    arguments=[
        '--x', '0.0', 
        '--y', '-0.17', 
        '--z', '-0.275', 
        '--yaw', '-1.5708',  # <--- 90 degrees Left around base_link Z
        '--pitch', '0.0', 
        '--roll', '0.0', 
        '--frame-id', 'base_link', 
        '--child-frame-id', 'oak-d-base-frame'
    ]
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

    return LaunchDescription(
        declared_arguments +
    [
        urdf_launch,
        static_tf_node,
        detection_container,
        detection_include
    ])
