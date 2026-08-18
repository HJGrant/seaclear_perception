from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
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
        DeclareLaunchArgument('launch_foxglove', default_value='true', description='Whether to start the Foxglove Bridge node')
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
    
    launch_foxglove_bool = LaunchConfiguration('launch_foxglove')

    # Path to the other launch file
    # Include another launch file
    oak_params_file = os.path.join(
        get_package_share_directory('oakd_isaac_ros'),
        'config', 'oak_d', 'oakd_dnn_depth-bremen.yaml'
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

    # NOTE: the static transform publishers are removed from the launch file, since they are handled by the transform tree package in SC2
    # static transform for BlueROV/MAVROS tests, the velocities are commanded in the base_link frame, but need to be published in the 'map' frame for MAVROS
    static_tf_map_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_link_static_tf',
        arguments=[
            '--x', '0.0', 
            '--y', '0.0', 
            '--z', '0.0', 
            '--yaw', '0.0', 
            '--pitch', '0.0', 
            '--roll', '0.0', 
            '--frame-id', 'map', 
            '--child-frame-id', 'base_link'
        ]
    )

    static_tf_sc2_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sc2_to_base_link_static_tf',
        arguments=[
            '--x', '0.0', 
            '--y', '0.0', 
            '--z', '0.0', 
            '--yaw', '0.0', # 180 degrees rotation 3.14159
            '--pitch', '0.0', 
            '--roll', '0.0', 
            '--frame-id', 'base_link', 
            '--child-frame-id', 'sc2_base_link'
        ]
    )

    #publish a transform from the robot base_frame to the oak-d-base-frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_base_link_broadcaster',
        arguments=[
            '--x', '0.0', 
            '--y', '-0.115', 
            '--z', '-0.275', 
            '--yaw', '-1.5708',  # <--- 90 degrees Left around base_link Z
            '--pitch', '0.0', 
            '--roll', '0.0', 
            '--frame-id', 'sc2_base_link', 
            '--child-frame-id', 'oak-d-base-frame'
        ]
    )

    optical_to_image_plane_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='optical_to_image_plane_tf',
        arguments=[
            '--x', '0.0', 
            '--y', '0.0', 
            '--z', '0.0', 
            '--yaw', '0.0', 
            '--pitch', '0.0', 
            '--roll', '-1.5708', 
            '--frame-id', 'oak_rgb_camera_optical_frame', 
            '--child-frame-id', 'image_plane'
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

    detection_container = ComposableNodeContainer(
        name='detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            camera_node,
            #rectify_node,
            image_converter,
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

    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge_node',
        condition=IfCondition(launch_foxglove_bool),
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'use_compression': True
        }]
    )

    return LaunchDescription(
        declared_arguments +
    [
        #urdf_launch,
        #foxglove_node,
        #static_tf_map_to_base_link,
        #static_tf_node,
        #optical_to_image_plane_tf,
        #static_tf_sc2_to_base_link,
        detection_container,
        detection_include
    ])
