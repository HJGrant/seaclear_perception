import yaml
from typing import Any, List, Tuple
import os

from isaac_ros_launch_utils.all_types import (
    Action,
    ComposableNode,
    ComposableNodeContainer,
    LaunchDescription,
    LoadComposableNodes
)
import isaac_ros_launch_utils as lu
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_ros.actions import Node
import launch_ros.descriptions

#COMMON_CONFIG = 'common_config'
#NVBLOX_CONFIG = 'nvblox_config'

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxPeopleSegmentation
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        'rosbag', 'None', description='Path to rosbag (running on sensor if not set).', cli=True)
    args.add_arg('rosbag_args', '--loop',
                 description='Additional args for ros2 bag play.', cli=True)
    
    args.add_arg('use_rosbag', default='false', description='Use rosbag instead of live OAK-D camera', cli=True)

    args.add_arg('log_level', 'info', choices=[
                 'debug', 'info', 'warn'], cli=True)
    args.add_arg('num_cameras', 1,
                 description='How many cameras to use.', cli=True)
    args.add_arg(
        'mode',
        default=NvbloxMode.people_detection,
        choices=NvbloxMode.names(),
        description='The nvblox mode.',
        cli=True)
    
    args.add_arg(
        'voxel_size',
        default=0.05,
        description='Set the Voxel Size.',
        cli=True)

    args.add_arg(
        'people_segmentation',
        default=NvbloxPeopleSegmentation.peoplesemsegnet_vanilla,
        choices=[
            str(NvbloxPeopleSegmentation.peoplesemsegnet_vanilla),
            str(NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg)
        ],
        description='The  model type of PeopleSemSegNet (only used when mode:=people_segmentation).',
        cli=True)
    args.add_arg(
        'attach_to_container',
        'False',
        description='Add components to an existing component container.',
        cli=True)
    args.add_arg(
        'container_name',
        NVBLOX_CONTAINER_NAME,
        description='Name of the component container.')
    args.add_arg(
        'run_oakd',
        'True',
        description='Launch OAK-D driver')
    args.add_arg(
        'use_foxglove_whitelist',
        True,
        description='Disable visualization of bandwidth-heavy topics',
        cli=True)
    actions = args.get_launch_actions()


    # Globally set use_sim_time if we're running from bag or sim
    actions.append(
        SetParameter('use_sim_time', True, condition=IfCondition(lu.is_valid(args.use_rosbag))))

    run_oakd_driver = UnlessCondition(
        OrSubstitution(lu.is_valid(args.rosbag), lu.is_false(args.run_oakd)))
    
    camera_input_topics = ['/oak/rgb/image_raw_rgb8']
    input_camera_info_topics = ['/oak/rgb/camera_info']
    output_resized_image_topics = ['/oak/segmentation/image_resized']
    output_resized_camera_info_topics = ['/oak/segmentation/camera_info_resized']
    camera_mode = 'oakd'

    # launch the OAK-D ROS driver, the config is in config/oak_d/oakd_rgbd.yaml
    actions.append(
        lu.include(
            'oakd_isaac_ros',
            'launch/tools/camera.launch.py',
            launch_arguments={
                'container_name': args.container_name,
                'params_file': os.path.join(str(get_package_share_directory('oakd_isaac_ros')),'config', 'oak_d', 'oakd_rgbd.yaml'),
                # Remap /oak/imu/data -> /imu/data_ned through camera.launch.py
                'imu_topic_remapped': '/imu/data_ned',
                },
            condition=UnlessCondition(LaunchConfiguration('use_rosbag'))
            ))


    # Run people segmentation if the right flag has been raised
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/perception/segmentation.launch.py',
            launch_arguments={
                'container_name': args.container_name,
                'people_segmentation': args.people_segmentation,
                #'namespace_list': camera_namespaces,
                'input_topic_list': camera_input_topics,
                'input_camera_info_topic_list': input_camera_info_topics,
                'output_resized_image_topic_list': output_resized_image_topics,
                'output_resized_camera_info_topic_list': output_resized_camera_info_topics,
                'num_cameras': args.num_cameras,
                # fixing rosbag replay dropping fps
                'one_container_per_camera': True
            },
            condition=IfCondition(lu.has_substring(args.mode, NvbloxMode.people_segmentation))))

    # Run the object segmentation code by launching the 'people_segmentation' pipeline, in fact runs the SeaClear 1.0 YOLO inference
    actions.append(
        lu.include(
            'oakd_isaac_ros',
            'launch/tools/detection.launch.py',
            launch_arguments={
                #'namespace_list': camera_namespaces,
                'input_topic_list': camera_input_topics,
                'num_cameras': args.num_cameras,
                'container_name': args.container_name,
                # fixing rosbag replay dropping fps
                'one_container_per_camera': True
            },
            condition=IfCondition(lu.has_substring(args.mode, NvbloxMode.people_detection))))

    # image converter
    actions.append(
        Node(
            package='oakd_isaac_ros',
            executable='image_converter.py',
            name='image_converter_node',
            output='screen',
            remappings=[
                ('/oak/rgb/image_raw', '/oak/rgb/image_raw'),
                ('/oak/rgb/image_raw_rgb8', '/oak/rgb/image_raw_rgb8')
            ]
        )
    )
    
    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        )
    )
    actions.append(
    Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "1.0", "0", "3.14", "1.5708", "odom", "oak-d-base-frame"],
        )
    )

    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "oak-d-base-frame", "oak_rgb_camera_optical_frame"],
        )
    )

    # Nvblox
    actions.append(
        lu.include(
            'oakd_isaac_ros',
            'launch/tools/nvblox.launch.py',
            launch_arguments={
                'container_name': args.container_name,
                'mode': args.mode,
                'camera': camera_mode,
                'num_cameras': args.num_cameras,
                'voxel_size' : args.voxel_size,
            }))

    # Play ros2bag
    actions.append(
        lu.play_rosbag(
            bag_path=os.path.join(str(get_package_share_directory('oakd_isaac_ros')),'assets', 'underwater_test_bag'),
            additional_bag_play_args=args.rosbag_args,
            condition=IfCondition(lu.is_valid(args.use_rosbag))))

    # Visualization
    actions.append(
        lu.include(
            'oakd_isaac_ros',
            'launch/tools/visualization.launch.py',
            launch_arguments={
                'mode': args.mode,
                #'camera': camera_mode,
                'use_foxglove_whitelist': args.use_foxglove_whitelist,
            }))

    # run WebRTC Stream for RGB
    actions.append(
    IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_deep_learning'),
                'launch',
                'video_output.ros2.launch'
            )
        ),
        launch_arguments={
            'topic': '/oak/rgb/image_raw',
            'output': 'webrtc://@:8554/rgb',
            'output_codec': 'h264',
            'output_bitrate': '4000000'
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_rosbag'))
        )
    )   

    # run WebRTC Stream for Object Detection
    actions.append(
    IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_deep_learning'),
                'launch',
                'video_output.ros2.launch'
            )
        ),
        launch_arguments={
            'topic': '/mask_overlay',
            'output': 'webrtc://@:8555/yolo',
            'output_codec': 'h264',
            'output_bitrate': '4000000'
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_rosbag'))
        )
    )   

    # run WebRTC Stream for Depth Map Overlay
    actions.append(
    IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_deep_learning'),
                'launch',
                'video_output.ros2.launch'
            )
        ),
        launch_arguments={
            'topic': '/nvblox_node/dynamic_depth_frame_overlay',
            'output': 'webrtc://@:8556/depth',
            'output_codec': 'h264',
            'output_bitrate': '4000000'
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_rosbag'))
        )
    ) 

    actions.append(
    IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('isaac_ros_jetson_stats'),
                'launch',
                'jtop.launch.py'
            ))
        )
    )

    actions.append(
        lu.component_container(
            NVBLOX_CONTAINER_NAME, condition=UnlessCondition(args.attach_to_container),
            log_level=args.log_level))
    
    actions.append(
        lu.component_container('detection_container', log_level=args.log_level)
    )

    return LaunchDescription(actions)

