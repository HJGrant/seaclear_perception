from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, SetRemap
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    container_name = "detection_container" 

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
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[],
        output='screen'
    )

    # Include the detection pipeline, passing arguments and attaching encoder to detection_container
    yolo_sc1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('oakd_isaac_ros'), 'launch', 'tools', 'detection.launch.py')
        ),
        launch_arguments={
            'num_cameras': '1',
            'input_topic_list': '["/oak/rgb/image_raw_rgb8"]',
            'one_container_per_camera': 'False',
            'container_name': container_name,
            'target_class_id' : '3',
            "confidence_threshold": "0.5",
        }.items()
    )

    # Second instance with target_class_id=5
    yolo_sc2 = GroupAction([
        PushRosNamespace('yolo_sc2'),
        # remap absolute outputs so they don't collide with the first pipeline
        SetRemap('/tensor_pub', '/yolo_sc2/tensor_pub'),
        # ensure detections topic is namespaced if decoder uses absolute name
        SetRemap('/detections_output', '/yolo_sc2/detections_output'),
        SetRemap('/camera0/detection/mask', '/yolo_sc2/camera0/detection/mask'),
        SetRemap('/camera0/detection/mask/camera_info', '/yolo_sc2/camera0/detection/mask/camera_info'),
        SetRemap('/camera0/detection/mask_resized', '/yolo_sc2/camera0/detection/mask_resized'),
        SetRemap('/camera0/detection/mask_resized/camera_info', '/yolo_sc2/camera0/detection/mask_resized/camera_info'),
        # optional, if your overlay topic is absolute and needs to be unique
        SetRemap('/mask_overlay', '/mask_overlay_sc2'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('oakd_isaac_ros'),
                    'launch', 'tools', 'detection.launch.py'
                )
            ),
            launch_arguments={
                'num_cameras': '1',
                'container_name': container_name,
                'input_topic_list': '["/oak/rgb/image_raw_rgb8"]',
                'one_container_per_camera': 'False',
                # set the second model here
                'model_file_path': '/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/sc2_v12s.onnx',
                'engine_file_path': '/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/sc2_v12s.plan',
                # per-model params
                'target_class_id': '3',
                'confidence_threshold': '0.5',
            }.items(),
            # add a condition here if you need it, e.g. condition=IfCondition(mode)
        ),
    ])


    yolo_dual_stream = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_deep_learning'),
                'launch',
                'video_output.ros2.launch'
            )
        ),
        launch_arguments={
            'topic': '/mask_overlay_dual',
            'output': 'rtp://192.168.2.94:8550', #'webrtc://@:8551/yolo_dual',
            'output_codec': 'h264',
            'output_bitrate': '4000000'
        }.items()
        )
    
    depth_mask_publisher = ComposableNode(
        name='depth_mask_publisher',
        package='oakd_isaac_ros',
        # class is registered as `publish_mask` in publish_mask_dual.cpp
        plugin='publish_mask_dual',
        parameters=[{
            'target_class_id': 3,
            'confidence_threshold' : 0.7
        }],
        extra_arguments=[{
            '--ros-args': '', 
            '--log-level': 'debug'}]
    )

    # Load the mask publisher component into the existing container
    depth_mask_loader = LoadComposableNodes(
        target_container=f'/{container_name}',
        composable_node_descriptions=[depth_mask_publisher]
    )


    return LaunchDescription([
        oakd_launch,
        image_converter,
        detection_container,
        yolo_sc1,
        yolo_sc2,
        yolo_dual_stream,
        depth_mask_loader
    ])
