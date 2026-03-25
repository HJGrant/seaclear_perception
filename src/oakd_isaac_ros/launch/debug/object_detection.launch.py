import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    container_name = LaunchConfiguration('container_name')
    detector_model_file_path = LaunchConfiguration('detector_model_file_path')
    detector_engine_file_path = LaunchConfiguration('detector_engine_file_path')
    verbose = LaunchConfiguration('verbose')
    target_class_id = LaunchConfiguration('target_class_id')
    package_share = get_package_share_directory('oakd_isaac_ros')
    oakd_launch_file = os.path.join(package_share, 'launch', 'tools', 'camera.launch.py')
    default_detector_model_dir = os.path.join(
        os.environ.get('ISAAC_ROS_WS', '/workspaces/oakd_isaac_ros'),
        'isaac_ros_assets',
        'models',
        'seaclear',
    )

    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oakd_launch_file),
        launch_arguments={
            'params_file': params_file,
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

    detection_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'tools', 'detection.launch.py')
        ),
        launch_arguments={
            'num_cameras': '1',
            'input_topic_list': '["/oak/rgb/image_raw_rgb8"]',
            'one_container_per_camera': 'False',
            'container_name': container_name,
            'model_file_path': detector_model_file_path,
            'engine_file_path': detector_engine_file_path,
            'verbose': verbose,
            'target_class_id': target_class_id,
        }.items()
    )

    delayed_detection_include = TimerAction(
        period=15.0,  # Delay in seconds (adjust as needed)
        actions=[detection_include]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_share, 'config', 'oak_d', 'oakd_dnn_depth.yaml'),
        ),
        DeclareLaunchArgument('container_name', default_value='detection_container'),
        DeclareLaunchArgument(
            'detector_model_file_path',
            default_value=os.path.join(default_detector_model_dir, 'v12n.onnx'),
        ),
        DeclareLaunchArgument(
            'detector_engine_file_path',
            default_value=os.path.join(default_detector_model_dir, 'v12n.plan'),
        ),
        DeclareLaunchArgument('verbose', default_value='True'),
        DeclareLaunchArgument('target_class_id', default_value='3'),
        oakd_launch,
        image_converter,
        detection_container,
        delayed_detection_include
    ])
