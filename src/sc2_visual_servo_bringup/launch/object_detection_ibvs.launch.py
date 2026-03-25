import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Define the Launch Configuration variable
    launch_ibvs_bool = LaunchConfiguration('launch_ibvs')
    launch_foxglove_bool = LaunchConfiguration('launch_foxglove')

    # 2. Declare the argument so it shows up in --show-args
    declare_launch_ibvs_arg = DeclareLaunchArgument(
        'launch_ibvs',
        default_value='true',
        description='Whether to start the IBVS visual servo controller node'
    )

    declare_launch_foxglove_arg = DeclareLaunchArgument(
        'launch_foxglove',
        default_value='true',
        description='Whether to start the Foxglove Bridge node'
    )

    oakd_isaac_ros_dir = get_package_share_directory('oakd_isaac_ros')
    
    oakd_object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(oakd_isaac_ros_dir, 'launch', 'debug', 'object_detection.launch.py')
        )
    )
    
    # 3. Add the condition to your Node
    ibvs_node = Node(
        package='sc2_visual_servo_controller',
        executable='visual_servo_controller_node',
        name='visual_servo_controller_node',
        output='screen',
        condition=IfCondition(launch_ibvs_bool), # <--- The Magic Line
        parameters=[{
            'target_topic': '/sc2/visual_servo/target_obs',
            'cmd_topic': '/sc2/visual_servo/cmd_vel_body',
            'status_topic': '/sc2/visual_servo/status',
            'body_frame_id': 'base_link',
            'timeout_ms': 500,
            'target_point_timeout_ms': 500,
            'max_vx': 0.35,
            'max_vy': 0.25,
            'max_vz': 0.25,
            'max_yaw_rate': 0.35,
            'image_width': 1280.0,
            'image_height': 720.0,
            'target_u': 640.0,
            'target_v': 360.0,
            'desired_target_distance_m': 1.0
        }],
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

    return LaunchDescription([
        declare_launch_ibvs_arg,
        declare_launch_foxglove_arg,
        oakd_object_detection_launch,
        ibvs_node,
        foxglove_node
    ])