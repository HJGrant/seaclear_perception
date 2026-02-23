from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_stub = LaunchConfiguration('use_stub_target')
    csv_path = LaunchConfiguration('csv_path')
    controller_mode = LaunchConfiguration('controller_mode')

    return LaunchDescription([
        DeclareLaunchArgument('use_stub_target', default_value='true'),
        DeclareLaunchArgument('csv_path', default_value='/tmp/sc2_visual_servo_mvp.csv'),
        DeclareLaunchArgument('controller_mode', default_value='ibvs'),

        Node(
            package='sc2_visual_servo_controller',
            executable='target_obs_publisher',
            name='target_obs_publisher',
            condition=IfCondition(use_stub),
            output='screen',
        ),

        Node(
            package='sc2_visual_servo_perception',
            executable='target_observer_node',
            name='target_observer_node',
            condition=UnlessCondition(use_stub),
            output='screen',
            parameters=[{
                'detections_topic': '/detections_output',
                'depth_topic': '/oak/stereo/image_raw',
                'output_topic': '/sc2/visual_servo/target_obs',
                'use_depth': True,
            }],
        ),

        Node(
            package='sc2_visual_servo_controller',
            executable='visual_servo_controller_node',
            name='visual_servo_controller_node',
            output='screen',
            parameters=[{
                'controller_mode': controller_mode,
                'target_topic': '/sc2/visual_servo/target_obs',
                'cmd_topic': '/sc2/visual_servo/cmd_vel_body',
                'status_topic': '/sc2/visual_servo/status',
                'body_frame_id': 'base_link',
                'timeout_ms': 500,
                'max_vx': 0.35,
                'max_vy': 0.25,
                'max_vz': 0.25,
                'max_yaw_rate': 0.35,
                'image_width': 1280.0,
                'image_height': 720.0,
                'target_u': 640.0,
                'target_v': 360.0,
            }],
        ),

        Node(
            package='sc2_visual_servo_controller',
            executable='fake_vehicle_node',
            name='fake_vehicle_node',
            output='screen',
            parameters=[{
                'cmd_topic': '/sc2/visual_servo/cmd_vel_body',
                'pose_topic': '/sc2/visual_servo/fake_pose',
            }],
        ),

        Node(
            package='sc2_visual_servo_controller',
            executable='csv_logger_node',
            name='csv_logger_node',
            output='screen',
            parameters=[{
                'target_topic': '/sc2/visual_servo/target_obs',
                'cmd_topic': '/sc2/visual_servo/cmd_vel_body',
                'csv_path': csv_path,
            }],
        ),

        Node(
            package='sc2_visual_servo_controller',
            executable='cmd_vel_to_mavlink_stub',
            name='cmd_vel_to_mavlink_stub',
            output='screen',
            parameters=[{'cmd_topic': '/sc2/visual_servo/cmd_vel_body'}],
        ),
    ])
