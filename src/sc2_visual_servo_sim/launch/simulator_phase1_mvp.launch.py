from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    lighting = LaunchConfiguration('lighting')

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='dynamic_velocity'),
        DeclareLaunchArgument('detector_mode', default_value='synthetic'),
        DeclareLaunchArgument('lighting', default_value='bright'),
        DeclareLaunchArgument('target_world_x_m', default_value='1.0'),
        DeclareLaunchArgument('target_world_y_m', default_value='0.6'),
        DeclareLaunchArgument('target_world_z_m', default_value='-4.5'),
        DeclareLaunchArgument('initial_z_m', default_value='-1.5'),
        DeclareLaunchArgument('initial_yaw_rad', default_value='0.0'),

        Node(
            package='sc2_visual_servo_sim',
            executable='sim_static_frames_node',
            name='sim_static_frames_node',
            output='screen',
            parameters=[{
                'body_frame_id': 'base_link',
                'camera_link_frame_id': 'down_camera_link',
                'camera_optical_frame_id': 'down_camera_optical_frame',
            }],
        ),

        Node(
            package='sc2_visual_servo_sim',
            executable='sim_direct_velocity_backend_node',
            name='sim_direct_velocity_backend_node',
            output='screen',
            condition=IfCondition(PythonExpression([LaunchConfiguration('backend'), " == 'direct_velocity'"])),
            parameters=[{
                'cmd_topic': '/sc2/visual_servo/cmd_vel_body',
                'odom_topic': '/sim/odom',
                'initial_z_m': LaunchConfiguration('initial_z_m'),
                'initial_yaw_rad': LaunchConfiguration('initial_yaw_rad'),
            }],
        ),

        Node(
            package='sc2_visual_servo_sim',
            executable='sim_dynamic_velocity_backend_node',
            name='sim_dynamic_velocity_backend_node',
            output='screen',
            condition=UnlessCondition(PythonExpression([LaunchConfiguration('backend'), " == 'direct_velocity'"])),
            parameters=[{
                'cmd_topic': '/sc2/visual_servo/cmd_vel_body',
                'odom_topic': '/sim/odom',
                'wrench_topic': '/sim/backend/wrench',
                'initial_z_m': LaunchConfiguration('initial_z_m'),
                'initial_yaw_rad': LaunchConfiguration('initial_yaw_rad'),
            }],
        ),

        Node(
            package='sc2_visual_servo_sim',
            executable='odom_to_body_twist_node',
            name='odom_to_body_twist_node',
            output='screen',
            parameters=[{
                'odom_topic': '/sim/odom',
                'body_twist_topic': '/sim/body_velocity',
                'body_frame_id': 'base_link',
            }],
        ),

        Node(
            package='sc2_visual_servo_sim',
            executable='sim_downward_camera_node',
            name='sim_downward_camera_node',
            output='screen',
            parameters=[{
                'odom_topic': '/sim/odom',
                'image_topic': '/sim/down/image_raw',
                'depth_topic': '/sim/down/depth/image_raw',
                'camera_info_topic': '/sim/down/camera_info',
                'camera_optical_frame_id': 'down_camera_optical_frame',
                'target_world_x_m': LaunchConfiguration('target_world_x_m'),
                'target_world_y_m': LaunchConfiguration('target_world_y_m'),
                'target_world_z_m': LaunchConfiguration('target_world_z_m'),
                'lighting_mode': lighting,
            }],
        ),

        Node(
            package='sc2_visual_servo_synth',
            executable='synthetic_bbox_publisher_node',
            name='synthetic_bbox_publisher_node',
            output='screen',
            condition=IfCondition(PythonExpression([LaunchConfiguration('detector_mode'), " == 'synthetic'"])),
            parameters=[{
                'detections_topic': '/detections_output',
                'best_detection_topic': '/best_detection',
                'frame_id': 'down_camera_optical_frame',
                'pattern': 'fixed_offsets',
                'base_u_px': 640.0,
                'base_v_px': 360.0,
                'fixed_offset_u_px': 180.0,
                'fixed_offset_v_px': 100.0,
            }],
        ),

        Node(
            package='sc2_visual_servo_sim',
            executable='rgb_target_detector_node',
            name='rgb_target_detector_node',
            output='screen',
            condition=UnlessCondition(PythonExpression([LaunchConfiguration('detector_mode'), " == 'synthetic'"])),
            parameters=[{
                'image_topic': '/sim/down/image_raw',
                'detections_topic': '/detections_output',
                'best_detection_topic': '/best_detection',
                'frame_id': 'down_camera_optical_frame',
            }],
        ),

        Node(
            package='sc2_visual_servo_perception',
            executable='target_point_estimator_node',
            name='target_point_estimator_node',
            output='screen',
            parameters=[{
                'target_bounding_box': '/best_detection',
                'depth_topic': '/sim/down/depth/image_raw',
                'camera_info_topic': '/sim/down/camera_info',
                'target_point_topic': '/sc2/visual_servo/target_point',
                'default_frame_id': 'down_camera_optical_frame',
            }],
        ),

        Node(
            package='sc2_visual_servo_controller',
            executable='visual_servo_controller_node',
            name='visual_servo_controller_node',
            output='screen',
            parameters=[{
                'controller_mode': 'staged_pid',
                'target_bounding_box': '/best_detection',
                'target_point_topic': '/sc2/visual_servo/target_point',
                'cmd_topic': '/sc2/visual_servo/cmd_vel_body',
                'vel_measurement_topic': '/sim/body_velocity',
                'camera_info_topic': '/sim/down/camera_info',
                'camera_optical_frame_id': 'down_camera_optical_frame',
                'body_frame_id': 'base_link',
                'target_u': 640.0,
                'target_v': 360.0,
                'image_width': 1280.0,
                'image_height': 720.0,
                'max_vx': 0.35,
                'max_vy': 0.25,
                'max_vz': 0.20,
                'start_enabled': True,
                'staged_pid_horizontal_hold_sec': 1000.0,
                'staged_pid_z_source': 'target_pose',
            }],
        ),
    ])
