from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pattern = LaunchConfiguration('pattern')
    rate_hz = LaunchConfiguration('rate_hz')
    publish_direct = LaunchConfiguration('publish_target_obs_direct')

    detections_topic = LaunchConfiguration('detections_topic')
    target_topic = LaunchConfiguration('target_topic')
    cmd_topic = LaunchConfiguration('cmd_topic')
    status_topic = LaunchConfiguration('status_topic')
    csv_path = LaunchConfiguration('csv_path')

    controller_mode = LaunchConfiguration('controller_mode')
    mavros_backend = LaunchConfiguration('mavros_backend')
    deadman = LaunchConfiguration('deadman')
    deadman_topic = LaunchConfiguration('deadman_topic')
    deadman_required = LaunchConfiguration('deadman_required')
    timeout_ms = LaunchConfiguration('timeout_ms')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_debug_viz = LaunchConfiguration('enable_debug_viz')
    debug_markers_topic = LaunchConfiguration('debug_markers_topic')

    return LaunchDescription([
        DeclareLaunchArgument('pattern', default_value='fixed_offsets'),
        DeclareLaunchArgument('rate_hz', default_value='15.0'),
        DeclareLaunchArgument('publish_target_obs_direct', default_value='false'),
        DeclareLaunchArgument('detections_topic', default_value='/detections_output'),
        DeclareLaunchArgument('target_topic', default_value='/sc2/visual_servo/target_obs'),
        DeclareLaunchArgument('cmd_topic', default_value='/sc2/visual_servo/cmd_vel_body'),
        DeclareLaunchArgument('status_topic', default_value='/sc2/visual_servo/status'),
        DeclareLaunchArgument('csv_path', default_value='/tmp/sc2_visual_servo_mvp.csv'),

        DeclareLaunchArgument('controller_mode', default_value='bbox_approach'),
        DeclareLaunchArgument('mavros_backend', default_value='rc_override'),
        DeclareLaunchArgument('deadman', default_value='true'),
        DeclareLaunchArgument('deadman_topic', default_value='/sc2/visual_servo/deadman'),
        DeclareLaunchArgument('deadman_required', default_value='true'),
        DeclareLaunchArgument('timeout_ms', default_value='600'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_debug_viz', default_value='true'),
        DeclareLaunchArgument('debug_markers_topic', default_value='/sc2/visual_servo/debug_markers'),

        Node(
            package='sc2_visual_servo_synth',
            executable='synthetic_bbox_publisher_node',
            name='synthetic_bbox_publisher_node',
            output='screen',
            parameters=[{
                'pattern': pattern,
                'rate_hz': rate_hz,
                'detections_topic': detections_topic,
                'target_obs_topic': target_topic,
                'publish_target_obs_direct': publish_direct,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='sc2_visual_servo_perception',
            executable='target_observer_node',
            name='target_observer_node',
            condition=UnlessCondition(publish_direct),
            output='screen',
            parameters=[{
                'detections_topic': detections_topic,
                'depth_topic': '',
                'output_topic': target_topic,
                'target_class': '',
                'min_confidence': 0.1,
                'pick_strategy': 'highest_confidence',
                'max_age_ms': 400,
                'timeout_ms': timeout_ms,
                'use_depth': False,
                'source': 'synthetic_detection2d',
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='sc2_visual_servo_controller',
            executable='visual_servo_controller_node',
            name='visual_servo_controller_node',
            output='screen',
            parameters=[{
                'controller_mode': controller_mode,
                'target_topic': target_topic,
                'cmd_topic': cmd_topic,
                'status_topic': status_topic,
                'body_frame_id': 'base_link',
                'timeout_ms': timeout_ms,
                'rate_hz': 20.0,
                'max_vx': 0.30,
                'max_vy': 0.20,
                'max_vz': 0.18,
                'max_yaw_rate': 0.35,
                'slew_vx': 0.8,
                'slew_vy': 0.8,
                'slew_vz': 0.8,
                'slew_yaw_rate': 1.0,
                'image_width': 1280.0,
                'image_height': 720.0,
                'target_u': 640.0,
                'target_v': 360.0,
                'desired_depth_m': 1.0,
                'bbox_stop_area_px': 22000.0,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='sc2_visual_servo_controller',
            executable='csv_logger_node',
            name='csv_logger_node',
            output='screen',
            parameters=[{
                'target_topic': target_topic,
                'cmd_topic': cmd_topic,
                'csv_path': csv_path,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='sc2_visual_servo_controller',
            executable='cmd_vel_to_mavros_bridge',
            name='cmd_vel_to_mavros_bridge',
            output='screen',
            parameters=[{
                'backend': mavros_backend,
                'cmd_topic': cmd_topic,
                'deadman_topic': deadman_topic,
                'deadman_initial': deadman,
                'deadman_required': deadman_required,
                'timeout_ms': timeout_ms,
                'max_vx': 0.30,
                'max_vy': 0.20,
                'max_vz': 0.18,
                'max_yaw_rate': 0.35,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='sc2_visual_servo_controller',
            executable='visual_servo_debug_viz_node',
            name='visual_servo_debug_viz_node',
            condition=IfCondition(enable_debug_viz),
            output='screen',
            parameters=[{
                'target_topic': target_topic,
                'cmd_topic': cmd_topic,
                'markers_topic': debug_markers_topic,
                'body_frame_id': 'base_link',
                'image_width': 1280.0,
                'image_height': 720.0,
                'target_u': 640.0,
                'target_v': 360.0,
                'use_grasp_pose': False,
                'use_sim_time': use_sim_time,
            }],
        ),

    ])
