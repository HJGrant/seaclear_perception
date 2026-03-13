from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_stub = LaunchConfiguration('use_stub_target')
    csv_path = LaunchConfiguration('csv_path')
    controller_mode = LaunchConfiguration('controller_mode')

    detections_topic = LaunchConfiguration('detections_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    target_class = LaunchConfiguration('target_class')
    min_confidence = LaunchConfiguration('min_confidence')
    pick_strategy = LaunchConfiguration('pick_strategy')
    max_age_ms = LaunchConfiguration('max_age_ms')
    observer_timeout_ms = LaunchConfiguration('observer_timeout_ms')
    use_depth = LaunchConfiguration('use_depth')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_target_point_estimator = LaunchConfiguration('enable_target_point_estimator')
    target_point_topic = LaunchConfiguration('target_point_topic')
    target_point_status_topic = LaunchConfiguration('target_point_status_topic')
    mask_topic = LaunchConfiguration('mask_topic')
    use_mask_gating = LaunchConfiguration('use_mask_gating')
    min_valid_points = LaunchConfiguration('min_valid_points')
    num_nearest_points = LaunchConfiguration('num_nearest_points')
    max_search_radius_px = LaunchConfiguration('max_search_radius_px')
    hold_last_valid = LaunchConfiguration('hold_last_valid')
    max_hold_ms = LaunchConfiguration('max_hold_ms')
    controller_target_point_timeout_ms = LaunchConfiguration('controller_target_point_timeout_ms')
    desired_target_distance_m = LaunchConfiguration('desired_target_distance_m')
    k_target_distance = LaunchConfiguration('k_target_distance')
    require_target_point = LaunchConfiguration('require_target_point')
    enable_debug_viz = LaunchConfiguration('enable_debug_viz')
    debug_markers_topic = LaunchConfiguration('debug_markers_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_stub_target', default_value='true'),
        DeclareLaunchArgument('csv_path', default_value='/tmp/sc2_visual_servo_mvp.csv'),
        DeclareLaunchArgument('controller_mode', default_value='ibvs'),

        DeclareLaunchArgument('detections_topic', default_value='/detections_output'),
        DeclareLaunchArgument('depth_topic', default_value='/oak/stereo/image_raw'),
        DeclareLaunchArgument('target_class', default_value=''),
        DeclareLaunchArgument('min_confidence', default_value='0.3'),
        DeclareLaunchArgument('pick_strategy', default_value='highest_confidence'),
        DeclareLaunchArgument('max_age_ms', default_value='300'),
        DeclareLaunchArgument('observer_timeout_ms', default_value='500'),
        DeclareLaunchArgument('use_depth', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_target_point_estimator', default_value='false'),
        DeclareLaunchArgument('target_point_topic', default_value='/sc2/visual_servo/target_point'),
        DeclareLaunchArgument('target_point_status_topic', default_value='/sc2/visual_servo/target_point_status'),
        DeclareLaunchArgument('mask_topic', default_value=''),
        DeclareLaunchArgument('use_mask_gating', default_value='false'),
        DeclareLaunchArgument('min_valid_points', default_value='20'),
        DeclareLaunchArgument('num_nearest_points', default_value='25'),
        DeclareLaunchArgument('max_search_radius_px', default_value='40.0'),
        DeclareLaunchArgument('hold_last_valid', default_value='true'),
        DeclareLaunchArgument('max_hold_ms', default_value='500'),
        DeclareLaunchArgument('controller_target_point_timeout_ms', default_value='500'),
        DeclareLaunchArgument('desired_target_distance_m', default_value='0.6'),
        DeclareLaunchArgument('k_target_distance', default_value='0.6'),
        DeclareLaunchArgument('require_target_point', default_value='false'),
        DeclareLaunchArgument('enable_debug_viz', default_value='true'),
        DeclareLaunchArgument('debug_markers_topic', default_value='/sc2/visual_servo/debug_markers'),

        Node(
            package='sc2_visual_servo_controller',
            executable='target_obs_publisher',
            name='target_obs_publisher',
            condition=IfCondition(use_stub),
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='sc2_visual_servo_perception',
            executable='target_observer_node',
            name='target_observer_node',
            condition=UnlessCondition(use_stub),
            output='screen',
            parameters=[{
                'detections_topic': detections_topic,
                'depth_topic': depth_topic,
                'output_topic': '/sc2/visual_servo/target_obs',
                'target_class': target_class,
                'min_confidence': min_confidence,
                'pick_strategy': pick_strategy,
                'max_age_ms': max_age_ms,
                'timeout_ms': observer_timeout_ms,
                'use_depth': use_depth,
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
                'target_topic': '/sc2/visual_servo/target_obs',
                'target_point_topic': target_point_topic,
                'cmd_topic': '/sc2/visual_servo/cmd_vel_body',
                'status_topic': '/sc2/visual_servo/status',
                'body_frame_id': 'base_link',
                'timeout_ms': 500,
                'target_point_timeout_ms': controller_target_point_timeout_ms,
                'max_vx': 0.35,
                'max_vy': 0.25,
                'max_vz': 0.25,
                'max_yaw_rate': 0.35,
                'image_width': 1280.0,
                'image_height': 720.0,
                'target_u': 640.0,
                'target_v': 360.0,
                'k_target_distance': k_target_distance,
                'desired_target_distance_m': desired_target_distance_m,
                'require_target_point': require_target_point,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='sc2_visual_servo_perception',
            executable='target_point_estimator_node',
            name='target_point_estimator_node',
            condition=IfCondition(enable_target_point_estimator),
            output='screen',
            parameters=[{
                'source_mode': 'bbox_depth_roi',
                'target_obs_topic': '/sc2/visual_servo/target_obs',
                'depth_topic': depth_topic,
                'camera_info_topic': '/oak/stereo/camera_info',
                'mask_topic': mask_topic,
                'target_point_topic': target_point_topic,
                'status_topic': target_point_status_topic,
                'depth_min_m': 0.2,
                'depth_max_m': 20.0,
                'min_valid_points': min_valid_points,
                'num_nearest_points': num_nearest_points,
                'max_search_radius_px': max_search_radius_px,
                'use_uv_median_depth': True,
                'use_mask_gating': use_mask_gating,
                'hold_last_valid': hold_last_valid,
                'max_hold_ms': max_hold_ms,
                'use_sim_time': use_sim_time,
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
                'use_sim_time': use_sim_time,
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
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='sc2_visual_servo_controller',
            executable='cmd_vel_to_mavlink_stub',
            name='cmd_vel_to_mavlink_stub',
            output='screen',
            parameters=[{'cmd_topic': '/sc2/visual_servo/cmd_vel_body', 'use_sim_time': use_sim_time}],
        ),

        Node(
            package='sc2_visual_servo_controller',
            executable='visual_servo_debug_viz_node',
            name='visual_servo_debug_viz_node',
            condition=IfCondition(enable_debug_viz),
            output='screen',
            parameters=[{
                'target_topic': '/sc2/visual_servo/target_obs',
                'cmd_topic': '/sc2/visual_servo/cmd_vel_body',
                'target_point_topic': target_point_topic,
                'markers_topic': debug_markers_topic,
                'body_frame_id': 'base_link',
                'image_width': 1280.0,
                'image_height': 720.0,
                'target_u': 640.0,
                'target_v': 360.0,
                'use_target_point': enable_target_point_estimator,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
