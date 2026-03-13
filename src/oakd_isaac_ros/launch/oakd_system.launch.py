import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('oakd_isaac_ros')
    tools_dir = os.path.join(package_share, 'launch', 'tools')

    run_oakd = LaunchConfiguration('run_oakd')
    enable_detection = LaunchConfiguration('enable_detection')
    enable_nvblox = LaunchConfiguration('enable_nvblox')
    enable_visualization = LaunchConfiguration('enable_visualization')
    enable_visual_servo = LaunchConfiguration('enable_visual_servo')
    enable_target_point_estimator = LaunchConfiguration('enable_target_point_estimator')
    enable_debug_viz = LaunchConfiguration('enable_debug_viz')
    enable_mavros_bridge = LaunchConfiguration('enable_mavros_bridge')

    detections_topic = LaunchConfiguration('detections_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    mask_topic = LaunchConfiguration('mask_topic')
    target_topic = LaunchConfiguration('target_topic')
    target_point_topic = LaunchConfiguration('target_point_topic')
    target_point_status_topic = LaunchConfiguration('target_point_status_topic')
    cmd_topic = LaunchConfiguration('cmd_topic')
    status_topic = LaunchConfiguration('status_topic')
    debug_markers_topic = LaunchConfiguration('debug_markers_topic')
    csv_path = LaunchConfiguration('csv_path')

    controller_mode = LaunchConfiguration('controller_mode')
    target_class = LaunchConfiguration('target_class')
    min_confidence = LaunchConfiguration('min_confidence')
    pick_strategy = LaunchConfiguration('pick_strategy')
    max_age_ms = LaunchConfiguration('max_age_ms')
    observer_timeout_ms = LaunchConfiguration('observer_timeout_ms')
    use_depth = LaunchConfiguration('use_depth')
    min_valid_points = LaunchConfiguration('min_valid_points')
    num_nearest_points = LaunchConfiguration('num_nearest_points')
    max_search_radius_px = LaunchConfiguration('max_search_radius_px')
    use_mask_gating = LaunchConfiguration('use_mask_gating')
    hold_last_valid = LaunchConfiguration('hold_last_valid')
    max_hold_ms = LaunchConfiguration('max_hold_ms')
    controller_target_point_timeout_ms = LaunchConfiguration('controller_target_point_timeout_ms')
    desired_target_distance_m = LaunchConfiguration('desired_target_distance_m')
    k_target_distance = LaunchConfiguration('k_target_distance')
    require_target_point = LaunchConfiguration('require_target_point')
    use_sim_time = LaunchConfiguration('use_sim_time')
    body_frame_id = LaunchConfiguration('body_frame_id')

    mavros_backend = LaunchConfiguration('mavros_backend')
    deadman_topic = LaunchConfiguration('deadman_topic')
    deadman_initial = LaunchConfiguration('deadman_initial')
    deadman_required = LaunchConfiguration('deadman_required')
    mavros_timeout_ms = LaunchConfiguration('mavros_timeout_ms')

    visual_servo_and_target_point = IfCondition(
        PythonExpression([
            '"', enable_visual_servo, '" == "true" and "',
            enable_target_point_estimator, '" == "true"'
        ])
    )

    launch_description = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('run_oakd', default_value='true'),
        DeclareLaunchArgument('enable_detection', default_value='true'),
        DeclareLaunchArgument('enable_nvblox', default_value='true'),
        DeclareLaunchArgument('enable_visualization', default_value='true'),
        DeclareLaunchArgument('enable_visual_servo', default_value='true'),
        DeclareLaunchArgument('enable_target_point_estimator', default_value='true'),
        DeclareLaunchArgument('enable_debug_viz', default_value='true'),
        DeclareLaunchArgument('enable_mavros_bridge', default_value='false'),

        DeclareLaunchArgument('detections_topic', default_value='/detections_output'),
        DeclareLaunchArgument('depth_topic', default_value='/oak/stereo/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/oak/stereo/camera_info'),
        DeclareLaunchArgument('mask_topic', default_value='/camera0/detection/mask'),
        DeclareLaunchArgument('target_topic', default_value='/sc2/visual_servo/target_obs'),
        DeclareLaunchArgument('target_point_topic', default_value='/sc2/visual_servo/target_point'),
        DeclareLaunchArgument('target_point_status_topic', default_value='/sc2/visual_servo/target_point_status'),
        DeclareLaunchArgument('cmd_topic', default_value='/sc2/visual_servo/cmd_vel_body'),
        DeclareLaunchArgument('status_topic', default_value='/sc2/visual_servo/status'),
        DeclareLaunchArgument('debug_markers_topic', default_value='/sc2/visual_servo/debug_markers'),
        DeclareLaunchArgument('csv_path', default_value='/tmp/sc2_visual_servo_mvp.csv'),
        DeclareLaunchArgument('body_frame_id', default_value='base_link'),

        DeclareLaunchArgument('controller_mode', default_value='bbox_target_point'),
        DeclareLaunchArgument('target_class', default_value=''),
        DeclareLaunchArgument('min_confidence', default_value='0.3'),
        DeclareLaunchArgument('pick_strategy', default_value='highest_confidence'),
        DeclareLaunchArgument('max_age_ms', default_value='300'),
        DeclareLaunchArgument('observer_timeout_ms', default_value='500'),
        DeclareLaunchArgument('use_depth', default_value='true'),
        DeclareLaunchArgument('min_valid_points', default_value='20'),
        DeclareLaunchArgument('num_nearest_points', default_value='25'),
        DeclareLaunchArgument('max_search_radius_px', default_value='40.0'),
        DeclareLaunchArgument('use_mask_gating', default_value='false'),
        DeclareLaunchArgument('hold_last_valid', default_value='true'),
        DeclareLaunchArgument('max_hold_ms', default_value='500'),
        DeclareLaunchArgument('controller_target_point_timeout_ms', default_value='500'),
        DeclareLaunchArgument('desired_target_distance_m', default_value='0.6'),
        DeclareLaunchArgument('k_target_distance', default_value='0.6'),
        DeclareLaunchArgument('require_target_point', default_value='false'),

        DeclareLaunchArgument('mavros_backend', default_value='dry_run'),
        DeclareLaunchArgument('deadman_topic', default_value='/sc2/visual_servo/deadman'),
        DeclareLaunchArgument('deadman_initial', default_value='false'),
        DeclareLaunchArgument('deadman_required', default_value='true'),
        DeclareLaunchArgument('mavros_timeout_ms', default_value='600'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tools_dir, 'camera.launch.py')),
            launch_arguments={
                'params_file': os.path.join(package_share, 'config', 'oak_d', 'oakd_rgbd.yaml'),
            }.items(),
            condition=IfCondition(run_oakd),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tools_dir, 'detection.launch.py')),
            launch_arguments={
                'container_name': 'nvblox_container',
                'num_cameras': '1',
                'one_container_per_camera': 'True',
            }.items(),
            condition=IfCondition(enable_detection),
        ),

        Node(
            package='oakd_isaac_ros',
            executable='image_converter.py',
            name='image_converter_node',
            output='screen',
            remappings=[
                ('/oak/rgb/image_raw', '/oak/rgb/image_raw'),
                ('/oak/rgb/image_raw_rgb8', '/oak/rgb/image_raw_rgb8'),
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '1.0', '0', '3.14', '1.5708', 'odom', 'oak-d-base-frame'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'oak-d-base-frame', 'oak_rgb_camera_optical_frame'],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tools_dir, 'nvblox.launch.py')),
            launch_arguments={
                'container_name': 'nvblox_container',
                'mode': 'people_detection',
                'camera': 'oakd',
                'num_cameras': '1',
                'voxel_size': '0.05',
            }.items(),
            condition=IfCondition(enable_nvblox),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tools_dir, 'visualization.launch.py')),
            launch_arguments={
                'run_foxglove': 'true',
                'use_foxglove_whitelist': 'false',
                'enable_people_segmentation': 'false',
            }.items(),
            condition=IfCondition(enable_visualization),
        ),

        Node(
            package='sc2_visual_servo_perception',
            executable='target_observer_node',
            name='target_observer_node',
            condition=IfCondition(enable_visual_servo),
            output='screen',
            parameters=[{
                'detections_topic': detections_topic,
                'depth_topic': depth_topic,
                'output_topic': target_topic,
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
            package='sc2_visual_servo_perception',
            executable='target_point_estimator_node',
            name='target_point_estimator_node',
            condition=visual_servo_and_target_point,
            output='screen',
            parameters=[{
                'source_mode': 'bbox_depth_roi',
                'target_obs_topic': target_topic,
                'depth_topic': depth_topic,
                'camera_info_topic': camera_info_topic,
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
            executable='visual_servo_controller_node',
            name='visual_servo_controller_node',
            condition=IfCondition(enable_visual_servo),
            output='screen',
            parameters=[{
                'controller_mode': controller_mode,
                'target_topic': target_topic,
                'target_point_topic': target_point_topic,
                'cmd_topic': cmd_topic,
                'status_topic': status_topic,
                'body_frame_id': body_frame_id,
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
            package='sc2_visual_servo_controller',
            executable='csv_logger_node',
            name='csv_logger_node',
            condition=IfCondition(enable_visual_servo),
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
            executable='visual_servo_debug_viz_node',
            name='visual_servo_debug_viz_node',
            condition=IfCondition(enable_debug_viz),
            output='screen',
            parameters=[{
                'target_topic': target_topic,
                'cmd_topic': cmd_topic,
                'target_point_topic': target_point_topic,
                'markers_topic': debug_markers_topic,
                'body_frame_id': body_frame_id,
                'image_width': 1280.0,
                'image_height': 720.0,
                'target_u': 640.0,
                'target_v': 360.0,
                'use_target_point': enable_target_point_estimator,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='sc2_visual_servo_controller',
            executable='cmd_vel_to_mavros_bridge',
            name='cmd_vel_to_mavros_bridge',
            condition=IfCondition(enable_mavros_bridge),
            output='screen',
            parameters=[{
                'backend': mavros_backend,
                'cmd_topic': cmd_topic,
                'deadman_topic': deadman_topic,
                'deadman_initial': deadman_initial,
                'deadman_required': deadman_required,
                'timeout_ms': mavros_timeout_ms,
                'max_vx': 0.35,
                'max_vy': 0.25,
                'max_vz': 0.25,
                'max_yaw_rate': 0.35,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])

    return launch_description
