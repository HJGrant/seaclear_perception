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
    ),
    # Add your arguments here
    launch_arguments={
        'launch_foxglove': launch_foxglove_bool,
        'camera_model': 'OAK-D',
        'parent_frame': 'oak-d-base-frame'
        }.items()
    )
    
    target_pose_node = Node(
        package='sc2_visual_servo_perception',
        executable='target_point_estimator_node',
        name='target_point_estimator_node',
        output='screen',
        parameters=[{
            # target_point PoseStamped convention for staged PID:
            #   position.x/y = metric target offset in the camera optical frame
            #   position.z   = target-relative range/depth in meters
            'target_bounding_box': '/best_detection',
            'depth_topic': '/oak/stereo/image_raw',
            'camera_info_topic': '/oak/stereo/camera_info',
            'status_topic': '/sc2/visual_servo/target_point_status',
            'default_frame_id': 'oak_rgb_camera_optical_frame',
            'max_hold_ms': 500.0,
            'hold_last_valid': True,
            'max_obs_age_ms': 300.0,
            'max_depth_age_ms': 300.0,
            'depth_min_m': 0.2,
            'depth_max_m': 20.0,
            'min_valid_points': 20,
            'num_nearest_points': 25,
            'max_search_radius_px': 40.0,
            'use_uv_median_depth': True,
            'use_mask_gating': False,
            'mask_threshold': 1,
            'enable_temporal_filter': True,
            'filter_alpha_uv': 0.4,
            'filter_alpha_depth': 0.3,
            'filter_alpha_size': 0.3,
            'max_uv_jump_px': 80.0,
            'max_depth_jump_m': 0.5,
            'reject_large_jumps': True
        }],
    )

    # Phase 2 staged_pid notes:
    # - This launch keeps the grapple IF and only switches the controller logic
    #   to staged PID.
    # - `valid_target_observation` / `target_tracked` mean ROI-consistent
    #   tracking continuity, not centered/aligned lock or vertical-ready.
    # - Horizontal x/y/yaw follows the Phase 1 staged PID behavior.
    # - Default staged PID vertical source behavior for the standard experiment
    #   command is:
    #     primary  = `/sc2/visual_servo/target_point` (`PoseStamped`)
    #     fallback = DVL altitude, only while target_point is stale/unavailable.
    # - Search/recovery and terminal hold-zero are intentionally not launched.
    # - The controller still publishes through the grapple command adapter:
    #   TwistStamped plus Vector6 on `cmd_topic_vec_6`.
    # 3. Add the condition to your Node
    ibvs_node = Node(
        package='sc2_visual_servo_controller',
        executable='visual_servo_controller_node',
        name='visual_servo_controller_node',
        output='screen',
        condition=IfCondition(launch_ibvs_bool), # <--- The Magic Line
        parameters=[{
            'target_topic': '/sc2/visual_servo/target_obs',
            'target_point_topic': '/sc2/visual_servo/target_point',
            'controller_mode': 'staged_pid',
            'cmd_topic': '/mavros/setpoint_velocity/cmd_vel',           # velocity command for MAVROS
            'cmd_topic_vec_6': '/sc2/velocity_controller_node/outer/console_signals', # velocity command for SC2 (msg type: Vector6)
            'status_topic': '/sc2/visual_servo/status',
            'dvl_altitude_topic' : '/waterlinked_dvl_driver/velocity_report',
            'body_frame_id': 'base_link',
            'rate_hz' : 10.0,
            'timeout_ms': 500,
            'target_point_timeout_ms': 500,
            'max_vx': 0.20,
            'max_vy': 0.20,
            'max_vz': 0.20,
            'max_yaw_rate': 0.35,
            'image_width': 640.0,
            'image_height': 640.0,
            'target_u': 320.0,
            'target_v': 320.0,
            'desired_target_distance_m': 1.0,
            'desired_vertical_distance_m': 1.0,
            'staged_pid_z_source': 'target_point_primary_dvl_fallback',
            'staged_pid_z_kp': 0.8,
            'staged_pid_z_ki': 0.0,
            'staged_pid_z_kd': 0.10,
            'staged_pid_z_integral_limit': 1.0,
            'staged_pid_enable_yaw': True,
            # Phase 2 vertical gate: both u and v must stay within 150 px.
            # At 0.5 m range with HFOV ~= 66 deg and 640 px width,
            # 150 px is roughly 15 cm lateral offset.
            'staged_pid_horizontal_tolerance_px': 150.0,
            'staged_pid_horizontal_tolerance_x_px': 150.0,
            'staged_pid_horizontal_tolerance_y_px': 150.0,
            # Deprecated alias retained for older configs; controller uses the
            # x/y horizontal tolerances for vertical on/off.
            'staged_pid_vertical_tolerance_px': 150.0,
            'staged_pid_distance_tolerance_m': 0.08,
            'staged_pid_vertical_gate_window_ticks': 10,
            'staged_pid_vertical_enter_ratio': 0.8,
            'staged_pid_vertical_exit_ratio': 0.4,
            'staged_pid_vertical_enter_hold_ticks': 2,
            'staged_pid_vertical_exit_hold_ticks': 2,
            'staged_pid_hold_zero_distance_m': 0.5,
            'tracked_enter_window_obs': 10,
            'tracked_enter_min_obs': 2,
            'tracked_exit_no_obs_sec': 0.6,
            'ibvs_lambda': 0.1, # IBVS gain parameter 
            'alpha_1': 0.5, # EMA filter coefficient 
            'decay_rate': 0.1, # decay rate, how fast to shut down the control signal
            # IBVS depth mode. In staged_pid Phase 2 this selects the fallback
            # subscriber; target_point remains the primary vertical source.
            'depth_estimation_mode': 'DVL'
        }],
    )

    return LaunchDescription([
        declare_launch_ibvs_arg,
        declare_launch_foxglove_arg,
        oakd_object_detection_launch,
        target_pose_node,
        ibvs_node
    ])
