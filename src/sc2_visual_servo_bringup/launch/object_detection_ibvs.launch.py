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
            'target_bounding_box': '/best_detection',
            'depth_topic': '/oak/stereo/image_raw',
            'camera_info_topic': '/oak/stereo/camera_info',
            'status_topic': '/sc2/visual_servo/target_point_status',
            'default_frame_id': 'oak_rgb_camera_optical_frame',
            'max_hold_ms': 500.0,
            'hold_last_valid': True,
            'max_obs_age_ms': 300.0,
            'depth_min_m': 0.2,
            'depth_max_m': 20.0,
            'min_valid_points': 20,
            'num_nearest_points': 25,
            'max_search_radius_px': 40.0,
            'use_uv_median_depth': True,
            'use_mask_gating': False,
            'mask_threshold': 1
        }],
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
            'cmd_topic': '/mavros/setpoint_velocity/cmd_vel',
            'status_topic': '/sc2/visual_servo/status',
            'body_frame_id': 'base_link',
            'rate_hz' : 10.0,
            'timeout_ms': 500,
            'target_point_timeout_ms': 500,
            'max_vx': 0.20,
            'max_vy': 0.20,
            'max_vz': 0.0,
            'max_yaw_rate': 0.35,
            'image_width': 640.0,
            'image_height': 640.0,
            'target_u': 320.0,
            'target_v': 320.0,
            'desired_target_distance_m': 1.0,
            'ibvs_lambda': 0.1,
            'alpha_1': 0.5,  #apply no filtering
            'decay_rate': 0.1,
            'mavros_depth': True
        }],
    )

    return LaunchDescription([
        declare_launch_ibvs_arg,
        declare_launch_foxglove_arg,
        oakd_object_detection_launch,
        target_pose_node,
        ibvs_node
    ])