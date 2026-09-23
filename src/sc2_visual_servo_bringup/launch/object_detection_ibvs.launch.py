import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def _package_available(name: str) -> bool:
    try:
        get_package_share_directory(name)
        return True
    except PackageNotFoundError:
        return False


def _optional_detection_stack(context):
    launch_detection_stack = LaunchConfiguration('launch_detection_stack').perform(context)
    launch_foxglove = LaunchConfiguration('launch_foxglove').perform(context)

    base_required = [
        'oakd_isaac_ros',
        'depthai_ros_driver',
        'depthai_descriptions',
        'isaac_ros_dnn_image_encoder',
        'isaac_ros_tensor_rt',
        'isaac_ros_yolov8',
        'isaac_ros_image_proc',
    ]
    if _as_bool(launch_foxglove):
        base_required.extend([
            'isaac_ros_h264_encoder',
            'foxglove_bridge',
        ])

    missing = [pkg for pkg in base_required if not _package_available(pkg)]
    mode = str(launch_detection_stack).strip().lower()
    if mode not in {'auto', 'true', 'false'}:
        raise RuntimeError(
            f'Invalid launch_detection_stack={launch_detection_stack!r}; use auto, true, or false'
        )

    if mode == 'false':
        return [
            LogInfo(msg='[object_detection_ibvs] Skipping OAK-D detection stack by launch_detection_stack:=false'),
        ]

    if missing:
        if mode == 'true':
            raise RuntimeError(
                'launch_detection_stack:=true requested, but required packages are missing: ' +
                ', '.join(missing)
            )
        return [
            LogInfo(
                msg='[object_detection_ibvs] Detection stack unavailable in this environment; '
                    'launching observer/controller only. Missing packages: ' + ', '.join(missing)
            ),
        ]

    oakd_isaac_ros_dir = get_package_share_directory('oakd_isaac_ros')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(oakd_isaac_ros_dir, 'launch', 'debug', 'object_detection.launch.py')
            ),
            launch_arguments={
                'launch_foxglove': LaunchConfiguration('launch_foxglove'),
                'camera_model': 'OAK-D',
                'parent_frame': 'oak-d-base-frame',
            }.items()
        )
    ]


def _default_config_file() -> str:
    oakd_isaac_ros_dir = get_package_share_directory('oakd_isaac_ros')
    return os.path.join(oakd_isaac_ros_dir, 'config', 'oak_d', 'object_detection_ibvs.yaml')


def generate_launch_description():
    launch_ibvs_bool = LaunchConfiguration('launch_ibvs')
    launch_foxglove_bool = LaunchConfiguration('launch_foxglove')
    launch_target_point_bool = LaunchConfiguration('launch_target_point')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    centroid_kf_enable = LaunchConfiguration('centroid_kf_enable')
    camera_optical_frame_id = LaunchConfiguration('camera_optical_frame_id')
    body_frame_id = LaunchConfiguration('body_frame_id')
    image_jacobian_swap_uv = LaunchConfiguration('image_jacobian_swap_uv')
    depth_estimation_mode = LaunchConfiguration('depth_estimation_mode')
    image_jacobian_focal_scale = LaunchConfiguration('image_jacobian_focal_scale')

    declare_launch_ibvs_arg = DeclareLaunchArgument(
        'launch_ibvs',
        default_value='true',
        description='Whether to start the IBVS visual servo controller node',
    )

    declare_launch_foxglove_arg = DeclareLaunchArgument(
        'launch_foxglove',
        default_value='true',
        description='Whether to start the Foxglove Bridge node',
    )

    declare_launch_target_point_arg = DeclareLaunchArgument(
        'launch_target_point',
        default_value='false',
        description='Compatibility switch for the old target_point path; tracker/selector is the primary path',
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use ROS simulation time, for example when replaying rosbag2 with /clock',
    )

    declare_launch_detection_stack_arg = DeclareLaunchArgument(
        'launch_detection_stack',
        default_value='auto',
        description='auto: include OAK-D detection stack only when all required packages are available; false: launch observer/controller only; true: require detection stack and fail if unavailable',
    )

    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=_default_config_file(),
        description='YAML parameter file for the visual-servoing bringup nodes',
    )

    # Centroid Kalman filter overrides -- exposed here (defaults matching the
    # node's own declared defaults, so omitting them is a no-op) so bag
    # replay / KF validation doesn't need a separate `ros2 run ... -p ...`
    # invocation alongside this launch file. See
    # visual_servo_controller_node_implementation.py's own declare_parameter
    # calls for what these do.
    declare_centroid_kf_enable_arg = DeclareLaunchArgument(
        'centroid_kf_enable',
        default_value='false',
        description='Enable the image-Jacobian body-velocity-fused centroid Kalman filter',
    )

    declare_camera_optical_frame_id_arg = DeclareLaunchArgument(
        'camera_optical_frame_id',
        default_value='oak_rgb_camera_optical_frame',
        description='TF frame the centroid KF looks up the body->camera extrinsic against',
    )

    declare_body_frame_id_arg = DeclareLaunchArgument(
        'body_frame_id',
        default_value='base_link',
        description='TF frame used as the body side of the body->camera extrinsic lookup',
    )

    declare_image_jacobian_swap_uv_arg = DeclareLaunchArgument(
        'image_jacobian_swap_uv',
        default_value='false',
        description='Swap u/v before evaluating the image Jacobian, for detectors that report axes rotated from the standard pinhole convention',
    )

    declare_depth_estimation_mode_arg = DeclareLaunchArgument(
        'depth_estimation_mode',
        default_value='DVL',
        description='Staged-PID depth source: MAVROS (BlueROV only), DVL (real SC2 field tests), RGBD, or SC2_ALTITUDE (sim only, reads /sc2/altitude directly)',
    )

    declare_image_jacobian_focal_scale_arg = DeclareLaunchArgument(
        'image_jacobian_focal_scale',
        default_value='1.3',
        description='Empirical scale on the image Jacobian\'s fx/fy (e.g. a refraction correction); 1.0 is a no-op',
    )

    # Phase 2 staged_pid notes:
    # - This launch keeps the grapple IF and only switches the controller logic
    #   to staged PID.
    # - `valid_target_observation` / `target_tracked` mean ROI-consistent
    #   tracking continuity, not centered/aligned lock or vertical-ready.
    # - Horizontal x/y/yaw follows the Phase 1 staged PID behavior.
    # - Tracker/selector TargetObservation is the primary horizontal contract.
    # - target_point remains a compatibility path, disabled by default here.
    # - Staged PID vertical source is DVL for the P1-P4 tracker/selector path.
    # - Search/recovery and terminal hold-zero are intentionally not launched.
    # - The controller still publishes through the grapple command adapter:
    #   TwistStamped plus Vector6 on `cmd_topic_vec_6`.
    target_pose_node = Node(
        package='sc2_visual_servo_perception',
        executable='target_point_estimator_node',
        name='target_point_estimator_node',
        output='screen',
        condition=IfCondition(launch_target_point_bool),
        parameters=[config_file, {'use_sim_time': use_sim_time}],
    )

    target_observer_node = Node(
        package='sc2_visual_servo_perception',
        executable='target_observer_node',
        name='target_observer_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
    )

    ibvs_node = Node(
        package='sc2_visual_servo_controller',
        executable='visual_servo_controller_node',
        name='visual_servo_controller_node',
        output='screen',
        condition=IfCondition(launch_ibvs_bool),
        parameters=[config_file, {
            'use_sim_time': use_sim_time,
            'centroid_kf_enable': centroid_kf_enable,
            'camera_optical_frame_id': camera_optical_frame_id,
            'body_frame_id': body_frame_id,
            'image_jacobian_swap_uv': image_jacobian_swap_uv,
            'depth_estimation_mode': depth_estimation_mode,
            'image_jacobian_focal_scale': image_jacobian_focal_scale,
        }],
    )

    return LaunchDescription([
        declare_launch_ibvs_arg,
        declare_launch_foxglove_arg,
        declare_launch_target_point_arg,
        declare_use_sim_time_arg,
        declare_launch_detection_stack_arg,
        declare_config_file_arg,
        declare_centroid_kf_enable_arg,
        declare_camera_optical_frame_id_arg,
        declare_body_frame_id_arg,
        declare_image_jacobian_swap_uv_arg,
        declare_depth_estimation_mode_arg,
        OpaqueFunction(function=_optional_detection_stack),
        # target_observer_node,
        target_pose_node,
        ibvs_node,
    ])
