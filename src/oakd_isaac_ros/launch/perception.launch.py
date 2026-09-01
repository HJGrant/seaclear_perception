import math
import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('oakd_isaac_ros')

    # Python launch files aren't a package, so pull in the shared RTP helper
    # by path rather than a normal import.
    tools_dir = os.path.join(package_share, 'launch', 'tools')
    if tools_dir not in sys.path:
        sys.path.insert(0, tools_dir)
    from rtp_streams import rtp_composable_nodes

    with open(os.path.join(package_share, 'config', 'system_params.yaml')) as f:
        params = yaml.safe_load(f)

    # Same directory detection.launch.py's own model_file_path/engine_file_path
    # defaults resolve against -- kept in sync here since object_detection_model
    # below is filename-only (see system_params.yaml's comment on it).
    detector_model_dir = os.path.join(
        os.environ.get('ISAAC_ROS_WS', '/workspaces/oakd_isaac_ros'),
        'isaac_ros_assets', 'models', 'seaclear')

    # --- launch args, defaulted from config/system_params.yaml -------------
    declared_arguments = [
        DeclareLaunchArgument(
            'target_class_id',
            default_value='3',
            description='Detection class id to track (also gates nvblox masking)',
        ),
        DeclareLaunchArgument(
            'object_detection_model',
            default_value=params['perception']['object_detection_model'],
            description='Detector .onnx filename, must exist in '
                        f'{detector_model_dir}/ (engine_file_path is derived '
                        'by swapping the extension to .plan)',
        ),
        DeclareLaunchArgument(
            'nvblox_mode',
            default_value=params['nvblox']['mode'],
            description='nvblox multi_mapper mode (mask-gated reconstruction)',
        ),
        DeclareLaunchArgument(
            'voxel_size',
            default_value=str(params['nvblox']['voxel_size']),
            description='nvblox TSDF voxel size, in meters',
        ),
        DeclareLaunchArgument(
            'enable_object_detection',
            default_value=str(params['perception']['enable_object_detection']),
        ),
        DeclareLaunchArgument(
            'enable_nvblox',
            default_value=str(params['perception']['enable_nvblox']),
        ),
        DeclareLaunchArgument(
            'enable_multi_frame_mapping',
            default_value=str(params['nvblox']['multi_frame_mapping']['enabled']),
            description='Fuse depth over multiple re-anchored vehicle poses '
                        '(see map_frame_anchor) instead of one fixed frame',
        ),
        DeclareLaunchArgument(
            'reset_every_n_frames',
            default_value=str(params['nvblox']['multi_frame_mapping']['reset_every_n_frames']),
            description='How many depth frames between multi-frame mapping resets',
        ),
        DeclareLaunchArgument(
            'pose_source',
            default_value=params['nvblox']['multi_frame_mapping']['pose_source'],
            description="'state_estimation' (real EKF, must already be running "
                        "externally) or 'oakd_imu' (IMU-only fallback we launch "
                        "ourselves, for debugging with just the OAK-D connected)",
        ),
        DeclareLaunchArgument(
            'enable_camera',
            default_value=str(params['camera']['enable_camera']),
            description='Run the real depthai_ros_driver camera node. Turn off '
                        'for simulation testing -- something else (e.g. a '
                        'simulator bridge) must then publish the topics below.',
        ),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value=params['camera']['rgb_topic'],
        ),
        DeclareLaunchArgument(
            'rgb_camera_info_topic',
            default_value=params['camera']['rgb_camera_info_topic'],
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value=params['camera']['depth_topic'],
        ),
        DeclareLaunchArgument(
            'depth_camera_info_topic',
            default_value=params['camera']['depth_camera_info_topic'],
        ),
    ]

    target_class_id = LaunchConfiguration('target_class_id')
    object_detection_model = LaunchConfiguration('object_detection_model')
    nvblox_mode = LaunchConfiguration('nvblox_mode')
    voxel_size = LaunchConfiguration('voxel_size')
    enable_object_detection = LaunchConfiguration('enable_object_detection')
    enable_nvblox = LaunchConfiguration('enable_nvblox')
    enable_multi_frame_mapping = LaunchConfiguration('enable_multi_frame_mapping')
    reset_every_n_frames = LaunchConfiguration('reset_every_n_frames')
    pose_source = LaunchConfiguration('pose_source')
    enable_camera = LaunchConfiguration('enable_camera')
    rgb_topic = LaunchConfiguration('rgb_topic')
    rgb_camera_info_topic = LaunchConfiguration('rgb_camera_info_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    depth_camera_info_topic = LaunchConfiguration('depth_camera_info_topic')

    # perception.launch.py's own map->odom->oak-d-base-frame chain below is
    # only correct when nothing else owns those frames. That's true in
    # today's single-frame default (nothing else runs), and in the
    # multi-frame "oakd_imu" debug mode (imu_pose_estimate.launch.py only
    # publishes odom -> sc2_base_link, not the map or camera extrinsic). It
    # is NOT true in multi-frame "state_estimation" mode -- the real vehicle
    # stack (ekf_fusing.launch.py) is assumed already running and owns all
    # of those transforms; publishing them here too would create the same
    # two-publishers-one-child-frame conflict we just fixed there.
    own_camera_tf_condition = PythonExpression([
        "'", enable_multi_frame_mapping, "' == 'False' or '", pose_source, "' == 'oakd_imu'"
    ])

    # --- camera --------------------------------------------------------------
    oak_params_file = os.path.join(
        package_share, 'config', 'oak_d', 'oakd_dnn_depth-bremen.yaml'
    )

    camera_node = ComposableNode(
        package='depthai_ros_driver',
        plugin='depthai_ros_driver::Camera',
        name='oak',
        parameters=[oak_params_file, {
            'rgb.i_fps': float(params['camera']['rgb_fps']),
            'left.i_fps': float(params['camera']['depth_fps']),
            'right.i_fps': float(params['camera']['depth_fps']),
        }],
        remappings=[
            ('/oak/imu/data', '/sc2/imu/imu'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # Camera + detection + nvblox + RTP all share this one container so
    # NITROS can negotiate zero-copy transport wherever both ends are
    # NITROS-typed, and so rclcpp intra-process comms can do the same for
    # the plain-message hops (notably the RTP video_output nodes below).
    # Starts empty -- camera_node loads conditionally below like everything
    # else, so enable_camera:=false skips it cleanly for simulation testing.
    detection_container = ComposableNodeContainer(
        name='detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[],
        output='screen'
    )

    camera_include = LoadComposableNodes(
        condition=IfCondition(enable_camera),
        target_container='detection_container',
        composable_node_descriptions=[camera_node],
    )

    # image_converter only feeds the detection pipeline's input topic --
    # skip it entirely when detection is off.
    image_converter = LoadComposableNodes(
        condition=IfCondition(enable_object_detection),
        target_container='detection_container',
        composable_node_descriptions=[
            ComposableNode(
                name='oak_format_conv_node',
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                parameters=[{
                    'image_width': 1280,
                    'image_height': 720,
                    'encoding_desired': 'rgb8'
                }],
                remappings=[
                    ('image_raw', rgb_topic),
                    ('image', '/oak/rgb/image_raw_rgb8')
                ],
                # NOT intra-process-enabled: NITROS's type-negotiation uses
                # TRANSIENT_LOCAL durability internally, which throws at
                # construction if the node has use_intra_process_comms set.
            )
        ],
    )

    # --- object detection (optional) -----------------------------------------
    # engine_file_path is the same model, just with the extension swapped --
    # TensorRT (re)builds it from the .onnx automatically if it's missing or
    # stale, so only the .onnx needs to actually exist up front.
    detector_model_file_path = PathJoinSubstitution([detector_model_dir, object_detection_model])
    detector_engine_file_path = PathJoinSubstitution([
        detector_model_dir,
        PythonExpression(["'", object_detection_model, "'.replace('.onnx', '.plan')"]),
    ])

    detection_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'tools', 'detection.launch.py')
        ),
        launch_arguments={
            'num_cameras': '1',
            'input_topic_list': '["/oak/rgb/image_raw_rgb8"]',
            'one_container_per_camera': 'False',
            'container_name': 'detection_container',
            'verbose': 'True',
            'target_class_id': target_class_id,
            'model_file_path': detector_model_file_path,
            'engine_file_path': detector_engine_file_path,
        }.items(),
        condition=IfCondition(enable_object_detection),
    )

    # --- nvblox (optional) -----------------------------------------------------
    # Minimal static TF chain nvblox needs to integrate depth into a global
    # frame. Cheap enough to always run regardless of enable_nvblox -- but
    # see own_camera_tf_condition above for when these get skipped in favor
    # of an externally-running state_estimation stack.
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(own_camera_tf_condition),
    )
    # Only published in the plain single-frame default -- in "oakd_imu"
    # multi-frame mode, imu_pose_estimate.launch.py publishes this live
    # instead (publishing it here too would conflict, same as the
    # ekf_fusing.launch.py bug we fixed).
    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'sc2_base_link'],
        condition=UnlessCondition(enable_multi_frame_mapping),
    )
    # Camera's static extrinsic, parented to sc2_base_link (not odom
    # directly) so a live odom -> sc2_base_link (from either pose source)
    # actually moves the camera -- same reasoning as ekf_fusing.launch.py's
    # equivalent transform. Values match the transform_tree package's
    # odom -> oak-d-base-frame transform (0, 0.115, 0 m; roll=0, pitch=pi,
    # yaw=pi) -- that package's odom/base_link/sc2_base_link chain is all
    # identity, so the numbers carry over directly onto sc2_base_link here.
    static_tf_base_to_oak_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_oak_base_static_tf',
        arguments=[
            '--x', '0.0', '--y', '0.115', '--z', '0.0',
            '--roll', '0.0', '--pitch', str(math.pi), '--yaw', str(math.pi),
            '--frame-id', 'sc2_base_link', '--child-frame-id', 'oak-d-base-frame',
        ],
        condition=IfCondition(own_camera_tf_condition),
    )
    static_tf_oak_base_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oak_base_to_optical_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'oak-d-base-frame', 'oak_rgb_camera_optical_frame'],
        condition=IfCondition(own_camera_tf_condition),
    )
    # IMU extrinsic -- not needed by anything until imu_pose_estimate's
    # ekf_node started requiring a TF path from oak_imu_frame (the IMU
    # message's frame_id) to sc2_base_link to rotate measurements into the
    # base frame before fusing them. transform_tree parents sc2_imu_link off
    # oak-d-base-frame (not sc2_base_link directly), and that parent is not
    # coincident with sc2_base_link (see static_tf_base_to_oak_base above),
    # so this must chain off oak-d-base-frame too rather than reuse the same
    # roll/pitch/yaw numbers against a different parent.
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_static_tf',
        arguments=['0', '0', '0', str(1.5707963267948966), '0', '0', 'oak-d-base-frame', 'sc2_imu_link'],
        condition=IfCondition(own_camera_tf_condition),
    )
    static_tf_imu_to_oak_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_oak_imu_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'sc2_imu_link', 'oak_imu_frame'],
        condition=IfCondition(own_camera_tf_condition),
    )

    # --- multi-frame mapping pose source (optional) ---------------------------
    imu_pose_estimate_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'tools', 'imu_pose_estimate.launch.py')
        ),
        condition=IfCondition(PythonExpression([
            "'", enable_multi_frame_mapping, "' == 'True' and '", pose_source, "' == 'oakd_imu'"
        ])),
    )

    # --- multi-frame mapping map anchor (optional) -----------------------------
    # Doesn't care which pose source is active -- just needs a live
    # odom -> sc2_base_link, from whichever of the above is providing it.
    map_frame_anchor_node = LoadComposableNodes(
        condition=IfCondition(enable_multi_frame_mapping),
        target_container='detection_container',
        composable_node_descriptions=[
            ComposableNode(
                name='map_frame_anchor',
                package='oakd_isaac_ros',
                plugin='map_frame_anchor',
                parameters=[{
                    'parent_frame': 'odom',
                    'base_frame': 'sc2_base_link',
                    'map_frame': 'tsdf_map_origin',
                    'trigger_topic': depth_topic,
                    'reset_every_n_frames': ParameterValue(reset_every_n_frames, value_type=int),
                    'clear_map_service': '/nvblox_node/clear_map',
                }],
            )
        ],
    )

    # NOTE: nvblox_mode "people_detection" needs a live /camera0/detection/mask
    # from the detection pipeline to reconstruct anything via the masked
    # path -- if you disable object detection while leaving this mode set,
    # nvblox will come up fine but won't integrate depth into the static map.
    # Set nvblox_mode to "static" if you want nvblox without detection.
    nvblox_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'tools', 'nvblox.launch.py')
        ),
        launch_arguments={
            'container_name': 'detection_container',
            'mode': nvblox_mode,
            'camera': 'oakd',
            'num_cameras': '1',
            'voxel_size': voxel_size,
            # Only override nvblox_base.yaml's default ("odom") in
            # multi-frame mode -- map_frame_anchor re-anchors
            # tsdf_map_origin periodically; single-frame mode fuses
            # straight into the (static, in that mode) odom frame.
            'global_frame': PythonExpression([
                "'tsdf_map_origin' if '", enable_multi_frame_mapping, "' == 'True' else ''"
            ]),
            'depth_topic': depth_topic,
            'depth_camera_info_topic': depth_camera_info_topic,
            'rgb_topic': rgb_topic,
            'rgb_camera_info_topic': rgb_camera_info_topic,
        }.items(),
        condition=IfCondition(enable_nvblox),
    )

    # --- RTP streaming (optional, independent of detection/nvblox) -----------
    rtp_cfg = params['rtp']
    rtp_actions = []

    if rtp_cfg['enable']:
        destination_ips = rtp_cfg['destination_ips']
        headless = rtp_cfg['headless']

        rgb_nodes = []
        if rtp_cfg['rgb']['enabled']:
            # No host-side resize needed -- oakd_dnn_depth-bremen.yaml
            # configures the camera's own ISP to output natively at
            # rtp.rgb.width x height (1280x720 by default) via i_isp_num/
            # i_isp_den, so /oak/rgb/image_raw is already the right size.
            # (Previously this ran a GPU ResizeNode here, which shared the
            # GPU with TensorRT and made RGB RTP latency track detection
            # load -- see the comment in that config file.)
            #
            # Streams straight off the raw camera topic (not the detection
            # pipeline's converted copy) so RGB RTP works independently of
            # enable_object_detection.
            rgb_nodes = rtp_composable_nodes(
                'rgb', rgb_topic, rtp_cfg['rgb']['port'],
                destination_ips, headless,
            )

        mask_nodes = []
        if rtp_cfg['mask_overlay']['enabled']:
            # Only actually publishes once object detection is running --
            # /mask_overlay comes from publish_mask_nitros.cpp.
            mask_nodes = rtp_composable_nodes(
                'mask', '/mask_overlay', rtp_cfg['mask_overlay']['port'],
                destination_ips, headless,
            )

        if rgb_nodes or mask_nodes:
            rtp_actions.append(LoadComposableNodes(
                target_container='detection_container',
                composable_node_descriptions=rgb_nodes + mask_nodes,
            ))

        if rtp_cfg['depth']['enabled']:
            depth_colorizer = ComposableNode(
                package='oakd_isaac_ros',
                plugin='depth_colorizer',
                name='depth_colorizer',
                parameters=[{
                    'depth_topic': depth_topic,
                    'output_topic': '/oak/stereo/depth_color',
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
            depth_video_nodes = rtp_composable_nodes(
                'depth', '/oak/stereo/depth_color', rtp_cfg['depth']['port'],
                destination_ips, headless,
            )
            rtp_actions.append(LoadComposableNodes(
                target_container='detection_container',
                composable_node_descriptions=[depth_colorizer] + depth_video_nodes,
            ))

    return LaunchDescription(
        declared_arguments + [
            detection_container,
            camera_include,
            image_converter,
            detection_include,
            static_tf_map_to_odom,
            static_tf_odom_to_base,
            static_tf_base_to_oak_base,
            static_tf_oak_base_to_optical,
            static_tf_base_to_imu,
            static_tf_imu_to_oak_imu,
            imu_pose_estimate_include,
            nvblox_include,
            map_frame_anchor_node,
        ] + rtp_actions
    )
