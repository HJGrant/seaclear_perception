import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


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

    # --- launch args, defaulted from config/system_params.yaml -------------
    declared_arguments = [
        DeclareLaunchArgument(
            'target_class_id',
            default_value='3',
            description='Detection class id to track (also gates nvblox masking)',
        ),
        DeclareLaunchArgument(
            'nvblox_mode',
            default_value=params['nvblox']['mode'],
            description='nvblox multi_mapper mode (mask-gated reconstruction)',
        ),
        DeclareLaunchArgument(
            'voxel_size',
            default_value='0.05',
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
    ]

    target_class_id = LaunchConfiguration('target_class_id')
    nvblox_mode = LaunchConfiguration('nvblox_mode')
    voxel_size = LaunchConfiguration('voxel_size')
    enable_object_detection = LaunchConfiguration('enable_object_detection')
    enable_nvblox = LaunchConfiguration('enable_nvblox')

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
    detection_container = ComposableNodeContainer(
        name='detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[camera_node],
        output='screen'
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
                    ('image_raw', '/oak/rgb/image_raw'),
                    ('image', '/oak/rgb/image_raw_rgb8')
                ],
                # NOT intra-process-enabled: NITROS's type-negotiation uses
                # TRANSIENT_LOCAL durability internally, which throws at
                # construction if the node has use_intra_process_comms set.
            )
        ],
    )

    # --- object detection (optional) -----------------------------------------
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
        }.items(),
        condition=IfCondition(enable_object_detection),
    )

    # --- nvblox (optional) -----------------------------------------------------
    # Minimal static TF chain nvblox needs to integrate depth into a global
    # frame. Cheap enough to always run regardless of enable_nvblox.
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    static_tf_odom_to_oak_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_oak_base_static_tf',
        arguments=['0', '0', '1.0', '0', '3.14', '1.5708', 'odom', 'oak-d-base-frame'],
    )
    static_tf_oak_base_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oak_base_to_optical_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'oak-d-base-frame', 'oak_rgb_camera_optical_frame'],
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
                'rgb', '/oak/rgb/image_raw', rtp_cfg['rgb']['port'],
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
                    'depth_topic': '/oak/stereo/image_raw',
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
            image_converter,
            detection_include,
            static_tf_map_to_odom,
            static_tf_odom_to_oak_base,
            static_tf_oak_base_to_optical,
            nvblox_include,
        ] + rtp_actions
    )
