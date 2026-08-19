import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_share = get_package_share_directory('oakd_isaac_ros')

    tools_dir = os.path.join(package_share, 'launch', 'tools')
    if tools_dir not in sys.path:
        sys.path.insert(0, tools_dir)
    from rtp_streams import rtp_composable_nodes

    with open(os.path.join(package_share, 'config', 'system_params.yaml')) as f:
        params = yaml.safe_load(f)

    rgb_fps = params['camera']['rgb_fps']
    depth_fps = params['camera']['depth_fps']
    rtp_cfg = params['rtp']
    destination_ips = rtp_cfg['destination_ips']
    headless = rtp_cfg['headless']

    # Include OAK-D launch. tools/camera.launch.py creates a
    # ComposableNodeContainer named "<name>_container" (default name "oak"
    # -> "oak_container"); everything else below is loaded into that same
    # container so it's in-process with the camera driver and can benefit
    # from rclcpp intra-process comms.
    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'tools', 'camera.launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(
                package_share, 'config', 'oak_d', 'oakd_rgbd.yaml'
            ),
            'rgb_fps': str(rgb_fps),
            'depth_fps': str(depth_fps),
        }.items()
    )

    # CUDA-accelerated NITROS resize (replaces the CPU-bound Python
    # image_resizer.py node -- same isaac_ros_image_proc::ResizeNode used
    # elsewhere in this repo's detection pipeline). depthai_ros_driver
    # itself isn't NITROS-typed, so the camera->resize hop still can't be
    # true zero-copy, but the resize compute itself runs on the GPU instead
    # of CPU/OpenCV.
    resize = LoadComposableNodes(
        target_container='/oak_container',
        composable_node_descriptions=[
            ComposableNode(
                name='oak_image_resize_node_for_rtp',
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                parameters=[{
                    'input_width': 1920,
                    'input_height': 1080,
                    'output_width': rtp_cfg['rgb']['width'],
                    'output_height': rtp_cfg['rgb']['height'],
                    'encoding_desired': 'bgr8',
                }],
                remappings=[
                    ('image', '/oak/rgb/image_raw'),
                    ('camera_info', '/oak/rgb/camera_info'),
                    ('resize/image', '/oak/rgb/image_resized'),
                    ('resize/camera_info', '/oak/rgb/camera_resized_info'),
                ],
                # NOT intra-process-enabled: NITROS's type-negotiation uses
                # TRANSIENT_LOCAL durability internally, which throws at
                # construction if the node has use_intra_process_comms set.
            )
        ],
    )

    actions = [oakd_launch, resize]

    if rtp_cfg['enable'] and rtp_cfg['rgb']['enabled']:
        rgb_nodes = rtp_composable_nodes(
            'rgb', '/oak/rgb/image_resized', rtp_cfg['rgb']['port'],
            destination_ips, headless,
        )
        actions.append(LoadComposableNodes(
            target_container='/oak_container',
            composable_node_descriptions=rgb_nodes,
        ))

    if rtp_cfg['enable'] and rtp_cfg['depth']['enabled']:
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
        actions.append(LoadComposableNodes(
            target_container='/oak_container',
            composable_node_descriptions=[depth_colorizer] + depth_video_nodes,
        ))

    return LaunchDescription(actions)
