import yaml
from typing import Any, List, Tuple
import os

from isaac_ros_launch_utils.all_types import (
    Action,
    ComposableNode,
    ComposableNodeContainer,
    LaunchDescription,
    LoadComposableNodes
)
import isaac_ros_launch_utils as lu
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_ros.actions import Node
import launch_ros.descriptions

COMMON_CONFIG = 'common_config'
NVBLOX_CONFIG = 'nvblox_config'

# the output of the OAK-D camera is BGR, and as far as I can tell there is no way to change this in the camera config, so we transform the data to rgb manually, since this is what nvBlox requires
def convert_bgr_to_rgb():
    return LaunchDescription([
        Node(
            package='oakd_isaac_ros',           # Name of your package
            executable='image_converter',       # The name of your node
            name='image_converter_node',        # The name of the node
            output='screen',                    # Output to the screen
            remappings=[                       # Optional: remap topics if necessary
                ('/oak/rgb/image_raw', '/oak/rgb/image_raw'),
                ('/oak/rgb/image_raw_rgb8', '/oak/rgb/image_raw_rgb8')
            ]
        )
    ]) 

# extract the parameters from the top_level config file and read in the base_level config
def get_nvblox_params(num_cameras: int, nvblox_config: dict, common_config: dict) -> List[Any]:

    # initiate a list to store the parameters
    parameters = []
    # first check the additional config files
    for config_file in nvblox_config.get('config_files', []):
        assert 'path' in config_file, 'No `path` provided in config_files'
        
        print("### NVBLOX BASE LEVEL CONFIG: " + str(config_file) + " ###" )

        parameters.append(lu.get_path(config_file.get('package', 'oakd_isaac_ros'),
                                      config_file.get('path')))

    # this is the moment where the overriden parameters are taken from
    parameters.extend(nvblox_config.get('parameters', []))
    parameters.append({'num_cameras': num_cameras})

    if 'robot_frame' in common_config:
        parameters.append({'map_clearing_frame_id': common_config.get('robot_frame')})
        parameters.append({'esdf_slice_bounds_visualization_attachment_frame_id':
                           common_config.get('robot_frame')})
        parameters.append({'workspace_height_bounds_visualization_attachment_frame_id':
                           common_config.get('robot_frame')})
    if 'odom_frame' in common_config:
        parameters.append({'global_frame': common_config.get('odom_frame')})

    return parameters

# next thing is to get the nvblox remappings
def get_nvblox_remappings(cameras: dict) -> List[Tuple[str, str]]:
    remappings = []

    for i in range(len(cameras)):
        depth = cameras[i].get('depth')
        color = cameras[i].get('color')
        mask = cameras[i].get('mask')
        remappings.append((f'camera_{i}/depth/image', depth.get('image')))
        remappings.append((f'camera_{i}/depth/camera_info', depth.get('info')))
        remappings.append((f'camera_{i}/color/image', color.get('image')))
        remappings.append((f'camera_{i}/color/camera_info', color.get('info')))
        remappings.append((f'camera_{i}/mask/image', mask.get('image')))
        remappings.append((f'camera_{i}/mask/camera_info', mask.get('info')))
    return remappings

def start_segmentation():
    segmentation_launch = os.path.join(
        get_package_share_directory('nvblox_examples_bringup'), 'launch', 'perception', 'detection.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(segmentation_launch),
            launch_arguments={
            'input_topic_list': '["/oak/rgb/image_raw_rgb8"]',
            'input_camera_info_topic_list': '["/oak/rgb/camera_info"]',
        }.items()
        ),
    ])

# start the nvBLOX pipeline
# the argument container is passed and also the nvblox and common config values are passsed as dictionaries
def start_nvblox(args: lu.ArgumentContainer, nvblox_config: dict,
                 common_config: dict) -> List[Action]:

    # check which camera is used and check that there is at least one camera
    cameras = nvblox_config.get('remappings', [])
    num_cameras = len(cameras)
    assert num_cameras != 0, 'You need to provide at least one input channel'
    assert num_cameras <= 4, 'No more than 4 input channels must be provided'

    # now we run another function to open the nvBLOX params
    parameters = get_nvblox_params(num_cameras, nvblox_config, common_config)

    ## set the voxel size dynamically on launch
    voxel_size = args.voxel_size
    parameters.append({'voxel_size': voxel_size})

    remappings = get_nvblox_remappings(cameras)

    nvblox_node = ComposableNode(
        name=nvblox_config.get('node_name', 'nvblox_node'),
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters,
    )

    actions = []

    container_name = nvblox_config.get('container_name', 'nvblox_container')

    if nvblox_config.get('attach_to_container', False):
        actions.append(
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
                    nvblox_node
                ]
            ))
    else:
        actions.append(
            ComposableNodeContainer(
                name=container_name,
                package='rclcpp_components',
                namespace='',
                executable='component_container_mt',
                arguments=['--ros-args', '--log-level', args.log_level],
                composable_node_descriptions=[
                    nvblox_node,
                ],
            ))
        
    return actions

# generate frame transforms in order to place the 3D model and the camera at the right orientation
def generate_static_transforms():
    return LaunchDescription([
        Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"]),

        Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "odom", "oak-d-base-frame"]),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "oak-d-base-frame", "oak_rgb_camera_optical_frame"],
        ),
    ])

# here we define the implicit launch description
def generate_launch_description_impl(args: lu.ArgumentContainer) -> List[Action]:
    nvblox_config_file = lu.get_path('oakd_isaac_ros', args.nvblox_config_file)
    oakd_config_file_path = lu.get_path('oakd_isaac_ros', args.oakd_config_file)

    with open(nvblox_config_file, "r", encoding="utf-8") as file:
        nvblox_config = yaml.safe_load(file)

    actions = []

    # extract common config and the nvblox config from the config file and launch nvBLOX
    common_config = nvblox_config.get(COMMON_CONFIG, {})
    if NVBLOX_CONFIG in nvblox_config and lu.is_false(args.disable_nvblox):
        nvblox_config = nvblox_config.get(NVBLOX_CONFIG)
        actions.extend(start_nvblox(args, nvblox_config, common_config))

    # launch the visualization pipeline
    actions.append(
        lu.include(
            'oakd_isaac_ros',
            'launch/tools/visualization.launch.py',
            launch_arguments={'use_foxglove_whitelist': args.use_foxglove_whitelist},
        ))
    

    # append the launch file for the OAK-D
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('oakd_isaac_ros'),
                    'launch/tools/camera.launch.py'
                )
            ),
            launch_arguments={
                'params_file': str(oakd_config_file_path),
                # Add other arguments as needed
            }.items()
        )
    )

    actions.append(convert_bgr_to_rgb())

    actions.append(start_segmentation())

    actions.append(generate_static_transforms())

    return actions

# this function is the top-level functionality where we delcase the launch files and config files to be used in our setup
def generate_launch_description() -> LaunchDescription:
    package_name = 'oakd_isaac_ros'  # Change this to match your package name

    # get the nvBLOX config file
    nvblox_default_config = os.path.join(
        str(get_package_share_directory(package_name)),
        'config',
        'nvblox_segmentation.yaml'
    )

    # get the OAK-D config file
    oak_d_default_config = os.path.join(
        str(get_package_share_directory(package_name)),
        'config',
        'oak_d',
        'oakd_rgbd.yaml'
    )

    # Define arguments
    args = lu.ArgumentContainer()
    args.add_arg('disable_nvblox', False, description='Disable nvblox', cli=True)
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)

    # use Foxglove whitelist or show all topics?
    args.add_arg('use_foxglove_whitelist', False, cli=True)
    # Use the dynamically found config paths instead of hardcoding them
    args.add_arg('nvblox_config_file', str(nvblox_default_config),
                 description='Path to the config file', cli=True)
    args.add_arg('oakd_config_file', str(oak_d_default_config),
                 description='Path to the config file', cli=True)
    # set a default voxel_size but can be changed during launch
    args.add_arg('voxel_size', 0.05, description='Set nvBlox Voxel Size', cli=True)

    args.add_opaque_function(generate_launch_description_impl)

    return LaunchDescription(args.get_launch_actions())

