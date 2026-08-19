# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from typing import List, Tuple

from launch import Action, LaunchDescription
from launch_ros.descriptions import ComposableNode
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def get_oakd_remappings(
        mode: NvbloxMode, depth_topic: str, depth_camera_info_topic: str,
        rgb_camera_info_topic: str) -> List[Tuple[str, str]]:
    remappings = []
    remappings.append((f'camera_0/depth/image', depth_topic))
    remappings.append((f'camera_0/depth/camera_info', depth_camera_info_topic))

    if mode is NvbloxMode.people_segmentation:
            # nvblox takes resized images from semseg inputs
            remappings.append(
                (f'camera_0/color/image', f'/camera0/segmentation/image_resized'))
            remappings.append(
                (f'camera_0/color/camera_info', f'/camera0/segmentation/camera_info_resized'))
            remappings.append((f'camera_0/mask/image', f'/camera0/segmentation/people_mask'))
            remappings.append(
                (f'camera_0/mask/camera_info', f'/camera0/segmentation/camera_info_resized'))

    else:
        # /oak/rgb/image_raw_rgb8 is the detection pipeline's own internal
        # converted copy (see perception.launch.py's oak_format_conv_node),
        # not a raw camera topic -- not part of the configurable set.
        remappings.append((f'camera_0/color/image', f'/oak/rgb/image_raw_rgb8'))
        remappings.append((f'camera_0/color/camera_info', rgb_camera_info_topic))

        if mode is NvbloxMode.people_detection:
            remappings.append((f'camera_0/mask/image', f'/camera0/detection/mask_resized'))
            remappings.append((f'camera_0/mask/camera_info', f'/camera0/detection/mask_resized/camera_info'))

    return remappings

def add_nvblox(args: lu.ArgumentContainer) -> List[Action]:
    package_name = 'oakd_isaac_ros'

    mode = NvbloxMode[args.mode]
    camera = args.camera
    num_cameras = int(args.num_cameras)
    use_lidar = lu.is_true(args.lidar)

    base_config = lu.get_path(package_name, 'config/nvblox/nvblox_base.yaml')
    segmentation_config = lu.get_path(package_name,
                                      'config/nvblox/specializations/nvblox_segmentation.yaml')
    detection_config = lu.get_path(package_name,
                                   'config/nvblox/specializations/nvblox_detection.yaml')
    dynamics_config = lu.get_path(package_name,
                                  'config/nvblox/specializations/nvblox_dynamics.yaml')
    oakd_config = lu.get_path(package_name,
                                   'config/nvblox/specializations/nvblox_oakd.yaml')

    # select the reconstruction mode
    if mode is NvbloxMode.static:
        mode_config = {}
    elif mode is NvbloxMode.people_segmentation:
        mode_config = segmentation_config
        assert not use_lidar, 'Can not run lidar with people segmentation mode.'
    elif mode is NvbloxMode.people_detection:
        mode_config = detection_config
        assert not use_lidar, 'Can not run lidar with people detection mode.'
    elif mode is NvbloxMode.dynamic:
        mode_config = dynamics_config
        assert not use_lidar, 'Can not run lidar with dynamic mode.'
    else:
        raise Exception(f'Mode {mode} not implemented for nvblox.')

    # select the camera
    if camera == "oakd":
        print("##### Setting Camera to nvBLOX!! #####")
        remappings = get_oakd_remappings(
            mode, args.depth_topic, args.depth_camera_info_topic, args.rgb_camera_info_topic)
        camera_config = oakd_config
        assert num_cameras == 1, 'OAK-D example can only run with 1 camera.'
        assert not use_lidar, 'Can not run lidar for OAK-D example.'
    else:
        raise Exception(f'Camera {camera} not implemented for nvblox.')

    parameters = []
    parameters.append(base_config)
    parameters.append(mode_config)
    parameters.append(camera_config)
    parameters.append({'num_cameras': num_cameras})
    parameters.append({'use_lidar': use_lidar})
    parameters.append({'voxel_size': args.voxel_size})
    # Overrides nvblox_base.yaml's global_frame (default "odom") -- used by
    # multi-frame mapping to fuse into the periodically re-anchored
    # tsdf_map_origin frame instead of the continuously-drifting live
    # estimate. Empty string (the default here) means "leave it alone".
    if args.global_frame:
        parameters.append({'global_frame': args.global_frame})

    # Add the nvblox node.
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters,
    )

    actions = [] 
    if args.run_standalone:
        actions.append(lu.component_container(args.container_name))
    actions.append(lu.load_composable_nodes(args.container_name, [nvblox_node]))

    print("Remappings:", remappings)

    actions.append(
        lu.log_info(
            ["Starting nvblox with the '",
             str(camera), "' camera in '",
             str(mode), "' mode."]))
    
    return actions

def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('mode')
    args.add_arg('camera')
    args.add_arg('num_cameras', 1)
    args.add_arg('lidar', 'False')
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')
    args.add_arg('voxel_size', 0.05)
    args.add_arg('global_frame', '')
    # Defaults match the real depthai_ros_driver's topics -- overridden by
    # perception.launch.py from config/system_params.yaml's camera section
    # when testing against a simulator with different topic names.
    args.add_arg('depth_topic', '/oak/stereo/image_raw')
    args.add_arg('depth_camera_info_topic', '/oak/stereo/camera_info')
    args.add_arg('rgb_topic', '/oak/rgb/image_raw')
    args.add_arg('rgb_camera_info_topic', '/oak/rgb/camera_info')

    args.add_opaque_function(add_nvblox)
    return LaunchDescription(args.get_launch_actions())