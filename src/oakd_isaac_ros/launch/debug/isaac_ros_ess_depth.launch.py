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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    camera = LaunchConfiguration('camera')
    camera_arg = DeclareLaunchArgument(
        'camera',
        description='camera',
    )

    engine_file_path = LaunchConfiguration('engine_file_path')
    engine_file_path_arg = DeclareLaunchArgument(
        'engine_file_path',
        default_value='isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/ess.engine',
        description='The absolute path to the ESS engine plan.')

    threshold = LaunchConfiguration('threshold')
    threshold_arg = DeclareLaunchArgument(
        'threshold',
        default_value='0.0',
        description='Threshold value ranges between 0.0 and 1.0 '
                    'for filtering disparity with confidence.')
    launch_args = [
        camera_arg,
        engine_file_path_arg,
        threshold_arg,
    ]

    container_name = LaunchConfiguration('container_name')
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='detection_container',
        description='Name of the existing container to load nodes into'
    )

    left_image_rect_topic_arg = DeclareLaunchArgument(
        'left_image_topic',
        default_value='/oak/left/image_raw', # Fallback default
        description='Topic for the left rectified image'
    )

    right_image_rect_topic_arg = DeclareLaunchArgument(
        'right_image_topic',
        default_value='/oak/right/image_raw',
        description='Topic for the right rectified image'
    )

    left_camera_info_topic_arg = DeclareLaunchArgument(
        'left_camera_info_topic',
        default_value='/oak/left/camera_info',
        description='Topic for the left camera info.'
    )

    right_camera_info_topic_arg = DeclareLaunchArgument(
        'right_camera_info_topic',
        default_value='/oak/right/camera_info',
        description='Topic for the right camera info.'
    )
    
    # 2. Access the values
    left_image_topic = LaunchConfiguration('left_image_topic')
    right_image_topic = LaunchConfiguration('right_image_topic')
    left_camera_info_topic = LaunchConfiguration('left_camera_info_topic')
    right_camera_info_topic = LaunchConfiguration('right_camera_info_topic')

    # define the image height
    input_width_arg = DeclareLaunchArgument(
        'input_width',
        default_value='1280',
        description='Input width of the stereo pairs.'
    )

    input_height_arg = DeclareLaunchArgument(
        'input_height',
        default_value='800',
        description='Input height of the stereo pairs.'
    )

    stereo_input_width = LaunchConfiguration('input_width')
    stereo_input_height = LaunchConfiguration('input_height')
    

    left_image_topic_rgb8 = PythonExpression(["'/oak/left/image_rect_rgb8'"])
    right_image_topic_rgb8 = PythonExpression(["'/oak/right/image_rect_rgb8'"])


    right_format_conv_node =ComposableNode(
        name='right_format_conv_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        parameters=[{
            'image_width': 1280,
            'image_height': 800,
            'encoding_desired': 'rgb8'
        }],
        remappings=[
            ('image_raw', right_image_topic),
            ('image', right_image_topic_rgb8)
        ]
    )

    left_format_conv_node =ComposableNode(
        name='left_format_conv_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        parameters=[{
            'image_width': 1280,
            'image_height': 800,
            'encoding_desired': 'rgb8'
        }],
        remappings=[
            ('image_raw', left_image_topic),
            ('image', left_image_topic_rgb8)
        ]
    )

    right_rectify_node =ComposableNode(
        name='right_rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'output_width': 1280,
            'output_height': 800,
            'interpolation': 'linear',
        }],
        remappings=[
            ('image_raw', right_image_topic_rgb8),
            ('camera_info', right_camera_info_topic),
            ('image_rect', '/right/rectified'),
            ('camera_info_rect', '/right/rectified/camera_info'),
        ]
    )

    left_rectify_node =ComposableNode(
        name='left_rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'output_width': 1280,
            'output_height': 800,
            'interpolation': 'linear',
        }],
        remappings=[
            ('image_raw', left_image_topic_rgb8),
            ('camera_info', left_camera_info_topic),
            ('image_rect', '/left/rectified'),
            ('camera_info_rect', '/left/rectified/camera_info'),
        ]
    )

    right_resize_node =ComposableNode(
        name='right_resize_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'input_width': 1280,
            'input_height': 800,
            'output_width': 960,
            'output_height': 576,
            'encoding_desired': 'rgb8'
        }],
        remappings=[
            ('image', '/right/rectified'),
            ('camera_info', '/right/rectified/camera_info'),
            ('resize/image', '/right/image_resized'),
            ('resize/camera_info', '/right/camera_info_resized')
        ]
    )

    left_resize_node =ComposableNode(
        name='left_resize_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'input_width': 1280,
            'input_height': 800,
            'output_width': 960,
            'output_height': 576,
            'encoding_desired': 'rgb8'
        }],
        remappings=[
            ('image', '/left/rectified'),
            ('camera_info', '/left/rectified/camera_info'),
            ('resize/image', '/left/image_resized'),
            ('resize/camera_info', '/left/camera_info_resized')
        ]
    )

    disparity_node = ComposableNode(
        name='disparity',
        package='isaac_ros_ess',
        plugin='nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode',
        parameters=[{
            'engine_file_path': engine_file_path,
            'threshold': threshold,
            'input_layer_width': 960,
            'input_layer_height': 576,
            'type_negotiation_duration_s': 5,
            'max_disparity': 256,
        }],
        remappings=[
            ('left/image_rect', '/left/image_resized'),
            ('right/image_rect', '/right/image_resized'),
            ('left/camera_info', '/right/camera_info_resized'), # Fixed cross-talk
            ('right/camera_info', '/left/camera_info_resized'), # Fixed cross-talk
        ]
    )

    disparity_to_depth_node = ComposableNode(
        name='DisparityToDepthNode',
        package='isaac_ros_stereo_image_proc',
        plugin='nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode',
        remappings=[
        ('disparity', '/disparity'), # Check if ESS publishes here
        ('depth', '/depth')          # Check if nvblox listens here
        ]
    )

    dnn_depth_visualizer = ComposableNode(
        package='oakd_isaac_ros',
        plugin='oakd_isaac_ros::DNNDepthVisualizer',
        name='dnn_depth_visualizer',
        remappings=[
            ('disparity', '/disparity'),
            ('depth_color', '/depth_rgb8')
        ],
        parameters=[{'max_disp': 256.0}]
    )

    encoder_node = ComposableNode(
        name='encoder',
        package='isaac_ros_h264_encoder',
        plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
        parameters=[{
                'input_height': 576,
                'input_width': 960,
                'config': 'pframe_cqp',
        }],
        remappings=[
            ('image_raw', '/depth_rgb8'),
            ('image_compressed', '/dnn_depth_color/h264')
        ]
        )

    dnn_depth_video_node = ComposableNode(
        package='oakd_isaac_ros',
        plugin='oakd_isaac_ros::H264ToFoxgloveBridge',
        name='foxglove_bridge_dnn_depth',
        remappings=[
            ('input/h264', '/dnn_depth_color/h264'),
            ('output/foxglove', '/dnn_depth/video')
        ]
    )

    load_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            right_rectify_node,
            left_rectify_node,
            right_format_conv_node,
            left_format_conv_node,
            right_resize_node,
            left_resize_node,
            disparity_node,
            disparity_to_depth_node,
            dnn_depth_visualizer,
            encoder_node,
            dnn_depth_video_node
        ])

    return (LaunchDescription(launch_args + [container_name_arg, load_nodes]))
