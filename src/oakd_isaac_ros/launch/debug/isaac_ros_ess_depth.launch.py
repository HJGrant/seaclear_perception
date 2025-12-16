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
from launch_ros.actions import ComposableNodeContainer
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
        default_value='',
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

    left_image_topic = PythonExpression(["'/' + '", camera, "' + '/left/image_raw/compressed'"])
    left_info_topic = PythonExpression(["'/' + '", camera, "' + '/left/camera_info'"])
    right_image_topic = PythonExpression(["'/' + '", camera, "' + '/right/image_raw/compressed'"])
    right_info_topic = PythonExpression(["'/' + '", camera, "' + '/right/camera_info'"])
    left_rect_image_topic_mono8 = PythonExpression(["'/' + '", camera, "' + '/left/image_rect'"])
    right_rect_image_topic_mono8 = PythonExpression(["'/' + '", camera, "' + '/right/image_rect'"])
    left_rect_image_topic_rgb8 = PythonExpression(["'/' + '", camera, "' + '/left/image_rect_rgb8'"])
    right_rect_image_topic_rgb8 = PythonExpression(["'/' + '", camera, "' + '/right/image_rect_rgb8'"])

    

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
            ('image_raw', right_rect_image_topic_mono8),
            ('image', right_rect_image_topic_rgb8)
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
            ('image_raw', left_rect_image_topic_mono8),
            ('image', left_rect_image_topic_rgb8)
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
            ('image', right_rect_image_topic_rgb8),
            ('camera_info', right_info_topic),
            ('resize/image', 'right/image_resized'),
            ('resize/camera_info', 'right/camera_info_resized')
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
            ('image', left_rect_image_topic_rgb8),
            ('camera_info', left_info_topic),
            ('resize/image', 'left/image_resized'),
            ('resize/camera_info', 'left/camera_info_resized')
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
            ('left/image_rect', 'left/image_resized'),
            ('right/image_rect', 'right/image_resized'),
            ('left/camera_info', 'right/camera_info_resized'),
            ('right/camera_info', 'left/camera_info_resized'),
        ]
    )

    disparity_to_depth_node = ComposableNode(
        name='DisparityToDepthNode',
        package='isaac_ros_stereo_image_proc',
        plugin='nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode',
    )

    depth_format_conversion_node = ComposableNode(
        name='depth_format_conv_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        parameters=[{
            'image_width': 960,
            'image_height': 576,
            'encoding_desired': 'rgb8'
        }],
        remappings=[
            ('image_raw', '/depth'),
            ('image', '/depth_rgb8')
        ]
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
            ('image_compressed', '/dnn_depth_compressed')
        ]
        )

    container = ComposableNodeContainer(
        name='depth_container',
        namespace='depth',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            right_format_conv_node,
            left_format_conv_node,
            right_resize_node,
            left_resize_node,
            disparity_node,
            disparity_to_depth_node,
            #depth_format_conversion_node,
            #encoder_node,
        ],
        output='screen'
    )

    return (LaunchDescription(launch_args + [container]))
