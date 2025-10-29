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

from typing import List

from launch import Action, LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import isaac_ros_launch_utils as lu
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource

from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME, \
    PEOPLENET_INPUT_IMAGE_WIDTH, PEOPLENET_INPUT_IMAGE_HEIGHT

def create_detection_pipeline(args: lu.ArgumentContainer) -> List[Action]:

    # DNN Image Encoder parameters
    input_image_width = args.input_image_width
    input_image_height = args.input_image_height
    network_image_width = args.network_image_width
    network_image_height = args.network_image_height
    image_mean = args.image_mean
    image_stddev = args.image_stddev

    # TensorRT parameters
    model_file_path = args.model_file_path
    engine_file_path = args.engine_file_path
    input_tensor_names = args.input_tensor_names
    input_binding_names = args.input_binding_names
    output_tensor_names = args.output_tensor_names
    output_binding_names = args.output_binding_names
    verbose = args.verbose
    force_engine_update = args.force_engine_update

    # YOLOv8 Decoder parameters
    confidence_threshold = args.confidence_threshold
    nms_threshold = args.nms_threshold

    #depth mask parameters
    mask_class_id = args.target_class_id
    #confidence_threshold = arg.confidence_threshold

    #namespace = "camera0"
    #input_topic = "camera0/color/image_raw"
    #input_info_topic = "camera0/color/image_raw/camera_info"
    #input_topic='/oak/rgb/image_raw_rgb8'
    #input_info_topic='/oak/rgb/camera_info'
    input_topic = args.input_topic_list[0]
    input_info_topic = args.input_info_topic_list[0]


    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
    yolov8_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')]
        ),
        launch_arguments={
            'input_image_width': str(input_image_width),
            'input_image_height': str(input_image_height),
            'network_image_width': str(network_image_width),
            'network_image_height': str(network_image_height),
            'image_mean': str(image_mean),
            'image_stddev': str(image_stddev),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'detection_container',
            'dnn_image_encoder_namespace': 'yolov8_encoder',
            'keep_aspect_ratio': 'false',
            'image_input_topic': str(input_topic),
            'camera_info_input_topic': str(input_info_topic),
            'tensor_output_topic': '/tensor_pub',
        }.items(),
    )

    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            'model_file_path': model_file_path,
            'engine_file_path': engine_file_path,
            'output_binding_names': output_binding_names,
            'output_tensor_names': output_tensor_names,
            'input_tensor_names': input_tensor_names,
            'input_binding_names': input_binding_names,
            'verbose': verbose,
            'force_engine_update': force_engine_update
        }]
    )

    yolov8_decoder_node = ComposableNode(
        name='yolov8_decoder_node',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        parameters=[{
            'confidence_threshold': confidence_threshold,
            'nms_threshold': nms_threshold,
            'tensor_name': 'output_tensor',
        }]
    )

    mask_resize_node = ComposableNode(
        name='mask_resize_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        #namespace=namespace,
        extra_arguments=[{'--ros-args': '', '--log-level': 'debug'}],
        parameters=[{
            'input_width': 640,
            'input_height': 640,
            'output_width': 1280,
            'output_height': 720,
            'encoding_desired': 'mono8',
            'keep_aspect_ratio': False,
            'disable_padding': False,
            'num_blocks': 40,
            'input_qos_depth': 10,
            'output_qos_depth': 10,
            'camera_info_qos_depth': 10,
            'image_qos_depth': 10,
        }],
        remappings=[
            ('/image', '/camera0/detection/mask'),
            ('/camera_info', '/camera0/detection/mask/camera_info'), 
            ('/resize/image', '/camera0/detection/mask_resized'),
            ('/resize/camera_info', '/camera0/detection/mask_resized/camera_info'),
        ]
    )

    depth_mask_publisher = ComposableNode(
        name='depth_mask_publisher',
        package='oakd_isaac_ros',
        plugin='publish_mask',
        parameters=[{
            'target_class_id': int(mask_class_id),
            'confidence_threshold' : confidence_threshold
        }],
        extra_arguments=[{
            '--ros-args': '', 
            '--log-level': 'debug'}]
    )

    #detection_node = ComposableNodeContainer(
    #    name='tensor_rt_container',
    #    package='rclcpp_components',
    #    executable='component_container_mt',
    #    composable_node_descriptions=[tensor_rt_node, yolov8_decoder_node, mask_resize_node, depth_mask_publisher],
    #    output='screen',
    #    arguments=['--ros-args', '--log-level', 'INFO'],
    #    namespace=''
    #)

    detection_node = lu.load_composable_nodes(
            'detection_container',
            [
                tensor_rt_node, yolov8_decoder_node, mask_resize_node, depth_mask_publisher
            ],
        )

    return [yolov8_encoder_launch, detection_node]

def add_detection(args: lu.ArgumentContainer) -> List[Action]:
    # Only one camera, so just return the pipeline actions
    return create_detection_pipeline(args)


def generate_launch_description() -> LaunchDescription:
    '''
    Launch the resize node, DNN Image preprocessing node, TRT node and UNet decoder node
    for each camera.
    '''
    args = lu.ArgumentContainer()
    args.add_arg('num_cameras', '1',
                 description='Number of cameras requiring people detection pipeline')
    args.add_arg('namespace_list', '["camera0"]',
                 description='List of namespaces for each detection inference pipeline')
    args.add_arg(
        'input_topic_list',
        '["/oak/rgb/image_raw_rgb8"]',
        description='List of camera image input topics for each detection inference pipeline')

    args.add_arg(
        'input_info_topic_list',
        '["/oak/rgb/camera_info"]',
        description='List of camera image info input topics for each detection inference pipeline')

    args.add_arg('input_image_width', '1280',
                 description='Width of the input image.')
    args.add_arg('input_image_height', '720',
                 description='Height of the input image.')
    args.add_arg('confidence_threshold', '0.6',
                 description='Confidence threshold to filter candidate detections during NMS')
    args.add_arg('nms_threshold', '0.45',
                description='NMS IOU threshold')
    args.add_arg('image_mean', '[0.0, 0.0, 0.0]',
                 description='The mean for image normalization')
    args.add_arg('image_stddev', '[1.0, 1.0, 1.0]',
                 description='The stddev for image normalization')

    # Yolov8 decoder mask parameters
    # mask will be the same resolution as input image
    args.add_arg('network_image_width', '640',
                 description='Number of columns for network input tensor image')
    args.add_arg('network_image_height', '640',
                 description='Number of rows for network input tensor image')
    # TRT Node parameters
    args.add_arg('verbose', 'False',
                 description='TensorRT verbosely log if True')
    args.add_arg('force_engine_update', 'False',
                 description='TensorRT update the TensorRT engine file if True')
    #args.add_arg('model_file_path',
    #             lu.get_isaac_ros_ws_path() +
    #             '/isaac_ros_assets/models/yolov8/yolov8s.onnx',
    #             description='Full path to detection model TRT engine')
    args.add_arg('model_file_path',
                 '/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.onnx',
                 description='Full path to detection model TRT engine')
    args.add_arg('engine_file_path',
                 '/workspaces/oakd_isaac_ros/isaac_ros_assets/models/seaclear/v12s.plan',
                 description='Full path to detection model TRT engine')
    args.add_arg('input_binding_names', '["images"]',
                 description='List of TRT input tensor binding names')
    args.add_arg('input_tensor_names', '["input_tensor"]',
                 description='List of TRT input tensor names')
    args.add_arg('input_tensor_formats', '["nitros_tensor_list_nchw_rgb_f32"]',
                 description='List of TRT input tensor nitros type formats')
    args.add_arg('output_tensor_names', '["output_tensor"]',
                 description='List of TRT output tensor names')
    args.add_arg('output_binding_names', '["output0"]',
                 description='List of TRT output tensor binding names')
    args.add_arg('output_tensor_formats', '["nitros_tensor_list_nhwc_rgb_f32"]',
                 description='List of TRT output tensor nitros type formats')

    # Additional arguments
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME,
                 description='Name of container where detection nodes are')
    args.add_arg('run_standalone', 'False',
                 description='Run in a standalone container if True')
    # To avoid fps drops in rosbag replay uncompressed sensor topics, e.g. multi-rs
    args.add_arg('one_container_per_camera', 'False',
                 description='Run per-camera based detection nodes in separate containers if True')
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)

    args.add_arg('target_class_id', '0', 
                 description="Select the class id which the mask will be created for.")

    # Unet Decoder
    args.add_arg('network_output_type', 'sigmoid')
    args.add_arg('color_segmentation_mask_encoding', 'rgb8')
    args.add_opaque_function(add_detection)
    return LaunchDescription(args.get_launch_actions())
