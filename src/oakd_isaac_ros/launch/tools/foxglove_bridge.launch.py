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

import isaac_ros_launch_utils.all_types as lut
import isaac_ros_launch_utils as lu

# Whitelisted topics are the only ones propagated to Foxglove. Set "use_foxglove_whitelist:=False"
# to make all topics available.
TOPIC_WHITELIST = [
    # nvblox
    '/nvblox_node/combined_esdf_pointcloud',
    '/nvblox_node/.*_layer',
    '/nvblox_node/esdf_slice_bounds',
    '/nvblox_node/mesh',
    '/nvblox_node/static_esdf_pointcloud',
    # CUVSLAM
    '/visual_slam/status',
    '/visual_slam/tracking/odometry',
    '/visual_slam/tracking/slam_path',
    '/visual_slam/tracking/vo_path',
    '/visual_slam/tracking/vo_pose',
    '/visual_slam/vis/landmarks_cloud',
    '/visual_slam/vis/observations_cloud',
    # Navigation
    '/map',
    '/plan',
    '/local_costmap/costmap',
    '/local_costmap/local_costmap',
    '/local_costmap/published_footprint',
    '/global_costmap/costmap',
    '/global_costmap/global_costmap',
    '/global_costmap/published_footprint',
    '/initialpose',
    '/goal_pose',
    # Lidar
    '/back_2d_lidar/scan',
    '/front_2d_lidar/scan',
    '/front_3d_lidar/scan',
    '/front_3d_lidar/lidar_points_filtered',
    # Robot
    '/robot_description',
    '/tf',
    '/tf_static',
    '/diagnostics',
    # Global localization
    '/visual_localization/pose',

    # Bar30 Pressure Sensor
    '/bar30/depth',
    '/bar30/odom',
    '/bar30/pressure',
    '/bar30/temperature',

    # DVL-A50

]

IMAGE_TOPIC_WHITELIST = ["depth", "left/image_resized"]


MY_TOPIC_LIST = [
    #nvblox 
<<<<<<< HEAD
    '/nvblox_node/tsdf_layer',
    '/nvblox_node/color_layer_marker',
    '/nvblox_node/color_layer',
    '/nvblox_node/mesh',
    '/map',

    #oak-d
    '/oak',
    '/oak_rgb/camera_info',
    '/oak/stereo/image_raw',
    '/oak/stereo/camera_info',
    '/oak/rgb/image_raw/compressed',
    '/oak/compressed_video',
=======
    #'/nvblox_node/tsdf_layer',
    '/nvblox_node/color_layer_marker',
    '/nvblox_node/dynamic_occupancy_layer',
    '/nvblox_node/dynamic_points',
    #'/nvblox_node/color_layer',
    #'/nvblox_node/mesh',
    '/map',

    #oak-d
    '/sc2/imu/imu',    #remapped topic from /oak/imu/data 
    '/imu/data',
    #'/oak/imu/data',
    #'/oak/rgb/camera_info',
    '/oak/stereo/image_raw',
    #'/oak/stereo/camera_info',
    #'/oak/rgb/image_raw/compressed',
    #'/oak/rgb/image_raw_rgb8/nitros',
    #'/oak/compressed_video',

    #YOLOv8
    #'/camera0/detection/mask',
    #'/mask_overlay',
    #'/detections_output',

>>>>>>> grasp_point_detection

    #URDF topcis
    '/robot_description',
    '/joint_states',
<<<<<<< HEAD
    '/position_rad',
=======
    #'/position_rad',
    '/decoupled_position_rad',
>>>>>>> grasp_point_detection
    '/command/set_rel_pos_rad',
    '/grapple_percentage',

    # Robot
    '/robot_description',
    '/tf',
    '/tf_static',
    '/diagnostics',
    '/visual_localization/pose',

    #system stat topics
    '/system_stats',

<<<<<<< HEAD
    # torque gauge topics
    '/torque_gauge',
    '/torque_nm',
=======
    # senso control
    '/torque_gauge',
    '/abs_torque_nm',
    '/abs_torque_nm_per_joint',
    '/position_rad',  
>>>>>>> grasp_point_detection

    # Bar30 Pressure Sensor
    '/bar30/depth',
    '/bar30/odom',
    '/bar30/pressure',
    '/bar30/temperature',

<<<<<<< HEAD
=======
    # visualization
    '/seabed_grid',

    #DVL
    '/waterlinked_dvl_driver/dead_reckoning_report',
    '/waterlinked_dvl_driver/transition_event',
    '/waterlinked_dvl_driver/velocity_report'

    # sc2 topics
    '/sc2/altitude',
    '/sc2/aruco_pose/pose',
    '/sc2/dvl/dvl',
    '/sc2/dvl/odometry',
    '/sc2/gps/gps',
    '/sc2/imu/data_ned',
    '/sc2/imu/imu',
    '/sc2/odometry_ekf',
    '/sc2/position_controller_node/console_desireds',
    '/sc2/position_controller_node/control_errors',
    '/sc2/position_controller_node/control_signals',
    '/sc2/position_controller_node/outer/console_signals',
    '/sc2/position_controller_node/outer/control_signals',
    '/sc2/position_controller_node/outer/gamepad_signals',
    '/sc2/pwm_out',
    '/sc2/set_pose',
    '/sc2/state_estimation_node/depth',
    '/sc2/state_estimation_node/detected_objects/target',
    '/sc2/state_estimation_node/dvl',
    '/sc2/state_estimation_node/estimated_position',
    '/sc2/state_estimation_node/estimated_position_exploring',
    '/sc2/state_estimation_node/estimated_position_visual',
    '/sc2/state_estimation_node/estimated_state',
    '/sc2/state_estimation_node/estimated_velocity',
    '/sc2/thruster_manager/input',
    '/sc2/velocity_controller_node/console_desireds',
    '/sc2/velocity_controller_node/control_errors',
    '/sc2/velocity_controller_node/control_signals',
    '/sc2/velocity_controller_node/estimated_values_exploring',
    '/sc2/velocity_controller_node/estimated_values_visual',
    '/sc2/velocity_controller_node/outer/console_signals',
    '/sc2/velocity_controller_node/outer/gamepad_signals',
    '/sc2/wrench_order_enable',
    '/sc2/wrench_rov',
    '/sc2/wrench_rov_console',
    '/sc2/wrench_tortuga_interface',
    
>>>>>>> grasp_point_detection
]

def get_topic_whitelist():
    topic_whitelist = MY_TOPIC_LIST

    for camera in [
            '/front_stereo_camera', '/back_stereo_camera', '/left_stereo_camera',
            '/right_stereo_camera'
    ]:
        topic_whitelist.extend([f'{camera}/{topic}' for topic in IMAGE_TOPIC_WHITELIST])

    return topic_whitelist


def add_foxglove(args: lu.ArgumentContainer) -> list[lut.Action]:

    params = [{
        'send_buffer_limit': int(args.send_buffer_limit),
        'max_qos_depth': 30,
        'use_compression': True,
        'capabilities': ['clientPublish', 'connectionGraph', 'assets'],
    }]

    if lu.is_true(args.use_foxglove_whitelist):
        params[0].update({'topic_whitelist': get_topic_whitelist()})

    actions = []
    actions.append(
        lut.Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            parameters=params,
            # Use error log level to reduce terminal cluttering from "send_buffer_limit reached"
            # warnings.
            arguments=['--ros-args', '--log-level', 'ERROR'],
        ))

    return actions


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('use_foxglove_whitelist')
    args.add_arg('send_buffer_limit', 50000000)

    args.add_opaque_function(add_foxglove)
    return lu.LaunchDescription(args.get_launch_actions())
