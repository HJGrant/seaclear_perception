"""IMU-only pose estimate for multi-frame nvblox mapping when the vehicle's
real state_estimation stack (DVL/GPS-fused EKF) isn't running -- e.g.
debugging the mapping logic with just the OAK-D connected on a bench.

Runs the same robot_localization::ekf_node the real stack uses (see
state_estimation's ekf_fusing.launch.py), configured with only the OAK-D's
onboard IMU as input. Orientation tracks reasonably; position does not --
without DVL/GPS there's nothing to correct double-integrated accelerometer
drift, so treat any translation from this mode as unreliable. Good enough
to exercise map_frame_anchor's reset/re-anchor logic and rotation-only
consistency, not for anything needing real position.

Publishes the same odom -> sc2_base_link transform (publish_tf defaults to
true) that the real EKF would, so map_frame_anchor doesn't need to know or
care which pose source is active -- see perception.launch.py's pose_source
config switch.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_imu_only',
            namespace='sc2',
            output='screen',
            remappings=[
                ('/sc2/odometry/filtered', '/sc2/odometry_ekf'),
            ],
            parameters=[{
                'frequency': 60.0,
                'sensor_timeout': 0.5,
                'two_d_mode': False,
                'print_diagnostics': False,
                'publish_acceleration': False,
                'publish_tf': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'sc2_base_link',
                'world_frame': 'odom',

                'imu0': '/sc2/imu/imu',
                # Orientation + angular velocity only -- same axes
                # ekf_fusing.launch.py enables, no gravitational-acceleration
                # removal assumption needed since we're not using linear
                # acceleration for anything here.
                'imu0_config': [False, False, False, True, True, True,
                                False, False, False, True, True, True,
                                False, False, False],
                'imu0_nodelay': True,
                'imu0_differential': False,
                'imu0_relative': False,
                'imu0_queue_size': 5,
                'imu0_remove_gravitational_acceleration': False,

                'use_control': False,

                'initial_state': [
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0
                ],
            }],
        )
    ])
