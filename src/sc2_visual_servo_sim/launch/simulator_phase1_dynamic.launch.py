from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    launch_file = os.path.join(
        get_package_share_directory('sc2_visual_servo_sim'),
        'launch',
        'simulator_phase1_mvp.launch.py',
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'backend': 'dynamic_velocity', 'detector_mode': 'synthetic'}.items(),
        ),
    ])
