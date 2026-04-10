from setuptools import find_packages
from setuptools import setup

package_name = 'sc2_visual_servo_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SeaClear2',
    maintainer_email='noreply@seaclear2.local',
    description='Controller and simulation utilities for SeaClear2 visual servoing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'visual_servo_controller_node = sc2_visual_servo_controller.visual_servo_controller_node:main',
            'visual_servo_controller_node = sc2_visual_servo_controller.visual_servo_controller_node_implementation:main',
            'target_obs_publisher = sc2_visual_servo_controller.target_obs_publisher:main',
            'fake_vehicle_node = sc2_visual_servo_controller.fake_vehicle_node:main',
            'csv_logger_node = sc2_visual_servo_controller.csv_logger_node:main',
            'cmd_vel_to_mavlink_stub = sc2_visual_servo_controller.cmd_vel_to_mavlink_stub:main',
            'cmd_vel_to_mavros_bridge = sc2_visual_servo_controller.cmd_vel_to_mavros_bridge:main',
            'visual_servo_debug_viz_node = sc2_visual_servo_controller.visual_servo_debug_viz_node:main',
            'velocity_control_test_node = sc2_visual_servo_controller.velocity_control_test:main',
            'axes_test = sc2_visual_servo_controller.axes_test:main',
        ],
    },
)
