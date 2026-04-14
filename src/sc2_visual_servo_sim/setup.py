from setuptools import setup


package_name = 'sc2_visual_servo_sim'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/simulator_phase1_mvp.launch.py',
            'launch/simulator_phase1_dynamic.launch.py',
            'launch/simulator_phase1_direct.launch.py',
            'launch/simulator_phase1_actual_detector.launch.py',
        ]),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='SeaClear2',
    maintainer_email='noreply@seaclear2.local',
    description='Phase 1 simulator nodes and launches for SC2 visual servo validation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_direct_velocity_backend_node = sc2_visual_servo_sim.sim_direct_velocity_backend_node:main',
            'sim_dynamic_velocity_backend_node = sc2_visual_servo_sim.sim_dynamic_velocity_backend_node:main',
            'sim_downward_camera_node = sc2_visual_servo_sim.sim_downward_camera_node:main',
            'rgb_target_detector_node = sc2_visual_servo_sim.rgb_target_detector_node:main',
            'odom_to_body_twist_node = sc2_visual_servo_sim.odom_to_body_twist_node:main',
            'sim_static_frames_node = sc2_visual_servo_sim.sim_static_frames_node:main',
        ],
    },
)
