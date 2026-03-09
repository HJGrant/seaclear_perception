from setuptools import setup

package_name = 'sc2_visual_servo_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SeaClear2',
    maintainer_email='noreply@seaclear2.local',
    description='Perception wrappers for SeaClear2 visual servoing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_observer_node = sc2_visual_servo_perception.target_observer_node:main',
            'grasp_pose_estimator_node = sc2_visual_servo_perception.grasp_pose_estimator_node:main',
        ],
    },
)
