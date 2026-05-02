import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'gazebo_target_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'gui'), glob('gui/*.config')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'models', 'hit_explosion'), glob('models/hit_explosion/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Gazebo target + bridge to /drone/position.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_pose_to_drone_point_node = gazebo_target_sim.dynamic_pose_to_drone_point_node:main',
            'target_controller_node = gazebo_target_sim.target_controller_node:main',
            'interceptor_controller_node = gazebo_target_sim.interceptor_controller_node:main',
            'interception_logic_node = gazebo_target_sim.interception_logic_node:main',
            'noisy_measurement_node = gazebo_target_sim.noisy_measurement_node:main',
        ],
    },
)
