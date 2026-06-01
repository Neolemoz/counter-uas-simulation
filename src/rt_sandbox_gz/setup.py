import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rt_sandbox_gz'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (
            os.path.join('share', package_name, 'models', 'rt_drone'),
            glob('models/rt_drone/*'),
        ),
        (
            os.path.join('share', package_name, 'models', 'rt_radar'),
            glob('models/rt_radar/*'),
        ),
        (
            os.path.join('share', package_name, 'models', 'rt_interceptor'),
            glob('models/rt_interceptor/*'),
        ),
        (
            os.path.join('share', package_name, 'models', 'rt_waypoint'),
            glob('models/rt_waypoint/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='RT sandbox Gazebo bridge for PLAT-RT-G6.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rt_sandbox_gz_bridge_node = rt_sandbox_gz.gz_bridge_node:main',
        ],
    },
)
