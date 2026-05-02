import os
from glob import glob

from setuptools import setup

package_name = 'counter_uas'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Launch bringup for the counter-UAS simulation stack.',
    license='Apache-2.0',
)
