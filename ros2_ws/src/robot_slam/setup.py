import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='LiDAR-only SLAM bringup (RPLIDAR A1 + SLAM Toolbox)',
    license='Apache-2.0',
    entry_points={'console_scripts': ['imu_serial_node = robot_slam.imu_serial_node:main']},
)
