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
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.lua') + glob('config/*.rviz') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='LiDAR-only SLAM bringup with Guardian tilt-gating & APF bumper',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_serial_node = robot_slam.imu_serial_node:main',
            'hardleel_guardian_node = robot_slam.hardleel_guardian_node:main',
            'wall_follow_explorer_node = robot_slam.wall_follow_explorer_node:main',
            'sensor_prep = robot_slam.sensor_prep:main',
            'coverage_mission = robot_slam.coverage_mission:main',
            'competition_mission = robot_slam.competition_mission:main',
            'event_logger = robot_slam.event_logger:main',
            'save_pose = robot_slam.save_pose:main',
            'click_waypoints = robot_slam.click_waypoints:main',
            'drop_servo_node = robot_slam.drop_servo_node:main',
            'servo_calibrate = robot_slam.servo_calibrate:main',
            'frontier_explorer = robot_slam.frontier_explorer:main',
            'speed_governor = robot_slam.speed_governor:main',
            'map_freezer = robot_slam.map_freezer:main',
            'scan_gate = robot_slam.scan_gate:main',
            'imu_bias = robot_slam.imu_bias:main',
            'wall_heading = robot_slam.wall_heading:main',
            'loc_monitor = robot_slam.loc_monitor:main',
            'run_recorder = robot_slam.run_recorder:main',
            'ramp_dash = robot_slam.ramp_dash_node:main',
        ]
    },
)
