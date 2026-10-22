"""Pi side of the laptop-brain split: hardware drivers only.

The Pi streams /imu/data_raw, /imu/data and /camera/image_raw/compressed and drives the motors from /cmd_vel.
The lidar runs separately under scripts/pi/lidar_supervisor.sh (it needs a reset-and-retry loop).
Everything else (static TFs, EKF, Cartographer, Nav2, mission) runs on the laptop: brain.launch.py.
Start it with scripts/pi/robot_hw.sh, which sets ROS_DOMAIN_ID=7 and CycloneDDS.
"""
import glob
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def first(pattern, fallback):
    found = sorted(glob.glob(pattern))
    return found[0] if found else fallback


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('imu_port', default_value=first('/dev/serial/by-id/usb-Arduino*', '/dev/ttyACM1')),
        DeclareLaunchArgument('pico_port', default_value=first('/dev/serial/by-id/*Raspberry_Pi_Pico*', '/dev/ttyACM0')),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('camera_fps', default_value='10.0'),
        DeclareLaunchArgument('motors', default_value='true'),

        # Nano IMU -> /imu/data_raw (~100 Hz)
        Node(
            package='robot_slam', executable='imu_serial_node', name='imu_serial_node', output='screen',
            parameters=[{'port': LaunchConfiguration('imu_port'), 'frame_id': 'imu_link', 'gyro_calib_s': 5.0}],
        ),
        # Madgwick: /imu/data_raw -> /imu/data (same settings as imu.launch.py)
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter_madgwick_node',
            output='screen',
            parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu', 'gain': 0.02}],
            remappings=[('imu/data_raw', '/imu/data_raw'), ('imu/data', '/imu/data')],
        ),
        # /cmd_vel -> Pico; its 0.4 s command watchdog stops the motors if the laptop link drops
        Node(
            package='motor_teleop', executable='motor_bridge', name='motor_bridge', output='screen',
            condition=IfCondition(LaunchConfiguration('motors')),
            parameters=[{'port': LaunchConfiguration('pico_port'), 'send_rate': 10.0, 'max_pwm': 255}],
        ),
        # Pi Camera Module 3 -> /camera/image_raw/compressed
        ExecuteProcess(
            cmd=['python3', os.path.expanduser('~/pi_cam/pi_cam_node.py'), '--ros-args',
                 '-p', ['fps:=', LaunchConfiguration('camera_fps')]],
            condition=IfCondition(LaunchConfiguration('camera')),
            output='screen',
        ),
    ])
