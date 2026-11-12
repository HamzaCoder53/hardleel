"""Record the topics needed to rebuild a map or replay a run offline.

    ros2 launch robot_slam record_bag.launch.py                       # bags/run_<timestamp> (next to where you launch)
    ros2 launch robot_slam record_bag.launch.py bag_dir:=/home/hardleel/bags name:=track_01

Topics: /scan, /scan_filtered, /imu/* (data_raw, data, data_lpf), /cmd_vel.
Plain command for the same thing:
    ros2 bag record -o bags/run_01 /scan /scan_filtered /imu/data_raw /imu/data /imu/data_lpf /cmd_vel
"""
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

TOPICS = ['/scan', '/scan_filtered', '/imu/data_raw', '/imu/data', '/imu/data_lpf', '/cmd_vel']


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_dir', default_value='bags'),
        DeclareLaunchArgument('name', default_value=time.strftime('run_%Y%m%d_%H%M%S')),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', PathJoinSubstitution([LaunchConfiguration('bag_dir'),
                                                                      LaunchConfiguration('name')])] + TOPICS,
            output='screen'),
    ])
