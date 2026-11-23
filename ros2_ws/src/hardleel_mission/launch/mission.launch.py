"""hardleel waste-sorting mission (behaviour tree). Needs Nav2 running (laptop_nav.launch.py odom:=encoders on the
laptop + pi_encoder_mapping.launch.py slam:=false on the Pi) and the AI camera service (/classify_waste).

usage:  ros2 launch hardleel_mission mission.launch.py [mission_file:=~/mission.yaml] [dry_run:=true]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('hardleel_mission'), 'config', 'mission.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('mission_file', default_value=default),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true = simulate Nav2, camera and servo (test the tree logic)'),
        Node(package='hardleel_mission', executable='mission', name='mission', output='screen',
             parameters=[{'mission_file': LaunchConfiguration('mission_file'),
                          'dry_run': LaunchConfiguration('dry_run')}]),
    ])
