"""Mapping stack on the LAPTOP (WSL) in one launch: RViz, keyboard teleop and the tuning GUI.
Pi side: pi_mapping.launch.py (IMU, lidar, motor bridge, Cartographer, tuning helper).

usage (WSL Ubuntu-24.04, ROS settings from ~/.bashrc; laptop and Pi on the same Wi-Fi):
  ros2 launch robot_slam laptop_mapping.launch.py
  ros2 launch robot_slam laptop_mapping.launch.py teleop:=false tuning:=false

Teleop needs a real keyboard terminal and WSL has no terminal program, so it opens in its own Windows
terminal window (cmd.exe start -> wsl.exe). That window is independent: close it yourself (Ctrl+C in it);
stopping this launch does not close it. Teleop = 'motor_teleop' for the robot (/cmd_vel -> motor bridge).
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def teleop_window():
    teleop = 'ros2 run teleop_twist_keyboard teleop_twist_keyboard'
    if shutil.which('xterm'):                       # a Linux terminal, if one is ever installed
        return ['xterm', '-T', 'hardleel teleop', '-e', 'bash', '-ic', teleop]
    distro = os.environ.get('WSL_DISTRO_NAME', 'Ubuntu-24.04')
    return ['cmd.exe', '/c', 'start', 'hardleel teleop', 'wsl.exe', '-d', distro, '--', 'bash', '-ic', teleop]


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory('robot_slam'), 'rviz', 'hardleel.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='true'),
        DeclareLaunchArgument('tuning', default_value='true'),

        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_cfg], output='log',
             condition=IfCondition(LaunchConfiguration('rviz'))),
        # Staggered: WSLg lost the tuning GUI's window when it started at the same moment as RViz (15 Nov)
        TimerAction(period=8.0, actions=[
            Node(package='robot_slam', executable='tuning_gui', name='hardleel_tuning_gui', output='screen',
                 condition=IfCondition(LaunchConfiguration('tuning')))]),
        TimerAction(period=12.0, actions=[
            ExecuteProcess(cmd=teleop_window(), name='teleop_window', output='log',
                           condition=IfCondition(LaunchConfiguration('teleop')))]),
    ])
