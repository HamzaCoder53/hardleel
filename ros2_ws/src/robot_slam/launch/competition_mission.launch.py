"""Start only the competition mission node (Nav2 and the rest of the stack must already be running).

    ros2 launch robot_slam competition_mission.launch.py auto_start:=true
    ros2 topic pub --once /mission/start std_msgs/msg/Empty "{}"      # when auto_start is false
    ros2 topic pub --once /mission/abort std_msgs/msg/Empty "{}"      # stop everything
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_slam')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('course_file', default_value=os.path.join(share, 'config', 'competition_course.yaml')),
        Node(package='robot_slam', executable='competition_mission', name='competition_mission', output='screen',
             parameters=[os.path.join(share, 'config', 'competition_mission.yaml'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'auto_start': LaunchConfiguration('auto_start'),
                          'course_file': LaunchConfiguration('course_file')}]),
    ])
