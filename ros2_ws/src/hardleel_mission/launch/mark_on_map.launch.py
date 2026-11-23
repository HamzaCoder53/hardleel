"""Show a SAVED map in RViz to mark the mission places by clicking - no robot needed (laptop).

usage:  ros2 launch hardleel_mission mark_on_map.launch.py map:=$HOME/maps/mymap.yaml
then:   ros2 run hardleel_mission mark --click          (second terminal; follow its prompts)
RViz "2D Goal Pose" tool: click on the spot, drag towards where the robot must face. Places show as arrows.
Do not run this while Nav2 is running (its own map server and /goal_pose would clash).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hardleel_mission')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/maps/mymap.yaml'),
                              description='saved map .yaml'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': False}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_marking',
             output='screen', parameters=[{'autostart': True, 'node_names': ['map_server']}]),
        Node(package='rviz2', executable='rviz2', name='rviz_mark_places', output='log',
             arguments=['-d', os.path.join(share, 'config', 'mark_places.rviz')]),
    ])
