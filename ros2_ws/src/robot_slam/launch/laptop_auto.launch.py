"""Auto mapping on the LAPTOP: Nav2 navigation on Cartographer's live map + frontier explorer.

Pi (start_robot.sh auto): IMU (no EKF), lidar, rf2o + EKF (odom->base_link, /odom), Cartographer
(map->odom, /map), motor bridge, tuning helper.
Laptop (this file): Nav2 navigation only - no map_server and no AMCL, the map and map->odom come from
Cartographer - plus frontier_explorer, which starts DISABLED (Start in the tuning GUI's Auto tab).
nav2_params.yaml already lets the planner use unknown space (allow_unknown) and tracks unknown cells.

usage: ros2 launch robot_slam laptop_auto.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_slam.tuning_store import merged_params_file, with_saved   # tuning GUI saved values (~/.hardleel)


def generate_launch_description():
    pkg = get_package_share_directory('robot_slam')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=merged_params_file(
            os.path.join(pkg, 'config', 'nav2_params.yaml'),
            os.path.expanduser('~/.hardleel/nav2_auto_effective.yaml'))),

        # Nav2 without localization: planner, controller, smoother, behaviors, bt_navigator,
        # velocity_smoother, collision_monitor (-> /cmd_vel -> motor bridge on the Pi)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items()),

        Node(package='robot_slam', executable='frontier_explorer', name='frontier_explorer',
             output='screen', parameters=with_saved()),
    ])
