"""Nav2 on the simulated field (practice for the autonomous phase).

    ros2 launch rdj2026_sim nav.launch.py                          # ground-truth map, AMCL starts at START
    ros2 launch rdj2026_sim nav.launch.py map:=$HOME/maps/sim_map.yaml
Run sim.launch.py first. Send goals from RViz ("Nav2 Goal") or Lichtblick.
"""
import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('rdj2026_sim')
    nav2 = get_package_share_directory('nav2_bringup')
    layout = yaml.safe_load(open(os.path.join(pkg, 'arena', 'arena_layout.yaml')))
    sx, sy, syaw = layout['start']['pose']

    # Nav2 params with AMCL starting at the START pad (the truth map frame == the Gazebo world frame)
    params = yaml.safe_load(open(os.path.join(pkg, 'config', 'nav2_sim.yaml')))
    amcl = params['amcl']['ros__parameters']
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {'x': sx / 1000.0, 'y': sy / 1000.0, 'z': 0.0, 'yaw': math.radians(syaw)}
    tmp = tempfile.NamedTemporaryFile('w', suffix='_nav2.yaml', delete=False)
    yaml.safe_dump(params, tmp)
    tmp.close()

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'rdj2026_truth.yaml')),
        DeclareLaunchArgument('params_file', default_value=tmp.name),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'true',
                'autostart': 'true',
            }.items()),
    ])
