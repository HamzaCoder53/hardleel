"""Map the simulated field (practice for the 3-minute mapping phase / Virtual SLAM).

    ros2 launch rdj2026_sim mapping.launch.py                  # Cartographer (same config as the robot)
    ros2 launch rdj2026_sim mapping.launch.py slam:=slam_toolbox
Run sim.launch.py first, then drive with:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
Save when done:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/sim_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from robot_slam.carto_lua import config_for_platform


def generate_launch_description():
    pkg = get_package_share_directory('rdj2026_sim')
    carto_dir, carto_file = config_for_platform(os.path.join(pkg, 'config'), 'cartographer_sim.lua')
    slam = LaunchConfiguration('slam')
    is_carto = IfCondition(PythonExpression(["'", slam, "' == 'cartographer'"]))
    is_stb = IfCondition(PythonExpression(["'", slam, "' == 'slam_toolbox'"]))
    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='cartographer',
                              description='cartographer | slam_toolbox'),

        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
             output='screen', condition=is_carto,
             parameters=[{'use_sim_time': True}],
             arguments=['-configuration_directory', carto_dir,
                        '-configuration_basename', carto_file],
             remappings=[('scan', '/scan'), ('imu', '/imu/data_raw')]),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='occupancy_grid_node', output='screen', condition=is_carto,
             parameters=[{'use_sim_time': True}],
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']),

        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             output='screen', condition=is_stb,
             parameters=[os.path.join(pkg, 'config', 'slam_toolbox_sim.yaml'), {'use_sim_time': True}]),
    ])
