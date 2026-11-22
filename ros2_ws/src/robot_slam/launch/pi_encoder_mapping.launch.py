"""Mapping stack on the PI with WHEEL ENCODERS + LIDAR, no IMU (22 Nov, new 6 V encoder motors).

  lidar (lidar_supervisor.sh, port auto-detected)          -> /scan
  base_link -> laser (lidar 10 cm behind the robot centre)
  esp32_encoder_bridge: cmd_vel -> closed-loop motors; encoders -> odom->base_link TF + /odom
  Cartographer (cartographer_2d_encoders.lua: odometry on, IMU off) -> map->odom, /map
  tuning_helper (tuning GUI on the laptop)
No IMU node, no EKF. Laptop: hmap (RViz + tuning GUI + teleop) to map, hauto (Nav2 on the live map) to send
Nav2 goals from RViz.

usage (on the Pi):  ros2 launch robot_slam pi_encoder_mapping.launch.py              mapping
                    ros2 launch robot_slam pi_encoder_mapping.launch.py slam:=false  navigation (no Cartographer;
                    laptop: laptop_nav.launch.py odom:=encoders map:=..., AMCL owns map->odom)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_slam import tuning_store as ts
from robot_slam.serial_ports import ports_or_fallback

LASER_X, LASER_Y, LASER_Z, LASER_YAW = -0.10, 0.0, 0.15, 0.0   # lidar 10 cm behind the centre, facing forward
LUA = 'cartographer_2d_encoders.lua'


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('robot_slam'), 'config')
    if os.path.exists(ts.SESSION_CARTO):          # new run without last run's unsaved values
        os.remove(ts.SESSION_CARTO)
    ports = ports_or_fallback()                   # which ttyUSB is the lidar / the ESP32 (asked once)
    port_info = f"serial ports: lidar {ports['lidar']} ({ports['how'].get('lidar')}), " \
                f"ESP32 {ports['esp32']} ({ports['how'].get('esp32')})"

    slam = IfCondition(LaunchConfiguration('slam'))
    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true',
                              description='false = navigation: no Cartographer (AMCL on the laptop owns map->odom)'),
        LogInfo(msg=port_info),
        LogInfo(msg='encoder + lidar stack (no IMU): odom->base_link from the wheel encoders'),

        ExecuteProcess(cmd=['bash', os.path.expanduser('~/lidar_supervisor.sh')], name='lidar_supervisor',
                       output='screen', additional_env={'LIDAR_PORT': ports['lidar']}),

        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
             arguments=['--x', str(LASER_X), '--y', str(LASER_Y), '--z', str(LASER_Z),
                        '--yaw', str(LASER_YAW), '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser']),

        Node(package='motor_teleop', executable='esp32_encoder_bridge', name='esp32_encoder_bridge',
             output='screen',
             parameters=ts.with_saved({'port': ports['esp32'], 'publish_tf': True, 'odom_topic': '/odom',
                                       'max_speed': 3.0})),   # user 22 Nov: no speed cap (motors top out ~0.88 m/s)

        # respawn: the tuning GUI's Cartographer "Apply" restarts it with rewritten values
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
             output='screen', respawn=True, respawn_delay=2.0,
             parameters=[{'use_sim_time': False}],
             arguments=['-configuration_directory', ts.effective_carto_dir(cfg, LUA),
                        '-configuration_basename', LUA],
             remappings=[('scan', '/scan'), ('odom', '/odom')], condition=slam),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='occupancy_grid_node', output='screen',
             parameters=[{'use_sim_time': False}],
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'], condition=slam),

        Node(package='robot_slam', executable='tuning_helper', name='tuning_helper', output='screen'),
    ])
