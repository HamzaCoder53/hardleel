"""Mapping stack on the PI with WHEEL ENCODERS + LIDAR (+ optional IMU) (22 Nov, new 6 V encoder motors).

  lidar (lidar_supervisor.sh, port auto-detected)          -> /scan
  base_link -> laser (lidar 10 cm behind the robot centre)
  esp32_encoder_bridge: cmd_vel -> closed-loop motors; encoders -> odometry
  imu:=false  the bridge publishes odom->base_link + /odom from the encoders alone
  imu:=true   Nano IMU (imu.launch.py, no own EKF) + ekf_encoder_node (ekf_encoders.yaml): forward speed from
              the encoders (/wheel/odom), yaw rate from the gyro -> odom->base_link + /odom. Skid steer wheels
              slip in turns, the gyro does not: better heading, straighter walls.
              Default: true when the Nano is plugged in (/dev/serial/by-id/usb-Arduino*).
              KEEP THE ROBOT STILL for the first ~5 s (gyro calibration).
  SLAM -> map->odom, /map:
    slam:=toolbox (default, 23 Nov)  SLAM Toolbox async (slam_toolbox_encoders.yaml)
    slam:=cartographer               Cartographer (cartographer_2d_encoders.lua)
    slam:=false                      none (navigation: AMCL on the laptop owns map->odom)
  tuning_helper (tuning GUI on the laptop)
Laptop: hmap (RViz + tuning GUI + teleop) to map; navigation: Pi slam:=false + laptop
laptop_nav.launch.py odom:=encoders map:=...

usage (on the Pi):  ros2 launch robot_slam pi_encoder_mapping.launch.py                     mapping (SLAM Toolbox)
                    ros2 launch robot_slam pi_encoder_mapping.launch.py slam:=cartographer  mapping (Cartographer)
                    ros2 launch robot_slam pi_encoder_mapping.launch.py slam:=false         navigation
                    ... imu:=false / imu:=true   force without / with the IMU
Saving a SLAM Toolbox map: map_saver_cli (-> .yaml/.pgm) + /slam_toolbox/serialize_map (-> .posegraph/.data).
"""
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from robot_slam import tuning_store as ts
from robot_slam.serial_ports import ports_or_fallback

LASER_X, LASER_Y, LASER_Z, LASER_YAW = -0.10, 0.0, 0.15, 0.0   # lidar 10 cm behind the centre, facing forward
LUA = 'cartographer_2d_encoders.lua'


def generate_launch_description():
    pkg = get_package_share_directory('robot_slam')
    cfg = os.path.join(pkg, 'config')
    if os.path.exists(ts.SESSION_CARTO):          # new run without last run's unsaved values
        os.remove(ts.SESSION_CARTO)
    ports = ports_or_fallback()                   # which ttyUSB is the lidar / the ESP32 (asked once)
    port_info = f"serial ports: lidar {ports['lidar']} ({ports['how'].get('lidar')}), " \
                f"ESP32 {ports['esp32']} ({ports['how'].get('esp32')})"
    nano = bool(glob.glob('/dev/serial/by-id/usb-Arduino*'))

    def slam_is(*values):   # slam:=true (older usage) means the default, SLAM Toolbox
        return IfCondition(PythonExpression(["'", LaunchConfiguration('slam'), "' in ", repr(list(values))]))
    carto, toolbox = slam_is('cartographer'), slam_is('toolbox', 'true')
    with_imu, no_imu = IfCondition(LaunchConfiguration('imu')), UnlessCondition(LaunchConfiguration('imu'))
    bridge = {'port': ports['esp32'], 'max_speed': 3.0}   # user 22 Nov: no speed cap (motors top out ~0.88 m/s)

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='toolbox',
                              description='toolbox | cartographer | false (navigation: AMCL owns map->odom)'),
        DeclareLaunchArgument('imu', default_value='true' if nano else 'false',
                              description='true = encoder speed + IMU yaw rate in an EKF (default: Nano plugged in)'),
        LogInfo(msg=port_info),
        LogInfo(msg='odometry: encoders + IMU (EKF) - keep the robot still ~5 s for the gyro calibration',
                condition=with_imu),
        LogInfo(msg='odometry: encoders only (no IMU)', condition=no_imu),

        ExecuteProcess(cmd=['bash', os.path.expanduser('~/lidar_supervisor.sh')], name='lidar_supervisor',
                       output='screen', additional_env={'LIDAR_PORT': ports['lidar']}),

        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
             arguments=['--x', str(LASER_X), '--y', str(LASER_Y), '--z', str(LASER_Z),
                        '--yaw', str(LASER_YAW), '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser']),

        # encoders only: the bridge owns odom->base_link and /odom
        Node(package='motor_teleop', executable='esp32_encoder_bridge', name='esp32_encoder_bridge',
             output='screen', condition=no_imu,
             parameters=ts.with_saved({**bridge, 'publish_tf': True, 'odom_topic': '/odom'})),

        # with IMU: the bridge publishes /wheel/odom only, the EKF fuses it with the gyro
        Node(package='motor_teleop', executable='esp32_encoder_bridge', name='esp32_encoder_bridge',
             output='screen', condition=with_imu,
             parameters=ts.with_saved({**bridge, 'publish_tf': False, 'odom_topic': '/wheel/odom'})),
        IncludeLaunchDescription(                  # Nano IMU node + base_link->imu_link, without its own EKF
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'imu.launch.py')),
            launch_arguments={'use_ekf': 'false', 'use_madgwick': 'false'}.items(), condition=with_imu),
        Node(package='robot_localization', executable='ekf_node', name='ekf_encoder_node', output='screen',
             parameters=ts.with_saved(os.path.join(cfg, 'ekf_encoders.yaml')),
             remappings=[('odometry/filtered', '/odom')], condition=with_imu),

        # respawn: the tuning GUI's Cartographer "Apply" restarts it with rewritten values
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
             output='screen', respawn=True, respawn_delay=2.0,
             parameters=[{'use_sim_time': False}],
             arguments=['-configuration_directory', ts.effective_carto_dir(cfg, LUA),
                        '-configuration_basename', LUA],
             remappings=[('scan', '/scan'), ('odom', '/odom')], condition=carto),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='occupancy_grid_node', output='screen',
             parameters=[{'use_sim_time': False}],
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'], condition=carto),

        # SLAM Toolbox (async: drops scans when busy instead of lagging behind on the Pi)
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             output='screen', parameters=[os.path.join(cfg, 'slam_toolbox_encoders.yaml')],
             condition=toolbox),

        Node(package='robot_slam', executable='tuning_helper', name='tuning_helper', output='screen'),
    ])
