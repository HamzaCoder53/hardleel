import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Keep these identical to nav.launch.py
LASER_X, LASER_Y, LASER_Z, LASER_YAW = 0.0, 0.0, 0.15, 0.0
IMU_X, IMU_Y, IMU_Z = 0.0, 0.0, 0.05
IMU_ROLL, IMU_PITCH, IMU_YAW = 0.0, 0.0, 0.0


def find_nano():
    found = sorted(glob.glob('/dev/serial/by-id/usb-Arduino*'))
    return found[0] if found else '/dev/ttyACM1'


def static_tf(x, y, z, roll, pitch, yaw, parent, child, name):
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                   '--yaw', str(yaw), '--pitch', str(pitch), '--roll', str(roll),
                   '--frame-id', parent, '--child-frame-id', child])


def generate_launch_description():
    pkg = get_package_share_directory('robot_slam')
    sllidar_pkg = get_package_share_directory('sllidar_ros2')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    slam_pkg = get_package_share_directory('slam_toolbox')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_port', default_value=find_nano()),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('slam_params_file',
                              default_value=os.path.join(pkg, 'config', 'slam_toolbox_localization.yaml')),

        # LiDAR driver (if you added scan_mode to slam.launch.py, add it here too)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sllidar_pkg, 'launch', 'sllidar_a1_launch.py')),
            launch_arguments={
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': '115200',
                'frame_id': 'laser',
                'angle_compensate': 'true',
            }.items()),

        static_tf(LASER_X, LASER_Y, LASER_Z, 0.0, 0.0, LASER_YAW, 'base_link', 'laser', 'base_to_laser'),
        static_tf(IMU_X, IMU_Y, IMU_Z, IMU_ROLL, IMU_PITCH, IMU_YAW, 'base_link', 'imu_link', 'base_to_imu'),

        # Nano IMU -> /imu/data_raw (keep the robot still for the first 5 s)
        Node(package='robot_slam', executable='imu_serial_node', name='imu_serial_node',
             output='screen',
             parameters=[{'port': LaunchConfiguration('imu_port'),
                          'frame_id': 'imu_link', 'gyro_calib_s': 5.0}]),

        # LiDAR odometry (/odom_rf2o only; the EKF publishes the TF)
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', output='screen',
             parameters=[{'laser_scan_topic': '/scan',
                          'odom_topic': '/odom_rf2o',
                          'publish_tf': False,
                          'base_frame_id': 'base_link',
                          'odom_frame_id': 'odom',
                          'init_pose_from_topic': '',
                          'freq': 10.0}]),

        # EKF: odom -> base_link, output remapped to /odom for Nav2
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen',
             parameters=[os.path.join(pkg, 'config', 'ekf_nav.yaml')],
             remappings=[('odometry/filtered', '/odom')]),

        # SLAM Toolbox in localization mode: publishes map -> odom and /map (replaces map_server + AMCL)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_pkg, 'launch', 'localization_launch.py')),
            launch_arguments={
                'slam_params_file': LaunchConfiguration('slam_params_file'),
                'use_sim_time': 'false',
            }.items()),

        # Nav2 navigation only: planner, controller, costmaps, behaviors (no map_server, no AMCL)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items()),
    ])
