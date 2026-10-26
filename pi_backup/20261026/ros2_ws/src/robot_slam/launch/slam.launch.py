import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_pkg = get_package_share_directory('slam_toolbox')
    sllidar_pkg = get_package_share_directory('sllidar_ros2')
    my_pkg = get_package_share_directory('robot_slam')

    def static_tf(x, y, z, yaw, parent, child, name):
        return Node(
            package='tf2_ros', executable='static_transform_publisher', name=name,
            arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                       '--yaw', str(yaw), '--pitch', '0', '--roll', '0',
                       '--frame-id', parent, '--child-frame-id', child])

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),

        # LiDAR driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sllidar_pkg, 'launch', 'sllidar_a1_launch.py')),
            launch_arguments={
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': '115200',
                'frame_id': 'laser',
                'angle_compensate': 'true',
            }.items()),

        # base_link -> laser. EDIT x, y, z (metres, from robot centre; x forward,
        # y left, z up) and yaw (radians) to match where the LiDAR is mounted.
        static_tf(0.0, 0.0, 0.15, 0.0, 'base_link', 'laser', 'base_to_laser'),

        # odom -> base_link: fixed at zero because there are no encoders

        # SLAM Toolbox publishes map -> odom
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': os.path.join(my_pkg, 'config', 'slam_toolbox.yaml'),
                'use_sim_time': 'false',
            }.items()),
    ])
