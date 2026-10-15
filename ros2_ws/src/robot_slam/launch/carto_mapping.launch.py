import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Keep identical to slam.launch.py
LASER_X, LASER_Y, LASER_Z, LASER_YAW = 0.0, 0.0, 0.15, 3.14159


def generate_launch_description():
    pkg = get_package_share_directory('robot_slam')
    sllidar_pkg = get_package_share_directory('sllidar_ros2')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),

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

        # base_link -> laser
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
             arguments=['--x', str(LASER_X), '--y', str(LASER_Y), '--z', str(LASER_Z),
                        '--yaw', str(LASER_YAW), '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser']),

        # Cartographer: SLAM (publishes map -> odom)
        Node(package='cartographer_ros', executable='cartographer_node',
             name='cartographer_node', output='screen',
             parameters=[{'use_sim_time': False}],
             arguments=['-configuration_directory', os.path.join(pkg, 'config'),
                        '-configuration_basename', 'cartographer_2d.lua'],
             remappings=[('scan', '/scan'), ('imu', '/imu/data_raw')]),

        # Cartographer's submaps -> /map occupancy grid
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='occupancy_grid_node', output='screen',
             parameters=[{'use_sim_time': False}],
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']),
    ])
