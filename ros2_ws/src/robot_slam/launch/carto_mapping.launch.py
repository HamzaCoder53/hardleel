import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_slam.tuning_store import effective_carto_dir   # lua + tuning GUI overrides (saved / this run)

# Keep identical to slam.launch.py
# lidar 10 cm BEHIND the robot centre (user 15 Nov); robot 24 x 30.5 cm, base_link = robot centre
LASER_X, LASER_Y, LASER_Z, LASER_YAW = -0.10, 0.0, 0.15, 0.0   # yaw 0: lidar faces forward (was pi = reversed, 14 Nov)


def generate_launch_description():
    pkg = get_package_share_directory('robot_slam')
    sllidar_pkg = get_package_share_directory('sllidar_ros2')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        # false when ~/lidar_supervisor.sh runs the lidar driver (reset + restart on timeouts)
        DeclareLaunchArgument('use_lidar', default_value='true'),

        # LiDAR driver (if you added scan_mode to slam.launch.py, add it here too)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sllidar_pkg, 'launch', 'sllidar_a1_launch.py')),
            condition=IfCondition(LaunchConfiguration('use_lidar')),
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

        # odom -> base_link and base_link -> imu_link come from imu.launch.py (EKF + static TF):
        # start imu.launch.py first.

        # Cartographer: SLAM (publishes map -> odom)
        # respawn: the tuning GUI's "Apply" rewrites the lua and stops Cartographer; it comes back with it
        Node(package='cartographer_ros', executable='cartographer_node',
             name='cartographer_node', output='screen', respawn=True, respawn_delay=2.0,
             parameters=[{'use_sim_time': False}],
             # default lua, or ~/.hardleel/carto/ = default + values saved/applied in the tuning GUI
             arguments=['-configuration_directory', effective_carto_dir(os.path.join(pkg, 'config')),
                        '-configuration_basename', 'cartographer_2d.lua'],
             remappings=[('scan', '/scan'), ('imu', '/imu/data_raw')]),

        # Cartographer's submaps -> /map occupancy grid
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='occupancy_grid_node', output='screen',
             parameters=[{'use_sim_time': False}],
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']),
    ])
