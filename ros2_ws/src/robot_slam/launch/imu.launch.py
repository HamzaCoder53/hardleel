import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Where the Nano sits relative to base_link (metres) and how it is rotated (radians).
# Board flat with components up: leave the angles at 0.
# Board upside down (components down): IMU_ROLL = 3.14159
IMU_X, IMU_Y, IMU_Z = 0.0, 0.0, 0.05
IMU_ROLL, IMU_PITCH, IMU_YAW = 0.0, 0.0, 0.0


def find_nano():
    found = sorted(glob.glob('/dev/serial/by-id/usb-Arduino*'))
    return found[0] if found else '/dev/ttyACM1'


def generate_launch_description():
    pkg = get_package_share_directory('robot_slam')

    return LaunchDescription([
        DeclareLaunchArgument('imu_port', default_value=find_nano()),

        # Nano IMU -> /imu/data_raw
        Node(
            package='robot_slam',
            executable='imu_serial_node',
            name='imu_serial_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('imu_port'),
                'frame_id': 'imu_link',
                'gyro_calib_s': 5.0,
            }],
        ),

        # base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['--x', str(IMU_X), '--y', str(IMU_Y), '--z', str(IMU_Z),
                       '--yaw', str(IMU_YAW), '--pitch', str(IMU_PITCH), '--roll', str(IMU_ROLL),
                       '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        ),

        # EKF: publishes odom -> base_link from the gyro yaw rate
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
        ),
    ])
