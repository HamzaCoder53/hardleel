"""Odometry on the PI for auto mapping (start_robot.sh auto): rf2o lidar odometry + navigation EKF.

On the Pi rf2o gets every scan at full rate (on the laptop, over Wi-Fi, it managed only 2-3 Hz).
The EKF (ekf_nav.yaml: rf2o forward speed + IMU yaw rate) owns odom->base_link and publishes /odom for
Nav2; Cartographer then adds map->odom. Run imu.launch.py with use_ekf:=false so there is one EKF only.
base_link->laser comes from carto_mapping.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from robot_slam.tuning_store import with_saved   # values saved in the tuning GUI (~/.hardleel)


def generate_launch_description():
    pkg = get_package_share_directory('robot_slam')
    return LaunchDescription([
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', output='screen',
             parameters=with_saved({'laser_scan_topic': '/scan',
                                    'odom_topic': '/odom_rf2o',
                                    'publish_tf': False,
                                    'base_frame_id': 'base_link',
                                    'odom_frame_id': 'odom',
                                    'init_pose_from_topic': '',
                                    'freq': 10.0})),
        Node(package='robot_localization', executable='ekf_node', name='ekf_nav_node',
             output='screen',
             parameters=with_saved(os.path.join(pkg, 'config', 'ekf_nav.yaml')),
             remappings=[('odometry/filtered', '/odom')]),
    ])
