"""Nav2 on the LAPTOP (WSL Jazzy), robot drivers on the Pi.

Pi runs: lidar (lidar_supervisor.sh), imu.launch.py use_ekf:=false (IMU node + base_link->imu_link),
         esp32_teleop_bridge (cmd_vel -> motors). Cartographer must NOT run (AMCL owns map->odom).
Laptop runs (this file): base_link->laser, rf2o lidar odometry, EKF (rf2o + IMU yaw rate ->
         odom->base_link and /odom), Nav2 bringup (map server, AMCL, planner, controller, ...).
Same nodes and configs as nav.launch.py, minus the drivers.

usage: ros2 launch robot_slam laptop_nav.launch.py map:=$HOME/maps/<name>.yaml
Then in RViz: "2D Pose Estimate" where the robot is, then "Nav2 Goal".
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_slam.tuning_store import merged_params_file, with_saved   # tuning GUI saved values (~/.hardleel)

# Keep identical to slam.launch.py / carto_mapping.launch.py
LASER_X, LASER_Y, LASER_Z, LASER_YAW = 0.0, 0.0, 0.15, 0.0   # yaw 0: lidar faces forward (was pi = reversed, 14 Nov)


def generate_launch_description():
    pkg = get_package_share_directory('robot_slam')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/maps/my_map.yaml')),
        # nav2_params.yaml + values saved in the tuning GUI (merged into ~/.hardleel when anything is saved)
        DeclareLaunchArgument('params_file',
                              default_value=merged_params_file(os.path.join(pkg, 'config', 'nav2_params.yaml'))),

        # base_link -> laser (carto_mapping.launch.py published this during mapping)
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
             arguments=['--x', str(LASER_X), '--y', str(LASER_Y), '--z', str(LASER_Z),
                        '--yaw', str(LASER_YAW), '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser']),

        # LiDAR odometry: publishes /odom_rf2o only (the EKF publishes the TF)
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', output='screen',
             parameters=with_saved({'laser_scan_topic': '/scan',
                                    'odom_topic': '/odom_rf2o',
                                    'publish_tf': False,
                                    'base_frame_id': 'base_link',
                                    'odom_frame_id': 'odom',
                                    'init_pose_from_topic': '',
                                    'freq': 10.0})),

        # EKF: odom -> base_link, output topic remapped to /odom for Nav2
        Node(package='robot_localization', executable='ekf_node', name='ekf_nav_node',
             output='screen',
             parameters=with_saved(os.path.join(pkg, 'config', 'ekf_nav.yaml')),
             remappings=[('odometry/filtered', '/odom')]),

        # Nav2: map server, AMCL, planner, controller, costmaps, behaviors
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'slam': 'False',
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items()),
    ])
