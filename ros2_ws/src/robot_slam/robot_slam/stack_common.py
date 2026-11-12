"""Building blocks shared by mapping_stack.launch.py and mission_stack.launch.py.

Command chain (the guardian node is NOT used in these stacks; the collision monitor replaces it):
    teleop -> /cmd_vel_teleop \\
                                twist_mux -> /cmd_vel_mux -> velocity_smoother -> /cmd_vel_smoothed
    Nav2   -> /cmd_vel_nav    /                                   -> collision_monitor -> /cmd_vel -> motor_bridge
Scan chain: /scan -> sensor_prep -> /scan_filtered -> Cartographer, Nav2 costmaps, collision monitor.
"""
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# base_link -> laser / imu_link. LASER_YAW was 3.14159 on 15 Oct (LiDAR mounted facing backwards) and is 0.0
# in the current launch files; set it to match how the LiDAR is mounted now.
LASER_X, LASER_Y, LASER_Z, LASER_YAW = 0.0, 0.0, 0.15, 0.0
IMU_X, IMU_Y, IMU_Z = 0.0, 0.0, 0.05

TF_REMAP = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


def pkg_share(name='robot_slam'):
    return get_package_share_directory(name)


def find_nano():
    found = sorted(glob.glob('/dev/serial/by-id/usb-Arduino*'))
    return found[0] if found else '/dev/ttyACM1'


def static_tf(x, y, z, yaw, parent, child, name):
    return Node(package='tf2_ros', executable='static_transform_publisher', name=name,
                arguments=['--x', str(x), '--y', str(y), '--z', str(z), '--yaw', str(yaw),
                           '--pitch', '0', '--roll', '0', '--frame-id', parent, '--child-frame-id', child])


def lidar(lidar_port):
    """RPLIDAR A1 driver, Sensitivity scan mode (8k samples/s, about 1400 points per revolution)."""
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')),
            launch_arguments={'serial_port': lidar_port, 'serial_baudrate': '115200', 'frame_id': 'laser',
                              'angle_compensate': 'true', 'scan_mode': 'Sensitivity'}.items()),
        static_tf(LASER_X, LASER_Y, LASER_Z, LASER_YAW, 'base_link', 'laser', 'base_to_laser'),
        static_tf(IMU_X, IMU_Y, IMU_Z, 0.0, 'base_link', 'imu_link', 'base_to_imu'),
    ]


def imu_and_prep(imu_port):
    """Nano IMU -> /imu/data_raw -> Madgwick -> /imu/data, then sensor_prep (/scan_filtered, /imu/data_lpf)."""
    return [
        Node(package='robot_slam', executable='imu_serial_node', name='imu_serial_node', output='screen',
             parameters=[{'port': imu_port, 'frame_id': 'imu_link', 'gyro_calib_s': 5.0}]),
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter_madgwick_node', output='screen',
             parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu', 'gain': 0.02}],
             remappings=[('imu/data_raw', '/imu/data_raw'), ('imu/data', '/imu/data')]),
        Node(package='robot_slam', executable='sensor_prep', name='sensor_prep', output='screen',
             parameters=[{'lidar_height': LASER_Z}]),
    ]


def cartographer(localization=False, pbstream=''):
    """Cartographer on /scan_filtered + /imu/data. Localization mode loads a frozen .pbstream."""
    args = ['-configuration_directory', os.path.join(pkg_share(), 'config'),
            '-configuration_basename',
            'cartographer_2d_localization.lua' if localization else 'cartographer_2d.lua']
    if localization:
        args += ['-load_state_filename', pbstream, '-load_frozen_state=true']
    return [
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
             output='screen', parameters=[{'use_sim_time': False}], arguments=args,
             remappings=[('scan', '/scan_filtered'), ('imu', '/imu/data')]),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='occupancy_grid_node', output='screen', parameters=[{'use_sim_time': False}],
             arguments=['-resolution', '0.05', '-publish_period_sec', '0.5']),
    ]


def safety_chain(profile, nav2_params):
    """twist_mux -> velocity_smoother -> collision_monitor (profile: 'mapping' | 'mission')."""
    cfg = os.path.join(pkg_share(), 'config')
    monitor = f'collision_monitor_{profile}'
    return [
        Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
             parameters=[os.path.join(cfg, 'twist_mux.yaml')],
             remappings=[('cmd_vel_out', 'cmd_vel_mux')]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='screen', parameters=[nav2_params],
             remappings=TF_REMAP + [('cmd_vel', 'cmd_vel_mux'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name=monitor, output='screen',
             parameters=[os.path.join(cfg, 'collision_monitor.yaml')], remappings=TF_REMAP),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_safety',
             output='screen', parameters=[{'autostart': True, 'node_names': ['velocity_smoother', monitor]}]),
    ]


def nav2_nodes(nav2_params):
    """Nav2 navigation servers. Controller and behaviors publish /cmd_vel_nav (into twist_mux)."""
    nav_names = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
                 'bt_navigator', 'waypoint_follower']
    remaps = {'controller_server': [('cmd_vel', 'cmd_vel_nav')], 'behavior_server': [('cmd_vel', 'cmd_vel_nav')]}
    pkgs = {'controller_server': 'nav2_controller', 'smoother_server': 'nav2_smoother',
            'planner_server': 'nav2_planner', 'behavior_server': 'nav2_behaviors',
            'bt_navigator': 'nav2_bt_navigator', 'waypoint_follower': 'nav2_waypoint_follower'}
    nodes = [Node(package=pkgs[n], executable=n, name=n, output='screen', parameters=[nav2_params],
                  remappings=TF_REMAP + remaps.get(n, [])) for n in nav_names]
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='lifecycle_manager_navigation', output='screen',
                      parameters=[{'autostart': True, 'node_names': nav_names, 'bond_timeout': 10.0}]))
    return nodes


def odom_for_nav2():
    """/odom for Nav2 from rf2o + gyro EKF. TF publishing is off: Cartographer owns odom -> base_link."""
    cfg = os.path.join(pkg_share(), 'config')
    return [
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node', name='rf2o_laser_odometry',
             output='screen',
             parameters=[{'laser_scan_topic': '/scan_filtered', 'odom_topic': '/odom_rf2o', 'publish_tf': False,
                          'base_frame_id': 'base_link', 'odom_frame_id': 'odom', 'init_pose_from_topic': '',
                          'freq': 10.0}]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[os.path.join(cfg, 'ekf_nav.yaml'), {'publish_tf': False}],
             remappings=[('odometry/filtered', '/odom')]),
    ]
