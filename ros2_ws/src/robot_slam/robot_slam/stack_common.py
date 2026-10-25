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
CAM_X, CAM_Y, CAM_Z = 0.10, 0.0, 0.20   # Pi Camera Module 3, looking forward (nominal; measure on the robot)

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


def transforms():
    """Static TFs of the robot body. One owner only: the machine that runs the consumers (brain.launch.py on
    the laptop, or the single-machine stacks below)."""
    return [
        static_tf(LASER_X, LASER_Y, LASER_Z, LASER_YAW, 'base_link', 'laser', 'base_to_laser'),
        static_tf(IMU_X, IMU_Y, IMU_Z, 0.0, 'base_link', 'imu_link', 'base_to_imu'),
        static_tf(CAM_X, CAM_Y, CAM_Z, 0.0, 'base_link', 'camera', 'base_to_camera'),
    ]


def lidar(lidar_port, scan_mode='Boost'):
    """RPLIDAR A1 driver + body TFs. A1 firmware 1.29 offers Standard (~360 points/rev), Express (~730)
    and Boost (~1450); on the Pi prefer scripts/pi/lidar_supervisor.sh, which also retries."""
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')),
            launch_arguments={'serial_port': lidar_port, 'serial_baudrate': '115200', 'frame_id': 'laser',
                              'angle_compensate': 'true', 'scan_mode': scan_mode}.items()),
        *transforms(),
    ]


def imu_hw(imu_port):
    """Nano IMU -> /imu/data_raw -> Madgwick -> /imu/data."""
    return [
        Node(package='robot_slam', executable='imu_serial_node', name='imu_serial_node', output='screen',
             parameters=[{'port': imu_port, 'frame_id': 'imu_link', 'gyro_calib_s': 5.0}]),
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter_madgwick_node', output='screen',
             parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu', 'gain': 0.02}],
             remappings=[('imu/data_raw', '/imu/data_raw'), ('imu/data', '/imu/data')]),
    ]


def sensor_prep(use_sim_time=False):
    """/scan -> /scan_filtered (tilt gate), /imu/data_raw + /imu/data -> /imu/data_lpf.
    In the sim there is no Pi, so the Madgwick filter that makes /imu/data runs here too."""
    nodes = [Node(package='robot_slam', executable='sensor_prep', name='sensor_prep', output='screen',
                  parameters=[{'lidar_height': LASER_Z, 'use_sim_time': use_sim_time}])]
    if use_sim_time:
        nodes.append(Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
                          name='imu_filter_madgwick_node', output='log',
                          parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu', 'gain': 0.02,
                                       'use_sim_time': True}],
                          remappings=[('imu/data_raw', '/imu/data_raw'), ('imu/data', '/imu/data')]))
    return nodes


def imu_and_prep(imu_port):
    """Nano IMU -> /imu/data_raw -> Madgwick -> /imu/data, then sensor_prep (/scan_filtered, /imu/data_lpf)."""
    return [*imu_hw(imu_port), *sensor_prep()]


def cartographer(localization=False, pbstream='', use_sim_time=False, initial_pose=(0.0, 0.0, 0.0)):
    """Cartographer on /scan_filtered + /imu/data. Localization mode loads a frozen .pbstream and starts the
    live trajectory AT `initial_pose` (x, y, yaw in the saved map; the start pose is (0, 0, 0) because mapping
    began there), so it does not have to search the whole map to find itself."""
    cfg = os.path.join(pkg_share(), 'config')
    basename = 'cartographer_2d_localization.lua' if localization else 'cartographer_2d.lua'
    args = ['-configuration_directory', cfg, '-configuration_basename', basename]
    if localization:
        args += ['-load_state_filename', pbstream, '-load_frozen_state=true',
                 '-start_trajectory_with_default_topics=false']
    nodes = [
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
             output='screen', parameters=[{'use_sim_time': use_sim_time}], arguments=args,
             remappings=[('scan', '/scan_filtered'), ('imu', '/imu/data')]),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='occupancy_grid_node', output='screen', parameters=[{'use_sim_time': use_sim_time}],
             arguments=['-resolution', '0.05', '-publish_period_sec', '0.5']),
    ]
    if localization:
        import math
        from launch.actions import ExecuteProcess, TimerAction
        x, y, yaw = initial_pose
        req = (f"{{configuration_directory: '{cfg}', configuration_basename: '{basename}', use_initial_pose: true, "
               f"initial_pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, "
               f"z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}}}, relative_to_trajectory_id: 0}}")
        nodes.append(TimerAction(period=4.0, actions=[ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/start_trajectory', 'cartographer_ros_msgs/srv/StartTrajectory', req],
            output='screen')]))
    return nodes


def record_bag(bag_dir, name, topics=None):
    """ros2 bag record (mcap) of everything needed to rebuild the map / tune offline."""
    import time
    topics = topics or ['/scan', '/scan_filtered', '/imu/data_raw', '/imu/data', '/imu/data_lpf', '/imu/calib_status',
                        '/tf', '/tf_static', '/cmd_vel', '/cmd_vel_teleop', '/cmd_vel_nav', '/odom',
                        '/mission/state', '/mission/event', '/collision_monitor_state', '/classifier/result']
    from launch.actions import ExecuteProcess
    bag_dir = os.path.expanduser(bag_dir)
    os.makedirs(bag_dir, exist_ok=True)
    out = os.path.join(bag_dir, time.strftime(f'{name}_%Y%m%d_%H%M%S'))
    return [ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', out] + topics, output='log')]


# Nav2 plugin names that Humble (Nav2 1.1) still spells with '/'; Jazzy uses '::'.
HUMBLE_PLUGINS = {
    'nav2_navfn_planner::NavfnPlanner': 'nav2_navfn_planner/NavfnPlanner',
    'nav2_behaviors::Spin': 'nav2_behaviors/Spin',
    'nav2_behaviors::BackUp': 'nav2_behaviors/BackUp',
    'nav2_behaviors::DriveOnHeading': 'nav2_behaviors/DriveOnHeading',
    'nav2_behaviors::Wait': 'nav2_behaviors/Wait',
    'nav2_behaviors::AssistedTeleop': 'nav2_behaviors/AssistedTeleop',
}


def nav2_params_for_distro(path, use_sim_time=False):
    """Return a params file usable on this ROS distro: on Humble the Jazzy-only plugin names are rewritten,
    and use_sim_time is set for every node. The source yaml stays the single (Jazzy-style) source of truth."""
    distro = os.environ.get('ROS_DISTRO', 'jazzy')
    if distro != 'humble' and not use_sim_time:
        return path
    import tempfile
    import yaml
    with open(path) as f:
        params = yaml.safe_load(f)

    def fix(node):
        if isinstance(node, dict):
            return {k: fix(v) for k, v in node.items()}
        if isinstance(node, list):
            return [fix(v) for v in node]
        if distro == 'humble' and isinstance(node, str):
            return HUMBLE_PLUGINS.get(node, node)
        return node

    params = fix(params)
    for section in params.values():
        if isinstance(section, dict) and isinstance(section.get('ros__parameters'), dict):
            rp = section['ros__parameters']
            rp['use_sim_time'] = use_sim_time
            if distro == 'humble':                 # collision_monitor polygons: Humble wants a flat list
                for v in rp.values():
                    if isinstance(v, dict) and isinstance(v.get('points'), str):
                        v['points'] = [float(c) for pt in yaml.safe_load(v['points']) for c in pt]
    out = tempfile.NamedTemporaryFile('w', prefix=f'{os.path.basename(path)[:-5]}_{distro}_', suffix='.yaml',
                                      delete=False)
    yaml.safe_dump(params, out)
    out.close()
    return out.name


def safety_chain(profile, nav2_params, use_sim_time=False):
    """twist_mux -> velocity_smoother -> collision_monitor. One monitor config for mapping and mission
    (`profile` is kept for the older launch files and ignored): the node outlives the mapping phase."""
    cfg = os.path.join(pkg_share(), 'config')
    monitor = 'collision_monitor'
    sim = {'use_sim_time': use_sim_time}
    monitor_params = nav2_params_for_distro(os.path.join(cfg, 'collision_monitor.yaml'), use_sim_time)
    return [
        Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
             parameters=[os.path.join(cfg, 'twist_mux.yaml'), sim],
             remappings=[('cmd_vel_out', 'cmd_vel_mux')]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='screen', parameters=[nav2_params, sim],
             remappings=TF_REMAP + [('cmd_vel', 'cmd_vel_mux'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name=monitor, output='screen',
             parameters=[monitor_params, sim], remappings=TF_REMAP),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_safety',
             output='screen',
             parameters=[{'autostart': True, 'node_names': ['velocity_smoother', monitor], **sim}]),
    ]


def nav2_nodes(nav2_params, use_sim_time=False):
    """Nav2 navigation servers. Controller and behaviors publish /cmd_vel_nav (into twist_mux)."""
    nav_names = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
                 'bt_navigator', 'waypoint_follower']
    remaps = {'controller_server': [('cmd_vel', 'cmd_vel_nav')], 'behavior_server': [('cmd_vel', 'cmd_vel_nav')]}
    pkgs = {'controller_server': 'nav2_controller', 'smoother_server': 'nav2_smoother',
            'planner_server': 'nav2_planner', 'behavior_server': 'nav2_behaviors',
            'bt_navigator': 'nav2_bt_navigator', 'waypoint_follower': 'nav2_waypoint_follower'}
    sim = {'use_sim_time': use_sim_time}
    nodes = [Node(package=pkgs[n], executable=n, name=n, output='screen', parameters=[nav2_params, sim],
                  remappings=TF_REMAP + remaps.get(n, [])) for n in nav_names]
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='lifecycle_manager_navigation', output='screen',
                      parameters=[{'autostart': True, 'node_names': nav_names, 'bond_timeout': 10.0, **sim}]))
    return nodes


def odom_for_nav2(use_sim_time=False):
    """/odom for Nav2 from rf2o + gyro EKF. TF publishing is off: Cartographer owns odom -> base_link."""
    cfg = os.path.join(pkg_share(), 'config')
    return [
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node', name='rf2o_laser_odometry',
             output='screen',
             parameters=[{'laser_scan_topic': '/scan_filtered', 'odom_topic': '/odom_rf2o', 'publish_tf': False,
                          'base_frame_id': 'base_link', 'odom_frame_id': 'odom', 'init_pose_from_topic': '',
                          'freq': 10.0, 'use_sim_time': use_sim_time}]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[os.path.join(cfg, 'ekf_nav.yaml'), {'publish_tf': False, 'use_sim_time': use_sim_time}],
             remappings=[('odometry/filtered', '/odom')]),
    ]
