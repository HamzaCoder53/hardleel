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


def lidar(lidar_port, scan_mode='Standard'):          # NOT Boost: it timed out on the robot (26 Oct)
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


def sensor_prep(use_sim_time=False, laser_filter=False):
    """/scan -> /scan_filtered (tilt gate; laser_filter: + range / footprint filter, final stack item 7),
    /imu/data_raw + /imu/data -> /imu/data_lpf.
    In the sim there is no Pi, so the Madgwick filter that makes /imu/data runs here too."""
    nodes = [Node(package='robot_slam', executable='sensor_prep', name='sensor_prep', output='screen',
                  parameters=[{'lidar_height': LASER_Z, 'use_sim_time': use_sim_time, 'laser_filter': laser_filter,
                               'laser_xy': [LASER_X, LASER_Y]}])]
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


# Final-stack localization switches (all OFF by default; laptop launch args of the same name, loc_v2:=true = all).
LOC_SWITCHES = ('scan_gate', 'imu_bias', 'wall_heading', 'carto_odom', 'carto_stationary', 'loc_monitor')


def loc_options(context):
    """Launch configurations -> {switch: bool}. loc_v2:=true turns every switch on."""
    lc = context.launch_configurations
    on = lambda n: str(lc.get(n, 'false')).lower() in ('1', 'true', 'yes', 'on')
    allv2 = on('loc_v2')
    # carto_odom is NOT part of loc_v2: Cartographer waits for /odom_loc, the EKF for rf2o, and rf2o for the odom
    # frame only Cartographer publishes -> nothing starts (sim 26 Oct). Separate switch until that loop is broken.
    opts = {k: (allv2 and k != 'carto_odom') or on(k) for k in LOC_SWITCHES}
    opts['apply_corrections'] = on('loc_apply_corrections')
    opts['laser_filter'] = on('laser_filter')
    return opts


def loc_launch_args():
    from launch.actions import DeclareLaunchArgument
    return [DeclareLaunchArgument('loc_v2', default_value='false', description='NEW, off: all localization v2 parts'),
            *[DeclareLaunchArgument(k, default_value='false', description='NEW, off (final stack item 1)')
              for k in LOC_SWITCHES],
            DeclareLaunchArgument('loc_apply_corrections', default_value='false',
                                  description='NEW, off: loc_monitor relocalizes Cartographer (else it only logs)'),
            DeclareLaunchArgument('laser_filter', default_value='false',
                                  description='NEW, off: drop returns < 0.15 m, > 7 m and inside the robot body')]


def carto_config_dir(opts):
    """Config dir for Cartographer: the package's, or a temp copy with the v2 changes patched in."""
    cfg = os.path.join(pkg_share(), 'config')
    if not opts or not (opts.get('carto_odom') or opts.get('carto_stationary')):
        return cfg
    import shutil
    import tempfile
    out = tempfile.mkdtemp(prefix='carto_v2_')
    for f in ('cartographer_2d.lua', 'cartographer_2d_localization.lua'):
        shutil.copy(os.path.join(cfg, f), out)
    path = os.path.join(out, 'cartographer_2d.lua')
    text = open(path).read()
    if opts.get('carto_odom'):                       # rf2o + gyro + wall-heading EKF (/odom_loc), never PWM
        text = text.replace('use_odometry = false,', 'use_odometry = true,')
    if opts.get('carto_stationary'):                 # still scans inserted during stops (checkpoints, turns)
        text = text.replace('motion_filter.max_time_seconds = 2.0', 'motion_filter.max_time_seconds = 1.0')
    open(path, 'w').write(text)
    return out


def cartographer(localization=False, pbstream='', use_sim_time=False, initial_pose=(0.0, 0.0, 0.0), opts=None):
    """Cartographer on /scan_filtered + /imu/data (v2 switches: /scan_mapping from scan_gate, /imu/data_v2 from
    imu_bias, /odom_loc odometry). Localization mode loads a frozen .pbstream and starts the
    live trajectory AT `initial_pose` (x, y, yaw in the saved map; the start pose is (0, 0, 0) because mapping
    began there), so it does not have to search the whole map to find itself."""
    opts = opts or {}
    cfg = carto_config_dir(opts)
    basename = 'cartographer_2d_localization.lua' if localization else 'cartographer_2d.lua'
    args = ['-configuration_directory', cfg, '-configuration_basename', basename]
    if localization:
        args += ['-load_state_filename', pbstream, '-load_frozen_state=true',
                 '-start_trajectory_with_default_topics=false']
    remaps = [('scan', '/scan_mapping' if opts.get('scan_gate') else '/scan_filtered'),
              # raw (boot-bias-corrected) gyro + accel: Cartographer never used the Madgwick orientation, and the
              # team dropped Madgwick on 26 Oct
              ('imu', '/imu/data_v2' if opts.get('imu_bias') else '/imu/data_raw')]
    if opts.get('carto_odom'):
        remaps.append(('odom', '/odom_loc'))
    nodes = [
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
             output='screen', parameters=[{'use_sim_time': use_sim_time}], arguments=args,
             remappings=remaps),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='occupancy_grid_node', output='screen', parameters=[{'use_sim_time': use_sim_time}],
             arguments=['-resolution', '0.05', '-publish_period_sec', '0.5']),
        # /map -> /map_nav for Nav2's global costmap, frozen at /mission/start (no ghost obstacles)
        Node(package='robot_slam', executable='map_freezer', name='map_freezer', output='log',
             parameters=[{'use_sim_time': use_sim_time}]),
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


def loc_v2(opts, use_sim_time=False, localization=False):
    """Final-stack localization nodes for the switches that are on (laptop only; nothing runs on the Pi)."""
    if not opts or not any(opts.get(k) for k in LOC_SWITCHES):
        return []
    cfg = os.path.join(pkg_share(), 'config')
    sim = {'use_sim_time': use_sim_time}
    nodes = []
    if opts.get('scan_gate'):
        nodes.append(Node(package='robot_slam', executable='scan_gate', name='scan_gate', output='screen',
                          parameters=[sim]))
    if opts.get('imu_bias'):
        nodes += [Node(package='robot_slam', executable='imu_bias', name='imu_bias', output='screen', parameters=[sim]),
                  Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_madgwick_v2',
                       output='log', parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu',
                                                  'gain': 0.02, **sim}],
                       remappings=[('imu/data_raw', '/imu/data_raw_unbiased'), ('imu/data', '/imu/data_v2')])]
    need_odom = opts.get('wall_heading') or opts.get('carto_odom') or opts.get('loc_monitor')
    if need_odom:                                    # own rf2o + EKF: also works in manual mapping (no Nav2 chain)
        nodes += [
            Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node', name='rf2o_loc', output='log',
                 parameters=[{'laser_scan_topic': '/scan_filtered', 'odom_topic': '/odom_rf2o_loc', 'publish_tf': False,
                              'base_frame_id': 'base_link', 'odom_frame_id': 'odom', 'init_pose_from_topic': '',
                              'freq': 10.0, **sim}]),
            Node(package='robot_localization', executable='ekf_node', name='ekf_loc', output='log',
                 parameters=[os.path.join(cfg, 'ekf_nav_v2.yaml'),
                             {'imu0': '/imu/data_raw_unbiased' if opts.get('imu_bias') else '/imu/data_raw', **sim}],
                 remappings=[('odometry/filtered', '/odom_loc')])]
    if opts.get('wall_heading'):
        nodes.append(Node(package='robot_slam', executable='wall_heading', name='wall_heading', output='screen',
                          parameters=[{'odom_topic': '/odom_loc', **sim}]))
    if opts.get('loc_monitor'):
        cdir = carto_config_dir(opts)
        nodes.append(Node(package='robot_slam', executable='loc_monitor', name='loc_monitor', output='screen',
                          parameters=[{'apply_corrections': bool(opts.get('apply_corrections')),
                                       'carto_config_dir': cdir,
                                       'carto_config_basename': 'cartographer_2d_localization.lua' if localization
                                       else 'cartographer_2d.lua', **sim}],
                          remappings=[('/odom_rf2o', '/odom_rf2o_loc')]))
    return nodes


def record_bag(bag_dir, name, topics=None):
    """ros2 bag record (mcap) of everything needed to rebuild the map / tune offline."""
    import time
    topics = topics or ['/scan', '/scan_filtered', '/imu/data_raw', '/imu/data', '/imu/data_lpf', '/imu/calib_status',
                        '/tf', '/tf_static', '/cmd_vel', '/cmd_vel_teleop', '/cmd_vel_nav', '/odom',
                        '/mission/state', '/mission/event', '/collision_monitor_state', '/classifier/result',
                        '/motor/pwm_state', '/motor/speed_scale', '/exploration/state',   # pwm_state: speed fit
                        # final-stack localization (only present when the switches are on)
                        '/scan_gate/state', '/imu/bias', '/imu/data_v2', '/wall_heading/imu', '/wall_heading/state',
                        '/odom_loc', '/odom_rf2o_loc', '/localization/confidence', '/localization/state',
                        '/localization/event', '/localization/recovering', '/tracked_pose', '/map_nav']
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
                    if isinstance(v, dict) and v.get('type') == 'velocity_polygon':   # no such type on Humble:
                        first = v['velocity_polygons'][0]                            # use the forward box
                        v['points'] = v[first]['points']
                        v['type'] = 'polygon'
                        for k in v.pop('velocity_polygons'):
                            v.pop(k, None)
                        v.pop('holonomic', None)
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
             # bond OFF (0.0): even with 10 s the bond over rmw_zenoh failed, the manager deactivated the collision
             # monitor and crashed -> teleop silently did nothing (real robot, 26 Oct, twice). Autostart still
             # configures + activates both nodes; nothing deactivates them afterwards.
             parameters=[{'autostart': True, 'node_names': ['velocity_smoother', monitor], 'bond_timeout': 0.0,
                          'attempt_respawn_reconnection': True, 'bond_respawn_max_duration': 15.0, **sim}]),
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
    extra = {n: {} for n in nav_names}
    if os.environ.get('ROS_DISTRO', 'jazzy') != 'humble':      # BT.CPP v4 tree; Humble keeps the default one
        extra['bt_navigator'] = {'default_nav_to_pose_bt_xml':
                                 os.path.join(pkg_share(), 'config', 'bt_mission_nav_to_pose.xml')}
    nodes = [Node(package=pkgs[n], executable=n, name=n, output='screen', parameters=[nav2_params, sim, extra[n]],
                  remappings=TF_REMAP + remaps.get(n, [])) for n in nav_names]
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='lifecycle_manager_navigation', output='screen',
                      parameters=[{'autostart': True, 'node_names': nav_names, 'bond_timeout': 0.0,   # bond off: fails over zenoh (26 Oct)
                                   'service_timeout': 30.0,          # bring-up under load timed out in the sim
                                   'attempt_respawn_reconnection': True, 'bond_respawn_max_duration': 15.0, **sim}]))
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
