"""Phase 1 (mapping) on the laptop. Leave it RUNNING into phase 2: laptop_mission.launch.py only adds Nav2,
the classifier, the logger and the mission on top, so Cartographer never restarts and the map origin stays
at the start pose.

    static TFs (base_link -> laser / imu_link / camera)       one owner: this launch
    /scan -> sensor_prep -> /scan_filtered (tilt gate)
    Cartographer SLAM (/scan_filtered + /imu/data) -> map -> odom -> base_link, /map
    teleop (/cmd_vel_teleop, keyboard in another terminal) -> twist_mux -> velocity_smoother -> collision_monitor
        -> /cmd_vel -> Pi
    ros2 bag record (mcap) of scans, IMU, TF, commands -> ~/hardleel_bags (record_bag:=false to skip)

    ros2 launch robot_slam laptop_mapping.launch.py [rviz:=true] [sim:=false]
        [loc_v2:=true | scan_gate:= imu_bias:= wall_heading:= carto_odom:= carto_stationary:= loc_monitor:=]
        (final-stack localization, all OFF by default; loc_apply_corrections:=true lets loc_monitor relocalize)
    Save the map at the end of phase 1: scripts/laptop/save_map.sh <name>
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch_ros.actions import Node

from robot_slam import stack_common as sc


def truthy(context, name):
    return context.launch_configurations[name].lower() in ('1', 'true', 'yes')


def setup(context):
    sim = truthy(context, 'sim')
    cfg = os.path.join(sc.pkg_share(), 'config')
    nav2_params = sc.nav2_params_for_distro(os.path.join(cfg, 'nav2_params_mission.yaml'), sim)
    actions = [] if sim else sc.transforms()          # the sim publishes the body TFs itself
    actions += sc.sensor_prep(sim, sc.loc_options(context)["laser_filter"])
    opts = sc.loc_options(context)                    # final-stack localization switches, all off by default
    actions += sc.cartographer(localization=False, use_sim_time=sim, opts=opts)
    actions += sc.loc_v2(opts, sim)
    actions += sc.safety_chain('mapping', nav2_params, sim)
    mode = context.launch_configurations['mapping_mode']
    if mode == 'auto':
        # OFF by default. Nav2 + frontier exploration that stops by itself (time_limit_s < 180 s). Then start
        # laptop_mission with nav2:=false (Nav2 is already up). auto_calibrate: a short straight speed test
        # (motor_calibrator, wall drive) before exploring; the explorer waits for it.
        cal = truthy(context, 'auto_calibrate')
        actions += ([] if sim else sc.odom_for_nav2(sim)) + sc.nav2_nodes(nav2_params, sim)
        if cal:
            actions.append(Node(package='motor_teleop', executable='motor_calibrator', name='motor_calibrator',
                                output='screen', parameters=[{'use_sim_time': sim}]))
            actions.append(TimerAction(period=20.0, actions=[ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/motor/calibrate_speed', 'std_srvs/srv/Trigger', '{}'],
                output='screen')]))
        actions.append(TimerAction(period=20.0, actions=[Node(
            package='robot_slam', executable='frontier_explorer', name='frontier_explorer', output='screen',
            parameters=[os.path.join(cfg, 'frontier_explorer.yaml'),
                        {'use_sim_time': sim, 'start_delay_s': 25.0 if cal else 2.0}])]))
    elif mode != 'manual':
        raise RuntimeError(f'mapping_mode must be manual or auto, not "{mode}"')
    if truthy(context, 'rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                            arguments=['-d', os.path.join(cfg, 'brain.rviz')], parameters=[{'use_sim_time': sim}]))
    if truthy(context, 'record_run'):                 # final stack: run folders, split by phase (replaces record_bag)
        actions.append(Node(package='robot_slam', executable='run_recorder', name='run_recorder', output='screen',
                            parameters=[{'role': 'laptop', 'use_sim_time': sim}]))
    elif truthy(context, 'record_bag'):
        actions += sc.record_bag(context.launch_configurations['bag_dir'], 'mapping')
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('mapping_mode', default_value='manual', description='manual | auto (frontier, OFF)'),
        DeclareLaunchArgument('auto_calibrate', default_value='false',
                              description='auto mode only: speed calibration drive before exploring'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('bag_dir', default_value='~/hardleel_bags'),
        DeclareLaunchArgument('record_run', default_value='false',
                              description='NEW, off: run_recorder (~/runs/run_*, per phase, MCAP zstd)'),
        *sc.loc_launch_args(),
        OpaqueFunction(function=setup),
    ])
