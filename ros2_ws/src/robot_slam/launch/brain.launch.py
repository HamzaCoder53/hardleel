"""Laptop side of the laptop-brain split: everything except the hardware drivers.

The Pi runs robot_hw.launch.py + lidar_supervisor.sh (scripts/pi/robot_hw.sh) and streams /scan, /imu/*, the
camera, and takes /cmd_vel. This file runs on the laptop (WSL, ROS 2 Humble) on the same ROS_DOMAIN_ID=7;
start it with scripts/laptop/brain.sh, which sets up CycloneDDS and the clock check.

    brain.launch.py mode:=mapping                         # Cartographer mapping + safe teleop chain
    brain.launch.py mode:=mission pbstream:=<map>.pbstream                     # localize + Nav2, robot still
    brain.launch.py mode:=mission pbstream:=<map>.pbstream run_mission:=true   # + competition mission node
    ... sim:=true                                         # against rdj2026_sim (use_sim_time, sim owns body TFs)

Drive by hand (mapping): teleop_twist_keyboard remapped to /cmd_vel_teleop.
Mission: waits for `ros2 topic pub --once /mission/start std_msgs/msg/Empty {}` unless auto_start:=true.
Every run records a bag (record_bag:=false to skip) so maps can be rebuilt offline.
"""
import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch_ros.actions import Node

from robot_slam import stack_common as sc

BAG_TOPICS = ['/scan', '/scan_filtered', '/imu/data_raw', '/imu/data', '/imu/data_lpf', '/cmd_vel',
              '/cmd_vel_teleop', '/cmd_vel_nav', '/odom', '/odom_rf2o', '/tf', '/tf_static', '/mission/state']


def truthy(context, name):
    return context.launch_configurations[name].lower() in ('1', 'true', 'yes')


def setup(context):
    mode = context.launch_configurations['mode']
    sim = truthy(context, 'sim')
    cfg = os.path.join(sc.pkg_share(), 'config')
    actions = [] if sim else sc.transforms()     # the sim publishes the body TFs itself
    actions += sc.sensor_prep(sim)

    if mode == 'mapping':
        nav2_params = sc.nav2_params_for_distro(os.path.join(cfg, 'nav2_params.yaml'), sim)
        actions += sc.cartographer(localization=False, use_sim_time=sim)
        actions += sc.safety_chain('mapping', nav2_params, sim)
    elif mode == 'mission':
        pbstream = os.path.expanduser(context.launch_configurations['pbstream'])
        if not os.path.isfile(pbstream):
            raise RuntimeError(f'mode:=mission needs pbstream:=<saved map>.pbstream (got "{pbstream}")')
        nav2_params = sc.nav2_params_for_distro(os.path.join(cfg, 'nav2_params_mission.yaml'), sim)
        actions += sc.cartographer(localization=True, pbstream=pbstream, use_sim_time=sim)
        actions += sc.odom_for_nav2(sim)
        actions += sc.safety_chain('mission', nav2_params, sim)
        actions += sc.nav2_nodes(nav2_params, sim)
        if truthy(context, 'run_mission'):
            # give Nav2 time to configure and activate before the mission sends goals
            actions.append(TimerAction(period=20.0, actions=[Node(
                package='robot_slam', executable='competition_mission', name='competition_mission',
                output='screen',
                parameters=[os.path.join(cfg, 'competition_mission.yaml'),
                            {'use_sim_time': sim, 'auto_start': truthy(context, 'auto_start')}],
                remappings=[('scan', '/scan_filtered')])]))
    else:
        raise RuntimeError(f'mode must be mapping or mission, not "{mode}"')

    if truthy(context, 'rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                            arguments=['-d', os.path.join(cfg, 'brain.rviz')],
                            parameters=[{'use_sim_time': sim}]))
    if truthy(context, 'record_bag'):
        bag_dir = os.path.expanduser(context.launch_configurations['bag_dir'])
        os.makedirs(bag_dir, exist_ok=True)
        name = os.path.join(bag_dir, time.strftime(f'{mode}_%Y%m%d_%H%M%S'))
        actions.append(ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', name] + BAG_TOPICS, output='log'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', description='mapping | mission'),
        DeclareLaunchArgument('pbstream', default_value='', description='saved map for mode:=mission'),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('run_mission', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('bag_dir', default_value='~/hardleel_bags'),
        OpaqueFunction(function=setup),
    ])
