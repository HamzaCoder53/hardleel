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
    Save the map at the end of phase 1: scripts/laptop/save_map.sh <name>
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from robot_slam import stack_common as sc


def truthy(context, name):
    return context.launch_configurations[name].lower() in ('1', 'true', 'yes')


def setup(context):
    sim = truthy(context, 'sim')
    cfg = os.path.join(sc.pkg_share(), 'config')
    nav2_params = sc.nav2_params_for_distro(os.path.join(cfg, 'nav2_params_mission.yaml'), sim)
    actions = [] if sim else sc.transforms()          # the sim publishes the body TFs itself
    actions += sc.sensor_prep(sim)
    actions += sc.cartographer(localization=False, use_sim_time=sim)
    actions += sc.safety_chain('mapping', nav2_params, sim)
    if truthy(context, 'rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                            arguments=['-d', os.path.join(cfg, 'brain.rviz')], parameters=[{'use_sim_time': sim}]))
    if truthy(context, 'record_bag'):
        actions += sc.record_bag(context.launch_configurations['bag_dir'], 'mapping')
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('bag_dir', default_value='~/hardleel_bags'),
        OpaqueFunction(function=setup),
    ])
