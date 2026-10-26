"""FALLBACK for phase 2: use instead of laptop_mapping.launch.py when Cartographer had to be restarted.
Pure localization on the map saved at the end of phase 1, with the robot placed back on the START pose
(map origin), then laptop_mission.launch.py on top as usual.

    ros2 launch robot_slam laptop_localization.launch.py pbstream:=~/hardleel_maps/<name>.pbstream
        [initial_x:=0 initial_y:=0 initial_yaw:=0]   # only if the robot is NOT on the start pose
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from robot_slam import stack_common as sc


def truthy(context, name):
    return context.launch_configurations[name].lower() in ('1', 'true', 'yes')


def setup(context):
    lc = context.launch_configurations
    sim = truthy(context, 'sim')
    cfg = os.path.join(sc.pkg_share(), 'config')
    pbstream = os.path.expanduser(lc['pbstream'])
    if not os.path.isfile(pbstream):
        raise RuntimeError(f'pbstream:=<saved map>.pbstream needed (got "{pbstream}")')
    pose = (float(lc['initial_x']), float(lc['initial_y']), float(lc['initial_yaw']))
    nav2_params = sc.nav2_params_for_distro(os.path.join(cfg, 'nav2_params_mission.yaml'), sim)
    actions = [] if sim else sc.transforms()
    actions += sc.sensor_prep(sim, sc.loc_options(context)["laser_filter"])
    opts = sc.loc_options(context)                    # final-stack localization switches, all off by default
    actions += sc.cartographer(localization=True, pbstream=pbstream, use_sim_time=sim, initial_pose=pose, opts=opts)
    actions += sc.loc_v2(opts, sim, localization=True)
    actions += sc.safety_chain('mission', nav2_params, sim)
    if truthy(context, 'rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                            arguments=['-d', os.path.join(cfg, 'brain.rviz')], parameters=[{'use_sim_time': sim}]))
    if truthy(context, 'record_bag'):
        actions += sc.record_bag(lc['bag_dir'], 'localization')
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pbstream', default_value=''),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('bag_dir', default_value='~/hardleel_bags'),
        *sc.loc_launch_args(),
        OpaqueFunction(function=setup),
    ])
