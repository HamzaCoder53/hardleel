"""Mission stack: Cartographer localization on a saved map + Nav2 + the safe command chain + coverage mission.

    ros2 launch robot_slam mission_stack.launch.py pbstream:=/home/hardleel/maps/<name>.pbstream
    ros2 launch robot_slam mission_stack.launch.py pbstream:=... run_mission:=false   # bring up only, no driving

Collision profile: mission (stop 0.40 m, slowdown 0.52 m); Nav2 params: nav2_params_mission.yaml (0.30 m/s,
inflation 0.45 m). coverage_mission starts only when run_mission:=true (the default is false so that
nothing drives the robot unless asked); it needs Nav2 to be up and sends goals to navigate_to_pose.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_slam import stack_common as sc


def generate_launch_description():
    cfg = os.path.join(sc.pkg_share(), 'config')
    nav2_params = os.path.join(cfg, 'nav2_params_mission.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('pbstream', description='Cartographer state saved while mapping'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_port', default_value=sc.find_nano()),
        DeclareLaunchArgument('motor_bridge', default_value='false'),
        DeclareLaunchArgument('pico_port', default_value='/dev/pico'),
        DeclareLaunchArgument('run_mission', default_value='false'),
        *sc.lidar(LaunchConfiguration('lidar_port')),
        *sc.imu_and_prep(LaunchConfiguration('imu_port')),
        *sc.cartographer(localization=True, pbstream=LaunchConfiguration('pbstream')),
        *sc.odom_for_nav2(),
        *sc.safety_chain('mission', nav2_params),
        *sc.nav2_nodes(nav2_params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sc.pkg_share('motor_teleop'), 'launch', 'motor_bridge.launch.py')),
            launch_arguments={'port': LaunchConfiguration('pico_port')}.items(),
            condition=IfCondition(LaunchConfiguration('motor_bridge'))),
        # Give Nav2 time to configure and activate before the first goal.
        TimerAction(period=20.0, condition=IfCondition(LaunchConfiguration('run_mission')), actions=[
            Node(package='robot_slam', executable='coverage_mission', name='coverage_mission', output='screen',
                 parameters=[os.path.join(cfg, 'coverage_mission.yaml'), {'regime': 'mission'}]),
        ]),
    ])
