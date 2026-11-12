"""Full mapping stack: LiDAR + IMU + sensor_prep + Cartographer + the safe command chain.

    ros2 launch robot_slam mapping_stack.launch.py                    # robot stays still until commanded
    ros2 launch robot_slam mapping_stack.launch.py motor_bridge:=true # also start the motor bridge

Drive by publishing to /cmd_vel_teleop (the highest priority input), for example
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop -p speed:=0.15
Collision profile: mapping (stop 0.30 m, slowdown 0.42 m). The guardian node is not part of this stack.
This launch file starts nothing that moves the wheels unless motor_bridge:=true and a command arrives.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from robot_slam import stack_common as sc


def generate_launch_description():
    nav2_params = os.path.join(sc.pkg_share(), 'config', 'nav2_params.yaml')      # mapping variant
    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_port', default_value=sc.find_nano()),
        DeclareLaunchArgument('motor_bridge', default_value='false'),
        DeclareLaunchArgument('pico_port', default_value='/dev/pico'),
        *sc.lidar(LaunchConfiguration('lidar_port')),
        *sc.imu_and_prep(LaunchConfiguration('imu_port')),
        *sc.cartographer(localization=False),
        *sc.safety_chain('mapping', nav2_params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sc.pkg_share('motor_teleop'), 'launch', 'motor_bridge.launch.py')),
            launch_arguments={'port': LaunchConfiguration('pico_port')}.items(),
            condition=IfCondition(LaunchConfiguration('motor_bridge'))),
    ])
