"""RDJ 2026 game field + simulated hardleel in Gazebo (Ignition) Fortress.

    ros2 launch rdj2026_sim sim.launch.py            # field, robot at START, gate cycling
    ros2 launch rdj2026_sim sim.launch.py gui:=false # headless (faster on WSL)
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rdj2026_sim')
    world = os.path.join(pkg, 'worlds', 'rdj2026.sdf')
    urdf = open(os.path.join(pkg, 'urdf', 'hardleel_sim.urdf')).read()
    layout = yaml.safe_load(open(os.path.join(pkg, 'arena', 'arena_layout.yaml')))
    sx, sy, syaw = layout['start']['pose']
    gate = layout['gate']
    gx = (gate['from'][0] + gate['to'][0]) / 2000.0
    gy = (gate['from'][1] + gate['to'][1]) / 2000.0

    gui = LaunchConfiguration('gui')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo window'),
        DeclareLaunchArgument('gate', default_value='true', description='Run the time-based gate'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', pkg),

        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world],
            condition=IfCondition(gui), output='screen'),
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering', world],
            condition=IfCondition(PythonExpression(["'", gui, "' != 'true'"])), output='screen'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}], output='screen'),

        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml'), 'use_sim_time': True}],
             output='screen'),

        TimerAction(period=4.0, actions=[
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'rdj2026', '-name', 'hardleel', '-topic', 'robot_description',
                            '-x', str(sx / 1000.0), '-y', str(sy / 1000.0), '-z', '0.04',
                            '-Y', str(math.radians(syaw))]),
        ]),

        TimerAction(period=8.0, actions=[
            Node(package='rdj2026_sim', executable='gate_controller.py', output='screen',
                 condition=IfCondition(LaunchConfiguration('gate')),
                 parameters=[{'use_sim_time': True, 'x': gx, 'y': gy,
                              'height': gate['height'] / 1000.0,
                              'open_s': float(gate['open_s']), 'closed_s': float(gate['closed_s'])}]),
        ]),
    ])
