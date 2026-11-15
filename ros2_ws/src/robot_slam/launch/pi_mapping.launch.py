"""Mapping stack on the PI in one launch: IMU -> (after its gyro calibration) lidar + Cartographer,
plus the ESP32 motor bridge and the tuning helper. Laptop side: laptop_mapping.launch.py.

usage (on the Pi, ROS settings from ~/.bashrc):
  ros2 launch robot_slam pi_mapping.launch.py              manual mapping (teleop)
  ros2 launch robot_slam pi_mapping.launch.py auto:=true   auto mapping in the maze: IMU without EKF, rf2o/EKF
                                                           and the maze explorer on the Pi (laptop: hmap only;
                                                           Start in the tuning GUI Auto tab - NOT hauto/Nav2)
Keep the robot still until the IMU logs "Publishing." (gyro calibration). Ctrl+C stops everything.
Same nodes as start_robot.sh (which runs them in tmux windows instead). Values saved in the tuning GUI
are loaded; last run's unsaved Cartographer "Apply" values are dropped.
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from robot_slam import tuning_store as ts


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('robot_slam'), 'launch')
    auto = LaunchConfiguration('auto')
    if os.path.exists(ts.SESSION_CARTO):          # a new run starts without last run's unsaved values
        os.remove(ts.SESSION_CARTO)

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'imu.launch.py')),
        launch_arguments={'use_ekf': PythonExpression(["'false' if '", auto, "' == 'true' else 'true'"])}.items())

    # Started only after the IMU has calibrated (imu_serial_node logs "... Publishing.")
    after_imu = [
        LogInfo(msg='IMU publishing - starting lidar + Cartographer'),
        ExecuteProcess(cmd=['bash', os.path.expanduser('~/lidar_supervisor.sh')], name='lidar_supervisor',
                       output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'carto_mapping.launch.py')),
            launch_arguments={'use_lidar': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'pi_odom.launch.py')),
            condition=IfCondition(auto)),
        # auto mode: maze explorer (right-hand wall follower) on the Pi, starts DISABLED (Start in the GUI)
        Node(package='robot_slam', executable='maze_explorer', name='maze_explorer', output='screen',
             parameters=ts.with_saved(), condition=IfCondition(auto)),
    ]
    started = {'done': False}

    def on_imu_output(event):
        if not started['done'] and b'Publishing.' in event.text:
            started['done'] = True
            return after_imu
        return None

    return LaunchDescription([
        DeclareLaunchArgument('auto', default_value='false',
                              description='true = auto mapping (rf2o + EKF on the Pi, IMU without EKF)'),
        imu,
        # ROS logs go to stderr; check both streams of every process for the IMU's "Publishing."
        RegisterEventHandler(OnProcessIO(on_stdout=on_imu_output, on_stderr=on_imu_output)),

        Node(package='motor_teleop', executable='esp32_teleop_bridge', name='esp32_teleop_bridge',
             output='screen', parameters=ts.with_saved()),
        Node(package='robot_slam', executable='tuning_helper', name='tuning_helper', output='screen'),
    ])
