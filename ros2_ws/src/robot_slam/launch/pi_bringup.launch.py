"""Pi side of the competition stack: hardware drivers only (no TFs, no EKF - the laptop owns those).

    /imu/data_raw (imu_serial_node, startup gyro calibration with motion check) -> Madgwick -> /imu/data
    /cmd_vel -> motor_bridge -> Pico. Stops the motors after cmd_timeout (0.3 s) without a command, on the Pi,
               so safety never depends on Wi-Fi; the Pico firmware has its own watchdog behind that.
    Pi Camera -> /camera/image_raw/compressed (320x240 @ 5 fps by default)
    /dump (drop_servo_node; does nothing while has_drop_bin is false)
The lidar runs separately under scripts/pi/lidar_supervisor.sh (it needs a reset-and-retry loop).
Start everything with scripts/pi/robot_hw.sh (sets ROS_DOMAIN_ID=7 + CycloneDDS, tmux session "hw").
"""
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def first(patterns, fallback):
    if isinstance(patterns, str):
        patterns = [patterns]
    for pattern in patterns:
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return fallback


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('robot_slam'), 'config')
    L = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('imu_port', default_value=first('/dev/serial/by-id/usb-Arduino*', '/dev/ttyACM1')),
        DeclareLaunchArgument('pico_port', default_value=first([
            '/dev/serial/by-id/*CP210*',
            '/dev/serial/by-id/*CH340*',
            '/dev/serial/by-id/*1a86_USB_Serial*',
            '/dev/serial/by-id/*Silicon_Labs*',
            '/dev/serial/by-id/*Espressif*',
            '/dev/serial/by-id/*USB-Serial*',
            '/dev/serial/by-id/*Raspberry_Pi_Pico*',
        ], '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')),
        DeclareLaunchArgument('cmd_timeout', default_value='0.3'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('camera_fps', default_value='5.0'),
        DeclareLaunchArgument('camera_width', default_value='320'),
        DeclareLaunchArgument('camera_height', default_value='240'),
        DeclareLaunchArgument('camera_rotate_deg', default_value='0'),   # 13:00 25 Oct frame: upright
        DeclareLaunchArgument('motors', default_value='true'),
        DeclareLaunchArgument('drop', default_value='true'),
        DeclareLaunchArgument('yaw_feedback', default_value='false'),   # NEW, off: gyro turn-rate loop in motor_bridge
        DeclareLaunchArgument('record', default_value='false'),         # NEW, off: run_recorder role pi (~/runs/run_*)
        DeclareLaunchArgument('camera_v2', default_value='false'),      # NEW, off: pi_cam/camera_node.py (video+stills)
        DeclareLaunchArgument('ramp_dash', default_value='false'),      # NEW, off: ramp_dash node + bridge dash_input
        DeclareLaunchArgument('video_width', default_value='1920'),     # 1280 x 720 if the soak test shows heat
        DeclareLaunchArgument('video_height', default_value='1080'),

        Node(package='robot_slam', executable='imu_serial_node', name='imu_serial_node', output='screen',
             parameters=[{'port': L('imu_port'), 'frame_id': 'imu_link', 'gyro_calib_s': 3.0}]),
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter_madgwick_node',
             output='screen', parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu', 'gain': 0.02}],
             remappings=[('imu/data_raw', '/imu/data_raw'), ('imu/data', '/imu/data')]),
        # send_rate 10 Hz: 50 Hz used 70 % of a Pi core. The bridge checks the command age every tick, so the
        # motors stop 0.3-0.4 s after the last /cmd_vel.
        Node(package='motor_teleop', executable='motor_bridge', name='motor_bridge', output='screen',
             condition=IfCondition(L('motors')),
             parameters=[{'port': L('pico_port'), 'send_rate': 10.0, 'max_pwm': 255, 'cmd_timeout': L('cmd_timeout'),
                          'yaw_feedback': ParameterValue(L('yaw_feedback'), value_type=bool),
                          'dash_input': ParameterValue(L('ramp_dash'), value_type=bool)}]),
        # final stack item 5: the dash controller runs HERE on the Pi (control loop never crosses Wi-Fi)
        Node(package='robot_slam', executable='ramp_dash', name='ramp_dash', output='screen',
             condition=IfCondition(L('ramp_dash')), parameters=[os.path.join(cfg, 'ramp_dash.yaml')]),
        ExecuteProcess(
            cmd=['python3', os.path.expanduser('~/pi_cam/pi_cam_node.py'), '--ros-args',
                 '-p', ['fps:=', L('camera_fps')], '-p', ['width:=', L('camera_width')],
                 '-p', ['height:=', L('camera_height')], '-p', ['rotate_deg:=', L('camera_rotate_deg')]],
            condition=IfCondition(PythonExpression(["'", L('camera'), "' == 'true' and '", L('camera_v2'),
                                                    "' != 'true'"])), output='screen'),
        # final stack item 3 (OFF): 1080p30 hardware H.264 video + the same lores stream + stills + frame times
        ExecuteProcess(
            cmd=['python3', os.path.expanduser('~/pi_cam/camera_node.py'), '--ros-args',
                 '-p', ['lores_fps:=', L('camera_fps')], '-p', ['lores_width:=', L('camera_width')],
                 '-p', ['lores_height:=', L('camera_height')], '-p', ['rotate_deg:=', L('camera_rotate_deg')],
                 '-p', ['width:=', L('video_width')], '-p', ['height:=', L('video_height')]],
            condition=IfCondition(PythonExpression(["'", L('camera'), "' == 'true' and '", L('camera_v2'),
                                                    "' == 'true'"])), output='screen'),
        Node(package='robot_slam', executable='drop_servo_node', name='drop_servo', output='screen',
             condition=IfCondition(L('drop')), parameters=[os.path.join(cfg, 'drop_servo.yaml')]),
        # final stack: Pi-side recording (lowest priority; stops itself on heat / throttling / < 3 GB disk)
        Node(package='robot_slam', executable='run_recorder', name='run_recorder', output='screen',
             condition=IfCondition(L('record')), parameters=[{'role': 'pi'}]),
    ])
