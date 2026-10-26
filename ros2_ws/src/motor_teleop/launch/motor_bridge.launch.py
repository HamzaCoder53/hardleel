import glob

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def default_esp32_port():
    for pat in ('/dev/serial/by-id/*CP210*', '/dev/serial/by-id/*CH340*',
                '/dev/serial/by-id/*1a86_USB_Serial*', '/dev/serial/by-id/*Silicon_Labs*',
                '/dev/serial/by-id/*Espressif*', '/dev/serial/by-id/*USB-Serial*'):
        found = sorted(glob.glob(pat))
        if found:
            return found[0]
    return '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=default_esp32_port()),
        DeclareLaunchArgument('max_pwm', default_value='255'),
        Node(package='motor_teleop', executable='motor_bridge', name='motor_bridge',
             output='screen',
             parameters=[{
                 'port': LaunchConfiguration('port'),
                 'send_rate': 10.0,   # the old rate; 50 Hz used 70 % of a Pi core and starved Cartographer
                 'max_pwm': ParameterValue(LaunchConfiguration('max_pwm'), value_type=int),
             }]),
    ])
