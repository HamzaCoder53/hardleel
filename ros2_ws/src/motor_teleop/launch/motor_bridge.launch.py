from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('max_pwm', default_value='255'),
        Node(package='motor_teleop', executable='motor_bridge', name='motor_bridge',
             output='screen',
             parameters=[{
                 'port': LaunchConfiguration('port'),
                 'send_rate': 10.0,   # the old rate; 50 Hz used 70 % of a Pi core and starved Cartographer
                 'max_pwm': ParameterValue(LaunchConfiguration('max_pwm'), value_type=int),
             }]),
    ])
