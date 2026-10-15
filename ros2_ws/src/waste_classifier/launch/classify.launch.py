import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    models = os.path.join(get_package_share_directory('waste_classifier'), 'models')
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=os.path.join(models, 'waste_model.tflite')),
        DeclareLaunchArgument('labels_path', default_value=os.path.join(models, 'labels.txt')),
        DeclareLaunchArgument('backend', default_value='auto'),       # auto | picamera2 | rpicam
        DeclareLaunchArgument('settle_time', default_value='1.0'),
        DeclareLaunchArgument('save_dir', default_value=''),          # e.g. /home/pi/captures
        Node(
            package='waste_classifier',
            executable='classify_service',
            name='waste_classifier_service',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'labels_path': LaunchConfiguration('labels_path'),
                'backend': LaunchConfiguration('backend'),
                'settle_time': LaunchConfiguration('settle_time'),
                'save_dir': LaunchConfiguration('save_dir'),
            }],
        ),
    ])
