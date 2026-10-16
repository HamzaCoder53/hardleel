import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    models = os.path.join(get_package_share_directory('waste_classifier'), 'models')
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=os.path.join(models, 'waste_model.tflite')),
        DeclareLaunchArgument('labels_path', default_value=os.path.join(models, 'labels.txt')),
        DeclareLaunchArgument('min_conf', default_value='0.75'),
        DeclareLaunchArgument('min_margin', default_value='0.25'),
        DeclareLaunchArgument('save_dir', default_value=''),          # e.g. /home/hardleel/captures
        Node(
            package='waste_classifier',
            executable='classify_service',
            name='waste_classifier',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'labels_path': LaunchConfiguration('labels_path'),
                'min_conf': ParameterValue(LaunchConfiguration('min_conf'), value_type=float),
                'min_margin': ParameterValue(LaunchConfiguration('min_margin'), value_type=float),
                'save_dir': LaunchConfiguration('save_dir'),
            }],
        ),
    ])
