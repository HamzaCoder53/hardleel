#!/usr/bin/env bash
# create_waste_package.sh - creates the ROS 2 (Jazzy) package "waste_classifier" inside a workspace.
#
# Usage (run it in the folder that holds your files):
#     bash create_waste_package.sh ~/ros2_ws
#
# Files looked for in the CURRENT folder:
#     waste_classifier_service.py   (required)
#     waste_model.tflite, labels.txt (optional - copied into the package's models/ folder)
set -euo pipefail

WS="${1:-$HOME/ros2_ws}"
PKG="waste_classifier"
SRC="$WS/src"
DIR="$SRC/$PKG"

[ -f waste_classifier_service.py ] || { echo "ERROR: waste_classifier_service.py not found in $(pwd)"; exit 1; }
[ -d "$DIR" ] && { echo "ERROR: $DIR already exists - remove or rename it first"; exit 1; }

mkdir -p "$DIR/$PKG" "$DIR/launch" "$DIR/models" "$DIR/resource"
touch "$DIR/resource/$PKG" "$DIR/$PKG/__init__.py"

cp waste_classifier_service.py "$DIR/$PKG/waste_classifier_service.py"
[ -f waste_model.tflite ] && cp waste_model.tflite "$DIR/models/" || echo "note: no waste_model.tflite here - copy it to $DIR/models/ later"
[ -f labels.txt ]         && cp labels.txt         "$DIR/models/" || echo "note: no labels.txt here - copy it to $DIR/models/ later"

cat > "$DIR/package.xml" <<'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>waste_classifier</name>
  <version>0.1.0</version>
  <description>On-demand waste classification service (Pi camera + TFLite): plastic=blue, organic=white, metal=black</description>
  <maintainer email="pi@example.com">pi</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>std_srvs</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

cat > "$DIR/setup.py" <<'EOF'
from glob import glob
from setuptools import find_packages, setup

package_name = 'waste_classifier'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@example.com',
    description='On-demand waste classification service',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'classify_service = waste_classifier.waste_classifier_service:main',
        ],
    },
)
EOF

cat > "$DIR/setup.cfg" <<'EOF'
[develop]
script_dir=$base/lib/waste_classifier
[install]
install_scripts=$base/lib/waste_classifier
EOF

cat > "$DIR/launch/classify.launch.py" <<'EOF'
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
EOF

echo
echo "Package created at: $DIR"
find "$DIR" -type f | sed "s|$WS/||" | sort
echo
echo "Next:  cd $WS && colcon build --packages-select $PKG --symlink-install && source install/setup.bash"
