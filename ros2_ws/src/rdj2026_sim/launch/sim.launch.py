"""RDJ 2026 game field + simulated hardleel in Gazebo: Fortress (`ign gazebo`) on Humble, Harmonic (`gz sim`) on
Jazzy. The world/URDF/bridge files are written with Fortress names; on Jazzy they are rewritten at launch time.

    ros2 launch rdj2026_sim sim.launch.py            # field, robot at START, gate cycling
    ros2 launch rdj2026_sim sim.launch.py gui:=false # headless (faster on WSL)
"""
import math
import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


HARMONIC = os.environ.get('ROS_DISTRO', 'humble') != 'humble'


def for_distro(path, hardware=False):
    """Fortress file as is on Humble; on Jazzy a copy with Harmonic names (gz.msgs, gz_frame_id).
    hardware=True: the diff drive listens to /sim_cmd_vel (fake_pico) instead of /cmd_vel (motor_bridge's)."""
    if not HARMONIC and not hardware:
        return path
    import tempfile
    text = open(path).read()
    if HARMONIC:
        text = text.replace('ignition.msgs.', 'gz.msgs.').replace('ignition_frame_id', 'gz_frame_id')
        text = text.replace('ignition-gazebo-', 'gz-sim-').replace('ignition::gazebo::', 'gz::sim::')
    if hardware:
        text = text.replace('ros_topic_name: /cmd_vel\n', 'ros_topic_name: /sim_cmd_vel\n')
    out = tempfile.NamedTemporaryFile('w', suffix=os.path.splitext(path)[1], delete=False)
    out.write(text)
    out.close()
    return out.name


def generate_launch_description():
    pkg = get_package_share_directory('rdj2026_sim')
    # world:=rdj2026 (rulebook Figure 1) | course (the real course drawn by the team, start (0,0) facing east)
    W = LaunchConfiguration('world')
    world = [pkg, '/worlds/', W, '.sdf']
    if HARMONIC:
        # the worlds use Fortress plugin names; Harmonic needs a rewritten copy (resource path keeps the includes)
        world = PythonExpression(["'", for_distro(os.path.join(pkg, 'worlds', 'rdj2026.sdf')), "' if '", W,
                                  "' == 'rdj2026' else '", for_distro(os.path.join(pkg, 'worlds', 'course.sdf')), "'"])
    urdf = open(for_distro(os.path.join(pkg, 'urdf', 'hardleel_sim.urdf'))).read()
    gz = ['gz', 'sim'] if HARMONIC else ['ign', 'gazebo']
    layout = yaml.safe_load(open(os.path.join(pkg, 'arena', 'arena_layout.yaml')))
    sx, sy, syaw = layout['start']['pose']
    gate = layout['gate']
    gx = (gate['from'][0] + gate['to'][0]) / 2000.0
    gy = (gate['from'][1] + gate['to'][1]) / 2000.0

    gui = LaunchConfiguration('gui')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo window'),
        DeclareLaunchArgument('world', default_value='rdj2026', description='rdj2026 (Figure 1) | course (real course)'),
        DeclareLaunchArgument('hardware', default_value='false',
                              description='true: the REAL motor_bridge drives the robot through fake_pico (motor '
                                          'model with deadband / slow zone); implies no odom TF from the sim'),
        DeclareLaunchArgument('gate', default_value='false' if gate.get('hide') else 'true',
                              description='Run the time-based gate (off when the layout has no gate)'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', pkg),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg),

        ExecuteProcess(
            cmd=gz + ['-r', world],
            condition=IfCondition(gui), output='screen'),
        ExecuteProcess(
            cmd=gz + ['-r', '-s'] + ([] if sys.platform == 'darwin' else ['--headless-rendering']) + [world],
            condition=IfCondition(PythonExpression(["'", gui, "' != 'true'"])), output='screen'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}], output='screen'),

        DeclareLaunchArgument('noisy_odom', default_value='false',
                              description='Replace the perfect odom->base_link TF with a drifting one (no wheel encoders)'),
        DeclareLaunchArgument('odom_tf', default_value='true',
                              description='false: no odom->base_link TF from the sim at all (Cartographer provides it, '
                                          'like the real robot without encoders)'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('noisy_odom'), "' != 'true' and '",
                                                     LaunchConfiguration('odom_tf'), "' == 'true' and '",
                                                     LaunchConfiguration('hardware'), "' != 'true'"])),
             parameters=[{'config_file': for_distro(os.path.join(pkg, 'config', 'bridge.yaml')), 'use_sim_time': True}],
             output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             condition=IfCondition(LaunchConfiguration('hardware')),
             parameters=[{'config_file': for_distro(os.path.join(pkg, 'config', 'bridge_noisy_odom.yaml'), True),
                          'use_sim_time': True}], output='screen'),
        DeclareLaunchArgument('pico_side_noise', default_value='0.0', description='ramp-dash test: per-side slip noise'),
        DeclareLaunchArgument('pico_side_bias', default_value='0.0', description='ramp-dash test: right-left efficiency'),
        DeclareLaunchArgument('pico_bog_zone', default_value='-1,-1,-1,-1', description='x0,y0,x1,y1 (world)'),
        DeclareLaunchArgument('pico_bog_times', default_value='0'),
        Node(package='rdj2026_sim', executable='fake_pico.py', name='fake_pico', output='screen',
             condition=IfCondition(LaunchConfiguration('hardware')),
             parameters=[{'use_sim_time': True, 'slow_zone': [6.0, 1.3, 6.71, 2.4],   # right corridor, map x 1-2.1
                          'side_noise': ParameterValue(LaunchConfiguration('pico_side_noise'), value_type=float),
                          'side_bias': ParameterValue(LaunchConfiguration('pico_side_bias'), value_type=float),
                          'bog_zone': PythonExpression(["[float(v) for v in '", LaunchConfiguration('pico_bog_zone'),
                                                        "'.split(',')]"]),
                          'bog_times': ParameterValue(LaunchConfiguration('pico_bog_times'), value_type=int)}]),
        TimerAction(period=2.0, actions=[Node(
            package='motor_teleop', executable='motor_bridge', name='motor_bridge', output='screen',
            condition=IfCondition(LaunchConfiguration('hardware')),
            parameters=[{'use_sim_time': True, 'port': '/tmp/fake_pico', 'send_rate': 10.0, 'max_pwm': 255,
                         'cmd_timeout': 0.3, 'max_speed': 0.9, 'calib_file': '/tmp/sim_motor_calib.yaml'}])]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             condition=IfCondition(PythonExpression(["('", LaunchConfiguration('noisy_odom'), "' == 'true' or '",
                                                     LaunchConfiguration('odom_tf'), "' != 'true') and '",
                                                     LaunchConfiguration('hardware'), "' != 'true'"])),
             parameters=[{'config_file': for_distro(os.path.join(pkg, 'config', 'bridge_noisy_odom.yaml')), 'use_sim_time': True}],
             output='screen'),
        Node(package='rdj2026_sim', executable='noisy_odom.py', output='screen',
             condition=IfCondition(LaunchConfiguration('noisy_odom')),
             parameters=[{'use_sim_time': True}]),

        TimerAction(period=4.0, actions=[
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'rdj2026', '-name', 'hardleel', '-topic', 'robot_description',
                            '-x', PythonExpression(["'0.06' if '", W, f"' == 'course' else '{sx / 1000.0}'"]),   # course: 6 cm off the start-box wall (sim only)
                            '-y', PythonExpression(["'0.0' if '", W, f"' == 'course' else '{sy / 1000.0}'"]), '-z', '0.04', '-Y', PythonExpression(["'0.0' if '", W, f"' == 'course' else '{math.radians(syaw)}'"])]),
        ]),

        TimerAction(period=8.0, actions=[
            Node(package='rdj2026_sim', executable='gate_controller.py', output='screen',
                 condition=IfCondition(LaunchConfiguration('gate')),
                 parameters=[{'use_sim_time': True, 'x': gx, 'y': gy,
                              'height': gate['height'] / 1000.0,
                              'open_s': float(gate['open_s']), 'closed_s': float(gate['closed_s'])}]),
        ]),
    ])
