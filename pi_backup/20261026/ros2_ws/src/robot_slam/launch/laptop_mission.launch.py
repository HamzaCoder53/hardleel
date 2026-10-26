"""Phase 2 on the laptop, ON TOP of the running laptop_mapping.launch.py (or laptop_localization.launch.py).
Adds only: /odom for Nav2, the Nav2 servers, the CLIP classifier, the event logger and the mission node.
Nothing here restarts Cartographer, the TFs or the safety chain.

    ros2 launch robot_slam laptop_mission.launch.py [waypoints:=~/hardleel_ws/waypoints/waypoints_course.yaml]
        [classifier:=clip|stub] [stub_class:=plastic] [sim:=false]
    The mission waits for ONE start command:  scripts/laptop/go.sh   (= ros2 topic pub --once /mission/start ...)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch_ros.actions import Node

from robot_slam import stack_common as sc


def truthy(context, name):
    return context.launch_configurations[name].lower() in ('1', 'true', 'yes')


def setup(context):
    lc = context.launch_configurations
    sim = truthy(context, 'sim')
    cfg = os.path.join(sc.pkg_share(), 'config')
    nav2_params = sc.nav2_params_for_distro(os.path.join(cfg, 'nav2_params_mission.yaml'), sim)
    simp = {'use_sim_time': sim}
    # sim: the simulator's /odom (twist only, its TF is off with sim odom_tf:=false) stands in for rf2o + EKF
    actions = []
    if truthy(context, 'nav2'):                   # nav2:=false when mapping_mode:=auto already started Nav2
        actions += ([] if sim else sc.odom_for_nav2(sim)) + sc.nav2_nodes(nav2_params, sim)
    # (the mission node itself switches the collision monitor from the mapping stop box to the mission one)
    # --- new design, every switch OFF by default ---
    turn_cal = truthy(context, 'turn_calibration')
    if turn_cal:                                  # in-place spin test at the start of the run (gyro, Pi PWM)
        actions.append(Node(package='motor_teleop', executable='motor_calibrator', name='motor_calibrator',
                            output='screen', parameters=[simp]))
    if truthy(context, 'speed_feedback'):         # forward speed from the Cartographer pose -> /motor/speed_scale
        actions.append(Node(package='robot_slam', executable='speed_governor', name='speed_governor',
                            output='screen', parameters=[simp]))
    actions.append(Node(package='robot_slam', executable='event_logger', name='event_logger', output='screen',
                        parameters=[os.path.join(cfg, 'event_logger.yaml'), simp]))
    stub = '' if lc['classifier'] == 'clip' else lc['stub_class']
    if not stub:
        wc_cfg = os.path.join(sc.pkg_share('waste_classifier'), 'config', 'clip_classifier.yaml')
        actions.append(Node(package='waste_classifier', executable='clip_classifier', name='clip_classifier',
                            output='screen', parameters=[wc_cfg, simp]))
    waypoints = os.path.expanduser(lc['waypoints'])
    # Nav2 needs ~15 s to configure and activate before the mission can send goals; the mission itself also
    # waits for the navigate_to_pose server, this delay only keeps the start-up log readable.
    actions.append(TimerAction(period=5.0, actions=[Node(
        package='robot_slam', executable='competition_mission', name='competition_mission', output='screen',
        parameters=[os.path.join(cfg, 'competition_mission.yaml'),
                    {**simp, 'waypoints_file': waypoints, 'stub_class': stub,
                     'auto_start': truthy(context, 'auto_start'), 'turn_calibration': turn_cal}],
        remappings=[('scan', '/scan_filtered')])]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('waypoints', default_value='~/hardleel_ws/waypoints/waypoints_course.yaml'),
        DeclareLaunchArgument('classifier', default_value='clip', description='clip | stub'),
        DeclareLaunchArgument('stub_class', default_value='plastic'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('nav2', default_value='true', description='false if auto mapping already runs Nav2'),
        DeclareLaunchArgument('turn_calibration', default_value='false', description='NEW, off: spin test at start'),
        DeclareLaunchArgument('speed_feedback', default_value='false', description='NEW, off: speed_governor'),
        OpaqueFunction(function=setup),
    ])
