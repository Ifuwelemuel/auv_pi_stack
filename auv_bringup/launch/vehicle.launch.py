"""Everything that runs on the vehicle Raspberry Pi.

    ros2 launch auv_bringup vehicle.launch.py                       # drivers + safety + teleop
    ros2 launch auv_bringup vehicle.launch.py autopilot:=true       # + heading/depth hold
    ros2 launch auv_bringup vehicle.launch.py mission:=box_20m      # + autopilot + navigator
    ros2 launch auv_bringup vehicle.launch.py record:=true          # + rosbag
    ros2 launch auv_bringup vehicle.launch.py depth:=false gps:=false mavros:=false
                                                  # thrusters-only bench work
    ros2 launch auv_bringup vehicle.launch.py local_joy:=true       # gamepad plugged into the Pi

mission:= takes a file name in config/missions/ (with or without .yaml) or
an absolute path. A mission implies the autopilot.

On the laptop: ros2 launch auv_bringup operator.launch.py

Refuses to start if stack nodes are already running (stacked launches gave
two publishers per topic in v1 — D10). Run scripts/kill_stack.sh first.
"""
import os
import subprocess
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RECORD_TOPICS = [
    '/joy', '/heartbeat', '/estop', '/estop_request',
    '/cmd/safety', '/cmd/teleop', '/cmd/autonomy', '/cmd/actuators',
    '/autopilot/heading_setpoint', '/autopilot/depth_setpoint',
    '/autopilot/surge_setpoint',
    '/depth', '/pressure', '/gps/fix', '/gps/status',
    '/navigator/position', '/navigator/origin', '/navigator/status',
    '/mavros/mavros/data', '/mavros/mavros/data_raw', '/mavros/mavros/mag',
    '/mavros/imu/data', '/mavros/imu/data_raw', '/mavros/imu/mag',
    '/mavros/state', '/mavros/battery',
]


def _ghost_guard():
    ghosts = subprocess.run(['pgrep', '-af', 'lib/auv_(control|drivers)/'],
                            capture_output=True, text=True).stdout.strip()
    if ghosts:
        raise RuntimeError('\n\nStack nodes already running — refusing to launch '
                           'a second copy.\nRun scripts/kill_stack.sh, then '
                           'relaunch.\nFound:\n' + ghosts + '\n')


def _resolve_mission(name, share):
    if not name:
        return ''
    if os.path.isabs(name):
        path = name
    else:
        path = os.path.join(share, 'config', 'missions',
                            name if name.endswith('.yaml') else name + '.yaml')
    if not os.path.isfile(path):
        raise RuntimeError(f'Mission file not found: {path}')
    return path


def _setup(context):
    _ghost_guard()
    share = get_package_share_directory('auv_bringup')
    cal = os.path.join(share, 'config', 'calibration.yaml')
    veh = os.path.join(share, 'config', 'vehicle.yaml')
    params = [cal, veh]
    arg = lambda n: LaunchConfiguration(n).perform(context).lower() == 'true'  # noqa
    mission = _resolve_mission(LaunchConfiguration('mission').perform(context), share)

    def node(pkg, exe, extra=None, **kw):
        return Node(package=pkg, executable=exe, name=exe, output='screen',
                    parameters=params + (extra or []), **kw)

    actions = [
        node('auv_control', 'safety_supervisor'),
        node('auv_control', 'actuator_mixer'),
        node('auv_control', 'teleop_node'),
        node('auv_drivers', 'thruster_driver'),
    ]
    if arg('local_joy'):
        actions.append(Node(package='joy', executable='joy_node', name='joy_node',
                            output='screen', parameters=[{'autorepeat_rate': 20.0}]))
    if arg('depth'):
        actions.append(node('auv_drivers', 'depth_sensor'))
    if arg('gps'):
        actions.append(node('auv_drivers', 'gps_node'))
    if arg('mavros'):
        actions.append(Node(
            package='mavros', executable='mavros_node', name='mavros',
            output='screen',
            parameters=[os.path.join(share, 'config', 'mavros.yaml'),
                        {'fcu_url': LaunchConfiguration('fcu_url').perform(context),
                         'gcs_url': LaunchConfiguration('gcs_url').perform(context)}]))
    if arg('autopilot') or mission:
        actions.append(node('auv_control', 'autopilot'))
    if mission:
        actions.append(node('auv_control', 'navigator',
                            extra=[{'mission_file': mission}]))
    if arg('record'):
        bag = os.path.expanduser(
            f'~/auv_bags/{datetime.now().strftime("%Y%m%d_%H%M%S")}')
        os.makedirs(os.path.dirname(bag), exist_ok=True)
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag] + RECORD_TOPICS,
            output='screen'))
    return actions


def generate_launch_description():
    args = [
        ('depth', 'true', 'Bar02 depth sensor'),
        ('gps', 'true', 'buoy GPS on the Pi serial port'),
        ('mavros', 'true', 'Pixhawk IMU/compass bridge'),
        ('autopilot', 'false', 'heading/depth hold (implied by mission)'),
        ('mission', '', 'mission name in config/missions/ or absolute path'),
        ('record', 'false', 'record a rosbag to ~/auv_bags/'),
        ('local_joy', 'false', 'run joy_node on the Pi (gamepad plugged in)'),
        ('fcu_url', '/dev/pixhawk:115200', 'Pixhawk serial device:baud'),
        ('gcs_url', '', 'optional QGC forwarding, e.g. udp://@<laptop-ip>:14550'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args]
        + [OpaqueFunction(function=_setup)])
