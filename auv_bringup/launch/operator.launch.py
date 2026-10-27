"""Operator laptop: the gamepad, nothing else (D4).

    export ROS_DOMAIN_ID=42
    ros2 launch auv_bringup operator.launch.py

Only /joy crosses the WiFi. teleop_node runs on the Pi, so if the link drops
(the vehicle dives, or you walk out of range) /joy stops, the heartbeat
stops, and the vehicle is stopped by the watchdog if teleop was in command.
autorepeat keeps /joy streaming while the sticks are held still.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy_node', output='screen',
             parameters=[{'autorepeat_rate': 20.0, 'deadzone': 0.05}]),
    ])
