from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4ctrl',
            executable='thrust_calibrate.py',
            name='thrust_calibrate',
            output='screen',
            parameters=[{
                'time_interval': 1.0,
                'mass_kg': 10.7,
            }],
        ),
    ])
