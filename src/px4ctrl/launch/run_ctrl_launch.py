# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("px4ctrl")
    config_path = os.path.join(pkg_share, 'config', 'ctrl_param_fpv.yaml')

    px4ctrl_node = Node(
        package='px4ctrl',
        executable='px4ctrl_node',
        name='px4ctrl',
        output='screen',
        parameters=[config_path],
        remappings=[
            ('odom', '/vins_fusion/imu_propagate'),
            ('cmd', '/position_cmd')
        ]
    )
    return LaunchDescription([
        px4ctrl_node
    ])
