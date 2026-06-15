import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('abb_control')

    return LaunchDescription([
        Node(
            package='abb_control',
            executable='arm_control_node',
            name='arm_control',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, 'config', 'arm_control.yaml'),
                os.path.join(pkg_dir, 'config', 'obstacles.yaml'),
            ],
        )
    ])
