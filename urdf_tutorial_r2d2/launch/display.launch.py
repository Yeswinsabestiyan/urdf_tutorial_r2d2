import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('urdf_tutorial_r2d2')
    xacro_file = os.path.join(pkg_share, 'urdf', 'r2d2.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'r2d2.rviz')

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='urdf_tutorial_r2d2',
            executable='state_publisher',
            name='r2d2_state_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
