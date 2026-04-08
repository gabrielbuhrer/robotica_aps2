import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_description'),
                'launch',
                'robot_simulation.py'
            )
        )
    )

    rotate_node = Node(
        package='patrol_bot',
        executable='rotate_node',
        name='rotate_node',
        output='screen'
    )

    patrol_node = Node(
        package='patrol_bot',
        executable='patrol_node',
        name='patrol_node',
        output='screen'
    )

    delayed_rotate = TimerAction(
        period=2.0,
        actions=[rotate_node]
    )

    delayed_patrol = TimerAction(
        period=2.5,
        actions=[patrol_node]
    )

    return LaunchDescription([
        sim_launch,
        delayed_rotate,
        delayed_patrol,
    ])