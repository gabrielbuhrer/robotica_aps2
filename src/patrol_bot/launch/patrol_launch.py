from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    patrol_node = Node(
        package='patrol_bot',
        executable='patrol_node',
        name='patrol_node',
        output='screen'
    )

    rotate_node = Node(
        package='patrol_bot',
        executable='rotate_node',
        name='rotate_node',
        output='screen'
    )

    return LaunchDescription([
        patrol_node,
        rotate_node,
    ])
