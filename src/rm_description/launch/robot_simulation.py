import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_name = 'rm_description'
    pkg_share = get_package_share_directory(pkg_name)

    # Mundo (use o nome que você salvou)
    world_file = os.path.join(pkg_share, 'world', 'casa_mobiliada.sdf')

    # Xacro do robô
    xacro_file = os.path.join(pkg_share, 'urdf', 'differential_robot.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    # Configs
    bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'aps1.rviz')

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Spawn do robô
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rm_robot',
            '-topic', 'robot_description',
            '-x', '2.0',
            '-y', '2.0',
            '-z', '0.10',
        ],
        output='screen'
    )

    # Bridge de tópicos
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen'
    )

    # Bridge da imagem (melhor para /camera/image_raw)
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen'
    )

    # RViz (vai falhar se aps1.rviz não existir ainda — a gente cria já já)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        bridge,
        image_bridge,
        rviz
    ])
