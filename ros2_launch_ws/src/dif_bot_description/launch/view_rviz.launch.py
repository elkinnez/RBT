import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('dif_bot_description').find('dif_bot_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model_path),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')]  # si no existe, lo ignora
        )
    ])