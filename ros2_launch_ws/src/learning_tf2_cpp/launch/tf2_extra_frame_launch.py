import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Incluye el launch anterior (broadcasters + listener)
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('learning_tf2_cpp'),
                'launch',
                'tf2_bro_lis_launch.py'
            )
        )
    )

    # Nodo del broadcaster dinámico (crea el frame carrot1)
    dynamic_broadcaster = Node(
        package='learning_tf2_cpp',
        executable='dynamic_frame_tf2_broadcaster',
        name='dynamic_broadcaster'
    )

    return LaunchDescription([
        demo_nodes,
        dynamic_broadcaster,
    ])
