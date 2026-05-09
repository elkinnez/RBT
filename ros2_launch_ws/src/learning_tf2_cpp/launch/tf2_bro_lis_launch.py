from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de turtlesim (la simulación)
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="sim"
        ),
        # Broadcaster para turtle1
        Node(
            package="learning_tf2_cpp",
            executable="turtle_tf2_broadcaster",
            name="broadcaster1",
            parameters=[{"turtlename": "turtle1"}]
        ),
        # Argumento para el frame objetivo (se puede cambiar al lanzar)
        DeclareLaunchArgument(
            "target_frame",
            default_value="turtle1",
            description="Target frame name."
        ),
        # Broadcaster para turtle2
        Node(
            package="learning_tf2_cpp",
            executable="turtle_tf2_broadcaster",
            name="broadcaster2",
            parameters=[{"turtlename": "turtle2"}]
        ),
        # Listener (usa el argumento target_frame)
        Node(
            package="learning_tf2_cpp",
            executable="turtle_tf2_listener",
            name="listener",
            parameters=[{"target_frame": LaunchConfiguration("target_frame")}]
        ),
    ])
