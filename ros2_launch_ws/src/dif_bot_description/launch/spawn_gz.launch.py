import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('dif_bot_description')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default=os.path.join(pkg_share, 'worlds', 'my_world.sdf'))

    # Xacro del robot
    robot_xacro = os.path.join(pkg_share, 'urdf', 'robot_gz.urdf.xacro')
    robot_description = Command(['xacro ', robot_xacro])

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )

    # Lanzar Gazebo (gz sim) directamente
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_file],
        output='screen'
    )

    # Spawn del robot después de 5 segundos
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'dif_bot', '-z', '0.2'],
        output='screen'
    )
    spawn_after_gz = TimerAction(period=5.0, actions=[spawn_entity])

    # Bridge ROS ↔ GZ
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=world_file),
        gz_sim,
        rsp_node,
        spawn_after_gz,
        bridge,
    ])