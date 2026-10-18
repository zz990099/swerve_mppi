"""Run the Nav2 FollowPath server, plugin and final-velocity chassis bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters = LaunchConfiguration('parameters')
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('parameters', default_value=PathJoinSubstitution([
            FindPackageShare('swerve_mppi_ros'), 'config', 'controller.yaml'])),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             namespace=namespace, output='screen', parameters=[parameters]),
        Node(package='swerve_mppi_ros', executable='chassis_bridge', name='mppi_chassis_bridge',
             namespace=namespace, output='screen', parameters=[parameters]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_controller', namespace=namespace, output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['controller_server']}]),
    ])
