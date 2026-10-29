"""Start a Gazebo Harmonic chassis and the Nav2 FollowPath MPPI plugin."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('chassis_config', default_value=PathJoinSubstitution([
            FindPackageShare('swerve_gazebo_sim'), 'config', 'swerve.yaml'])),
        DeclareLaunchArgument('publish_ground_truth_map_tf', default_value='false'),
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('yaw', default_value='0'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('publish_ground_truth', default_value='false'),
        DeclareLaunchArgument('parameters', default_value=PathJoinSubstitution([
            FindPackageShare('swerve_mppi_ros'), 'config', 'controller.yaml'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('swerve_gazebo_sim'), 'launch', 'demo_gz.launch.py'])),
            launch_arguments={'config': LaunchConfiguration('chassis_config'),
                              'headless': LaunchConfiguration('headless'),
                              'odom_topic': 'odom',
                              'publish_odom_tf': 'true',
                              'publish_ground_truth': LaunchConfiguration('publish_ground_truth'),
                              'publish_ground_truth_map_tf': LaunchConfiguration('publish_ground_truth_map_tf'),
                              'x': LaunchConfiguration('x'), 'y': LaunchConfiguration('y'),
                              'yaw': LaunchConfiguration('yaw')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('swerve_mppi_ros'), 'launch', 'controller.launch.py'])),
            launch_arguments={'parameters': LaunchConfiguration('parameters')}.items()),
    ])
