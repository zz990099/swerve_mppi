"""Start a Gazebo Harmonic chassis and its independent MPPI path follower."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('publish_ground_truth', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('swerve_gazebo_sim'), 'launch', 'demo_gz.launch.py'])),
            launch_arguments={'headless': LaunchConfiguration('headless'),
                              'publish_ground_truth': LaunchConfiguration('publish_ground_truth')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('swerve_mppi_ros'), 'launch', 'controller.launch.py']))),
    ])
