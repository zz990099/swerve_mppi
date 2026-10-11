"""Start an unconfigured lifecycle controller; task/evidence producers are external."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('parameters', default_value=PathJoinSubstitution([
            FindPackageShare('swerve_mppi_ros'), 'config', 'controller.yaml'])),
        LifecycleNode(
            package='swerve_mppi_ros', executable='controller', name='mppi_controller',
            namespace=LaunchConfiguration('namespace'), output='screen',
            parameters=[LaunchConfiguration('parameters')]),
    ])
