"""Start a Gazebo Harmonic chassis and the Nav2 FollowPath MPPI plugin."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('chassis_config', default_value=PathJoinSubstitution([
            FindPackageShare('swerve_gazebo_sim'), 'config', 'swerve.yaml'])),
        DeclareLaunchArgument('use_imu_odometry', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('publish_ground_truth', default_value='false'),
        DeclareLaunchArgument('parameters', default_value=PathJoinSubstitution([
            FindPackageShare('swerve_mppi_ros'), 'config', 'controller.yaml'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('swerve_gazebo_sim'), 'launch', 'demo_gz.launch.py'])),
            launch_arguments={'config': LaunchConfiguration('chassis_config'),
                              'headless': LaunchConfiguration('headless'),
                              'odom_topic': PythonExpression(["'wheel/odom' if '", LaunchConfiguration('use_imu_odometry'), "' == 'true' else 'odom'"]),
                              'publish_odom_tf': PythonExpression(["'false' if '", LaunchConfiguration('use_imu_odometry'), "' == 'true' else 'true'"]),
                              'publish_ground_truth': LaunchConfiguration('publish_ground_truth')}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='imu_bridge', arguments=['/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('use_imu_odometry'))),
        Node(package='swerve_mppi_ros', executable='imu_odometry.py', output='screen',
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('use_imu_odometry'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('swerve_mppi_ros'), 'launch', 'controller.launch.py'])),
            launch_arguments={'parameters': LaunchConfiguration('parameters')}.items()),
    ])
