"""Run the Nav2 FollowPath server, continuous velocity plugin."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.utilities import prefix_namespace, to_parameters_list


def controller_nodes(context):
    parameters = Path(LaunchConfiguration('parameters').perform(context))
    namespace = LaunchConfiguration('namespace').perform(context)
    node_namespace = prefix_namespace(context.launch_configurations.get('ros_namespace'), namespace)
    inherited = []
    for parameter in context.launch_configurations.get('global_params', []) or []:
        if isinstance(parameter, tuple):
            name, value = parameter
            if name == 'use_sim_time':
                inherited.append({name: value})
        else:
            inherited.append(Path(parameter))
    resolved = to_parameters_list(
        context, 'controller_server', node_namespace, [*inherited, parameters])
    # Resolve the controller's clock once before starting the command pipeline.
    use_sim_time = next(
        (param.value for param in reversed(resolved) if param.name == 'use_sim_time'), False)
    clock = {'use_sim_time': use_sim_time}
    return [
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             namespace=namespace, output='screen', parameters=[str(parameters), clock],
             remappings=[('cmd_vel', 'cmd_vel_nav2')]),
        Node(package='swerve_mppi_ros', executable='cmd_vel_stamp_guard', name='cmd_vel_stamp_guard',
             namespace=namespace, output='screen', parameters=[clock]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_controller', namespace=namespace, output='screen',
             parameters=[{**clock, 'autostart': True, 'node_names': ['controller_server']}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('parameters', default_value=PathJoinSubstitution([
            FindPackageShare('swerve_mppi_ros'), 'config', 'controller.yaml'])),
        OpaqueFunction(function=controller_nodes),
    ])
