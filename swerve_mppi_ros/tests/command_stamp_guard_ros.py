"""Exercise the installed command relay, including its ROS clock and chassis contract."""
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from swerve_chassis_msgs.msg import ChassisState
import yaml


NAMESPACE = '/stamp_test'


@pytest.fixture
def graph():
    rclpy.init()
    node = rclpy.create_node('stamp_test_probe', namespace=NAMESPACE)
    processes = []

    def start(*args):
        processes.append(subprocess.Popen(['ros2', *args], start_new_session=True))

    def wait(predicate, description, *, timeout=10., publish=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publish:
                publish()
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return
            for process in processes:
                assert process.poll() is None, f'{process.args} exited with {process.returncode}'
        pytest.fail(f'Timed out waiting for {description}')

    try:
        yield node, start, wait
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


def ros_stamp(ns):
    sec, nanosec = divmod(ns, 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def nanoseconds(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def command(ns, velocity):
    msg = TwistStamped()
    msg.header.frame_id = 'base_footprint'
    msg.header.stamp = ros_stamp(ns)
    msg.twist.linear.x = velocity
    return msg


def start_controller(start, tmp_path, sim_time):
    share = Path(get_package_share_directory('swerve_mppi_ros'))
    config = yaml.safe_load((share / 'config/controller.yaml').read_text())
    controller = config['controller_server']['ros__parameters']
    if sim_time is None:
        controller.pop('use_sim_time')
    else:
        controller['use_sim_time'] = sim_time
    params = tmp_path / 'controller.yaml'
    params.write_text(yaml.safe_dump({NAMESPACE: config}))
    start('launch', 'swerve_mppi_ros', 'controller.launch.py',
          f'namespace:={NAMESPACE}', f'parameters:={params}')


@pytest.mark.parametrize('sim_time', [False, None])
def test_legacy_params_without_a_clock(graph, tmp_path, sim_time):
    node, start, wait = graph
    wire = []
    commands = node.create_publisher(TwistStamped, 'cmd_vel_nav2', 10)
    output = node.create_subscription(TwistStamped, 'cmd_vel', wire.append, 10)
    start_controller(start, tmp_path, sim_time)
    wait(lambda: commands.get_subscription_count() and output.get_publisher_count(),
         'relay discovery')

    message = command(node.get_clock().now().nanoseconds, .2)
    wait(lambda: wire, 'a system-time command without /clock', timeout=2.,
         publish=lambda: commands.publish(message))
    assert wire[0] == message


def test_same_stamp_stop_at_the_future_limit(graph, tmp_path):
    node, start, wait = graph
    wire = []
    state = None

    def receive_state(message):
        nonlocal state
        state = message

    commands = node.create_publisher(TwistStamped, 'cmd_vel_nav2', 10)
    clock = node.create_publisher(Clock, '/clock', 10)
    feedback = node.create_publisher(JointState, 'joint_states', 10)
    node.create_subscription(TwistStamped, 'cmd_vel', wire.append, 10)
    node.create_subscription(
        ChassisState, 'chassis_controller/state', receive_state, qos_profile_sensor_data)
    start_controller(start, tmp_path, True)
    start('run', 'swerve_chassis_controller', 'swerve_controller', '--ros-args',
          '-r', f'__ns:={NAMESPACE}', '-p', 'use_sim_time:=true',
          '-p', 'cmd_future_tolerance:=0.03', '-p', 'feedback_timeout:=3.0',
          '-p', 'wall_timeout:=3.0')
    wait(lambda: commands.get_subscription_count() and feedback.get_subscription_count() and
         {'cmd_vel_stamp_guard', 'swerve_controller'} <= {
             endpoint.node_name for endpoint in node.get_subscriptions_info_by_topic('/clock')} and
         any(endpoint.node_name == 'swerve_controller' for endpoint in
             node.get_subscriptions_info_by_topic(f'{NAMESPACE}/cmd_vel')),
         'chassis and relay clock and command subscriptions')

    now = 10_000_000_000
    clock_msg = Clock(clock=ros_stamp(now))
    wait(lambda: state is not None and nanoseconds(state.header.stamp) == now,
         'the frozen chassis clock', publish=lambda: clock.publish(clock_msg))
    joints = JointState()
    joints.header.stamp = clock_msg.clock
    joints.name = [f'{wheel}_{joint}_joint' for wheel in ('fl', 'fr', 'rl', 'rr')
                   for joint in ('steering', 'wheel')]
    joints.position = joints.velocity = [0.] * 8
    wait(lambda: state.feedback_valid, 'fresh chassis feedback',
         publish=lambda: feedback.publish(joints))

    start_stamp = now - 50_000_000
    retry_at = 0.

    def publish_startup_stop():
        nonlocal start_stamp, retry_at
        clock.publish(clock_msg)
        wall_time = time.monotonic()
        if wall_time >= retry_at:
            # A repeated stamp cannot recover a stop lost while DDS endpoints match.
            start_stamp += 1_000_000
            retry_at = wall_time + .1
            commands.publish(command(start_stamp, 0.))

    wait(lambda: state.fault == 0 and abs(nanoseconds(state.command_stamp) - start_stamp) < 1_000
         and wire and nanoseconds(wire[-1].header.stamp) == start_stamp,
         'chassis acknowledgement of the latest startup stop',
         timeout=2., publish=publish_startup_stop)
    sequence = state.command_sequence

    edge = now + 30_000_000
    commands.publish(command(edge, .6))
    wait(lambda: state.fault == 0 and state.command_sequence == sequence + 1 and
         state.requested_velocity.linear.x == .6, 'motion accepted at the limit', timeout=1.)
    commands.publish(command(edge, 0.))
    wait(lambda: state.fault == 0 and state.command_sequence == sequence + 2 and
         state.requested_velocity.linear.x == 0. and len(wire) >= 3,
         'same-stamp stop accepted immediately', timeout=1.)
    assert nanoseconds(state.header.stamp) == now
    assert [nanoseconds(msg.header.stamp) for msg in wire[-2:]] == [edge - 1_000, edge]
    assert [msg.twist.linear.x for msg in wire[-2:]] == [.6, 0.]
