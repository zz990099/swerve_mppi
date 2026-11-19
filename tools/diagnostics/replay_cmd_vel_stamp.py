"""Replay the same-clock Nav2 motion/zero pair against the installed ROS nodes."""
import os
import signal
import subprocess
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from swerve_chassis_msgs.msg import ChassisState


def ros_stamp(ns):
    sec, nanosec = divmod(ns, 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def timestamp(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main():
    rclpy.init()
    node = rclpy.create_node('cmd_vel_replay')
    state = [None]
    wire = []
    node.create_subscription(TwistStamped, '/probe/cmd_vel', wire.append, 10)
    node.create_subscription(ChassisState, '/probe/chassis_controller/state',
                             lambda msg: state.__setitem__(0, msg), qos_profile_sensor_data)
    clock = node.create_publisher(Clock, '/clock', 10)
    feedback = node.create_publisher(JointState, '/probe/joint_states', 10)
    commands = node.create_publisher(TwistStamped, '/probe/cmd_vel_nav2', 10)
    processes = []
    ns = 13_635_000_000

    def wait(predicate, timeout=.18, repeat=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if repeat:
                repeat()
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return
        raise AssertionError(f'No expected result. State {state[0]}; last command {wire[-1:]}')

    def set_clock(ns, timeout=.18):
        wait(lambda: state[0] is not None and timestamp(state[0].header.stamp) == ns,
             timeout, lambda: clock.publish(Clock(clock=ros_stamp(ns))))

    def joints(ns):
        msg = JointState()
        msg.header.stamp = ros_stamp(ns)
        msg.name = [f'{w}_{joint}_joint' for w in ('fl', 'fr', 'rl', 'rr')
                    for joint in ('steering', 'wheel')]
        msg.position, msg.velocity = [0.] * 8, [0.] * 8
        feedback.publish(msg)

    def command(ns, x):
        msg = TwistStamped()
        msg.header.frame_id = 'base_footprint'
        msg.header.stamp = ros_stamp(ns)
        msg.twist.linear.x = x
        commands.publish(msg)

    try:
        for package, executable in [('swerve_mppi_ros', 'cmd_vel_stamp_guard'),
                                    ('swerve_chassis_controller', 'swerve_controller')]:
            processes.append(subprocess.Popen(
                ['ros2', 'run', package, executable, '--ros-args', '-r', '__ns:=/probe',
                 '-p', 'use_sim_time:=true'], start_new_session=True))
        wait(lambda: commands.get_subscription_count() == 1 and
             clock.get_subscription_count() == 2 and
             feedback.get_subscription_count() == 1 and
             any(e.node_name == 'swerve_controller' for e in
                 node.get_subscriptions_info_by_topic('/probe/cmd_vel')), 12)
        # Keep the first clock value available until both ROS clocks initialize.
        set_clock(ns, 3)
        wait(lambda: state[0].feedback_valid, 3, lambda: joints(ns))
        for _ in range(8):
            ns += 20_000_000
            set_clock(ns)
            wait(lambda: timestamp(state[0].feedback_stamp) == ns and state[0].feedback_valid,
                 repeat=lambda: joints(ns))
            command(ns, 0.)
            try:
                wait(lambda: state[0].fault == 0)
                break
            except AssertionError:
                continue
        else:
            raise AssertionError(f'Chassis did not accept a startup stop: {state[0]}')

        ns += 50_000_000
        set_clock(ns)
        joints(ns)
        wait(lambda: timestamp(state[0].feedback_stamp) == ns)
        command(ns, .60)
        wait(lambda: state[0].fault == 0 and state[0].requested_velocity.linear.x == .60)
        sequence = state[0].command_sequence
        command(ns, 0.)
        wait(lambda: state[0].fault == 0 and state[0].command_sequence == sequence + 1 and
             state[0].requested_velocity.linear.x == 0. and
             len(wire) >= 3)
        assert [timestamp(m.header.stamp) for m in wire[-2:]] == [ns, ns + 1000]
        assert [m.twist.linear.x for m in wire[-2:]] == [.60, 0.]
        assert timestamp(state[0].header.stamp) == ns
        print('PASS same-tick zero reached the chassis without a fault while ROS time was frozen',
              flush=True)

        command(ns, .60)
        command(ns, 0.)
        command(ns - 1, .60)
        ns += 50_000_000
        set_clock(ns)
        joints(ns)
        wait(lambda: timestamp(state[0].feedback_stamp) == ns)
        command(ns, .25)
        wait(lambda: state[0].fault == 0 and state[0].requested_velocity.linear.x == .25 and
             len(wire) >= 4)
        assert state[0].command_sequence == sequence + 2
        assert [timestamp(m.header.stamp) for m in wire[-3:]] == [ns - 50_000_000,
                                                                  ns - 49_999_000, ns]
        print('PASS same-tick repetitions and older motion did not reach the chassis', flush=True)
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        for process in processes:
            process.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
