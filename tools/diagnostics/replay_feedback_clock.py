"""Replay the chassis time boundary with real ROS messages and no MPPI process."""
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


def stamp(ns):
    sec, nanosec = divmod(ns, 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def nanoseconds(value):
    return value.sec * 1_000_000_000 + value.nanosec


def main():
    rclpy.init()
    node = rclpy.create_node('feedback_clock_replay', namespace='/feedback_replay')
    state = [None]
    node.create_subscription(ChassisState, 'chassis_controller/state',
                             lambda msg: state.__setitem__(0, msg), qos_profile_sensor_data)
    clock = node.create_publisher(Clock, '/clock', 10)
    feedback = node.create_publisher(JointState, 'joint_states', 10)
    commands = node.create_publisher(TwistStamped, 'cmd_vel', 10)
    process = subprocess.Popen(
        ['ros2', 'run', 'swerve_chassis_controller', 'swerve_controller', '--ros-args',
         '-r', '__ns:=/feedback_replay', '-p', 'use_sim_time:=true',
         '-p', 'feedback_timeout:=3.0', '-p', 'wall_timeout:=3.0',
         '-p', 'cmd_future_tolerance:=0.03'], start_new_session=True)
    now = 84_115_000_000

    def joint_sample(ns):
        msg = JointState()
        msg.header.stamp = stamp(ns)
        msg.name = [f'{wheel}_{joint}_joint' for wheel in ('fl', 'fr', 'rl', 'rr')
                    for joint in ('steering', 'wheel')]
        msg.position = msg.velocity = [0.] * 8
        feedback.publish(msg)

    def command(ns, vx):
        msg = TwistStamped()
        msg.header.stamp = stamp(ns)
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = vx
        commands.publish(msg)

    def wait(predicate, label, timeout=2., publish=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publish:
                publish()
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return
            assert process.poll() is None, f'Chassis exited with {process.returncode}'
        raise AssertionError(f'Timed out waiting for {label}: {state[0]}')

    try:
        wait(lambda: commands.get_subscription_count() and feedback.get_subscription_count()
             and clock.get_subscription_count(), 'topic discovery', timeout=10.)
        set_clock = lambda: clock.publish(Clock(clock=stamp(now)))
        wait(lambda: state[0] and nanoseconds(state[0].header.stamp) == now,
             'frozen clock', publish=set_clock)
        wait(lambda: state[0].feedback_valid and nanoseconds(state[0].feedback_stamp) == now,
             'feedback at the current clock', publish=lambda: joint_sample(now))
        startup_stamp = now - 100_000_000
        retry_at = 0.

        def send_startup():
            nonlocal startup_stamp, retry_at
            steady = time.monotonic()
            if steady >= retry_at:
                startup_stamp += 1_000_000
                retry_at = steady + .1
                command(startup_stamp, 0.)

        wait(lambda: state[0].fault == 0
             and abs(nanoseconds(state[0].command_stamp) - startup_stamp) <= 1,
             'startup zero', publish=send_startup)
        command(now, .55)
        wait(lambda: state[0].fault == 0 and state[0].requested_velocity.linear.x == .55,
             'accepted running command')
        sequence = state[0].command_sequence
        wait(lambda: state[0].fault == 0 and state[0].feedback_valid
             and nanoseconds(state[0].feedback_stamp) == now + 10_000_000,
             'valid feedback leading by exactly 10 ms',
             publish=lambda: joint_sample(now + 10_000_000))
        print('PASS: 10 ms lead accepted at clock 84.115 s', flush=True)
        wait(lambda: state[0].fault == 2, 'fault 2 on the first 15 ms feedback lead',
             publish=lambda: joint_sample(now + 15_000_000))
        assert nanoseconds(state[0].header.stamp) == now
        assert nanoseconds(state[0].feedback_stamp) == now + 10_000_000
        assert state[0].command_sequence == sequence
        assert state[0].command_fresh
        assert not state[0].generator_valid
        print('PASS: 15 ms lead latches fault 2 with a fresh and unchanged velocity command',
              flush=True)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
