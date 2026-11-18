"""Exercise the physical benchmark's pinned controller node under controlled clock skew."""
import subprocess
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from swerve_chassis_msgs.msg import ChassisState


def stamp(ms):
    seconds, remainder = divmod(ms, 1000)
    return Time(sec=seconds, nanosec=remainder * 1_000_000)


def millis(message):
    return message.sec * 1000 + (message.nanosec + 500_000) // 1_000_000


def main():
    rclpy.init()
    node = rclpy.create_node("chassis_clock_probe")
    latest = [None]
    node.create_subscription(
        ChassisState, "/probe/chassis_controller/state",
        lambda state: latest.__setitem__(0, state), rclpy.qos.qos_profile_sensor_data)
    clock = node.create_publisher(Clock, "/clock", 10)
    feedback = node.create_publisher(JointState, "/probe/joint_states", 10)
    commands = node.create_publisher(TwistStamped, "/probe/cmd_vel", 10)
    process = subprocess.Popen(["ros2", "run", "swerve_chassis_controller", "swerve_controller",
                                "--ros-args", "-p", "use_sim_time:=true", "-r", "__ns:=/probe"])

    def wait(predicate, timeout=0.18, resend=None):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if resend:
                resend()
            rclpy.spin_once(node, timeout_sec=.01)
            if latest[0] is not None and predicate(latest[0]):
                return latest[0]
        raise AssertionError(latest[0])

    def ack_clock(ms, timeout=0.18):
        def send_clock():
            clock.publish(Clock(clock=stamp(ms)))

        wait(lambda s: millis(s.header.stamp) == ms, timeout=timeout, resend=send_clock)

    def joints(ms):
        message = JointState()
        message.header.stamp = stamp(ms)
        message.name = [f"{corner}_{kind}_joint" for corner in ("fl", "fr", "rl", "rr")
                        for kind in ("steering", "wheel")]
        message.position = [0.] * 8
        message.velocity = [0.] * 8
        feedback.publish(message)

    def command(ms, vx=0.):
        message = TwistStamped()
        message.header.stamp = stamp(ms)
        message.header.frame_id = "base_footprint"
        message.twist.linear.x = vx
        commands.publish(message)

    def advance(ms):
        ack_clock(ms)
        joints(ms)
        wait(lambda s: millis(s.feedback_stamp) == ms and s.feedback_valid)

    def check_rejection(ms, code, text):
        prior = latest[0]
        rejected = wait(lambda s: millis(s.header.stamp) == ms and s.fault == code)
        assert rejected.command_sequence == prior.command_sequence, rejected
        assert rejected.command_fresh, rejected
        assert not rejected.generator_valid, rejected
        assert max(map(abs, rejected.target_wheel_speeds)) == 0., rejected
        print(text, flush=True)

    try:
        ack_clock(40640, timeout=12)
        wait(lambda s: s.feedback_valid, timeout=5, resend=lambda: joints(40640))
        wait(lambda s: s.fault == 0, timeout=5, resend=lambda: command(40640))
        advance(40655)
        command(40660, .08)
        wait(lambda s: s.fault == 0 and millis(s.command_stamp) == 40660)
        print("PASS command 5 ms ahead accepted", flush=True)

        advance(40700)
        command(40715, .08)
        check_rejection(40700, 1, "PASS command 15 ms ahead rejected with fresh history")
        advance(40720)
        command(40720)
        wait(lambda s: s.fault == 0 and millis(s.command_stamp) == 40720)
        print("PASS a fresh zero request clears the latched fault", flush=True)

        advance(40725)
        command(40720)
        check_rejection(40725, 1, "PASS duplicate stamp rejected with fresh history")
        advance(40730)
        command(40730)
        wait(lambda s: s.fault == 0 and millis(s.command_stamp) == 40730)
        advance(40735)
        joints(40750)
        check_rejection(40735, 2, "PASS feedback 15 ms ahead rejected with fresh command")
    finally:
        process.terminate()
        process.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
