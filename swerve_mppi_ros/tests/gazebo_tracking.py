"""Physical path-following acceptance; truth is an observer, never controller input."""
import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path as PathMessage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


def yaw(q):
    return math.atan2(2*q.w*q.z, 1-2*q.z*q.z)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--chassis-root', type=Path, required=True)
    parser.add_argument('--scenario', choices=('straight', 'crab', 'reverse', 'spin', 'final_yaw', 'curve'))
    args = parser.parse_args()
    sys.path.insert(0, str(args.chassis_root.resolve() / 'test'))
    from gazebo_truth import GazeboTruth

    rclpy.init()
    node = rclpy.create_node('tracking_acceptance', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    odom, completed = [None], [None]
    node.create_subscription(Odometry, '/odom', lambda m: odom.__setitem__(0, m), 10)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Bool, '/mppi_controller/goal_reached',
                             lambda m: completed.__setitem__(0, m.data), qos)
    publisher = node.create_publisher(PathMessage, '/path', 1)
    truth = GazeboTruth('/ground_truth/odom', 'gz')

    def wait(predicate, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError(f'tracking condition timed out, odom={odom[0]}')

    try:
        wait(lambda: odom[0] is not None and truth.latest is not None and
             publisher.get_subscription_count() > 0, 90)
        initial = odom[0].pose.pose
        initial_truth = truth.latest.pose.pose
        rotation = wrap(yaw(initial_truth.orientation)-yaw(initial.orientation))
        origin = (initial.position.x, initial.position.y,
                  initial_truth.position.x, initial_truth.position.y)
        scenarios = {
            'straight': [(0, 0, 0), (1, 0, 0)],
            'crab': [(0, 0, 0), (0, .8, 0)],
            'reverse': [(0, 0, 0), (-.8, 0, 0)],
            'spin': [(0, 0, 0), (0, 0, .9)],
            'final_yaw': [(0, 0, 0), (.8, 0, -.9)],
            'curve': [(0, 0, 0), (.4, .1, .2), (.9, .25, .2), (1.3, .25, 0)],
        }
        for name, points in scenarios.items():
            if args.scenario and name != args.scenario:
                continue
            print(f"START {name}", flush=True)
            base = odom[0].pose.pose
            a, bx, by = yaw(base.orientation), base.position.x, base.position.y
            message = PathMessage()
            message.header.frame_id = 'odom'
            message.header.stamp = node.get_clock().now().to_msg()
            for x, y, heading in points:
                p = PoseStamped()
                p.header = message.header
                p.pose.position.x = bx+math.cos(a)*x-math.sin(a)*y
                p.pose.position.y = by+math.sin(a)*x+math.cos(a)*y
                p.pose.orientation.z = math.sin((a+heading)/2)
                p.pose.orientation.w = math.cos((a+heading)/2)
                message.poses.append(p)
            completed[0] = None
            publisher.publish(message)
            wait(lambda: completed[0] is False, 5)
            start = time.monotonic()
            wait(lambda: completed[0] is True, 75)
            target = message.poses[-1].pose
            measured = truth.latest.pose.pose
            x, y = target.position.x-origin[0], target.position.y-origin[1]
            tx = origin[2]+math.cos(rotation)*x-math.sin(rotation)*y
            ty = origin[3]+math.sin(rotation)*x+math.cos(rotation)*y
            error = math.hypot(measured.position.x-tx, measured.position.y-ty)
            angle = abs(wrap(yaw(measured.orientation)-yaw(target.orientation)-rotation))
            print(f'{name}: wall_s={time.monotonic()-start:.2f} '
                  f'truth_position_error_m={error:.4f} truth_yaw_error_rad={angle:.4f}', flush=True)
            assert error < .15 and angle < .10, (name, error, angle)
        print('PASS physical path following and external goal completion', flush=True)
    finally:
        truth.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
