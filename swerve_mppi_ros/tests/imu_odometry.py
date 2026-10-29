#!/usr/bin/env python3
"""Demo localization: wheel body translation with independent IMU heading.

The Gazebo IMU is ideal in this experiment; this is not a calibrated hardware EKF.
"""
import copy
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from heading_odometry import HeadingOdometry


def stamp(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class ImuOdometry(Node):
    def __init__(self):
        super().__init__('imu_wheel_odometry')
        self.filter = HeadingOdometry()
        self.queue = deque(maxlen=64)
        self.last_clock = None
        self.last_imu_wall = 0.
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Imu, 'imu/data', self.imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'wheel/odom', self.wheel, 10)

    def clock(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if self.last_clock is not None and now < self.last_clock:
            self.filter = HeadingOdometry()
            self.queue.clear()
        self.last_clock = now
        return now

    def imu(self, msg):
        now = self.clock()
        q = msg.orientation
        norm = sum(v*v for v in (q.w, q.x, q.y, q.z))
        if (msg.orientation_covariance[0] < 0 or not math.isfinite(norm) or
                abs(norm-1) > .01 or stamp(msg) > now+.01 or now-stamp(msg) > .1):
            return
        if self.filter.add_imu(stamp(msg), yaw(q), msg.angular_velocity.z):
            self.last_imu_wall = time.monotonic()
            self.drain()

    def wheel(self, msg):
        now = self.clock()
        if stamp(msg) > now+.01 or now-stamp(msg) > .1:
            return
        if self.queue and stamp(msg) <= stamp(self.queue[-1]):
            return
        self.queue.append(msg)
        self.drain()

    def drain(self):
        if time.monotonic()-self.last_imu_wall > .1 or not self.filter.imu:
            return
        while self.queue:
            msg = self.queue[0]
            t = stamp(msg)
            if t > self.filter.imu[-1][0]+1e-8:
                break
            self.queue.popleft()
            pose = msg.pose.pose
            result = self.filter.wheel(t, msg.twist.twist.linear.x,
                                       msg.twist.twist.linear.y, yaw(pose.orientation),
                                       pose.position.x, pose.position.y)
            if result is None:
                continue
            out = copy.deepcopy(msg)
            x, y, angle, rate = result
            out.pose.pose.position.x, out.pose.pose.position.y = x, y
            q = out.pose.pose.orientation
            q.x = q.y = 0.
            q.z, q.w = math.sin(angle/2), math.cos(angle/2)
            out.twist.twist.angular.z = rate
            # Retain conservative wheel covariance; no sensor-accuracy claim.
            self.pub.publish(out)
            tf = TransformStamped()
            tf.header, tf.child_frame_id = out.header, out.child_frame_id
            tf.transform.translation.x, tf.transform.translation.y = x, y
            tf.transform.rotation = q
            self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = ImuOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
