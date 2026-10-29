"""Source-time aligned wheel translation plus independent IMU orientation.

No truth position input. No extrapolation through missing IMU samples.
"""
import bisect
import math


class HeadingOdometry:
    def __init__(self, max_gap=.1):
        self.max_gap = max_gap
        self.imu = []
        self.previous = None
        self.offset = None
        self.x = self.y = self.yaw = 0.

    def add_imu(self, stamp, yaw, rate):
        if not all(math.isfinite(v) for v in (stamp, yaw, rate)):
            return False
        if self.imu and stamp <= self.imu[-1][0]:
            return False
        self.imu.append((stamp, yaw, rate))
        self.imu = self.imu[-256:]
        return True

    def wheel(self, stamp, vx, vy, initial_yaw=0., initial_x=0., initial_y=0.):
        if not all(math.isfinite(v) for v in (stamp, vx, vy, initial_yaw, initial_x, initial_y)):
            return None
        if self.previous is not None and stamp <= self.previous:
            return None
        times = [v[0] for v in self.imu]
        i = bisect.bisect_left(times, stamp)
        if i < len(times) and abs(times[i]-stamp) < 1e-8:
            _, angle, rate = self.imu[i]
        elif i == 0 or i == len(times) or times[i]-times[i-1] > self.max_gap:
            return None
        else:
            a, b = self.imu[i-1], self.imu[i]
            f = (stamp-a[0])/(b[0]-a[0])
            angle = a[1]+f*math.remainder(b[1]-a[1], 2*math.pi)
            rate = a[2]+f*(b[2]-a[2])
        if self.offset is None:
            self.offset = initial_yaw-angle
            self.x, self.y, self.yaw = initial_x, initial_y, initial_yaw
        heading = math.remainder(angle+self.offset, 2*math.pi)
        if self.previous is not None:
            dt = stamp-self.previous
            if dt > self.max_gap:
                # Resume from the last published pose; unobserved translation is
                # deliberately not fabricated across a sensor gap.
                self.previous = None
                return None
            change = math.remainder(heading-self.yaw, 2*math.pi)
            mid = self.yaw+change/2
            gain = math.sin(change/2)/(change/2) if abs(change) > 1e-8 else 1.
            self.x += dt*gain*(vx*math.cos(mid)-vy*math.sin(mid))
            self.y += dt*gain*(vx*math.sin(mid)+vy*math.cos(mid))
        self.previous, self.yaw = stamp, heading
        return self.x, self.y, self.yaw, rate
