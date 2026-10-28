"""Stream real observations to CSV; retain partial recordings on a failed case."""
import csv
import json
import math
import time
from pathlib import Path

from trajectory_scenarios import TITLES


def heading(pose):
    q = pose.orientation
    return math.atan2(2*q.w*q.z, 1-2*q.z*q.z)


def local_pose(pose, origin):
    angle = heading(origin)
    dx, dy = pose.position.x-origin.position.x, pose.position.y-origin.position.y
    return (math.cos(angle)*dx+math.sin(angle)*dy,
            -math.sin(angle)*dx+math.cos(angle)*dy,
            math.remainder(heading(pose)-angle, 2*math.pi))


def stamp(message):
    s = message.header.stamp
    return s.sec + s.nanosec*1e-9


def path_error(x, y, points):
    errors = []
    for (ax, ay, _), (bx, by, _) in zip(points, points[1:]):
        dx, dy = bx-ax, by-ay
        u = max(0., min(1., ((x-ax)*dx+(y-ay)*dy)/max(1e-12, dx*dx+dy*dy)))
        errors.append(math.hypot(x-ax-u*dx, y-ay-u*dy))
    return min(errors) if errors else math.hypot(x-points[0][0], y-points[0][1])


class Recording:
    def __init__(self, directory, name, points, seed, odom_origin, truth_origin, initial_stamp):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.points = points
        self.odom_origin, self.truth_origin = odom_origin, truth_origin
        self.last_stamp = initial_stamp-1e-9
        self.start = time.monotonic()
        self.errors = []
        self.last_pose = None
        self.stopped = 0
        self.metadata = dict(scenario=name, title=TITLES.get(name, name), seed=seed,
                             reference=points, source='Independent Gazebo ground truth',
                             coordinate_frame='initial_body', status='running')
        self.file = (self.directory/'trajectory.csv').open('w', newline='', buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(['sim_time_s', 'wall_elapsed_s', 'truth_x_m', 'truth_y_m',
                              'truth_yaw_rad', 'odom_time_s', 'odom_x_m', 'odom_y_m',
                              'odom_yaw_rad', 'path_error_m', 'peak_wheel_rad_s',
                              'requested_vx_m_s', 'requested_vy_m_s', 'requested_wz_rad_s'])
        self._write_metadata()

    def _write_metadata(self):
        (self.directory/'result.json').write_text(json.dumps(self.metadata, indent=2)+'\n')

    def observe(self, history, odom, execution):
        for observation in history:
            timestamp = stamp(observation)
            if timestamp <= self.last_stamp:
                continue
            self.last_stamp = timestamp
            self.last_pose = local_pose(observation.pose.pose, self.truth_origin)
            x, y, a = self.last_pose
            error = path_error(x, y, self.points)
            self.errors.append(error)
            ox, oy, oa = local_pose(odom.pose.pose, self.odom_origin)
            peak = max(map(abs, execution.wheel_speeds)) if execution is not None else None
            requested = execution.requested_velocity if execution is not None else None
            self.stopped += peak is not None and peak < .05
            self.writer.writerow([timestamp, time.monotonic()-self.start, x, y, a,
                                  stamp(odom), ox, oy, oa, error, peak,
                                  requested.linear.x if requested else None,
                                  requested.linear.y if requested else None,
                                  requested.angular.z if requested else None])

    def close(self, success, reason, compute_times):
        self.file.close()
        count = len(self.errors)
        self.metadata.update(status='passed' if success else 'failed', reason=reason,
                             samples=count, wall_duration_s=time.monotonic()-self.start)
        if count:
            gx, gy, ga = self.points[-1]
            x, y, a = self.last_pose
            self.metadata.update(path_rms_m=math.sqrt(sum(v*v for v in self.errors)/count),
                                 path_peak_m=max(self.errors), stopped_fraction=self.stopped/count,
                                 final_position_error_m=math.hypot(x-gx, y-gy),
                                 final_yaw_error_rad=abs(math.remainder(a-ga, 2*math.pi)))
        if compute_times:
            ordered = sorted(compute_times)
            self.metadata['compute_ms'] = {
                str(p): ordered[min(len(ordered)-1, int(len(ordered)*p/100))]
                for p in (50, 95, 99)}
        self._write_metadata()
