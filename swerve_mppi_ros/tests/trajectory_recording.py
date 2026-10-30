"""Stream real observations to CSV; retain partial recordings on a failed case."""
import csv
import json
import math
import time
from pathlib import Path

from trajectory_scenarios import TITLES
from tracking_quality import evaluate, reference_match

DIAGNOSTICS = ('compute_ms', 'cost', 'evaluated', 'effective_samples', 'invalid_state',
               'invalid_input',
               'invalid_control', 'actuator_residual', 'generator_error', 'critic_rejected',
               'collision_rejected', 'critic_errors', 'controller_errors', 'braking_selected', 'presteering',
               'reference_speed', 'noise_scale', 'reorientation_active', 'reference_limit',
               'branch_distance', 'branch_phase', 'completed_transition', 'route_section', 'route_rotating', 'remaining_path_m') + tuple(
    f'{candidate}_{field}' for candidate in ('mean', 'fresh', 'brake', 'slow', 'recovery', 'selected')
    for field in ('valid', 'cost', 'path', 'heading', 'velocity', 'alignment', 'effort', 'progress', 'goal'))

def heading(pose):
    q = pose.orientation
    return math.atan2(2*q.w*q.z, 1-2*q.z*q.z)


def local_pose(pose, origin):
    angle = heading(origin)
    dx, dy = pose.position.x-origin.position.x, pose.position.y-origin.position.y
    return (math.cos(angle)*dx+math.sin(angle)*dy,
            -math.sin(angle)*dx+math.cos(angle)*dy,
            math.remainder(heading(pose)-angle, 2*math.pi))


def local_origin(pose):
    return dict(x=pose.position.x, y=pose.position.y, yaw=heading(pose))


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
        self.heading_errors = []
        self.last_request = None
        self.last_publication = None
        self.max_feedback_linear_rate = 0.
        self.max_feedback_angular_rate = 0.
        self.effective_samples = []
        self.max_request_linear_rate = 0.
        self.max_request_angular_rate = 0.
        self.stopped = 0
        self.moving_started = None
        self.first_stamp = None
        self.tracking_samples = 0
        self.tracking_stopped = 0
        self.execution_previous = None
        self.execution_seconds = [0.]*5
        self.metadata = dict(scenario=name, title=TITLES.get(name, name), seed=seed,
                             reference=points, source='Independent Gazebo ground truth',
                             coordinate_frame='initial_body', status='running',
                             origins=dict(odom=local_origin(odom_origin), truth=local_origin(truth_origin)))
        self.file = (self.directory/'trajectory.csv').open('w', newline='', buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(['sim_time_s', 'wall_elapsed_s', 'truth_x_m', 'truth_y_m',
                              'truth_yaw_rad', 'odom_time_s', 'odom_x_m', 'odom_y_m',
                              'odom_yaw_rad', 'path_error_m', 'peak_wheel_rad_s',
                              'requested_vx_m_s', 'requested_vy_m_s', 'requested_wz_rad_s',
                              'chassis_stamp_s', 'feedback_stamp_s', 'command_stamp_s', 'execution', 'fault', 'generator_valid'] +
                             [f'{field}_{wheel}' for field in ('steering_rad', 'wheel_rad_s',
                              'target_steering_rad', 'target_wheel_rad_s') for wheel in ('fl','fr','rl','rr')])
        self.tf_file = (self.directory/'map_to_odom.csv').open('w', newline='', buffering=1)
        self.tf_writer = csv.writer(self.tf_file)
        self.tf_writer.writerow(['stamp_s', 'x_m', 'y_m', 'yaw_rad'])
        self.wheel_odom_file = (self.directory/'wheel_odometry.csv').open('w', newline='', buffering=1)
        self.wheel_odom_writer = csv.writer(self.wheel_odom_file)
        self.wheel_odom_writer.writerow(['stamp_s', 'x_m', 'y_m', 'yaw_rad'])
        self.prediction_file = (self.directory/'predictions.csv').open('w', newline='', buffering=1)
        self.command_file = (self.directory/'commands.csv').open('w', newline='', buffering=1)
        self.command_writer = csv.writer(self.command_file)
        self.command_writer.writerow(['command_stamp_s', 'observer_stamp_s', 'vx_m_s', 'vy_m_s', 'wz_rad_s'])
        self.execution_file = (self.directory/'execution.csv').open('w', newline='', buffering=1)
        self.execution_writer = csv.writer(self.execution_file)
        self.execution_writer.writerow(['chassis_stamp_s', 'feedback_stamp_s', 'command_stamp_s',
            'command_sequence', 'execution', 'fault', 'generator_valid'] +
            [f'{field}_{wheel}' for field in ('steering_rad', 'wheel_rad_s',
             'target_steering_rad', 'target_wheel_rad_s') for wheel in ('fl', 'fr', 'rl', 'rr')])
        self.prediction_writer = csv.writer(self.prediction_file)
        self.prediction_writer.writerow(['prediction_stamp_s', 'horizon_step', 'x_m', 'y_m', 'yaw_rad'])
        self.diagnostic_file = (self.directory/'diagnostics.csv').open('w',newline='',buffering=1)
        self.diagnostic_writer = csv.writer(self.diagnostic_file)
        self.diagnostic_writer.writerow(['stamp_s']+list(DIAGNOSTICS))
        self._write_metadata()

    def transforms(self, message):
        for item in message.transforms:
            if item.header.frame_id != 'map' or item.child_frame_id != 'odom':
                continue
            q = item.transform.rotation
            angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.tf_writer.writerow([stamp(item), item.transform.translation.x,
                                    item.transform.translation.y, angle])

    def wheel_odometry(self, message):
        pose = message.pose.pose
        self.wheel_odom_writer.writerow([stamp(message), pose.position.x, pose.position.y, heading(pose)])

    def diagnostic(self, message):
        for status in message.status:
            values={item.key:item.value for item in status.values}
            if 'compute_ms' not in values:
                continue
            self.diagnostic_writer.writerow([stamp(message)]+[values.get(key,'') for key in DIAGNOSTICS])
            if 'effective_samples' in values:
                self.effective_samples.append(float(values['effective_samples']))

    def command(self, message, observer_stamp):
        timestamp = stamp(message)
        request = (message.twist.linear.x, message.twist.linear.y, message.twist.angular.z)
        self.command_writer.writerow([timestamp, observer_stamp, *request])
        if self.last_publication is not None:
            previous_time, previous = self.last_publication
            dt = timestamp-previous_time
            if dt > 1e-6:
                self.max_request_linear_rate = max(self.max_request_linear_rate,
                    math.hypot(request[0]-previous[0], request[1]-previous[1])/dt)
                self.max_request_angular_rate = max(self.max_request_angular_rate,
                    abs(request[2]-previous[2])/dt)
        self.last_publication = (timestamp, request)

    def execution(self, message):
        timestamp = stamp(message)
        if self.execution_previous is not None:
            previous, phase = self.execution_previous
            dt = timestamp-previous
            if 0 < dt <= .1 and 0 <= phase < 5:
                self.execution_seconds[phase] += dt
        self.execution_previous = (timestamp, message.execution)
        self.execution_writer.writerow([stamp(message),
            message.feedback_stamp.sec+message.feedback_stamp.nanosec*1e-9,
            message.command_stamp.sec+message.command_stamp.nanosec*1e-9,
            message.command_sequence, message.execution, message.fault, int(message.generator_valid)] +
            list(message.steering_angles)+list(message.wheel_speeds)+
            list(message.target_steering)+list(message.target_wheel_speeds))

    def prediction(self, message):
        for i, item in enumerate(message.poses):
            x, y, angle = local_pose(item.pose, self.odom_origin)
            self.prediction_writer.writerow([stamp(message), i, x, y, angle])

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
            _, reference_yaw = reference_match(x, y, self.points)
            self.heading_errors.append(abs(math.remainder(a-reference_yaw, 2*math.pi)))
            ox, oy, oa = local_pose(odom.pose.pose, self.odom_origin)
            peak = max(map(abs, execution.wheel_speeds)) if execution is not None else None
            if self.first_stamp is None:
                self.first_stamp = timestamp
            if peak is not None and peak >= .05 and self.moving_started is None:
                self.moving_started = timestamp
            if self.moving_started is not None and peak is not None:
                self.tracking_samples += 1
                self.tracking_stopped += peak < .05
            requested = execution.requested_velocity if execution is not None else None
            self.stopped += peak is not None and peak < .05
            extra = [None]*22
            if execution is not None:
                command_time = execution.command_stamp.sec+execution.command_stamp.nanosec*1e-9
                request = (requested.linear.x, requested.linear.y, requested.angular.z)
                if self.last_request is not None:
                    previous_time, previous, fault = self.last_request
                    dt = command_time-previous_time
                    if dt > 1e-6 and not fault and not execution.fault:
                        self.max_feedback_linear_rate = max(self.max_feedback_linear_rate,
                            math.hypot(request[0]-previous[0],request[1]-previous[1])/dt)
                        self.max_feedback_angular_rate = max(self.max_feedback_angular_rate,
                            abs(request[2]-previous[2])/dt)
                self.last_request = (command_time, request, execution.fault)
                extra = [stamp(execution), execution.feedback_stamp.sec+execution.feedback_stamp.nanosec*1e-9, command_time, execution.execution, execution.fault, int(execution.generator_valid)] + list(
                    execution.steering_angles)+list(execution.wheel_speeds)+list(
                    execution.target_steering)+list(execution.target_wheel_speeds)
            self.writer.writerow([timestamp, time.monotonic()-self.start, x, y, a,
                                  stamp(odom), ox, oy, oa, error, peak,
                                  requested.linear.x if requested else None,
                                  requested.linear.y if requested else None,
                                  requested.angular.z if requested else None] + extra)

    def close(self, success, reason, compute_times):
        self.tf_file.close()
        self.wheel_odom_file.close()
        self.file.close()
        self.prediction_file.close()
        self.command_file.close()
        self.execution_file.close()
        self.diagnostic_file.close()
        count = len(self.errors)
        self.metadata.update(status='passed' if success else 'failed', reason=reason,
                             samples=count, wall_duration_s=time.monotonic()-self.start)
        self.metadata['initial_wait_to_motion_s'] = (self.moving_started-self.first_stamp
            if self.moving_started is not None and self.first_stamp is not None else None)
        self.metadata['execution_duration_s'] = dict(zip(
            ('tracking', 'limited', 'braking', 'presteering', 'stopped'), self.execution_seconds))
        self.metadata['tracking_stopped_fraction'] = (self.tracking_stopped/self.tracking_samples
            if self.tracking_samples else None)
        if count:
            gx, gy, ga = self.points[-1]
            x, y, a = self.last_pose
            self.metadata.update(path_rms_m=math.sqrt(sum(v*v for v in self.errors)/count),
                                 path_peak_m=max(self.errors), stopped_fraction=self.stopped/count,
                                 final_position_error_m=math.hypot(x-gx, y-gy),
                                 final_yaw_error_rad=abs(math.remainder(a-ga, 2*math.pi)),
                                 heading_rms_rad=math.sqrt(sum(v*v for v in self.heading_errors)/count),
                                 heading_peak_rad=max(self.heading_errors),
                                 max_request_linear_rate=self.max_request_linear_rate,
                                 max_request_angular_rate=self.max_request_angular_rate,
                                 max_feedback_linear_rate=self.max_feedback_linear_rate,
                                 max_feedback_angular_rate=self.max_feedback_angular_rate)
        if compute_times:
            ordered = sorted(compute_times)
            self.metadata['compute_ms'] = {
                str(p): ordered[min(len(ordered)-1, int(len(ordered)*p/100))]
                for p in (50, 95, 99)}
        if compute_times:
            self.metadata['compute_p99_ms'] = self.metadata['compute_ms']['99']
        if self.effective_samples:
            self.metadata['effective_samples_mean'] = sum(self.effective_samples)/len(self.effective_samples)
            self.metadata['effective_samples_min'] = min(self.effective_samples)
        self.metadata['goal_success'] = success
        limits, failures = evaluate(self.metadata)
        self.metadata.update(quality_limits=limits, quality_failures=failures)
        if failures:
            self.metadata['status'] = 'failed'
        self._write_metadata()

