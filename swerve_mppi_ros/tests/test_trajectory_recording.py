"""Recording contracts, independent of ROS and Gazebo installation."""
import csv
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
import unittest

from trajectory_recording import Recording, local_pose, path_error
from trajectory_scenarios import BENCHMARK_CASES, PATHS
from tracking_quality import evaluate


def pose(x, y, angle):
    return NS(position=NS(x=x, y=y), orientation=NS(w=math.cos(angle/2), z=math.sin(angle/2)))


def observation(x, y, angle, seconds):
    return NS(header=NS(stamp=NS(sec=seconds, nanosec=0)), pose=NS(pose=pose(x, y, angle)))


class RecordingTests(unittest.TestCase):
    def test_execution_stream_and_startup_wait_are_distinct_from_tracking_stops(self):
        with TemporaryDirectory() as temporary:
            record = Recording(temporary, 'straight', [(0, 0, 0), (1, 0, 0)], 42,
                               pose(0, 0, 0), pose(0, 0, 0), 2)
            velocity = NS(linear=NS(x=.1, y=0.), angular=NS(z=0.))
            state = NS(header=NS(stamp=NS(sec=2, nanosec=0)),
                feedback_stamp=NS(sec=2, nanosec=0), command_stamp=NS(sec=2, nanosec=0),
                command_sequence=1, execution=3, fault=0, generator_valid=True,
                requested_velocity=velocity, steering_angles=[0.]*4, wheel_speeds=[0.]*4,
                target_steering=[0.]*4, target_wheel_speeds=[0.]*4)
            record.command(NS(header=state.header, twist=velocity), 2.01)
            record.execution(state)
            state.header.stamp = NS(sec=2, nanosec=20000000)
            state.execution = 0
            record.execution(state)
            for t, speed in [(2, 0.), (3, 1.), (4, 0.)]:
                state.wheel_speeds = [speed]*4
                obs = observation(.1*(t-2), 0, 0, t)
                record.observe([obs], obs, state)
            record.close(False, 'partial diagnostic example', [])
            self.assertEqual(record.metadata['initial_wait_to_motion_s'], 1.)
            self.assertAlmostEqual(record.metadata['execution_duration_s']['presteering'], .02)
            self.assertEqual(record.metadata['tracking_stopped_fraction'], .5)
            self.assertAlmostEqual(record.metadata['stopped_fraction'], 2/3)
            with (Path(temporary)/'execution.csv').open() as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(rows[0]['command_sequence'], '1')
            self.assertEqual(rows[0]['target_steering_rad_fl'], '0.0')
            with (Path(temporary)/'commands.csv').open() as file:
                commands = list(csv.DictReader(file))
            self.assertEqual(float(commands[0]['observer_stamp_s']), 2.01)

    def test_rotated_coordinate_frames(self):
        x, y, angle = local_pose(pose(10, 21, math.pi), pose(10, 20, math.pi/2))
        self.assertAlmostEqual(x, 1)
        self.assertAlmostEqual(y, 0)
        self.assertAlmostEqual(angle, math.pi/2)
        self.assertAlmostEqual(path_error(.5, .2, [(0, 0, 0), (1, 0, 0)]), .2)

    def test_partial_failure_preserves_unique_observations(self):
        with TemporaryDirectory() as temporary:
            record = Recording(temporary, 'straight', [(0, 0, 0), (1, 0, 0)], 42,
                               pose(0, 0, 0), pose(10, 20, math.pi/2), 2)
            history = [observation(10, 20, math.pi/2, 1),
                       observation(10, 20.5, math.pi/2, 2)]
            odom = observation(.5, 0, 0, 2)
            record.observe(history, odom, None)
            record.observe(history, odom, None)
            record.close(False, 'test interruption', [10, 20])
            with (Path(temporary)/'trajectory.csv').open() as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 1)
            self.assertAlmostEqual(float(rows[0]['truth_x_m']), .5)
            self.assertAlmostEqual(float(rows[0]['truth_y_m']), 0)
            result = json.loads((Path(temporary)/'result.json').read_text())
            self.assertEqual(result['status'], 'failed')
            self.assertEqual(result['samples'], 1)
            self.assertAlmostEqual(result['final_position_error_m'], .5)

    def test_endpoint_success_cannot_hide_bad_tracking(self):
        metadata = dict(scenario='straight', reference=[(0,0,0),(1,0,0)], samples=100,
                        path_rms_m=.01, path_peak_m=.02, heading_rms_rad=.01,
                        heading_peak_rad=.02, final_position_error_m=.01,
                        final_yaw_error_rad=.01, stopped_fraction=.2, wall_duration_s=5.,
                        max_request_linear_rate=.9, max_request_angular_rate=1.3, compute_p99_ms=20.)
        self.assertEqual(evaluate(metadata)[1], [])
        metadata['path_peak_m'] = .19
        self.assertTrue(any('path_peak_m' in value for value in evaluate(metadata)[1]))

    def test_required_initial_directions_and_dense_curves(self):
        self.assertEqual(len(BENCHMARK_CASES), 12)
        self.assertEqual(PATHS['crab'][-1], (0, .8, 0))
        self.assertEqual(PATHS['reverse'][-1], (-.8, 0, 0))
        self.assertGreater(len(PATHS['smooth_s']), 30)
        ys = [p[1] for p in PATHS['smooth_s']]
        self.assertGreater(max(ys), .2)
        self.assertLess(min(ys), -.2)


if __name__ == '__main__':
    unittest.main()
