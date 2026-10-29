import math
import unittest

from audit_execution import audit


def row(t, measured, target=1.):
    sample = dict(chassis_stamp_s=str(t), feedback_stamp_s=str(t), command_stamp_s=str(t-.01),
                  fault='0', generator_valid='1')
    for wheel in ('fl', 'fr', 'rl', 'rr'):
        sample[f'steering_rad_{wheel}'] = str(measured)
        sample[f'target_steering_rad_{wheel}'] = str(target)
    return sample


class ExecutionAuditTests(unittest.TestCase):
    def test_wheel_response_and_source_time_pose_comparison(self):
        rows = []
        for t, wheel in [(0., 0.), (.01, .4), (.02, .8), (.03, 1.)]:
            sample = row(t, 0.)
            for name in ('fl', 'fr', 'rl', 'rr'):
                sample[f'wheel_rad_s_{name}'] = str(wheel)
                sample[f'target_wheel_rad_s_{name}'] = '1'
            sample.update(odom_time_s=str(t), sim_time_s=str(t), odom_x_m=str(t),
                          odom_y_m='0', odom_yaw_rad='0', truth_x_m=str(t+.02),
                          truth_y_m='0', truth_yaw_rad='0')
            rows.append(sample)
        result = audit(rows)
        self.assertLess(result['wheel_response_rms_rad_s'], 1e-14)
        self.assertAlmostEqual(result['wheel_acceleration_rad_s2']['95'], 40.)
        self.assertEqual(result['odometry_truth']['samples'], 3)
        self.assertAlmostEqual(result['odometry_truth']['position_rms_m'], .02)

    def test_known_response_and_duplicate_feedback(self):
        rows = [row(i*.01, 1-math.exp(-i*.01/.1)) for i in range(30)]
        result = audit([r for r in rows for _ in range(2)], [.05, .1, .15], [0.])
        self.assertEqual(result['best']['tau_s'], .1)
        self.assertLess(result['best']['rms_rad'], 1e-14)
        self.assertEqual(result['best']['joint_samples'], 29*4)

    def test_future_target_is_not_used_for_earlier_encoder_interval(self):
        rows = [row(0., 0., 0.), row(.02, 0., 1.),
                row(.04, 1-math.exp(-.02/.1), 1.)]
        self.assertLess(audit(rows, [.1], [0.])['best']['rms_rad'], 1e-14)

    def test_target_changes_survive_duplicate_encoder_stamps(self):
        rows = [row(0., 0., 0.), row(.01, 0., 1.),
                row(.02, 1-math.exp(-.01/.1), 1.)]
        rows[1]['feedback_stamp_s'] = '0'
        result = audit(rows, [.1], [0.])
        self.assertEqual(result['best']['joint_samples'], 4)
        self.assertLess(result['best']['rms_rad'], 1e-14)

    def test_faults_gaps_and_clock_resets(self):
        bad = row(.01, .1)
        bad['fault'] = '1'
        self.assertIsNone(audit([row(0., 0.), bad, row(.02, .2)])['best'])
        self.assertIsNone(audit([row(0., 0.), row(1., .2)])['best'])
        with self.assertRaises(ValueError):
            audit([row(1., 0.), row(.5, 0.)])
        with self.assertRaises(ValueError):
            audit([], [0.])
