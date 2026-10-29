"""Localization regression for unobserved stationary steering yaw."""
import math
import unittest

from heading_odometry import HeadingOdometry


class HeadingTests(unittest.TestCase):
    def test_stationary_yaw_is_observed_before_lateral_translation(self):
        f = HeadingOdometry()
        for i in range(21):
            t = i*.01
            angle = -.02*min(1., t/.1)
            f.add_imu(t, angle, -.2 if t < .1 else 0.)
            result = f.wheel(t, 0., 0. if t <= .1 else 1.)
        self.assertAlmostEqual(result[2], -.02)
        self.assertAlmostEqual(result[0], .1*math.sin(.02))
        self.assertAlmostEqual(result[1], .1*math.cos(.02))

    def test_timestamp_interpolation_wrap_and_no_extrapolation(self):
        f = HeadingOdometry()
        f.add_imu(0., math.pi-.01, 1.)
        self.assertIsNone(f.wheel(.01, 0, 1))
        f.add_imu(.02, -math.pi+.01, 1.)
        self.assertIsNotNone(f.wheel(.01, 0, 1, 1.))
        f.add_imu(.04, -math.pi+.03, 1.)
        result = f.wheel(.03, 0, 0)
        self.assertAlmostEqual(result[2], 1.02)
        self.assertFalse(f.add_imu(.02, 0., 0.))
        self.assertFalse(f.add_imu(.06, float('nan'), 0.))
        self.assertIsNone(f.wheel(.03, 0, 0))

    def test_missing_sensor_gap_cannot_integrate_unobserved_motion(self):
        f = HeadingOdometry()
        f.add_imu(0., 0., 0.)
        self.assertIsNotNone(f.wheel(0., 0, 0))
        f.add_imu(1., 0., 0.)
        self.assertIsNone(f.wheel(.5, 1, 0))
        self.assertIsNone(f.wheel(1., 1, 0))
        self.assertEqual(f.x, 0.)


if __name__ == '__main__':
    unittest.main()
