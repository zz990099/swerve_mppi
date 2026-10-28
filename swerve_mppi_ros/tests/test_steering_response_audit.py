"""Offline actuator audit contracts."""
import math
import unittest

from audit_steering_response import response_errors


def sample(t, angle, fault='0'):
    row = dict(feedback_stamp_s=str(t), fault=fault, generator_valid='1')
    for wheel in ('fl', 'fr', 'rl', 'rr'):
        row[f'steering_rad_{wheel}'] = str(angle)
        row[f'target_steering_rad_{wheel}'] = '1'
    return row


class ResponseAuditTests(unittest.TestCase):
    def test_known_response_and_duplicate_source_samples(self):
        rows = [sample(i*.02, 1-math.exp(-i*.02/.1)) for i in range(10)]
        result = response_errors([item for row in rows for item in (row, row)], [.05, .1])
        self.assertEqual(result['intervals'], 9)
        self.assertLess(result['steering_rms_rad']['0.1'], 1e-14)
        self.assertGreater(result['steering_rms_rad']['0.05'], .01)

    def test_faults_gaps_and_missing_feedback_are_not_replayed(self):
        rows = [sample(0, 0), sample(.02, .1, '1'), sample(.04, .2),
                sample(1, .3), {}, sample(1.02, .4)]
        result = response_errors(rows, [.1])
        self.assertEqual(result['intervals'], 0)
        self.assertIsNone(result['steering_rms_rad']['0.1'])
        with self.assertRaises(ValueError):
            response_errors([], [float('nan')])


if __name__ == '__main__':
    unittest.main()
