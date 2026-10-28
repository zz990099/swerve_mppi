"""Audit one-step steering response against recorded feedback (no ROS required).

The recorded starting target is held over each feedback interval. Unobserved
target updates and source skew remain errors; this is a diagnostic comparison,
not a fitted or certified full chassis model.
"""
import argparse
import csv
import json
import math
from pathlib import Path


def response_errors(rows, time_constants):
    if not time_constants or any(not math.isfinite(t) or t <= 0 for t in time_constants):
        raise ValueError('positive finite time constants required')
    errors = {tau: [] for tau in time_constants}
    previous = None
    intervals = 0
    for row in rows:
        try:
            timestamp = float(row['feedback_stamp_s'])
            valid = (row['fault'] == '0' and row['generator_valid'] == '1'
                     and math.isfinite(timestamp))
            angles = [float(row[f'steering_rad_{w}']) for w in ('fl', 'fr', 'rl', 'rr')]
            targets = [float(row[f'target_steering_rad_{w}']) for w in ('fl', 'fr', 'rl', 'rr')]
            valid &= all(math.isfinite(v) for v in angles+targets)
        except (KeyError, ValueError, TypeError):
            valid = False
        if not valid:
            previous = None
            continue
        if previous is not None:
            start, initial, target = previous
            dt = timestamp-start
            if dt == 0:
                continue  # Same source feedback must not count as a new observation.
            if .005 <= dt <= .04:
                intervals += 1
                for tau in time_constants:
                    gain = -math.expm1(-dt/tau)
                    errors[tau].extend((a+gain*(t-a)-b)**2
                                       for a, t, b in zip(initial, target, angles))
        previous = timestamp, angles, targets
    return dict(intervals=intervals, steering_rms_rad={str(t):
                math.sqrt(sum(values)/len(values)) if values else None
                for t, values in errors.items()})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings', nargs='+', type=Path)
    parser.add_argument('--tau', nargs='+', type=float, default=[.05, .075, .1, .12, .15])
    args = parser.parse_args()
    result = {}
    for path in args.recordings:
        with path.open(newline='') as file:
            result[str(path)] = response_errors(csv.DictReader(file), args.tau)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
