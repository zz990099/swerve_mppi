"""Timestamp-aligned steering identification and execution timing diagnostics.

Targets use chassis publication stamps; measurements use encoder stamps.
Recorded targets are zero-order held, with optional transport delay. This fits
the recorded target/encoder chain only, not unrecorded intermediate targets or
tire dynamics. Use different seeds/files for identification and validation.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path


WHEELS = ('fl', 'fr', 'rl', 'rr')


def percentiles(values):
    values = sorted(v for v in values if math.isfinite(v))
    return {str(p): values[min(len(values)-1, int(len(values)*p/100))]
            if values else None for p in (50, 95, 99)}


def audit(rows, taus=(.05, .075, .1, .12, .15), delays=(0., .01, .02), wheel_acceleration=40.):
    if not taus or not delays or any(not math.isfinite(v) or v <= 0 for v in taus) or any(
            not math.isfinite(v) or v < 0 for v in delays):
        raise ValueError('finite positive tau and nonnegative delay required')
    if not math.isfinite(wheel_acceleration) or wheel_acceleration <= 0:
        raise ValueError('finite positive wheel acceleration required')
    rows = list(rows)
    targets, wheel_targets, measurements, feedback_age, command_age = {}, {}, [], [], []
    previous_stamp = None
    epoch_stamp = None
    for row in rows:
        try:
            publication = float(row['chassis_stamp_s'])
            encoder = float(row['feedback_stamp_s'])
            command = float(row['command_stamp_s'])
            angles = [float(row[f'steering_rad_{w}']) for w in WHEELS]
            target = [float(row[f'target_steering_rad_{w}']) for w in WHEELS]
            valid = row['fault'] == '0' and row['generator_valid'] == '1' and all(
                math.isfinite(v) for v in [publication, encoder, command]+angles+target)
        except (KeyError, TypeError, ValueError):
            valid = False
        if not valid:
            measurements.append(None)
            previous_stamp = None
            continue
        if epoch_stamp is not None and encoder < epoch_stamp:
            raise ValueError('clock reset: audit each clock epoch separately')
        epoch_stamp = encoder
        # Target publications and encoder samples are independent streams.
        # Preserve target changes even if this message repeats an encoder stamp.
        targets[publication] = target
        try:
            wheels = [float(row[f'wheel_rad_s_{w}']) for w in WHEELS]
            wheel_target = [float(row[f'target_wheel_rad_s_{w}']) for w in WHEELS]
            if not all(math.isfinite(v) for v in wheels+wheel_target):
                raise ValueError('nonfinite wheel observation')
            wheel_targets[publication] = wheel_target
        except (KeyError, ValueError, TypeError):
            wheels = None
        if encoder == previous_stamp:
            continue
        previous_stamp = encoder
        measurements.append((encoder, angles, wheels))
        feedback_age.append(publication-encoder)
        command_age.append(publication-command)
    times = sorted(targets)
    results = []
    for tau in taus:
        for delay in delays:
            squared, count = 0., 0
            for a, b in zip(measurements, measurements[1:]):
                if a is None or b is None:
                    continue
                start, initial, _ = a
                end, observed, _ = b
                if not .005 <= end-start <= .04:
                    continue
                index = bisect.bisect_right(times, start-delay)-1
                if index < 0:
                    continue
                prediction = list(initial)
                cursor = start
                # Integrate every recorded target change, with no future target
                # substituted for the target that was active at the encoder time.
                while cursor < end-1e-12:
                    stop = min(end, times[index+1]+delay) if index+1 < len(times) else end
                    gain = -math.expm1(-(stop-cursor)/tau)
                    prediction = [v+gain*(t-v) for v, t in zip(prediction, targets[times[index]])]
                    cursor = stop
                    index += 1
                squared += sum((v-o)**2 for v, o in zip(prediction, observed))
                count += len(WHEELS)
            results.append(dict(tau_s=tau, delay_s=delay, joint_samples=count,
                                rms_rad=math.sqrt(squared/count) if count else None))
    finite = [r for r in results if r['rms_rad'] is not None]
    wheel_errors, acceleration_samples = [], []
    wheel_times = sorted(wheel_targets)
    for a, b in zip(measurements, measurements[1:]):
        if a is None or b is None or a[2] is None or b[2] is None:
            continue
        dt = b[0]-a[0]
        if not .005 <= dt <= .04:
            continue
        index = bisect.bisect_right(wheel_times, a[0])-1
        if index < 0:
            continue
        prediction = list(a[2])
        cursor = a[0]
        while cursor < b[0]-1e-12:
            stop = min(b[0], wheel_times[index+1]) if index+1 < len(wheel_times) else b[0]
            step = wheel_acceleration*(stop-cursor)
            prediction = [v+max(-step, min(step, t-v))
                          for v, t in zip(prediction, wheel_targets[wheel_times[index]])]
            cursor = stop
            index += 1
        wheel_errors.extend((v-o)**2 for v, o in zip(prediction, b[2]))
        acceleration_samples.extend(abs(v-o)/dt for v, o in zip(a[2], b[2]))
    return dict(best=min(finite, key=lambda r: r['rms_rad']) if finite else None,
                candidates=results, feedback_age_s=percentiles(feedback_age),
                accepted_command_age_s=percentiles(command_age),
                wheel_response_rms_rad_s=(math.sqrt(sum(wheel_errors)/len(wheel_errors))
                                          if wheel_errors else None),
                wheel_acceleration_rad_s2=percentiles(acceleration_samples),
                wheel_acceleration_assumed_rad_s2=wheel_acceleration,
                odometry_truth=pose_errors(rows))


def pose_errors(rows):
    """Compare source-time-interpolated odometry and truth in the initial body frame."""
    odometry, truth = {}, {}
    for row in rows:
        try:
            ot, tt = float(row['odom_time_s']), float(row['sim_time_s'])
            o = [float(row[f'odom_{key}']) for key in ('x_m', 'y_m', 'yaw_rad')]
            t = [float(row[f'truth_{key}']) for key in ('x_m', 'y_m', 'yaw_rad')]
            if all(math.isfinite(v) for v in [ot, tt]+o+t):
                odometry[ot], truth[tt] = o, t
        except (KeyError, ValueError, TypeError):
            continue
    times = sorted(odometry)
    position, heading = [], []
    for t, observed in sorted(truth.items()):
        i = bisect.bisect_right(times, t)
        if not 0 < i < len(times) or times[i]-times[i-1] > .1:
            continue
        a, b = odometry[times[i-1]], odometry[times[i]]
        f = (t-times[i-1])/(times[i]-times[i-1])
        position.append(math.hypot(a[0]+f*(b[0]-a[0])-observed[0],
                                   a[1]+f*(b[1]-a[1])-observed[1]))
        yaw = a[2]+f*math.remainder(b[2]-a[2], 2*math.pi)
        heading.append(abs(math.remainder(yaw-observed[2], 2*math.pi)))
    return dict(samples=len(position), position_rms_m=(math.sqrt(sum(v*v for v in position)/len(position))
                if position else None), yaw_rms_rad=(math.sqrt(sum(v*v for v in heading)/len(heading))
                if heading else None))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings', nargs='+', type=Path)
    parser.add_argument('--tau', nargs='+', type=float, default=[.05, .075, .1, .12, .15])
    parser.add_argument('--delay', nargs='+', type=float, default=[0., .01, .02])
    parser.add_argument('--wheel-acceleration', type=float, default=40.)
    args = parser.parse_args()
    result = {}
    for path in args.recordings:
        with path.open(newline='') as file:
            result[str(path)] = audit(csv.DictReader(file), args.tau, args.delay, args.wheel_acceleration)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
