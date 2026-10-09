#!/usr/bin/env python3
"""Compare the core predictor with the actual companion Python chassis, no ROS."""
import argparse
import itertools
import math
from pathlib import Path
import subprocess
import sys


def run(tool, root, wheel_cap, wheel_accel):
    sys.path.insert(0, str(root))
    from swerve_gazebo_sim.chassis import Chassis, Command, Mode, Phase
    c = dict(max_linear_speed=.8, max_angular_speed=.8, max_linear_acceleration=.9,
             max_angular_acceleration=1.3, max_wheel_speed=wheel_cap / .1,
             max_wheel_acceleration=wheel_accel / .1, max_steering_rate=2.5,
             drive_steering_limit=.2, stopped_wheel_speed=.05,
             steering_alignment_tolerance=.05, steering_alignment_duration=.05,
             mode_switch_timeout=5., cmd_timeout=.5, wall_timeout=.5,
             feedback_timeout=.25, update_rate=100.)
    count = 0
    for source, target in itertools.permutations(Mode, 2):
        plant = Chassis(c, dict(wheelbase=.6, track_width=.5, wheel_radius=.1))
        plant.actual_mode = plant.requested_mode = source
        if source == Mode.SPIN:
            plant.angles = plant.kinematics.inverse(0, 0, 1, [0] * 4)[1]
        elif source == Mode.CRAB:
            plant.angles = [math.pi / 2] * 4
        plant.observe(plant.angles, [0] * 4, 0., 0.)
        plant.last_time = 0.
        rows, expected = [], []
        stamp = 0.
        entry = (0., 0., 0.)
        request_id = 0
        # Exercise warm acceleration, reversal, zero, all six explicit changes,
        # raw entry curvature, saturation, signed +/-90 ties and realignment.
        source_drive = {Mode.DUAL_ACKERMANN: (.6, 0., .3), Mode.SPIN: (0., 0., .7),
                        Mode.CRAB: (0., .6, 0.)}[source]
        target_drive = {Mode.DUAL_ACKERMANN: (.2, 0., .7), Mode.SPIN: (0., 0., -.7),
                        Mode.CRAB: (0., -.6, 0.)}[target]
        commands = [source_drive] * 110 + [tuple(-v for v in source_drive)] * 110 + [(0., 0., 0.)] * 50
        commands += [(0., 0., 0.)] * 150 + [tuple(8 * v for v in target_drive)] * 100
        if target == Mode.CRAB:
            commands += [(.6, 0., 0.)] * 130 + [(0., 0., 0.)] * 20 + [(0., .5, 0.)] * 130
        commands += [(0., 0., 0.)] * 50
        before_confirmation = False
        for tick, velocity in enumerate(commands):
            now = stamp + .01
            new_request = tick == 270
            if new_request:
                request_id += 1
                entry = target_drive
            measured_angles, measured_speeds = list(plant.angles), list(plant.speeds)
            plant.observe(measured_angles, measured_speeds, now, now)
            cmd = Command(now, target if request_id else source, velocity, request_id, entry)
            # Acceptance is at the previous endpoint, matching the explicit
            # predictor seed. Same-mode automatic entry happens on this tick.
            assert plant.receive(cmd, stamp, stamp)
            plant.step(now, now)
            assert plant.fault == 0, (source, target, tick, plant.fault)
            rows.append(' '.join(map(str, (now-stamp, round(stamp * 1e9), int(target if new_request else plant.actual_mode),
                       *velocity, int(new_request), *entry, *measured_angles,
                       *(v * .1 for v in measured_speeds)))))
            expected.append((int(plant.phase), *plant.angles, *(v * .1 for v in plant.speeds),
                             *plant.alignment, *plant.limited))
            if request_id and not plant.confirmed:
                before_confirmation = True
                if plant.phase == Phase.ALIGNING:
                    assert not any(plant.speeds)
            stamp = now
        assert before_confirmation and plant.actual_mode == target and plant.confirmed
        result = subprocess.run([tool, str(wheel_cap), str(wheel_accel)], input='\n'.join(rows) + '\n',
                                text=True, capture_output=True, check=True)
        actual = [tuple(map(float, row.split())) for row in result.stdout.splitlines()]
        assert len(actual) == len(expected)
        for tick, (a, e) in enumerate(zip(actual, expected)):
            assert a[0] == e[0], (source, target, tick, 'phase', a[0], e[0])
            assert max(abs(x-y) for x, y in zip(a[1:], e[1:])) < 2e-10, (source, target, tick, a, e)
            count += 1
    return count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tool', required=True)
    parser.add_argument('--chassis-root', required=True)
    args = parser.parse_args()
    root = Path(args.chassis_root)
    assert (root / 'swerve_gazebo_sim/chassis.py').is_file(), 'current companion source is required'
    count = sum(run(args.tool, root, cap, accel) for cap, accel in ((2., 4.), (.2, .4)))
    print(f'{count} Python/core command-cycle comparisons passed (six directed mode changes, two limit sets)')


if __name__ == '__main__':
    main()
