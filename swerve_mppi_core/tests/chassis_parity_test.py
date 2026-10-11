#!/usr/bin/env python3
"""Compare the core predictor with the actual companion Python chassis, no ROS."""
import argparse
import itertools
import math
from pathlib import Path
import subprocess
import sys


def simple_yaml(path):
    sections, section = {}, None
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line:
            continue
        if not line.startswith(" "):
            section = line.removesuffix(":")
            sections[section] = {}
            continue
        key, value = line.strip().split(":", 1)
        sections[section][key] = value.strip().strip('"')
    return sections


def message_fields(path):
    fields = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if line and "=" not in line:
            fields.append(" ".join(line.split()))
    return fields


def validate_contract(tool, root):
    command = message_fields(root / "msg/ChassisCommand.msg")
    state = message_fields(root / "msg/ChassisState.msg")
    assert command == [
        "std_msgs/Header header", "uint8 mode", "geometry_msgs/Twist velocity",
        "uint64 request_id", "geometry_msgs/Twist entry_velocity",
    ], command
    assert state == [
        "std_msgs/Header header", "uint64 request_id", "uint8 actual_mode",
        "uint8 requested_mode", "uint8 phase", "bool confirmed", "uint8 fault",
        "geometry_msgs/Twist velocity", "float64[4] steering_angles",
        "float64[4] wheel_speeds", "geometry_msgs/Twist accepted_entry_velocity",
        "float64[4] accepted_steering",
    ], state

    resolved = subprocess.run(
        [tool, "--print-interface"], text=True, capture_output=True, check=True
    ).stdout
    core = dict(line.split("=", 1) for line in resolved.splitlines())
    core = {key: float(value) for key, value in core.items()}
    cfg = simple_yaml(root / "config/swerve.yaml")
    geometry, control = cfg["geometry"], cfg["control"]
    radius = float(geometry["wheel_radius"])
    expected = {
        "wheelbase_m": float(geometry["wheelbase"]),
        "track_m": float(geometry["track_width"]),
        "wheel_radius_m": radius,
        "chassis_period_s": 1.0 / float(control["update_rate"]),
        "max_wheel_speed_mps": float(control["max_wheel_speed"]) * radius,
        "max_wheel_accel_mps2": float(control["max_wheel_acceleration"]) * radius,
        "max_steer_rate_radps": float(control["max_steering_rate"]),
        "steering_limit_rad": float(control["steering_limit"]),
        "chassis_max_linear_speed_mps": float(control["max_linear_speed"]),
        "chassis_max_angular_speed_radps": float(control["max_angular_speed"]),
        "max_linear_accel_mps2": float(control["max_linear_acceleration"]),
        "max_angular_accel_radps2": float(control["max_angular_acceleration"]),
        "drive_steering_limit_rad": float(control["drive_steering_limit"]),
        "steering_tolerance_rad": float(control["steering_alignment_tolerance"]),
        "alignment_min_s": float(control["steering_alignment_duration"]),
        "confirmation_timeout_s": float(control["mode_switch_timeout"]),
        "stopped_wheel_speed_mps": float(control["stopped_wheel_speed"]) * radius,
    }
    assert set(core) == set(expected) | {"command_lifetime_s"}, core
    for key, value in expected.items():
        assert abs(core[key] - value) <= 1e-12 * max(1.0, abs(value)), (key, core[key], value)
    assert core["command_lifetime_s"] <= float(control["cmd_timeout"])


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
    validate_contract(args.tool, root)
    count = sum(run(args.tool, root, cap, accel) for cap, accel in ((2., 4.), (.2, .4)))
    print(f'{count} Python/core command-cycle comparisons passed (six directed mode changes, two limit sets)')


if __name__ == '__main__':
    main()
