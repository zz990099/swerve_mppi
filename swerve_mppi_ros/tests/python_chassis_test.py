"""Drive the actual companion Python chassis with compiled core/ROS support.

No ROS or Gazebo runtime is used. Actuators follow targets with configurable lag;
pose and task completion are outside the core. This is not physical validation.
"""
import argparse
import math
from pathlib import Path
import subprocess
import sys


def run(tool, root, scenario, lag):
    sys.path.insert(0, str(root))
    from swerve_gazebo_sim.bringup import load_config
    from swerve_gazebo_sim.chassis import Chassis, Command
    from swerve_gazebo_sim.kinematics import integrate_pose

    cfg = load_config(root / 'config/swerve.yaml')
    chassis = Chassis(cfg['control'], {k: cfg['geometry'][k] for k in
                      ('wheelbase', 'track_width', 'wheel_radius')})
    angles, wheels, pose = [0.0]*4, [0.0]*4, (0.0, 0.0, 0.0)
    now = 1.0
    chassis.observe(angles, wheels, now, now)
    chassis.step(now, now)
    process = subprocess.Popen([tool, scenario], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, text=True)
    times = []
    try:
        for step in range(1000):
            velocity = chassis.kinematics.forward(wheels, angles)
            values = [now, *pose, *velocity, int(chassis.actual_mode),
                      int(chassis.requested_mode), int(not chassis.confirmed),
                      chassis.request_id, *chassis.entry, int(chassis.fault), *angles, *wheels]
            process.stdin.write(' '.join(map(str, values))+'\n')
            process.stdin.flush()
            line = process.stdout.readline()
            if not line:
                raise AssertionError('Compiled controller exited')
            vx, vy, wz, mode, request, ex, ey, ew, complete, duration = map(float, line.split())
            times.append(duration)
            assert chassis.receive(Command(now, int(mode), (vx, vy, wz), int(request),
                                           (ex, ey, ew)), now, now), chassis.fault
            if complete:
                print(f'PASS {scenario} lag={lag} time={now-1:.2f} pose={pose} '
                      f'p95_ms={sorted(times)[int(.95*(len(times)-1))]:.2f}', flush=True)
                return
            for _ in range(5):
                now += 0.01
                chassis.step(now, now)
                fraction = 1-math.exp(-0.01/lag) if lag else 1
                angles = [a+fraction*(b-a) for a, b in zip(angles, chassis.angles)]
                wheels = [a+fraction*(b-a) for a, b in zip(wheels, chassis.speeds)]
                velocity = chassis.kinematics.forward(wheels, angles)
                pose = integrate_pose(pose, velocity, 0.01)
                chassis.observe(angles, wheels, now, now)
                assert not chassis.fault, chassis.fault
        raise AssertionError(f'{scenario} did not settle: pose={pose}')
    finally:
        process.stdin.close()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        process.stdout.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--tool', required=True)
    parser.add_argument('--chassis-root', type=Path, required=True)
    args = parser.parse_args()
    for lag in (0.0, 0.03):
        for name in ('straight', 'crab', 'reverse', 'spin', 'final_yaw', 'curve'):
            run(args.tool, args.chassis_root.resolve(), name, lag)
