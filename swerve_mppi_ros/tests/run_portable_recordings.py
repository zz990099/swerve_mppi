"""Run selected seed-42 local numerical cases with explicit plant settings."""
import argparse
import csv
import json
import hashlib
from pathlib import Path
import subprocess

from trajectory_scenarios import BENCHMARK_CASES, scaled_path
from plot_portable import render


def run(executable, output, cases=('straight',), minimum_length=12., max_speed=1.,
        servo_time=.12, max_time=130., keep_path_yaw=False, plot=True):
    root = Path(output).resolve()
    root.mkdir(parents=True, exist_ok=False)
    (root/'references').mkdir()
    (root/'cases.txt').write_text('\n'.join(cases)+'\n')
    for name in cases:
        with (root/'references'/f'{name}.csv').open('w') as f:
            csv.writer(f).writerows(scaled_path(name, minimum_length))
    (root/'experiment.json').write_text(json.dumps(dict(
        source='local numerical closed loop, not Gazebo', cases=list(cases), seed=42,
        minimum_length=minimum_length, max_speed=max_speed, servo_time=servo_time,
        max_time=max_time, keep_path_yaw=keep_path_yaw,
        executable_sha256=hashlib.sha256(Path(executable).read_bytes()).hexdigest()), indent=2)+'\n')
    command = [str(Path(executable).resolve()), str(root), '--max-speed', str(max_speed),
               '--servo-time', str(servo_time), '--max-time', str(max_time)]
    if keep_path_yaw:
        command.append('--keep-path-yaw')
    with (root/'run.log').open('w') as log:
        subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
    print((root/'run.log').read_text(), end='')
    if plot:
        render(root)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True)
    parser.add_argument('--output', required=True)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--cases', nargs='+', choices=BENCHMARK_CASES, default=['straight'])
    selection.add_argument('--all-cases', action='store_true')
    parser.add_argument('--minimum-length', type=float, default=12.)
    parser.add_argument('--max-speed', type=float, default=1.)
    parser.add_argument('--servo-time', type=float, default=.12)
    parser.add_argument('--max-time', type=float, default=130.)
    parser.add_argument('--keep-path-yaw', action='store_true', help='Diagnostic ablation only')
    parser.add_argument('--no-plot', action='store_true')
    args = parser.parse_args()
    run(args.executable, args.output, BENCHMARK_CASES if args.all_cases else args.cases,
        args.minimum_length, args.max_speed, args.servo_time, args.max_time,
        args.keep_path_yaw, not args.no_plot)
