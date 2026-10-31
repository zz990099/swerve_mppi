#!/usr/bin/env python3
"""Incrementally build and run one local C++ scenario without ROS or CI."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--chassis-root', required=True, type=Path,
                        help='swerve_chassis_core directory, or its repository root')
    parser.add_argument('--case', dest='cases', nargs='+', default=['straight'])
    parser.add_argument('--build-dir', type=Path, default=Path('build-local'))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--cmake', default=shutil.which('cmake'))
    parser.add_argument('--minimum-length', type=float, default=12.)
    parser.add_argument('--max-speed', type=float, default=1.)
    parser.add_argument('--servo-time', type=float, default=.12)
    parser.add_argument('--max-time', type=float, default=130.)
    args = parser.parse_args()
    if args.cmake:
        args.cmake = shutil.which(args.cmake) or str(Path(args.cmake).resolve())
    if not args.cmake:
        parser.error('cmake is required; install cmake or pass --cmake')
    root = Path(__file__).resolve().parents[1]
    chassis = args.chassis_root.resolve()
    if (chassis/'swerve_chassis_core').is_dir():
        chassis = chassis/'swerve_chassis_core'
    build = args.build_dir.resolve()
    output = args.output or root/'artifacts-local'/(
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+'-'.join(args.cases))
    subprocess.run([args.cmake, '-S', str(root/'tools/local'), '-B', str(build),
                    '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_CXX_FLAGS=-Werror',
                    '-DSWERVE_CHASSIS_SOURCE_DIR='+str(chassis)], check=True)
    subprocess.run([args.cmake, '--build', str(build), '--target', 'record_portable',
                    '--parallel', '2'], check=True)
    subprocess.run([sys.executable, str(root/'swerve_mppi_ros/tests/run_portable_recordings.py'),
                    '--executable', str(build/'record_portable'), '--output', str(output),
                    '--cases', *args.cases, '--minimum-length', str(args.minimum_length),
                    '--max-speed', str(args.max_speed), '--servo-time', str(args.servo_time),
                    '--max-time', str(args.max_time)], check=True)
    print('Results:', output)


if __name__ == '__main__':
    main()
