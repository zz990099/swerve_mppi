"""Run isolated physical acceptance with bounded process-group cleanup."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import psutil


def stop(process):
    if process is None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)


def run_case(chassis_root, logs, scenario):
    logs.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, GZ_PARTITION='mppi_'+uuid.uuid4().hex)
    simulation = probe = None
    descendants = {}
    try:
        with (logs/'gazebo.log').open('w') as sim_log, \
             (logs/'tracking.log').open('w') as tracking_log:
            simulation = subprocess.Popen([
                'ros2', 'launch', 'swerve_mppi_ros', 'demo.launch.py',
                'headless:=true', 'publish_ground_truth:=true'],
                stdout=sim_log, stderr=subprocess.STDOUT,
                env=environment, start_new_session=True)
            probe = subprocess.Popen([
                sys.executable, '-u', str(Path(__file__).with_name('gazebo_tracking.py')),
                '--chassis-root', chassis_root, '--scenario', scenario],
                stdout=tracking_log, stderr=subprocess.STDOUT,
                env=environment, start_new_session=True)
            deadline = time.monotonic()+200
            while probe.poll() is None:
                try:
                    for child in psutil.Process(simulation.pid).children(recursive=True):
                        descendants[(child.pid, child.create_time())] = child
                except psutil.NoSuchProcess:
                    pass
                if simulation.poll() is not None:
                    raise RuntimeError('Gazebo launch exited before acceptance')
                if time.monotonic() > deadline:
                    raise TimeoutError('Physical acceptance exceeded 200 seconds')
                time.sleep(.2)
            if probe.returncode:
                raise RuntimeError(f'Physical acceptance failed: {probe.returncode}')
    finally:
        stop(probe)
        stop(simulation)
        remaining = []
        for child in descendants.values():
            try:
                child.terminate()
                remaining.append(child)
            except psutil.NoSuchProcess:
                pass
        _, alive = psutil.wait_procs(remaining, timeout=3)
        for child in alive:
            try:
                child.kill()
            except psutil.NoSuchProcess:
                pass
        for name in ('tracking.log', 'gazebo.log'):
            file = logs/name
            if file.exists():
                print(f'{name}:\n{file.read_text()[-12000:]}', flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--chassis-root', required=True)
    parser.add_argument('--logs', type=Path, required=True)
    args = parser.parse_args()
    for scenario in ('straight', 'crab', 'reverse', 'spin', 'final_yaw', 'curve',
                     'out_and_back', 'closed_loop', 'combined', 's_curve'):
        print(f'Independent physical scenario: {scenario}', flush=True)
        run_case(args.chassis_root, args.logs/scenario, scenario)


if __name__ == '__main__':
    main()
