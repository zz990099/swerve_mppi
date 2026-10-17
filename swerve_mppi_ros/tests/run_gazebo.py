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
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--chassis-root', required=True)
    parser.add_argument('--logs', type=Path, required=True)
    args = parser.parse_args()
    args.logs.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, GZ_PARTITION='mppi_'+uuid.uuid4().hex)
    simulation = probe = None
    descendants = {}
    try:
        with (args.logs/'gazebo.log').open('w') as sim_log, \
             (args.logs/'tracking.log').open('w') as tracking_log:
            simulation = subprocess.Popen([
                'ros2', 'launch', 'swerve_mppi_ros', 'demo.launch.py',
                'headless:=true', 'publish_ground_truth:=true'],
                stdout=sim_log, stderr=subprocess.STDOUT,
                env=environment, start_new_session=True)
            probe = subprocess.Popen([
                sys.executable, '-u', str(Path(__file__).with_name('gazebo_tracking.py')),
                '--chassis-root', args.chassis_root],
                stdout=tracking_log, stderr=subprocess.STDOUT,
                env=environment, start_new_session=True)
            deadline = time.monotonic()+600
            while probe.poll() is None:
                try:
                    for child in psutil.Process(simulation.pid).children(recursive=True):
                        descendants[(child.pid, child.create_time())] = child
                except psutil.NoSuchProcess:
                    pass
                if simulation.poll() is not None:
                    raise RuntimeError('Gazebo launch exited before acceptance')
                if time.monotonic() > deadline:
                    raise TimeoutError('Physical acceptance exceeded 600 seconds')
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
            file = args.logs/name
            if file.exists():
                print(f'{name}:\n{file.read_text()[-12000:]}', flush=True)


if __name__ == '__main__':
    main()
