"""Run isolated physical acceptance with bounded process-group cleanup."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import yaml

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


def run_case(chassis_root, logs, scenario, seed=None, record=False, minimum_length=0., max_speed=None, path_frame='odom'):
    logs.mkdir(parents=True, exist_ok=True)
    launch_options = []
    if path_frame == 'map':
        launch_options.extend(['publish_ground_truth_map_tf:=true',
                               'x:=2.0', 'y:=-1.0', 'yaw:=0.4'])
    probe_options = ['--path-frame', path_frame]
    if seed is not None or max_speed is not None:
        config = yaml.safe_load((Path(__file__).parents[1]/'config'/'controller.yaml').read_text())
        if seed is not None:
            config['controller_server']['ros__parameters']['FollowPath']['seed'] = seed
        if max_speed is not None:
            follow = config['controller_server']['ros__parameters']['FollowPath']
            follow.update(max_speed=max_speed, physical_linear_speed=max_speed)
            chassis = yaml.safe_load((chassis_root/'swerve_gazebo_sim/config/swerve.yaml').read_text())
            chassis['control']['max_linear_speed'] = max_speed
            chassis_path = (logs/'chassis.yaml').resolve()
            chassis_path.write_text(yaml.safe_dump(chassis))
            launch_options.append(f'chassis_config:={chassis_path}')
        parameters = (logs/'parameters.yaml').resolve()
        parameters.write_text(yaml.safe_dump(config))
        launch_options.append(f'parameters:={parameters}')
        if seed is not None:
            probe_options.extend(['--seed', str(seed)])
    probe_options.extend(['--min-path-length', str(minimum_length)])
    if record:
        probe_options.extend(['--output', str(logs.resolve())])
    environment = dict(os.environ, GZ_PARTITION='mppi_'+uuid.uuid4().hex)
    simulation = probe = None
    descendants = {}
    try:
        with (logs/'gazebo.log').open('w') as sim_log, \
             (logs/'tracking.log').open('w') as tracking_log:
            simulation = subprocess.Popen([
                'ros2', 'launch', 'swerve_mppi_ros', 'demo.launch.py',
                'headless:=true', 'publish_ground_truth:=true', *launch_options],
                stdout=sim_log, stderr=subprocess.STDOUT,
                env=environment, start_new_session=True)
            probe = subprocess.Popen([
                sys.executable, '-u', str(Path(__file__).with_name('gazebo_tracking.py')),
                '--chassis-root', str(chassis_root), '--scenario', scenario, *probe_options],
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
    failures=[]
    for scenario in ('straight', 'crab', 'reverse', 'spin', 'final_yaw', 'curve',
                     'out_and_back', 'closed_loop', 'combined', 's_curve', 'cancel'):
        print(f'Independent physical scenario: {scenario}', flush=True)
        try:
            run_case(args.chassis_root, args.logs/scenario, scenario, seed=42, record=True)
        except (RuntimeError, TimeoutError) as error:
            failures.append(scenario)
            print(f"FAIL {scenario}: {error}",flush=True)
    if failures:
        raise RuntimeError(f"Physical scenarios failed: {failures}")


if __name__ == '__main__':
    main()
