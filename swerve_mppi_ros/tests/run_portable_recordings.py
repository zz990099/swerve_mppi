"""Run one seed-42 numerical round at original route lengths; not Gazebo."""
import argparse
import csv
from pathlib import Path
import subprocess

from trajectory_scenarios import BENCHMARK_CASES, PATHS
from plot_portable import render


def run(executable, output):
    root = Path(output).resolve()
    (root/'references').mkdir(parents=True, exist_ok=True)
    (root/'cases.txt').write_text('\n'.join(BENCHMARK_CASES)+'\n')
    for name in BENCHMARK_CASES:
        with (root/'references'/f'{name}.csv').open('w') as f:
            csv.writer(f).writerows(PATHS[name])
    subprocess.run([str(Path(executable).resolve()), str(root)], check=True)
    render(root)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    run(args.executable, args.output)
