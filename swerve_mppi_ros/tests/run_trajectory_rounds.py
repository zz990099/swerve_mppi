"""Repeat isolated Nav2/Gazebo cases, record CSV/JSON and render one PNG per round."""
import argparse
import json
import os
from pathlib import Path
import subprocess

from plot_trajectories import plot_round
from run_gazebo import run_case
from trajectory_scenarios import BENCHMARK_CASES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--chassis-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rounds', type=int, default=3)
    parser.add_argument('--start-round', type=int, default=1)
    parser.add_argument('--seed-base', type=int, default=42)
    parser.add_argument('--commit', default=os.environ.get('GITHUB_SHA'))
    parser.add_argument('--cases', nargs='+', choices=BENCHMARK_CASES, default=BENCHMARK_CASES)
    args = parser.parse_args()
    if args.rounds < 1 or args.start_round < 1 or args.seed_base < 0:
        parser.error('rounds/start-round must be positive and seed-base nonnegative')
    repo = Path(__file__).resolve().parents[2]
    commit = args.commit or subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    failures = []
    for number in range(args.start_round, args.start_round+args.rounds):
        directory = args.output/f'round_{number:02d}'
        # Never overwrite an earlier experiment or combine samples from different runs.
        directory.mkdir(parents=True, exist_ok=False)
        seed = args.seed_base+number-1
        (directory/'round.json').write_text(json.dumps(dict(round=number, seed=seed,
            commit=commit, cases=list(args.cases)), indent=2)+'\n')
        try:
            for name in args.cases:
                print(f'ROUND {number} SEED {seed} CASE {name}', flush=True)
                try:
                    run_case(args.chassis_root.resolve(), directory/name, name,
                             seed=seed, record=True)
                except (RuntimeError, TimeoutError) as error:
                    failures.append((number, name, str(error)))
                    print(f'FAIL {number}/{name}: {error}', flush=True)
        finally:
            # A failed case must remain visible, not disappear from the round montage.
            print(f'PNG {plot_round(directory)}', flush=True)
    if failures:
        raise RuntimeError(f'Trajectory benchmark failures: {failures}')


if __name__ == '__main__':
    main()
