#!/usr/bin/env python3
"""Exercise the offline CLI and independently check serialized motion evidence."""
import argparse
import csv
import io
from pathlib import Path
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tool', required=True)
    parser.add_argument('--summary', required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        profile = root / 'resolved.conf'
        profile.write_text(subprocess.check_output([args.tool, '--print-config'], text=True))
        result = subprocess.run([args.tool, '--config', str(profile)], capture_output=True, text=True, check=True)
        calls = root / 'calls.csv'
        calls.write_text(result.stdout)
        report = subprocess.check_output([sys.executable, args.summary, str(calls)], text=True)
        assert '720 one-step comparisons' in report and '| body_lag |' in report
        rows = list(csv.DictReader(io.StringIO(result.stdout)))
        assert len(rows) == 720
        assert all(r['encoded_model_valid'] == '1' for r in rows)
        assert any(r['perturbation'] == 'body_lag' and r['phase'] == 'brake' and
                   r['body_stationary'] == '0' and r['encoder_stationary'] == '1' for r in rows)
        assert all(r['status'] == 'nominal_agreement' for r in rows if r['perturbation'] == 'wheel_lag')
        assert max(float(r['position_error_m']) for r in rows if r['perturbation'] == 'wheel_lag') > .001
        assert all(r['status'] == 'bounded_disagreement' and r['observed_model_valid'] == '0'
                   for r in rows if r['perturbation'] == 'noise')
        for altered in (result.stdout.splitlines()[0] + '\n', result.stdout + result.stdout.splitlines()[1] + '\n',
                        result.stdout.replace('nominal_agreement', 'invalid', 1)):
            calls.write_text(altered)
            assert subprocess.run([sys.executable, args.summary, str(calls)], capture_output=True).returncode != 0
        # A complete matrix with a corrupted residual must not produce a valid report.
        rows[0]['linear_residual_upper_mps'] = '0.5'
        buffer = io.StringIO()
        writer = csv.DictWriter(buffer, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
        calls.write_text(buffer.getvalue())
        assert subprocess.run([sys.executable, args.summary, str(calls)], capture_output=True).returncode != 0
        for options in (['--unknown'], ['--config'], ['--print-config', '--print-config'],
                        ['--config', str(profile), '--config', str(profile)], ['--config', str(root / 'missing')]):
            assert subprocess.run([args.tool, *options], capture_output=True).returncode != 0
        profile.write_text('dt_s=1.1\n')
        invalid = subprocess.run([args.tool, '--config', str(profile)], capture_output=True, text=True)
        assert invalid.returncode != 0 and not invalid.stdout
        profile.write_text('max_vx_mps=.01\n')
        invalid = subprocess.run([args.tool, '--config', str(profile)], capture_output=True, text=True)
        assert invalid.returncode != 0 and not invalid.stdout
    print('motion probe CLI, matrix and serialized evidence checks passed')


if __name__ == '__main__':
    main()
