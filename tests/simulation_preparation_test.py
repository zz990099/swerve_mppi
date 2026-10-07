"""Offline source-audit/export regression; requires the pinned companion checkout."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    import yaml

    parser = argparse.ArgumentParser()
    parser.add_argument('--tool', required=True, type=Path)
    parser.add_argument('--companion', required=True, type=Path)
    args = parser.parse_args()
    cases = 0

    def prepare(source, output, *extra, ok=True):
        nonlocal cases
        result = subprocess.run([
            sys.executable, str(ROOT / 'tools/prepare_simulation.py'),
            '--companion', str(source), '--output-dir', str(output),
            '--tool', str(args.tool.resolve()), *map(str, extra),
        ], capture_output=True, text=True)
        assert (result.returncode == 0) == ok, (result.stdout, result.stderr)
        cases += 1
        return result

    with tempfile.TemporaryDirectory() as folder:
        temp = Path(folder)
        source = temp / 'companion'
        baseline = json.loads((ROOT / 'preparation/simulation/baseline.json').read_text())
        for name in baseline['audited_blobs']:
            path = source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(args.companion / name, path)
        output = temp / 'nominal'
        prepare(source, output)
        manifest = json.loads((output / 'manifest.json').read_text())
        assert manifest['status'] == 'candidate_prepared_integration_blocked'
        assert manifest['candidate_startup_compatible']
        assert not any(manifest[k] for k in ('runtime_export', 'integration_ready', 'physical_acceptance'))
        assert len(manifest['remaining_gates']) == 10
        for name, sha in manifest['artifact_sha256'].items():
            assert hashlib.sha256((output / name).read_bytes()).hexdigest() == sha
        rows = json.loads((output / 'provenance.json').read_text())
        assert len(rows) == 136
        fields = {(row['participant'], row['name']): row for row in rows}
        assert fields['planner', 'compute_budget_ratio']['value'] == '0.59999999999999998'
        assert fields['executor', 'confirmation_timeout_s']['value'] == '5'
        assert fields['executor', 'minimum_mode_dwell_s']['value'] == '1'
        assert fields['executor', 'alignment_min_s']['value'] == '0.25'
        assert 'explicit core assumption' in fields['executor', 'max_linear_decel_mps2']['source']
        prior = (output / 'manifest.json').read_bytes()
        prepare(source, output, ok=False)
        assert (output / 'manifest.json').read_bytes() == prior
        # Every pinned source is checked, including fields not imported by this script.
        for i, name in enumerate(baseline['audited_blobs']):
            path = source / name
            original = path.read_bytes()
            path.write_bytes(original + b'\n')
            failed = temp / f'drift-{i}'
            result = prepare(source, failed, ok=False)
            assert name in result.stderr
            failure = json.loads((failed / 'manifest.json').read_text())
            assert failure['status'] == 'incomplete' and not failure['integration_ready']
            path.write_bytes(original)
        missing = source / 'msg/ChassisState.msg'
        original = missing.read_bytes()
        missing.unlink()
        prepare(source, temp / 'missing-source', ok=False)
        missing.write_bytes(original)
        prepare(source, source / 'forbidden-output', ok=False)
        assert not (source / 'forbidden-output').exists()
        custom = temp / 'custom.yaml'
        config = yaml.safe_load((source / 'config/swerve.yaml').read_text())
        config['geometry'].update(wheelbase=0.72, track_width=0.65, wheel_radius=0.12)
        config['control'].update(max_wheel_speed=15, max_wheel_acceleration=30,
                                 odom_frame='local_map', odom_child_frame='other_body')
        custom.write_text(yaml.safe_dump(config))
        changed = temp / 'custom'
        prepare(source, changed, '--config', custom, '--namespace', 'robot1', '--prefix', 'auto')
        m = json.loads((changed / 'manifest.json').read_text())
        assert m['parameters']['body_frame'] == 'robot1_base_footprint'
        assert m['parameters']['odom_frame'] == 'local_map'
        assert math.isclose(m['parameters']['max_wheel_speed_mps'], 1.8, abs_tol=1e-15)
        assert math.isclose(m['parameters']['max_wheel_accel_mps2'], 3.6, abs_tol=1e-15)
        assert m['parameters']['robot_radius_m'] > 0.6 - 0.05
        assert m['prefix'] == 'robot1_' and m['namespace'] == '/robot1'
        assert m['remaining_gates'][-1]['id'] == 'odometry_child_frame'
        base = temp / 'mechanical-base.conf'
        base.write_text((ROOT / 'config/default.conf').read_text().replace(
            'steering_limit_rad = 1.5707963267948966', 'steering_limit_rad = 2'))
        mechanical = temp / 'mechanical'
        prepare(source, mechanical, '--base', base)
        gates = json.loads((mechanical / 'manifest.json').read_text())['remaining_gates']
        assert any(gate['id'] == 'steering_travel' for gate in gates)
        base.write_text('wheelbase_m=0.6\n')
        failed_base = temp / 'partial-base'
        prepare(source, failed_base, '--base', base, ok=False)
        failure = json.loads((failed_base / 'manifest.json').read_text())
        assert failure['status'] == 'incomplete' and 'error' in failure
        assert (failed_base / 'preflight.log').exists()
        custom.write_text(yaml.safe_dump(config) + 'geometry: {}\n')
        prepare(source, temp / 'duplicate-yaml', '--config', custom, ok=False)
        config['control']['steering_limit'] = 0.3
        custom.write_text(yaml.safe_dump(config))
        prepare(source, temp / 'invalid-yaml', '--config', custom, ok=False)
        for i, extra in enumerate([
            ['--clock-epoch', '0'], ['--clock-epoch', str(2 ** 64)],
            ['--clock-domain', 'bad clock'], ['--namespace', 'bad namespace'],
        ]):
            prepare(source, temp / f'metadata-{i}', *extra, ok=False)
        # The owner source files stay byte-for-byte unchanged across all runs.
        for name, sha in baseline['audited_blobs'].items():
            data = (source / name).read_bytes()
            assert hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest() == sha
    print(f'{cases} offline companion preparation cases passed')


if __name__ == '__main__':
    main()
