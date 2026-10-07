"""Exercise offline CLI failures through the same public startup contracts."""

import argparse
import json
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tool', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    args = parser.parse_args()
    tool = args.tool.resolve()
    cases = 0

    def run(*arguments, ok=True):
        nonlocal cases
        result = subprocess.run([str(tool), *map(str, arguments)],
                                text=True, capture_output=True)
        assert (result.returncode == 0) == ok, (arguments, result.stdout, result.stderr)
        if not ok:
            assert 'preflight rejected:' in result.stderr
            assert not result.stdout
        cases += 1
        return result.stdout

    schema = json.loads(run('schema'))
    assert len(schema['fields']) == 68
    assert sum(f['scope'] == 'execution' for f in schema['fields']) == 33
    with tempfile.TemporaryDirectory() as folder:
        root = Path(folder)
        left, right, overrides = (root / name for name in ('left.conf', 'right.conf', 'overrides'))
        left.write_bytes(args.profile.read_bytes())
        overrides.write_text('compute_budget_ratio=0.6\nconfirmation_timeout_s=5\n')
        right.write_text(run('resolve', left, overrides))
        meta_text = '\n'.join([
            'schema_version=1', 'world_frame=odom', 'body_frame=base_footprint',
            'clock_domain=gazebo/world', 'clock_epoch=18446744073709551615',
            'motion_policy=NominalEncoderOnly', 'max_feedback_age_s=0.15',
            'max_command_age_s=0.15', 'period_tolerance_ratio=0.25', 'watchdog_s=0.15', '',
        ])
        meta, peer_meta = root / 'local.metadata', root / 'peer.metadata'
        meta.write_text(meta_text)
        peer_meta.write_text(meta_text)
        # Execution mismatch fails; planning-only budget changes succeed.
        run('compare', left, meta, right, peer_meta, ok=False)
        overrides.write_text('compute_budget_ratio=0.6\n')
        right.write_text(run('resolve', left, overrides))
        run('compare', left, meta, right, peer_meta)
        for line in meta_text.splitlines():
            peer_meta.write_text(meta_text.replace(line + '\n', ''))
            run('compare', left, meta, right, peer_meta, ok=False)
        for text in [
            meta_text + 'clock_epoch=1\n', meta_text + 'extra=1\n',
            meta_text.replace('schema_version=1', 'schema_version=4294967297'),
            meta_text.replace('clock_epoch=18446744073709551615', 'clock_epoch=18446744073709551616'),
            meta_text.replace('clock_epoch=18446744073709551615', 'clock_epoch=0'),
            meta_text.replace('clock_epoch=18446744073709551615', 'clock_epoch=-1'),
            meta_text.replace('clock_epoch=18446744073709551615', 'clock_epoch=1.0'),
            meta_text.replace('world_frame=odom', 'world_frame=base_footprint'),
            meta_text.replace('world_frame=odom', 'world_frame=bad frame'),
            meta_text.replace('motion_policy=NominalEncoderOnly', 'motion_policy=Unspecified'),
            meta_text.replace('watchdog_s=0.15', 'watchdog_s=nan'),
            meta_text.replace('watchdog_s=0.15', 'watchdog_s=0'),
            meta_text.replace('watchdog_s=0.15', 'watchdog_s=1e999'),
            meta_text.replace('watchdog_s=0.15', 'watchdog_s=0.15garbage'),
            meta_text.replace('max_feedback_age_s=0.15', 'max_feedback_age_s=-1'),
            meta_text.replace('period_tolerance_ratio=0.25', 'period_tolerance_ratio=1'),
            meta_text.replace('body_frame=base_footprint', 'body_frame='),
            'x' * 65537,
        ]:
            peer_meta.write_text(text)
            run('compare', left, meta, right, peer_meta, ok=False)
        # Independently valid metadata must still agree field-for-field.
        for old, new in [
            ('world_frame=odom', 'world_frame=map'),
            ('body_frame=base_footprint', 'body_frame=base_link'),
            ('clock_domain=gazebo/world', 'clock_domain=wall'),
            ('clock_epoch=18446744073709551615', 'clock_epoch=18446744073709551614'),
            ('motion_policy=NominalEncoderOnly', 'motion_policy=IndependentNominal'),
            ('max_feedback_age_s=0.15', 'max_feedback_age_s=0.14'),
            ('max_command_age_s=0.15', 'max_command_age_s=0.14'),
            ('period_tolerance_ratio=0.25', 'period_tolerance_ratio=0.20'),
            ('watchdog_s=0.15', 'watchdog_s=0.14'),
        ]:
            peer_meta.write_text(meta_text.replace(old, new))
            run('compare', left, meta, right, peer_meta, ok=False)
        peer_meta.write_text(meta_text)
        for text in ['compute_budget_ratio=0', 'wheel_radius_m=0', 'unknown=1',
                     'random_seed=1.5', 'dt_s=0.1\ndt_s=0.2', 'x' * 65537]:
            overrides.write_text(text)
            run('resolve', left, overrides, ok=False)
        saved = left.read_text()
        left.write_text(saved.replace('wheelbase_m = 0.6  # m; execution\n', ''))
        overrides.write_text('')
        run('resolve', left, overrides, ok=False)
        run('compare', left, meta, right, peer_meta, ok=False)
        run('resolve', root / 'absent', overrides, ok=False)
        run('compare', right, root / 'absent', right, peer_meta, ok=False)
        run('resolve', ok=False)
        run('unknown', ok=False)
    print(f'{cases} offline preflight cases passed')


if __name__ == '__main__':
    main()
