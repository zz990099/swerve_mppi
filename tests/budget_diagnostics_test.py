"""Exercise real pipeline reporting and matrix evidence without timing thresholds."""

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def check(condition, message):
    if not condition:
        raise AssertionError(message)


def read_csv(path):
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        check(reader.fieldnames and len(reader.fieldnames) == len(set(reader.fieldnames)),
              'CSV columns must be present and unique')
        return list(reader)


def nearest_rank(values, fraction):
    return sorted(values)[math.ceil(len(values) * fraction) - 1]


def check_reports(directory, repetitions):
    summaries = read_csv(directory / 'summary.csv')
    calls = read_csv(directory / 'calls.csv')
    expected = {(p, o) for p in (41, 401, 4096) for o in (0, 40, 128)}
    check(len(summaries) == 9 and len(calls) == 9 * repetitions, 'matrix calls were omitted')
    groups = {case: [] for case in expected}
    stages = ('controller', 'envelope', 'admission', 'install', 'sample')
    roots = ('ok', 'compute_timeout', 'controller_failure', 'admission_failure',
             'installation_failure', 'sampling_failure')
    for call in calls:
        key = (int(call['path_points']), int(call['obstacles']))
        check(key in expected, 'unexpected trace workload')
        groups[key].append(call)
        check(math.isclose(float(call['path_span_rad']), .7, abs_tol=1e-14),
              'density comparison changed the original curve geometry')
        check(int(call['repetition']) == len(groups[key]), 'repetitions must be ordered')
        for stage in ('setup', *stages, 'endpoint_sample'):
            value = float(call[stage + '_ms'])
            check(math.isfinite(value) and value >= 0, 'invalid measured stage duration')
        total = float(call['pipeline_ms'])
        check(math.isclose(total, sum(float(call[s + '_ms']) for s in stages), abs_tol=1e-9),
              'setup/endpoint work leaked into pipeline timing or a stage was omitted')
        budget = float(call['budget_ms'])
        check(int(call['pipeline_overrun']) == (total > budget), 'pipeline overrun is inconsistent')
        check(call['outcome'] in roots, 'unknown root failure category')
        check(int(call['feasible_rollouts']) <= int(call['evaluated_rollouts']), 'work counts invalid')
        # This profile permits only one branch and four evaluations. Actual
        # timeout work may be smaller; never substitute the configured maximum.
        check(int(call['branches']) <= 1 and int(call['evaluated_rollouts']) <= 4,
              'configured work differs from real single-branch planner work')
        if call['outcome'] == 'compute_timeout':
            check(call['failure_reason'] == 'ComputeTimeout' and call['budget_exhausted'] == '1',
                  'timeout diagnostics were lost')
            check(all(call[k] == '0' for k in ('command', 'actuation', 'installed', 'sampled', 'endpoint_sampled')),
                  'late planning output was authorized')
        elif call['outcome'] == 'ok':
            check(int(call['evaluated_rollouts']) == 4, 'successful work count was inferred incorrectly')
            check(call['failure_reason'] == 'None' and call['timing_error'] == 'None'
                  and call['safety_error'] == 'None', 'success concealed a failure')
        if call['pipeline_cpu_ms']:
            check(math.isclose(float(call['wall_minus_cpu_ms']), total - float(call['pipeline_cpu_ms']), abs_tol=1e-9),
                  'CPU gap data disagrees with the raw clocks')
        else:
            check(call['wall_minus_cpu_ms'] == '', 'unavailable CPU data was invented')
    check({(int(r['path_points']), int(r['obstacles'])) for r in summaries} == expected,
          'summary matrix is incomplete')
    for row in summaries:
        check(math.isclose(float(row['path_span_rad']), .7, abs_tol=1e-14), 'summary lost measured curve span')
        group = groups[(int(row['path_points']), int(row['obstacles']))]
        check(len(group) == repetitions, 'incorrect trace repetitions')
        for label, fraction in (('p50_ms', .5), ('p95_ms', .95), ('p99_ms', .99)):
            check(float(row[label]) == nearest_rank([float(c['pipeline_ms']) for c in group], fraction),
                  'summary percentile differs from the independent trace oracle')
        for stage in ('setup', *stages, 'endpoint_sample'):
            check(float(row[stage + '_p95_ms']) == nearest_rank([float(c[stage + '_ms']) for c in group], .95),
                  'stage percentile differs from the trace oracle')
        for root, label in zip(roots, ('successful_calls', 'timeout_only_calls', 'controller_failures',
                                     'admission_failures', 'installation_failures', 'sampling_failures')):
            check(int(row[label]) == sum(c['outcome'] == root for c in group), 'failure count disagrees with trace')
        check(int(row['other_failures']) == sum(c['outcome'] not in roots[:2] for c in group),
              'functional calls were double counted or omitted')
        check(int(row['compute_timeouts']) == sum(int(c['compute_timeout']) for c in group), 'timeout count mismatch')
        check(int(row['total_overruns']) == sum(int(c['pipeline_overrun']) for c in group), 'overrun count mismatch')
        check(int(row['controller_overruns']) + int(row['post_controller_overruns']) == int(row['total_overruns']),
              'pipeline overruns must have one measured location')
        for raw, summary in (('branches', 'branches_total'), ('evaluated_rollouts', 'evaluated_rollouts_total'),
                             ('feasible_rollouts', 'feasible_rollouts_total'), ('fallback_updates', 'fallback_updates_total'),
                             ('safety_reductions', 'safety_reductions_total')):
            check(int(row[summary]) == sum(int(c[raw]) for c in group), 'summary lost partial work')
        check(int(row['cpu_samples']) == sum(bool(c['pipeline_cpu_ms']) for c in group), 'CPU availability count mismatch')
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tool', required=True, type=Path)
    parser.add_argument('--runner', required=True, type=Path)
    parser.add_argument('--expected-version', required=True)
    args = parser.parse_args()
    tool = str(args.tool.resolve())
    with tempfile.TemporaryDirectory(prefix='swerve-budget-') as temporary:
        root = Path(temporary)
        profile = root / 'input.conf'
        profile.write_text('horizon_steps=2\nsamples_per_branch=2\niterations=1\n'
                           'noise_v_mps=0\nnoise_w_radps=0\nminimum_mode_dwell_s=100\n')
        for strict in (False, True):
            directory = root / ('strict' if strict else 'recording')
            command = [sys.executable, str(args.runner), '--executable', tool, '--output-dir', str(directory),
                       '--config', str(profile), '--repetitions', '2', '--budget-ratio', '0.000001' if strict else '1']
            if strict:
                command.append('--strict')
            result = subprocess.run(command, capture_output=True, text=True)
            check(result.returncode == (1 if strict else 0), f'matrix acceptance failed: {result.stdout}; {result.stderr}')
            summaries = check_reports(directory, 2)
            manifest = json.loads((directory / 'manifest.json').read_text())
            check(manifest['csv_schema_version'] == 2 and manifest['strict'] == strict, 'evidence schema/acceptance missing')
            check(manifest['status'] == 'budget_failure' if strict else manifest['status'].startswith('recorded_'),
                  'recording status falsely claimed or denied acceptance')
            check(manifest['tool_build']['version'] == args.expected_version and int(manifest['tool_build']['cplusplus']) >= 201703,
                  'build identity was lost')
            for filename, expected_hash in manifest['artifacts'].items():
                check(hashlib.sha256((directory / filename).read_bytes()).hexdigest() == expected_hash,
                      'manifest artifact fingerprint mismatch')
            if strict:
                check(sum(int(row['compute_timeouts']) for row in summaries) > 0, 'strict tiny-budget probe did not expire')
            previous = (directory / 'manifest.json').read_bytes()
            repeated = subprocess.run(command, capture_output=True, text=True)
            check(repeated.returncode == 2 and (directory / 'manifest.json').read_bytes() == previous,
                  'an existing report was overwritten')

        for flags, diagnostic in ((['--matrix', '--path-points', '401'], 'cannot combine'),
                                  (['--obstacles', '129'], '0..128'),
                                  (['--obstacles', '1', '--obstacles', '2'], 'duplicate'),
                                  (['--trace', str(profile), '--config', str(profile)], 'must differ'),
                                  (['--trace', str(root / 'same'), '--write-config', str(root / 'same')], 'must differ'),
                                  (['--trace', str(root)], 'cannot write call trace')):
            result = subprocess.run([tool, '2', *flags], capture_output=True, text=True)
            check(result.returncode == 1 and diagnostic in result.stderr and not result.stdout,
                  'invalid workload/output options were not rejected before measurement')
        # A selected workload may be admitted under smaller configured limits;
        # the default full workload must continue to reject those limits.
        profile.write_text(profile.read_text() + 'max_obstacles=1\n')
        result = subprocess.run([tool, '2', '--config', str(profile), '--obstacles', '1'], capture_output=True, text=True)
        check(result.returncode == 0 and '41,1,2,' in result.stdout, 'explicit workload was silently expanded')
        incomplete = root / 'incomplete'
        result = subprocess.run([sys.executable, str(args.runner), '--executable', tool,
                                 '--output-dir', str(incomplete), '--config', str(profile),
                                 '--repetitions', '2'], capture_output=True, text=True)
        check(result.returncode == 1 and json.loads((incomplete / 'manifest.json').read_text())['status'] == 'incomplete'
              and 'workload exceeds' in (incomplete / 'stderr.txt').read_text(),
              'rejected configuration falsely became a complete matrix')
        before = profile.read_bytes()
        hardlink, symlink = root / 'hard.conf', root / 'symbolic.conf'
        os.link(profile, hardlink)
        symlink.symlink_to(profile)
        for alias in (hardlink, symlink):
            result = subprocess.run([tool, '2', '--config', str(profile), '--trace', str(alias)], capture_output=True, text=True)
            check(result.returncode == 1 and 'must differ' in result.stderr and profile.read_bytes() == before,
                  'an output alias overwrote its input profile')
    print('Budget diagnostics and matrix evidence passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
