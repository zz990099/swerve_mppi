#!/usr/bin/env python3
"""Record the offline cold pipeline matrix and preserve reproducibility evidence."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(65536), b''):
            result.update(chunk)
    return result.hexdigest()


def source_snapshot(root):
    try:
        commit = subprocess.check_output(
            ['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True,
            stderr=subprocess.DEVNULL).strip()
        status = subprocess.check_output(
            ['git', '-C', str(root), 'status', '--porcelain'], text=True,
            stderr=subprocess.DEVNULL)
        return {'checkout_commit': commit, 'checkout_dirty': bool(status)}
    except (OSError, subprocess.CalledProcessError):
        return {'checkout_commit': None, 'checkout_dirty': None}


def build_cache(executable):
    cache = executable.parent / 'CMakeCache.txt'
    if not cache.is_file():
        cache = executable.parent.parent / 'CMakeCache.txt'
    keys = {'CMAKE_BUILD_TYPE', 'CMAKE_CXX_COMPILER', 'CMAKE_CXX_FLAGS',
            'CMAKE_CXX_FLAGS_DEBUG', 'CMAKE_CXX_FLAGS_RELEASE',
            'CMAKE_CXX_FLAGS_RELWITHDEBINFO', 'CMAKE_CXX_FLAGS_MINSIZEREL',
            'CMAKE_GENERATOR', 'CMAKE_HOME_DIRECTORY'}
    result = {}
    if cache.is_file():
        for line in cache.read_text().splitlines():
            if '=' in line and ':' in line.split('=', 1)[0]:
                key = line.split(':', 1)[0]
                if key in keys:
                    result[key] = line.split('=', 1)[1]
    return result


def host_snapshot():
    result = {'system': platform.system(), 'release': platform.release(),
              'machine': platform.machine(), 'processor': platform.processor() or None,
              'logical_cpus': os.cpu_count()}
    if result['system'] == 'Linux':
        try:
            with Path('/proc/cpuinfo').open() as stream:
                for line in stream:
                    if line.startswith('model name') and ':' in line:
                        result['processor'] = line.split(':', 1)[1].strip()
                        break
        except OSError:
            pass
    if hasattr(os, 'sched_getaffinity'):
        try:
            result['cpu_affinity'] = sorted(os.sched_getaffinity(0))
        except OSError:
            result['cpu_affinity'] = None
    return result


def read_matrix(directory, repetitions):
    with (directory / 'summary.csv').open(newline='') as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or len(set(reader.fieldnames)) != len(reader.fieldnames):
            raise ValueError('summary header is missing or contains duplicate columns')
        rows = list(reader)
    expected = {(p, o) for p in (41, 401, 4096) for o in (0, 40, 128)}
    actual = {(int(row['path_points']), int(row['obstacles'])) for row in rows}
    if actual != expected or len(rows) != len(expected):
        raise ValueError('summary does not contain the complete nine-case matrix')
    totals = {'compute_timeouts': 0, 'total_overruns': 0, 'other_failures': 0}
    root_counts = ('successful_calls', 'timeout_only_calls', 'controller_failures',
                   'admission_failures', 'installation_failures', 'sampling_failures')
    for row in rows:
        if int(row['repetitions']) != repetitions:
            raise ValueError('summary repetition count differs from the request')
        if sum(int(row[key]) for key in root_counts) != repetitions:
            raise ValueError('root outcome counts do not partition the calls')
        for key in totals:
            totals[key] += int(row[key])
    trace_counts = {case: 0 for case in expected}
    with (directory / 'calls.csv').open(newline='') as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            case = (int(row['path_points']), int(row['obstacles']))
            if case not in expected:
                raise ValueError('trace contains a case outside the matrix')
            trace_counts[case] += 1
            if int(row['repetition']) != trace_counts[case]:
                raise ValueError('trace repetitions are missing, duplicated or reordered')
    if any(count != repetitions for count in trace_counts.values()):
        raise ValueError('trace does not contain every requested call')
    return rows, totals


def write_report(directory, manifest, rows):
    lines = ['# Offline pipeline workload matrix', '',
             f"Status: **{manifest['status']}**. Tool exit code: {manifest['tool_exit_code']}.",
             '', 'Times are milliseconds, using nearest-rank percentiles. Setup and endpoint',
             'sampling are excluded from the live pipeline budget. Stage percentiles must',
             'not be added together. The fixed snapshots do not measure plant motion,',
             'ROS/DDS latency or executor scheduling; process CPU gaps do not identify a cause.',
             '', 'See manifest.json for source/build/host data and artifact SHA256 hashes.', '']
    if manifest.get('validation_error'):
        lines.extend([f"Incomplete evidence: {manifest['validation_error']}", ''])
    if rows:
        lines.extend(['| Path points | Obstacles | Pipeline P95 | Controller P95 | Admission P95 | '
                      'CPU gap P95 | Timeouts | Pipeline overruns | Functional failures |',
                      '| --- | --- | --- | --- | --- | --- | --- | --- | --- |'])
        for row in rows:
            values = [row[k] for k in ('path_points', 'obstacles', 'p95_ms', 'controller_p95_ms',
                                       'admission_p95_ms', 'wall_minus_cpu_p95_ms',
                                       'compute_timeouts', 'total_overruns', 'other_failures')]
            values[2:6] = [f'{float(v):.3f}' if v else 'unavailable' for v in values[2:6]]
            lines.append('| ' + ' | '.join(values) + ' |')
    (directory / 'report.md').write_text('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--repetitions', type=int, default=50)
    parser.add_argument('--budget-ratio', type=float, default=.6)
    parser.add_argument('--strict', action='store_true')
    args = parser.parse_args()
    if not 2 <= args.repetitions <= 10000:
        parser.error('repetitions must be 2..10000')
    if not math.isfinite(args.budget_ratio) or not 0 < args.budget_ratio <= 1:
        parser.error('budget ratio must be finite and in (0,1]')
    executable = args.executable.resolve()
    directory = args.output_dir.resolve()
    if directory.exists() and (not directory.is_dir() or any(directory.iterdir())):
        parser.error('output directory must be new or empty; existing evidence is preserved')
    try:
        info = subprocess.check_output([str(executable), '--build-info'], text=True,
                                       stderr=subprocess.PIPE, timeout=10)
        build = dict(line.split('=', 1) for line in info.splitlines())
        if set(build) != {'version', 'build_type', 'compiler', 'cplusplus'}:
            raise ValueError('unexpected build-info response')
        cache = build_cache(executable)
        source_root = Path(cache.get('CMAKE_HOME_DIRECTORY', Path(__file__).resolve().parents[1]))
        source = source_snapshot(source_root)
        binary_hash = digest(executable)
        input_profile = None
        if args.config:
            input_profile = {'path': str(args.config.resolve()), 'sha256': digest(args.config)}
        directory.mkdir(parents=True, exist_ok=True)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        parser.error(str(error))

    command = [str(executable), str(args.repetitions), '--matrix',
               '--budget-ratio', repr(args.budget_ratio), '--trace', str(directory / 'calls.csv'),
               '--write-config', str(directory / 'resolved.conf')]
    if args.config:
        command.extend(['--config', str(args.config.resolve())])
    if args.strict:
        command.append('--strict')
    manifest = {'schema_version': 1, 'csv_schema_version': 2, 'tool_build': build,
                'build_cache': cache, 'source': source, 'host': host_snapshot(),
                'executable_sha256': binary_hash, 'input_profile': input_profile,
                'command': command, 'strict': args.strict,
                'started_at': datetime.now(timezone.utc).isoformat()}
    start = time.monotonic()
    try:
        with (directory / 'summary.csv').open('w') as output:
            result = subprocess.run(command, stdout=output, stderr=subprocess.PIPE, text=True)
        exit_code, error_text = result.returncode, result.stderr
    except OSError as error:
        exit_code, error_text = 1, str(error)
    manifest.update(elapsed_s=time.monotonic() - start, tool_exit_code=exit_code)
    (directory / 'stderr.txt').write_text(error_text)
    rows = []
    try:
        rows, totals = read_matrix(directory, args.repetitions)
        manifest['totals'] = totals
        if totals['other_failures']:
            status = 'functional_failure'
        elif args.strict and (totals['compute_timeouts'] or totals['total_overruns']):
            status = 'budget_failure'
        elif exit_code:
            status = 'tool_failure'
        elif totals['compute_timeouts'] or totals['total_overruns']:
            status = 'recorded_budget_failures'
        else:
            status = 'recorded_no_failures'
        expected_failure = totals['other_failures'] > 0 or (
            args.strict and (totals['compute_timeouts'] > 0 or totals['total_overruns'] > 0))
        if bool(exit_code) != expected_failure:
            raise ValueError('tool exit code disagrees with the recorded acceptance counts')
        manifest['status'] = status
    except (OSError, ValueError, KeyError, TypeError) as error:
        manifest.update(status='incomplete', validation_error=str(error))
        exit_code = 1
    write_report(directory, manifest, rows)
    manifest['artifacts'] = {name: digest(directory / name)
                             for name in ('summary.csv', 'calls.csv', 'resolved.conf',
                                          'stderr.txt', 'report.md') if (directory / name).is_file()}
    (directory / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    print(f"{manifest['status']}: {directory / 'report.md'}")
    return 1 if exit_code else 0


if __name__ == '__main__':
    sys.exit(main())
