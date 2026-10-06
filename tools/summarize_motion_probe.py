#!/usr/bin/env python3
"""Validate and summarize the version-one independent motion probe CSV."""
import argparse
import collections
import csv
import math
from pathlib import Path
import sys

MODES = {'ackermann', 'crab', 'spin'}
PERTURBATIONS = {'nominal', 'wheel_lag', 'body_lag', 'slip', 'noise', 'delayed'}
STATUSES = {'nominal_agreement', 'bounded_disagreement', 'envelope_exceeded', 'unsynchronized'}


def load_rows(path):
    with Path(path).open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or len(reader.fieldnames) != len(set(reader.fieldnames)):
            raise ValueError('Missing or duplicate CSV headers')
        rows = list(reader)
    if len(rows) != 720:
        raise ValueError('Expected 720 rows for the complete matrix')
    seen = set()
    for row in rows:
        if None in row or None in row.values():
            raise ValueError('Malformed CSV row')
        key = (row['mode'], row['perturbation'], int(row['tick']))
        if key[0] not in MODES or key[1] not in PERTURBATIONS or not 0 <= key[2] < 40 or key in seen:
            raise ValueError('Duplicate or invalid matrix key')
        seen.add(key)
        if row['schema_version'] != '1' or row['phase'] != ('drive' if key[2] < 20 else 'brake'):
            raise ValueError('Unsupported schema or invalid phase')
        if row['status'] not in STATUSES:
            raise ValueError('Unexpected assessment status')
        for field in ('position_error_m', 'yaw_error_rad', 'linear_bound_mps', 'angular_bound_radps'):
            if not math.isfinite(float(row[field])) or float(row[field]) < 0:
                raise ValueError('Invalid error measurement')
        for field in ('body_stationary', 'encoder_stationary', 'encoded_model_valid', 'observed_model_valid'):
            if row[field] not in ('0', '1'):
                raise ValueError('Invalid boolean field')
        for field in ('encoder_vx', 'encoder_vy', 'encoder_wz', 'physical_vx', 'physical_vy',
                      'physical_wz', 'observed_vx', 'observed_vy', 'observed_wz'):
            if not math.isfinite(float(row[field])):
                raise ValueError('Invalid raw velocity')
        if int(row['encoder_stamp_ns']) < 0 or int(row['body_stamp_ns']) < 0:
            raise ValueError('Invalid source timestamp')
        if key[1] == 'delayed':
            if int(row['body_stamp_ns']) >= int(row['encoder_stamp_ns']) or row['status'] != 'unsynchronized':
                raise ValueError('Delayed sample admitted as coherent')
            if row['body_stationary'] != '0' or row['observed_model_valid'] != '0':
                raise ValueError('Delayed sample leaked motion evidence')
            continue
        if row['body_stamp_ns'] != row['encoder_stamp_ns']:
            raise ValueError('Unexpected source timestamp disagreement')
        # Independent CSV oracle: derive residuals and bounds from the raw observations.
        linear = math.hypot(float(row['observed_vx']) - float(row['encoder_vx']),
                            float(row['observed_vy']) - float(row['encoder_vy']))
        angular = abs(float(row['observed_wz']) - float(row['encoder_wz']))
        expected = {
            'linear_residual_mps': linear,
            'angular_residual_radps': angular,
            'linear_residual_upper_mps': linear + float(row['linear_bound_mps']),
            'angular_residual_upper_radps': angular + float(row['angular_bound_radps']),
            'body_speed_upper_mps': math.hypot(float(row['observed_vx']), float(row['observed_vy']))
                + float(row['linear_bound_mps']),
            'body_rate_upper_radps': abs(float(row['observed_wz'])) + float(row['angular_bound_radps']),
        }
        for field, value in expected.items():
            if not math.isfinite(value) or not math.isclose(float(row[field]), value, rel_tol=1e-10, abs_tol=1e-12):
                raise ValueError(f'CSV residual/bound mismatch: {field}')
        noise = math.hypot(float(row['observed_vx']) - float(row['physical_vx']),
                           float(row['observed_vy']) - float(row['physical_vy']))
        angular_noise = abs(float(row['observed_wz']) - float(row['physical_wz']))
        if noise > float(row['linear_bound_mps']) + 1e-12 or angular_noise > float(row['angular_bound_radps']) + 1e-12:
            raise ValueError('Fixture noise exceeds its declared deterministic bound')
    return rows


def summarize(rows):
    lines = ['# Independent motion probe', '',
             '720 one-step comparisons: three modes, six perturbations, 20 Drive and 20 Brake ticks per case.',
             'Encoder lag 0.15 s; body lag 0.20 s; slip fraction 0.25; measurement bounds 0.005 m/s and 0.01 rad/s.',
             'NominalAgreement is diagnostic evidence only. This probe does not certify physical stopping or authorize execution.', '',
             '| Perturbation | Max position error (m) | Max yaw error (rad) | Assessment counts | Encoder-only stopped samples |',
             '| --- | ---: | ---: | --- | ---: |']
    for name in sorted(PERTURBATIONS):
        group = [r for r in rows if r['perturbation'] == name]
        counts = collections.Counter(r['status'] for r in group)
        hidden = sum(r['phase'] == 'brake' and r['encoder_stationary'] == '1' and
                     r['body_stationary'] == '0' and r['status'] != 'unsynchronized' for r in group)
        position = max(float(r['position_error_m']) for r in group)
        yaw = max(float(r['yaw_error_rad']) for r in group)
        statuses = ', '.join(f'{k}: {v}' for k, v in sorted(counts.items()))
        lines.append(f'| {name} | {position:.9g} | {yaw:.9g} | {statuses} | {hidden} |')
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv')
    args = parser.parse_args()
    try:
        print(summarize(load_rows(args.csv)), end='')
    except (OSError, ValueError, KeyError) as error:
        print(f'Invalid motion evidence: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
