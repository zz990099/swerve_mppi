#!/usr/bin/env python3
"""Generate an offline migration candidate from a source-pinned companion audit.

No ROS/Gazebo imports, launch, runtime profile export, arming or command publication.
Only the audited pure configuration helper is evaluated. Requires PyYAML 6.0.3.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import types

ROOT = Path(__file__).resolve().parents[1]
MAPPINGS = {
    'wheelbase_m': 'geometry.wheelbase',
    'track_m': 'geometry.track_width',
    'wheel_radius_m': 'geometry.wheel_radius',
    'max_wheel_speed_mps': 'control.max_wheel_speed * geometry.wheel_radius',
    'max_wheel_accel_mps2': 'control.max_wheel_acceleration * geometry.wheel_radius',
    'max_steer_rate_radps': 'control.max_steering_rate',
    'confirmation_timeout_s': 'control.mode_switch_timeout',
    'robot_radius_m': 'bringup.safety_parameters(configuration).robot_radius_m',
    'collision_margin_m': 'safety.collision_margin',
}
GATES = [
    {'id': 'resolved_exchange', 'work': 'Export actual resolved participants, compare before arming.'},
    {'id': 'clock_epoch', 'work': 'Carry clock domain/epoch through snapshots, commands and resets.'},
    {'id': 'original_ns', 'work': 'Preserve original integer physics stamps before double conversion.'},
    {'id': 'coherent_ingress', 'work': 'Bind current payloads, source frames and context provenance.'},
    {'id': 'independent_motion', 'work': 'Provide independent observations; encoder FK is correlated.'},
    {'id': 'profile_revocation', 'work': 'Cancel profiles and serialize faults with final hardware writes.'},
    {'id': 'supervised_recovery', 'work': 'Require continuous independent stop, queue drain and bound acknowledgement.'},
    {'id': 'endpoint_scheduling', 'work': 'Measure blocked update/process loss; in-thread guard needs update progress.'},
    {'id': 'physical_bounds', 'work': 'Identify braking/steering/slip bounds before physical safety acceptance.'},
    {'id': 'target_budget', 'work': 'Prove full-pipeline and sample deadlines on the intended CPU.'},
]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def profile_values(text):
    """Read the core writer's output; validation belongs to config_preflight."""
    return {name.strip(): value.strip()
            for name, value in (line.split('#', 1)[0].split('=', 1) for line in text.splitlines())}


def strict_yaml(data):
    import yaml

    class UniqueLoader(yaml.SafeLoader):
        pass

    def mapping(loader, node):
        result = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node)
            if key in result:
                raise ValueError(f'duplicate YAML key: {key}')
            result[key] = loader.construct_object(value_node)
        return result

    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)
    return yaml.load(data, Loader=UniqueLoader)


def verify_sources(source, baseline):
    verified = {}
    for name, expected in baseline['audited_blobs'].items():
        path = source / name
        if not path.resolve().is_relative_to(source):
            raise ValueError(f'source escapes companion: {name}')
        data = path.read_bytes()
        actual = hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
        if actual != expected:
            raise ValueError(f'audited source drift: {name}; repeat the audit before migration')
        verified[name] = data
    return verified


def metadata(world, body, domain, epoch):
    # Explicit candidate values only. Epoch 1 is not a live allocation protocol.
    return '\n'.join([
        'schema_version=1', f'world_frame={world}', f'body_frame={body}',
        f'clock_domain={domain}', f'clock_epoch={epoch}',
        'motion_policy=NominalEncoderOnly',
        'max_feedback_age_s=0.15', 'max_command_age_s=0.15',
        'period_tolerance_ratio=0.25', 'watchdog_s=0.15', '',
    ])


def generate(args):
    source, output = args.companion.resolve(), args.output_dir.resolve()
    if output.is_relative_to(source):
        raise ValueError('output must be outside the companion source tree')
    # Never overwrite earlier evidence, including a failed preparation attempt.
    output.mkdir(parents=True, exist_ok=False)
    manifest = {
        'schema_version': 1, 'status': 'incomplete', 'kind': 'offline_migration_candidate',
        'runtime_export': False, 'integration_ready': False, 'physical_acceptance': False,
        'remaining_gates': copy.deepcopy(GATES),
    }

    def save():
        artifacts = {}
        for path in sorted(output.rglob('*')):
            if path.is_file() and path.name != 'manifest.json':
                artifacts[str(path.relative_to(output))] = digest(path.read_bytes())
        manifest['artifact_sha256'] = artifacts
        (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')

    def run(*arguments):
        result = subprocess.run([str(args.tool.resolve()), *map(str, arguments)],
                                text=True, capture_output=True, check=False)
        with (output / 'preflight.log').open('a') as log:
            log.write(f'operation={arguments[0]} exit={result.returncode}\n')
            log.write(result.stderr)
            if arguments[0] == 'compare':
                log.write(result.stdout)
        if result.returncode:
            raise ValueError(result.stderr.strip() or 'preflight failed')
        return result.stdout

    save()
    try:
        baseline = json.loads((ROOT / 'preparation/simulation/baseline.json').read_text())
        manifest['companion'] = baseline
        try:
            head = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'],
                                  text=True, capture_output=True, check=False)
            if head.returncode == 0:
                manifest['observed_core_git_head'] = head.stdout.strip()
                dirty = subprocess.run(['git', '-C', str(ROOT), 'status', '--porcelain'],
                                       text=True, capture_output=True, check=False)
                manifest['observed_core_git_dirty'] = bool(dirty.stdout.strip())
        except OSError:
            pass  # Source digests remain available for non-Git snapshots.
        verified = verify_sources(source, baseline)
        manifest['tool_sha256'] = digest(args.tool.read_bytes())
        base_data = args.base.read_bytes()
        manifest['base_profile_sha256'] = digest(base_data)
        manifest['preparation_source_sha256'] = {
            name: digest((ROOT / name).read_bytes()) for name in (
                'tools/prepare_simulation.py', 'preparation/config_preflight.cpp',
                'include/swerve_mppi/common/config.hpp', 'src/common/config_profile.cpp',
                'src/integration/adapter_contract.cpp')}
        cache = args.tool.resolve().parent / 'CMakeCache.txt'
        if cache.is_file():
            manifest['cmake_cache_sha256'] = digest(cache.read_bytes())
        (output / 'baseline.json').write_text(json.dumps(baseline, indent=2, sort_keys=True) + '\n')
        (output / 'base.conf').write_bytes(base_data)
        schema = json.loads(run('schema'))
        manifest['tool_version'] = schema['tool_version']
        inputs = output / 'inputs'
        inputs.mkdir()
        config_data = args.config.read_bytes() if args.config else verified['config/swerve.yaml']
        strict_yaml(config_data)  # Companion safe_load alone permits duplicate keys.
        (inputs / 'swerve.yaml').write_bytes(config_data)
        (inputs / 'controllers.yaml').write_bytes(verified['config/controllers.yaml'])
        manifest['custom_config'] = args.config is not None
        helper = types.ModuleType('audited_companion_bringup')
        # Evaluate exactly the verified bytes, avoiding an import/read race.
        exec(compile(verified['swerve_gazebo_sim/bringup.py'], 'audited_bringup.py', 'exec'),
             helper.__dict__)
        cfg = helper.load_config(inputs / 'swerve.yaml')
        namespace, prefix = helper.names(args.namespace, 'swerve_robot', args.prefix)
        mode_cfg = copy.deepcopy(cfg)
        # Reproduce mppi.launch.py's selection in memory; do not rewrite YAML or launch.
        mode_cfg['control']['chassis_control'] = True
        mode_cfg['control']['external_joint_control'] = False
        generated = helper.controller_config(inputs / 'controllers.yaml', mode_cfg, namespace, prefix)
        parameters = generated[f'{namespace}/chassis_controller']['ros__parameters']
        overrides = {name: parameters[name] for name in MAPPINGS}
        (output / 'executor.overrides').write_text(
            ''.join(f'{key}={value!r}\n' for key, value in overrides.items()))
        (output / 'planner.overrides').write_text(
            (output / 'executor.overrides').read_text() + 'compute_budget_ratio=0.6\n')
        for participant in ('executor', 'planner'):
            resolved = run('resolve', output / 'base.conf', output / f'{participant}.overrides')
            (output / f'{participant}.conf').write_text(resolved)
        candidate_meta = metadata(parameters['odom_frame'], parameters['body_frame'],
                                  args.clock_domain, args.clock_epoch)
        for participant in ('executor', 'planner'):
            (output / f'{participant}.metadata').write_text(candidate_meta)
        run('compare', output / 'planner.conf', output / 'planner.metadata',
            output / 'executor.conf', output / 'executor.metadata')
        values = {p: profile_values((output / f'{p}.conf').read_text())
                  for p in ('executor', 'planner')}
        provenance = []
        for field in schema['fields']:
            name = field['name']
            for p in ('executor', 'planner'):
                provenance.append({**field, 'participant': p, 'value': values[p][name].strip(),
                                   'source': MAPPINGS.get(name, 'base.conf: explicit core assumption')
                                   if name != 'compute_budget_ratio' or p != 'planner'
                                   else 'src/mppi_planner.cpp: default 0.6'})
        (output / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
        if float(values['executor']['steering_limit_rad']) != cfg['control']['steering_limit']:
            manifest['remaining_gates'].append({
                'id': 'steering_travel',
                'work': 'Explicit core steering travel differs from the YAML plant; resolve shared mechanical limits.'})
        actual_child = cfg['control']['odom_child_frame'] or prefix + 'base_footprint'
        if actual_child != parameters['body_frame']:
            manifest['remaining_gates'].append({
                'id': 'odometry_child_frame',
                'work': 'Python odometry child differs from the explicit chassis body frame; resolve provenance.'})
        manifest['candidate_startup_compatible'] = True
        manifest['parameters'] = parameters
        manifest['namespace'] = namespace
        manifest['prefix'] = prefix
        manifest['clock_epoch_note'] = 'Offline placeholder; allocate and exchange a fresh live incarnation.'
        manifest['status'] = 'candidate_prepared_integration_blocked'
        (output / 'report.md').write_text(
            '# Offline simulation preparation\n\n'
            f"Audited companion: `{baseline['commit']}`. Tool: `{schema['tool_version']}`.\n\n"
            'Startup candidates are compatible. Integration remains blocked. '
            'These are reconstructed candidates, not exports from running participants. '
            'No simulator was started and no commands were published.\n\n'
            'Mode: explicit chassis owner; motion policy: NominalEncoderOnly (development only). '
            'Unmapped settings remain explicit core assumptions, not plant calibration.\n\n'
            + '\n'.join(f"- `{gate['id']}`: {gate['work']}" for gate in manifest['remaining_gates'])
            + '\n')
        save()
        return manifest
    except Exception as error:
        manifest['error'] = str(error)
        save()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--companion', required=True, type=Path)
    parser.add_argument('--tool', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--base', type=Path, default=ROOT / 'config/default.conf')
    parser.add_argument('--config', type=Path, help='Optional startup YAML candidate; source still pinned')
    parser.add_argument('--namespace', default='')
    parser.add_argument('--prefix', default='')
    parser.add_argument('--clock-domain', default='gazebo/swerve_world')
    parser.add_argument('--clock-epoch', type=int, default=1)
    args = parser.parse_args()
    try:
        result = generate(args)
        print(f"{result['status']}: {args.output_dir / 'report.md'}")
        return 0
    except Exception as error:
        print(f'preparation rejected: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
