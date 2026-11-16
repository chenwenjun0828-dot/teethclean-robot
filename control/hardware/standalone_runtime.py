"""Portable two-file input and CLI; flattened into the generated robot script."""
import argparse
import base64
import binascii
import copy
from contextlib import contextmanager, nullcontext
import hashlib
import json
from pathlib import Path
import re
import tempfile
import time

import numpy as np

from .trajectory_contract import (JointTrajectory, bernstein_bounds, execution_blockers,
                                  load_bundle, read_json, sha256, write_json, unique_json_object)
from .standalone_plain import PlainTrajectoryInput
from .ros_runner import deployment_errors, run
from .ros1_trajectory import Ros1TrajectoryBackend
from .standalone_keyboard import KeyboardControl


PORTABLE_SCHEMA = 'ur7e_standalone_v1'
BASE_FILES = {'trajectory.json', 'export_report.json', 'source_manifest.json'}
AUDIT_FILES = {'ros_trace_audit.json', 'ros_transfer_audit.json'}


def prepare_execution_curve(curve, config, speed_factor):
    """Scale in memory; all references use the exact nanosecond ROS goal clock."""
    if (isinstance(speed_factor, bool) or not isinstance(speed_factor, (int, float))
            or not np.isfinite(speed_factor) or speed_factor <= 0):
        raise ValueError('speed_factor must be finite and positive')
    factor = float(speed_factor)
    original_duration = curve.duration
    rounding = 0.
    if factor != 1.:
        record = copy.deepcopy(curve.record)

        def scaled_time(t):
            value = t/factor
            if not np.isfinite(value) or value < 0 or value >= 2147483648:
                raise ValueError('Scaled clock exceeds ROS Duration range')
            return round(value*1e9)/1e9

        times = [scaled_time(t) for t in curve.times]
        if any(round(b*1e9) <= round(a*1e9) for a, b in zip(times[:-1], times[1:])):
            raise ValueError('speed_factor collapses nanosecond trajectory timestamps')
        rounding = max(abs(t-raw/factor) for t, raw in zip(times, curve.times))
        with np.errstate(over='raise', invalid='raise', under='ignore'):
            try:
                velocities = curve.v*factor
                accelerations = curve.a*factor*factor
            except FloatingPointError as error:
                raise ValueError('Scaled derivatives are not finite') from error
        for i, point in enumerate(record['points']):
            point.update(time_from_start=times[i], velocities=velocities[i].tolist(),
                         accelerations=accelerations[i].tolist())
        for phase in record['phases']:
            phase.update(start_s=scaled_time(phase['start_s']), end_s=scaled_time(phase['end_s']))
        curve = JointTrajectory(record)
        if not np.isfinite(curve.coefficients).all():
            raise ValueError('Scaled quintic coefficients are not finite')
    blockers = []
    if factor != 1. and config.get('execution_policy', {}).get('mode', 'full') == 'full':
        blockers.append('speed_factor.full_policy_requires_reaudit_and_export')
    audit = {'required': factor > 1., 'passed': None}
    if factor > 1.:
        limits = config.get('motion_limits', {})
        limits = limits if isinstance(limits, dict) else {}
        audit.update(joint_names=curve.names, method='quintic_bernstein_conservative_bounds',
                     limits=limits)
        for key, derivative in (('joint_velocity_rad_s', 1), ('joint_acceleration_rad_s2', 2)):
            value = limits.get(key)
            valid = (isinstance(value, list) and len(value) == 6 and all(
                not isinstance(x, bool) and isinstance(x, (int, float)) and np.isfinite(x) and x > 0
                for x in value))
            if not valid:
                blockers.append('motion_limits.'+key)
            bound = np.zeros(6)
            with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
                for c, dt in zip(curve.coefficients, np.diff(curve.times)):
                    low, high = bernstein_bounds(c, derivative, dt)
                    bound = np.maximum(bound, np.maximum(abs(low), abs(high)))
            if not np.isfinite(bound).all():
                raise ValueError('Scaled continuous derivative bounds are not finite')
            audit[key+'_bound'] = bound.tolist()
            if valid:
                for name in np.asarray(curve.names)[bound > np.asarray(value)]:
                    blockers.append('motion_limits.'+key+'.exceeded.'+str(name))
        audit['passed'] = not any(x.startswith('motion_limits.') for x in blockers)
    derived_hash = hashlib.sha256(json.dumps(curve.record, sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode('utf-8')).hexdigest()
    details = {'speed_factor': factor, 'original_duration_s': original_duration,
               'commanded_duration_s': curve.duration, 'execution_curve_sha256': derived_hash,
               'max_clock_rounding_s': rounding, 'speed_limit_audit': audit,
               'original_dynamics_audit_applies': factor == 1.}
    return curve, blockers, details


def portable_filename(name):
    return (isinstance(name, str) and
            re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*\.json', name) is not None and
            name != 'deployment.json')


def standalone_source_gate(curve, report, config):
    """A separate, explicit hardware policy; source facts remain immutable."""
    policy = config.get('execution_policy', {})
    mode = policy.get('mode', 'full')
    if mode == 'full':
        return execution_blockers(curve, report)
    if mode != 'geometry_only':
        return ['execution_policy.unsupported_mode']
    blockers = []
    if policy.get('acknowledge_unverified_motion') is not True:
        blockers.append('execution_policy.acknowledge_unverified_motion')
    if curve.record['source'].get('check_mode') != 'geometry_only':
        blockers.append('execution_policy.geometry_only_source_required')
    if report.get('interpolation_audit', {}).get('passed') is not True:
        blockers.append('interpolation_audit_failed')
    # Missing full checks are permitted; a known failed export audit is not.
    for name in ('trace_audit', 'transfer_audit'):
        if report.get(name) is not None and report[name].get('passed') is not True:
            blockers.append(name+'_failed')
    return blockers


@contextmanager
def restored_portable(path):
    """Restore exact original bytes, never execute content from the JSON."""
    path = Path(path).resolve()
    initial_digest = sha256(path)
    payload = json.loads(path.read_bytes().decode('utf-8'), object_pairs_hook=unique_json_object)
    if sha256(path) != initial_digest:
        raise ValueError('Portable input changed while loading')
    if not isinstance(payload, dict) or payload.get('schema') != PORTABLE_SCHEMA:
        raise ValueError('Expected ur7e_standalone_v1 packaged JSON, not the old trajectory file')
    files = payload.get('files')
    config = payload.get('deployment')
    if not isinstance(files, dict) or not BASE_FILES.issubset(files) or not isinstance(config, dict):
        raise ValueError('Missing portable files or deployment object')
    if any(not portable_filename(name) or not isinstance(data, str) for name, data in files.items()):
        raise ValueError('Invalid portable filename or encoded data')
    with tempfile.TemporaryDirectory(prefix='ur7e_saved_trajectory_') as temporary:
        root = Path(temporary)
        for name, encoded in files.items():
            try:
                data = base64.b64decode(encoded, validate=True)
            except (ValueError, binascii.Error) as error:
                raise ValueError('Invalid Base64 file: '+name) from error
            (root/name).write_bytes(data)
        manifest = read_json(root/'source_manifest.json')
        snapshots = manifest.get('snapshots')
        if not isinstance(snapshots, dict) or any(not portable_filename(name) for name in snapshots):
            raise ValueError('Invalid source snapshot names')
        expected = BASE_FILES | set(snapshots)
        if not expected.issubset(files) or set(files)-expected-AUDIT_FILES:
            raise ValueError('Unexpected or missing portable bundle files')
        write_json(root/'deployment.json', config)
        curve, report, _ = load_bundle(root/'trajectory.json')

        def unchanged():
            if sha256(path) != initial_digest:
                raise ValueError('portable_input_changed_before_send')

        yield root, curve, report, config, initial_digest, unchanged


def portable_summary(curve, report, config):
    warnings = execution_blockers(curve, report)
    policy = config.get('execution_policy', {'mode': 'full'})
    return {'points': len(curve.times), 'duration_s': curve.duration,
            'joint_names': curve.names, 'start_joints_rad': curve.q[0].tolist(),
            'end_joints_rad': curve.q[-1].tolist(), 'source': curve.record['source'],
            'source_warnings': warnings, 'execution_policy': policy,
            'project_source_restrictions_waived': warnings if policy.get('mode') == 'geometry_only' else [],
            'collision_dynamics_verified': not warnings,
            'hardware_validated': False}


@contextmanager
def standalone_inputs(path, config, reference, reference_digest, script_path):
    payload = json.loads(Path(path).read_bytes().decode('utf-8'), object_pairs_hook=unique_json_object)
    if isinstance(payload, dict) and payload.get('schema') == PORTABLE_SCHEMA:
        with restored_portable(path) as (root, curve, report, config, digest, unchanged):
            yield root/'trajectory.json', root/'deployment.json', curve, report, config, digest, unchanged, None
    else:
        adapter = PlainTrajectoryInput(path, config, reference, reference_digest, script_path)
        curve, report, config = adapter.load()
        yield Path(path), None, curve, report, config, adapter.digest, adapter.unchanged, adapter


def portable_operation(command, trajectory, output=None, *, backend_factory=None,
                       now=time.monotonic, sleep=time.sleep, speed_factor=1., operator=None,
                       plain_config=None, plain_reference=None, plain_reference_digest=None,
                       script_path=None):
    if command not in ('inspect', 'check', 'execute'):
        raise ValueError('Unknown standalone operation')
    if command != 'inspect' and output is None:
        raise ValueError('check/execute requires a new --out directory')
    if operator is not None and command != 'execute':
        raise ValueError('Keyboard controls require execute mode')
    with standalone_inputs(trajectory,
            plain_config if plain_config is not None else globals().get('SITE_DEPLOYMENT'),
            plain_reference if plain_reference is not None else globals().get('PLAIN_REFERENCE'),
            plain_reference_digest if plain_reference_digest is not None else globals().get('PLAIN_REFERENCE_SHA256'),
            script_path if script_path is not None else globals().get('PLAIN_SCRIPT_PATH')) as inputs:
        trajectory_path, deployment_path, curve, report, config, digest, unchanged, adapter = inputs
        summary = portable_summary(curve, report, config)
        binding = adapter.fingerprints if adapter is not None else {'portable_sha256': digest}
        gate = lambda c, r: standalone_source_gate(c, r, config)
        transform = lambda c, cfg: prepare_execution_curve(c, cfg, speed_factor)
        if command == 'inspect':
            _, speed_blockers, details = transform(curve, config)
            deployment_blockers = (adapter.deployment_errors(config, curve, report) if adapter is not None
                else deployment_errors(config, trajectory_path, curve, report))
            blockers = gate(curve, report)+deployment_blockers+speed_blockers
            summary.update(details)
            if speed_factor != 1.:
                summary['collision_dynamics_verified'] = False
            return dict(summary, status='blocked' if blockers else 'offline_checks_passed',
                        blockers=blockers, **binding, motion_sent=False,
                        motion_send_attempted=False)
        factory = Ros1TrajectoryBackend if backend_factory is None else backend_factory
        result = run(trajectory_path, deployment_path, output, factory,
                     execute=command == 'execute', source_gate=gate, before_send=unchanged,
                     now=now, sleep=sleep, curve_transform=transform, operator=operator,
                     input_adapter=adapter)
        result.update(summary, **binding)
        if speed_factor != 1.:
            result['collision_dynamics_verified'] = False
        write_json(Path(output)/'run_report.json', result)
        return result


def standalone_main(argv=None):
    parser = argparse.ArgumentParser(description='Standalone UR7e ROS 1 trajectory executor')
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ('inspect', 'check', 'execute'):
        command = commands.add_parser(name)
        command.add_argument('--trajectory', required=True, help='Paired plain points JSON or ur7e_standalone_v1 JSON')
        command.add_argument('--speed-factor', type=float, default=1.,
                             help='Positive time scaling: 2 doubles speed; 0.5 halves speed (default 1)')
        if name != 'inspect':
            command.add_argument('--out', required=True, help='New output directory')
        if name == 'execute':
            command.add_argument('--keyboard', action='store_true',
                                 help='Linux TTY: wait for S; Space terminates this run (no resume)')
    args = parser.parse_args(argv)
    try:
        context = KeyboardControl() if getattr(args, 'keyboard', False) else nullcontext(None)
        with context as operator:
            result = portable_operation(args.command, args.trajectory, getattr(args, 'out', None),
                                        speed_factor=args.speed_factor, operator=operator)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0 if result['status'] in ('offline_checks_passed', 'read_only_checked', 'completed',
                                         'cancelled_before_start', 'operator_stopped') else 2
    except (ValueError, RuntimeError, OSError, KeyError, TypeError, ImportError) as error:
        print('Standalone executor refused: '+str(error))
        return 2
