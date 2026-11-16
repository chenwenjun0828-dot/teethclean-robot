"""Protected action lifecycle, shared by ROS transport and deterministic tests."""
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import numpy as np

from .trajectory_contract import (execution_blockers, load_bundle, orientation_error_deg,
                                  read_json, sha256, write_json)

STREAMS = ('joints', 'wrench', 'tcp', 'robot_mode', 'safety_mode', 'program_running',
           'speed_scaling', 'watchdog', 'controllers', 'controller_state')
LIMITS = ('start_joint_rad', 'joint_tracking_rad', 'tcp_tracking_mm', 'tcp_angle_deg',
          'stopped_rad_s', 'max_state_age_s', 'max_feedback_age_s', 'max_stall_s',
          'max_run_wall_s', 'stop_timeout_s', 'settle_s', 'poll_s', 'goal_lead_s')
IDENTITY = ('robot_ip', 'serial_number', 'polyscope_version', 'driver_version', 'launch_command',
            'kinematic_calibration_sha256', 'tool_tcp_evidence', 'payload_evidence',
            'registration_evidence', 'force_units_compensation_evidence', 'fk_site_evidence',
            'stop_test_evidence', 'independent_watchdog_evidence', 'exclusive_command_evidence',
            'controller_progress_evidence')
ROS_FIELDS = ('action', 'controller', 'list_controllers_service', 'stop_service', 'joint_states_topic',
              'wrench_topic', 'wrench_frame', 'robot_mode_service', 'safety_mode_service',
              'program_running_service', 'speed_scaling_topic', 'watchdog_ready_topic',
              'heartbeat_topic', 'base_frame', 'tip_frame', 'controller_state_topic')


def log_state(stream, value):
    def sanitize(item):
        if isinstance(item, dict):
            return {key: sanitize(v) for key, v in item.items()}
        if isinstance(item, (list, tuple)):
            return [sanitize(v) for v in item]
        if isinstance(item, float) and not np.isfinite(item):
            return None
        return item
    stream.write(json.dumps(sanitize(value), allow_nan=False)+'\n')


def deployment_errors(config, trajectory_path, curve, report, *, require_authorization=True):
    missing = []
    if config.get('robot_model') != 'ur7e':
        missing.append('robot_model must be ur7e')
    for group, keys in (('identity', IDENTITY), ('ros', ROS_FIELDS)):
        for key in keys:
            if not isinstance(config.get(group, {}).get(key), str) or not config[group][key].strip():
                missing.append(group+'.'+key)
    if config.get('joint_names') != curve.names:
        missing.append('joint_names')
    if config.get('controller_interpolation') != 'quintic':
        missing.append('controller_interpolation')
    if config.get('command_owner') != 'exclusive_follow_joint_trajectory':
        missing.append('command_owner')
    for key in LIMITS:
        value = config.get('limits', {}).get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
            missing.append('limits.'+key)
    for phase in {p['name'] for p in curve.record['phases']}:
        for name in ('force_n', 'torque_nm'):
            value = config.get('phase_limits', {}).get(phase, {}).get(name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
                missing.append('phase_limits.'+phase+'.'+name)
    transform = np.asarray(config.get('planning_world_from_base_m'), dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-10)
            or not np.allclose(transform[:3, :3].T@transform[:3, :3], np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=1e-8)):
        missing.append('planning_world_from_base_m')
    if not require_authorization:
        return missing
    authorization = config.get('authorization', {})
    bundle = Path(trajectory_path).parent
    for key, path in (('trajectory_sha256', trajectory_path), ('manifest_sha256', bundle/'source_manifest.json'),
                      ('export_report_sha256', bundle/'export_report.json')):
        if authorization.get(key) != sha256(path):
            missing.append('authorization.'+key)
    if not authorization.get('approved_by'):
        missing.append('authorization.approved_by')
    try:
        expiry = datetime.fromisoformat((authorization.get('expires_utc') or '').replace('Z', '+00:00'))
        if expiry.tzinfo is None or expiry <= datetime.now(timezone.utc):
            missing.append('authorization.expired')
    except (KeyError, ValueError, TypeError):
        missing.append('authorization.expires_utc')
    return missing


def inspect(trajectory_path, deployment_path=None):
    curve, report, _ = load_bundle(trajectory_path)
    blockers = execution_blockers(curve, report)
    if deployment_path:
        blockers += deployment_errors(read_json(deployment_path), trajectory_path, curve, report)
    else:
        blockers.append('deployment_not_configured')
    return {'status': 'blocked' if blockers else 'offline_checks_passed', 'points': len(curve.times),
            'duration_s': curve.duration, 'joint_names': curve.names, 'blockers': blockers,
            'motion_sent': False, 'hardware_validated': False}


def state_fault(state, curve, config, *, active=False, final=False):
    limits = config['limits']
    if state.get('error'):
        return state['error']
    for name in STREAMS+ (('feedback',) if active else ()):
        age = state.get('ages', {}).get(name, float('inf'))
        maximum = limits['max_feedback_age_s'] if name == 'feedback' else limits['max_state_age_s']
        if not np.isfinite(age) or age < 0 or age > maximum:
            return 'stale_'+name
    if not state.get('controller_running') or state.get('conflicting_controllers'):
        return 'controller_not_exclusive'
    if (state.get('robot_mode') != 7 or state.get('safety_mode') != 1
            or state.get('program_running') is not True or state.get('watchdog_ready') is not True):
        return 'robot_or_watchdog_not_ready'
    scale = state.get('speed_scaling', float('nan'))
    if not np.isfinite(scale) or not 0 <= scale <= 1:
        return 'invalid_speed_scaling'
    vectors = {key: np.asarray(state.get(key, []), float) for key in ('q', 'dq', 'wrench', 'tcp')}
    if any(x.shape != ((7,) if key == 'tcp' else (6,)) or not np.isfinite(x).all()
           for key, x in vectors.items()):
        return 'invalid_state'
    if abs(np.linalg.norm(vectors['tcp'][3:])-1) > 1e-6:
        return 'invalid_tcp_orientation'
    progress = state.get('progress_s', 0.) if active else 0.
    if not np.isfinite(progress) or not 0 <= progress <= curve.duration+1e-9:
        return 'invalid_controller_progress'
    q, _, _, tcp, phase = curve.sample(curve.duration if final else progress)
    force_limits = config['phase_limits'][phase]
    if np.linalg.norm(vectors['wrench'][:3]) > force_limits['force_n']:
        return 'force_limit'
    if np.linalg.norm(vectors['wrench'][3:]) > force_limits['torque_nm']:
        return 'torque_limit'
    if active:
        desired = np.asarray(state.get('desired_q', []), float)
        if desired.shape != (6,) or not np.isfinite(desired).all() or np.max(abs(desired-q)) > 1e-6:
            return 'controller_reference_mismatch'
    tolerance = limits['joint_tracking_rad'] if active else limits['start_joint_rad']
    if np.max(abs(vectors['q']-q)) > tolerance:
        return 'joint_tracking' if active else 'start_mismatch'
    if np.linalg.norm(vectors['tcp'][:3]-tcp[:3])*1000 > limits['tcp_tracking_mm']:
        return 'tcp_tracking'
    if orientation_error_deg(vectors['tcp'][3:], tcp[3:]) > limits['tcp_angle_deg']:
        return 'tcp_orientation'
    if (not active or final) and np.max(abs(vectors['dq'])) > limits['stopped_rad_s']:
        return 'robot_not_stopped'
    return None


def stop_and_confirm(backend, config, now, sleep, initial_state=None, log=None):
    start, stable = now(), None
    errors = []
    for method in (backend.cancel, backend.stop):
        try:
            method()
        except Exception as error:
            errors.append(str(error))
    state = {}
    while now()-start < config['limits']['stop_timeout_s']:
        try:
            state = backend.snapshot()
            if log is not None:
                log_state(log, dict(stopping_elapsed_s=now()-start, **state))
            qd = np.asarray(state.get('dq', []), float)
            stopped = (qd.shape == (6,) and np.isfinite(qd).all()
                       and state.get('ages', {}).get('joints', float('inf')) <= config['limits']['max_state_age_s']
                       and np.max(abs(qd)) <= config['limits']['stopped_rad_s'])
            stable = (now() if stable is None else stable) if stopped else None
            if stable is not None and now()-stable >= config['limits']['settle_s']:
                result = {'confirmed': True, 'elapsed_s': now()-start, 'errors': errors}
                initial_tcp = np.asarray((initial_state or {}).get('tcp', []), float)
                final_tcp = np.asarray(state.get('tcp', []), float)
                if (initial_tcp.shape == final_tcp.shape == (7,) and np.isfinite(initial_tcp).all()
                        and np.isfinite(final_tcp).all()):
                    result['tcp_displacement_mm'] = float(np.linalg.norm(final_tcp[:3]-initial_tcp[:3])*1000)
                return result
        except Exception as error:
            errors.append(str(error))
        sleep(config['limits']['poll_s'])
    return {'confirmed': False, 'elapsed_s': now()-start, 'errors': errors}


def run(trajectory_path, deployment_path, output, backend_factory, *, execute=False,
        now=time.monotonic, sleep=time.sleep):
    """Backend construction follows offline gates. No goal or stop is sent by check."""
    output = Path(output)
    if output.exists():
        raise ValueError('Run output must be a new directory')
    output.mkdir(parents=True)
    curve, report, manifest = load_bundle(trajectory_path)
    config = read_json(deployment_path)
    blockers = deployment_errors(config, trajectory_path, curve, report, require_authorization=execute)
    if execute:
        blockers += execution_blockers(curve, report)
    result = {'status': 'blocked', 'motion_sent': False, 'motion_send_attempted': False, 'blockers': blockers,
              'trajectory_sha256': sha256(trajectory_path), 'deployment_sha256': sha256(deployment_path),
              'mode': 'execute' if execute else 'check', 'hardware_validated': False}
    if blockers:
        write_json(output/'run_report.json', result)
        return result
    backend = None
    attempted = False
    with (output/'states.jsonl').open('w', encoding='utf-8', buffering=1) as log:
        try:
            backend = backend_factory(config, curve)
            wait_start, stable = now(), None
            while True:
                if execute:
                    backend.heartbeat()
                state = backend.snapshot()
                log_state(log, state)
                fault = state_fault(state, curve, config)
                if fault is None:
                    stable = now() if stable is None else stable
                    if now()-stable >= config['limits']['settle_s']:
                        break
                else:
                    stable = None
                    if not fault.startswith('stale_') and fault != 'robot_or_watchdog_not_ready':
                        raise RuntimeError(fault)
                if now()-wait_start > config['limits']['max_feedback_age_s']+config['limits']['settle_s']:
                    raise RuntimeError(fault or 'preflight_not_stable')
                sleep(config['limits']['poll_s'])
            if not execute:
                result.update(status='read_only_checked', source_blockers=execution_blockers(curve, report))
                return result
            # Recheck fingerprints after preflight, immediately before the only send.
            if (sha256(trajectory_path) != result['trajectory_sha256']
                    or sha256(deployment_path) != result['deployment_sha256']
                    or config['authorization']['manifest_sha256'] != sha256(Path(trajectory_path).parent/'source_manifest.json')
                    or config['authorization']['export_report_sha256'] != sha256(Path(trajectory_path).parent/'export_report.json')
                    or any(sha256(Path(trajectory_path).parent/name) != digest
                           for name, digest in manifest['snapshots'].items())):
                raise RuntimeError('bundle_changed_during_preflight')
            backend.heartbeat()
            state = backend.snapshot()
            fault = state_fault(state, curve, config)
            if fault:
                raise RuntimeError(fault)
            attempted = True
            result.update(motion_sent=None, motion_send_attempted=True)
            backend.send(curve)
            result['motion_sent'] = True
            start, last_move, last_progress, stable = now(), now(), 0., None
            while True:
                backend.heartbeat()
                state = backend.snapshot()
                log_state(log, dict(wall_elapsed_s=now()-start, **state))
                feedback_pending = state.get('ages', {}).get('feedback', float('inf')) == float('inf')
                fault = state_fault(state, curve, config, active=not feedback_pending,
                                    final=state.get('action_state') == 'succeeded')
                if fault:
                    raise RuntimeError(fault)
                if feedback_pending:
                    if now()-start > config['limits']['max_feedback_age_s']:
                        raise RuntimeError('feedback_start_timeout')
                    sleep(config['limits']['poll_s'])
                    continue
                progress = state['progress_s']
                if progress < last_progress-1e-9:
                    raise RuntimeError('controller_progress_reversed')
                if progress > last_progress+1e-6:
                    last_move, last_progress = now(), progress
                status = state.get('action_state')
                if status == 'succeeded':
                    if state.get('result_code') != 0 or abs(progress-curve.duration) > 1e-6:
                        raise RuntimeError('action_result_incomplete')
                    stable = now() if stable is None else stable
                    if now()-stable >= config['limits']['settle_s']:
                        result.update(status='completed', controller_progress_s=progress, elapsed_s=now()-start)
                        break
                elif status not in ('pending', 'active'):
                    raise RuntimeError('action_'+str(status))
                if now()-last_move > config['limits']['max_stall_s']:
                    raise RuntimeError('controller_progress_stalled')
                if now()-start > config['limits']['max_run_wall_s']:
                    raise RuntimeError('execution_wall_timeout')
                sleep(config['limits']['poll_s'])
        except (Exception, KeyboardInterrupt) as error:
            result.update(status='fault_latched', fault=str(error) or type(error).__name__)
            if backend is not None and attempted:
                result['stop'] = stop_and_confirm(backend, config, now, sleep, state, log)
        finally:
            if backend is not None:
                try:
                    backend.close()
                except Exception as error:
                    result['close_error'] = str(error)
            write_json(output/'run_report.json', result)
    return result
