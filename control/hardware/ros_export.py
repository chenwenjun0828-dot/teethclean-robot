"""Offline export and FK audit; this module alone needs the planning environment."""
import math
from pathlib import Path
import shutil
import tempfile

import numpy as np

from .trajectory_contract import (JointTrajectory, SCHEMA, UNITS, UR_JOINTS, UR_MAPPING,
                                  execution_blockers, joint_order, orientation_error_deg,
                                  read_json, sha256, write_json)


def source_gates(report, request):
    session = request.get('perception_session') or report.get('perception_session') or {}
    return {'check_mode': report.get('check_mode'), 'execution_ready': report.get('execution_ready') is True,
            'simulation_only': report.get('simulation_only', True),
            'robot_execution_allowed': session.get('robot_execution_allowed') is True,
            'camera_verified': session.get('camera_frame_status') == 'live_tf_verified',
            'camera_frame_status': session.get('camera_frame_status'), 'status': report.get('status')}


def verify_fingerprints(value):
    if isinstance(value, dict):
        if 'path' in value and 'sha256' in value:
            if sha256(value['path']) != value['sha256']:
                raise ValueError('Source changed: '+str(value['path']))
        else:
            for item in value.values():
                verify_fingerprints(item)
    elif isinstance(value, list):
        for item in value:
            verify_fingerprints(item)


def sample_grid(plan, step_s, slow):
    boundaries = list(plan.times)
    trace = plan.constant_trace
    if trace is not None and hasattr(trace, 'clock_t'):
        boundaries.extend(plan.times[trace.start]+trace.clock_t*(plan.times[trace.end]-plan.times[trace.start]))
    boundaries = np.unique(np.asarray(boundaries))
    values = []
    for a, b in zip(boundaries[:-1], boundaries[1:]):
        values.extend(np.linspace(a, b, max(1, int(np.ceil((b-a)*slow/step_s)))+1))
    # ROS Duration is integer nanoseconds. Sample the source at the same clock.
    clock = np.unique(np.rint(np.asarray(values)*slow*1e9).astype(np.int64))
    original = set(np.rint(plan.times*slow*1e9).astype(np.int64))
    merged = []
    for value in clock:
        if merged and value-merged[-1] < 1000:
            if value in original:
                if merged[-1] in original:
                    raise ValueError('Original waypoints less than one microsecond apart')
                merged[-1] = value
        else:
            merged.append(value)
    return np.array(merged)/1e9


def bernstein_bounds(c, derivative, dt):
    p = np.polynomial.polynomial.polyder(c, derivative, axis=0)/dt**derivative
    n = len(p)-1
    matrix = np.array([[math.comb(k, j)/math.comb(n, j) if j <= k else 0
                        for j in range(n+1)] for k in range(n+1)])
    b = matrix@p
    return b.min(axis=0), b.max(axis=0)


def interpolation_audit(curve, plan, robot, order, slow, config):
    from scipy.spatial.transform import Rotation
    maximum = {'source_position_error_mm': 0., 'source_orientation_error_deg': 0.,
               'tcp_reference_error_mm': 0., 'tcp_reference_angle_deg': 0.,
               'max_joint_speed_rad_s': 0., 'max_joint_accel_rad_s2': 0.}
    bad = []
    inverse = np.argsort(order)
    for i, (left, right, c) in enumerate(zip(curve.times[:-1], curve.times[1:], curve.coefficients)):
        failed = False
        for derivative in (0, 1, 2):
            lo, hi = bernstein_bounds(c, derivative, right-left)
            if derivative == 0:
                failed |= bool(np.any(lo[inverse] < robot.lower-1e-9) or np.any(hi[inverse] > robot.upper+1e-9))
            else:
                key = 'max_joint_speed_rad_s' if derivative == 1 else 'max_joint_accel_rad_s2'
                bound = float(np.maximum(abs(lo), abs(hi)).max())
                maximum[key] = max(maximum[key], bound)
                limit = config['joint_speed_rad_s'] if derivative == 1 else config['joint_accel_rad_s2']
                failed |= bound > limit+1e-5
        for fraction in (.25, .5, .75):
            t = left+fraction*(right-left)
            q, _, _, target, _ = curve.sample(t)
            pose = robot.fk(q[inverse], full=True)
            reference = robot.fk(plan.sample(min(t/slow, plan.duration))['q'], full=True)
            quat = Rotation.from_matrix(pose[:3, :3]).as_quat()
            errors = [np.linalg.norm(pose[:3, 3]-reference[:3, 3])*1000,
                      orientation_error_deg(quat, Rotation.from_matrix(reference[:3, :3]).as_quat()),
                      np.linalg.norm(pose[:3, 3]-target[:3])*1000,
                      orientation_error_deg(quat, target[3:])]
            for key, value in zip(list(maximum)[:4], errors):
                maximum[key] = max(maximum[key], float(value))
            failed |= max(errors[0], errors[2]) > config['interpolation_position_mm']
            failed |= max(errors[1], errors[3]) > config['interpolation_angle_deg']
        if failed:
            bad.append(i)
    return dict(maximum, passed=not bad, failed_intervals=bad[:30], intervals=len(curve.times)-1,
                interior_samples_per_interval=3, rate_bounds='Bernstein polynomial hull',
                scope='sampled FK equivalence; not a continuous collision certificate')


class _ControllerView:
    """Keep original reference indices, but audit the ROS curve at original times."""
    def __init__(self, original, curve, order, slow, limits):
        self.original, self.curve, self.inverse, self.slow = original, curve, np.argsort(order), slow
        self._limits = limits

    def __getattr__(self, name):
        return getattr(self.original, name)

    def sample(self, time_s):
        q, v, a, _, _ = self.curve.sample(min(time_s*self.slow, self.curve.duration))
        state = self.original.sample(time_s)
        return dict(state, q=q[self.inverse], dq=v[self.inverse], ddq=a[self.inverse])

    def limits(self):
        return self._limits


def export_plan(directory, output, *, mapping=None, target_names=None, step_s=.01, slowdown=1.):
    from scipy.spatial.transform import Rotation
    from planning.visualization.view import load_scene
    from planning.motion_planning.audit import audit_trace
    from planning.motion_planning.free_motion import load_transfer_record, audit_transfers
    from planning.config import Config
    directory, output = Path(directory).resolve(), Path(output).resolve()
    if (not np.isfinite([step_s, slowdown]).all() or step_s <= 0 or step_s > .01 or slowdown < 1):
        raise ValueError('Require 0 < step_s <= 0.01 and slowdown >= 1')
    if output.exists():
        raise ValueError('Export output must be a new directory')
    report, request = read_json(directory/'planning_report.json'), read_json(directory/'request.json')
    verify_fingerprints(report)
    verify_fingerprints(request)
    checker = None
    if report.get('check_mode') == 'geometry_only':
        _, _, robot, _, _, _, plan, _ = load_scene(directory)
    else:
        from planning.motion_planning.execution import load_verified
        _, _, plan, checker = load_verified(directory)
        robot = checker.robot
    if plan is None:
        raise ValueError('No saved timed trajectory')
    gates = source_gates(report, request)
    mapping = UR_MAPPING if mapping is None else mapping
    target_names = UR_JOINTS if target_names is None else target_names
    order = joint_order(plan.joint_names, mapping, target_names)
    output.mkdir(parents=True)
    snapshots = {}
    for name in ('planning_report.json', 'request.json', 'sweep_reference.json', 'perception_reference.json',
                 'transfer_reference.json', 'dynamics_validation.json', 'tool_contact_manifest.json'):
        if (directory/name).is_file():
            shutil.copyfile(directory/name, output/name)
            snapshots[name] = sha256(output/name)
    manifest = {'schema': 1, 'gates': gates, 'snapshots': snapshots,
                'source_files': {p.name: sha256(p) for p in directory.iterdir() if p.is_file()}}
    write_json(output/'source_manifest.json', manifest)
    cuts = np.r_[0, np.flatnonzero(plan.phases[1:] != plan.phases[:-1])+1, len(plan.phases)]
    phases = [{'name': str(plan.phases[a]), 'start_s': round(float(plan.times[a])*slowdown, 9),
               'end_s': round(float(plan.times[b])*slowdown, 9)} for a, b in zip(cuts[:-1], cuts[1:])]
    for attempt in range(4):
        times = sample_grid(plan, step_s/2**attempt, slowdown)
        if len(times) > 200000:
            raise ValueError('Export refinement exceeds 200000 points')
        points = []
        for index, t in enumerate(times):
            state = plan.sample(min(t/slowdown, plan.duration))
            if index in (0, len(times)-1):
                if not np.allclose([state['dq'], state['ddq']], 0., atol=1e-8, rtol=0):
                    raise ValueError('Source endpoints are not stopped')
                # Preserve the saved home exactly instead of polynomial roundoff.
                state = dict(state, q=plan.joints[0 if index == 0 else -1],
                             dq=np.zeros(6), ddq=np.zeros(6))
            pose = robot.fk(state['q'], full=True)
            points.append({'time_from_start': float(t), 'positions': state['q'][order].tolist(),
                           'velocities': (state['dq'][order]/slowdown).tolist(),
                           'accelerations': (state['ddq'][order]/slowdown**2).tolist(),
                           'tcp_pose': np.r_[pose[:3, 3], Rotation.from_matrix(pose[:3, :3]).as_quat()].tolist()})
        record = {'schema': SCHEMA, 'units': UNITS, 'joint_names': target_names,
                  'mapping': mapping, 'source': gates, 'phases': phases,
                  'tcp_frame': report['tcp_frame_id'], 'tcp_reference_frame': 'planning_world',
                  'slowdown': slowdown, 'points': points}
        curve = JointTrajectory(record)
        audit = interpolation_audit(curve, plan, robot, order, slowdown, report['config'])
        print(f'ROS interpolation audit: {len(points)} points, passed={audit["passed"]}', flush=True)
        if audit['passed']:
            break
    if not audit['passed']:
        write_json(output/'failed_audit.json', audit)
        raise ValueError('ROS interpolation audit failed; see failed_audit.json')
    view = _ControllerView(plan, curve, order, slowdown, audit)
    with tempfile.TemporaryDirectory(prefix='ur7e_export_audit_') as temporary:
        for name in ('planning_report.json', 'sweep_reference.json', 'perception_reference.json'):
            if (directory/name).exists():
                shutil.copyfile(directory/name, Path(temporary)/name)
        trace_audit = audit_trace(temporary, view, robot)
        shutil.copyfile(Path(temporary)/'trace_audit.json', output/'ros_trace_audit.json')
    transfer = load_transfer_record(directory, report, request)
    transfer_audit = audit_transfers(view, robot, Config.from_saved(report['config']), transfer) if transfer else None
    motion_audit = None
    if checker is not None:
        from planning.infrastructure.audit import audit_motion
        from planning.infrastructure.trajectory import Trajectory
        inverse = np.argsort(order)
        controller_plan = Trajectory(curve.times, curve.q[:, inverse],
            [curve.sample((a+b)/2)[4] for a, b in zip(curve.times[:-1], curve.times[1:])],
            np.zeros(len(curve.times)-1, int), plan.joint_names,
            coefficients=np.asarray(curve.coefficients)[:, :, inverse])
        motion_audit = audit_motion(controller_plan, robot, checker, Config.from_saved(report['config']))
    write_json(output/'ros_transfer_audit.json', transfer_audit)
    write_json(output/'trajectory.json', record)
    result = {'schema': 1, 'source': gates, 'load_verified_passed': report.get('check_mode') == 'full',
              'trajectory_sha256': sha256(output/'trajectory.json'),
              'manifest_sha256': sha256(output/'source_manifest.json'),
              'interpolation_audit': audit, 'trace_audit': trace_audit, 'transfer_audit': transfer_audit,
              'ros_motion_audit': motion_audit,
              'points': len(points), 'duration_s': curve.duration, 'slowdown': slowdown,
              'refinements': attempt, 'time_quantization_s': 1e-9, 'clock_merge_tolerance_s': 1e-6}
    result['execution_blockers'] = execution_blockers(curve, result)
    result['hardware_execution_ready'] = False
    write_json(output/'export_report.json', result)
    return result
