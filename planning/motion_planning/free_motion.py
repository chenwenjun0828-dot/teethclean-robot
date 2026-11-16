"""Stopped-endpoint C2 free motion with independent Cartesian line references."""
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation, Slerp

from planning.infrastructure.io import fingerprint, validate_transform
from planning.infrastructure.robot import rotation_error
from planning.infrastructure.trajectory import Trajectory
from planning.infrastructure.transfer import straight_tcp_motion
from planning.pose_search.continuous import SweepFailure


TRANSFER_POLICY = 'straight_tcp_home_return_v1'
MAX_REFINEMENTS = 4


def _clock(t):
    return 10*t**3-15*t**4+6*t**5


def _reference(robot, start, end):
    return {'start_pose_world_m': robot.fk(start).tolist(),
            'end_pose_world_m': robot.fk(end).tolist(),
            'start_joints_rad': np.asarray(start).tolist(), 'end_joints_rad': np.asarray(end).tolist(),
            'orientation': 'shortest_slerp', 'clock': 'quintic_progress'}


def _fit(joints):
    u = np.linspace(0., 1., len(joints))
    path = CubicSpline(u, joints, axis=0)
    t = np.array([0., *[brentq(lambda x: _clock(x)-s, 0., 1.) for s in u[1:-1]], 1.])
    du = 30*t*t*(1-t)**2
    ddu = 60*t*(1-t)*(1-2*t)
    velocity = path(u, 1)*du[:, None]
    acceleration = path(u, 2)*du[:, None]**2+path(u, 1)*ddu[:, None]
    c = np.array([Trajectory._quintic_coefficients(a, b, va, vb, aa, ab, dt)
                  for a, b, va, vb, aa, ab, dt in zip(joints[:-1], joints[1:], velocity[:-1],
                      velocity[1:], acceleration[:-1], acceleration[1:], np.diff(t))])
    return np.diff(t), c


def retime_free(dt, coefficients, joints, robot, cfg):
    """Uniformly stretch a fixed polynomial curve; its normalized coefficients never change."""
    dt = np.array(dt, float, copy=True)
    plan = Trajectory(np.r_[0., np.cumsum(dt)], joints, ['transfer']*len(dt),
                      [0]*len(dt), robot.joint_names, coefficients)
    limits = plan.limits()
    scale = max(1., limits['max_joint_speed_rad_s']/cfg.joint_speed_rad_s,
                np.sqrt(limits['max_joint_accel_rad_s2']/cfg.joint_accel_rad_s2),
                limits['tcp_angular_speed_bound_deg_s']/cfg.orientation_speed_deg_s)
    speed = 0.
    for i in range(len(dt)):
        for t in plan.interval_times(i, cfg.sweep_step_rad):
            state = plan.sample(t)
            speed = max(speed, np.linalg.norm(robot.jacobian(state['q'])[:3]@state['dq'])*1000)
    return dt*max(scale, speed/cfg.transfer_speed_mm_s)*(1+1e-6)


def prepare_transfer(robot, checker, cfg, start, end):
    """Select fixed-endpoint IK, then refine only the free curve when interpolation fails."""
    start, end = np.array(start, float), np.array(end, float)
    reference = _reference(robot, start, end)
    last = None
    for refinement in range(MAX_REFINEMENTS):
        try:
            joints = np.asarray(straight_tcp_motion(robot, checker, start,
                np.array(reference['end_pose_world_m']), cfg, target_q=end, refinement=2**refinement))
        except SweepFailure as error:
            last = {'reason': str(error), **error.details}
            continue
        dt, coefficients = _fit(joints)
        dt = retime_free(dt, coefficients, joints, robot, cfg)
        plan = Trajectory(np.r_[0., np.cumsum(dt)], joints, ['transfer']*len(dt),
                          [0]*len(dt), robot.joint_names, coefficients)
        segment = dict(reference, phase='transfer', start_waypoint=0, end_waypoint=len(joints)-1)
        audit = audit_transfers(plan, robot, cfg, {'version': TRANSFER_POLICY, 'segments': [segment]},
                                require_return=False, raise_on_failure=False)
        if audit['passed']:
            return {'joints': joints, 'dt': dt, 'coefficients': coefficients,
                    'reference': dict(reference, refinement_level=refinement), 'audit': audit}
        last = audit
    raise SweepFailure('straight_transfer_failed', start_joints_rad=start.tolist(),
                       end_joints_rad=end.tolist(), diagnostic=last)


def validate_transfer_record(record):
    if record.get('version') != TRANSFER_POLICY or not isinstance(record.get('segments'), list):
        raise ValueError('Unsupported straight transfer reference')
    for segment in record['segments']:
        for name in ('start_pose_world_m', 'end_pose_world_m'):
            validate_transform(np.asarray(segment[name], float))
        if segment.get('orientation') != 'shortest_slerp' or segment.get('clock') != 'quintic_progress':
            raise ValueError('Unsupported straight transfer interpolation')


def audit_transfers(plan, robot, cfg, record, *, require_return=True, raise_on_failure=True):
    """Audit timed FK against finite line segments and their timed Slerp orientation."""
    validate_transfer_record(record)
    if require_return:
        if len(record['segments']) != 2:
            raise ValueError('Straight transfer requires entry and return')
        first, last = record['segments']
        trace = plan.constant_trace
        if (trace is None or first['phase'] != 'transfer' or last['phase'] != 'retreat'
                or (first['start_waypoint'], first['end_waypoint']) != (0, trace.start)
                or (last['start_waypoint'], last['end_waypoint']) != (trace.end, len(plan.joints)-1)
                or not np.allclose(plan.joints[0], plan.joints[-1], atol=1e-8, rtol=0)
                or not np.allclose(plan.joints[0], robot.home, atol=1e-8, rtol=0)):
            raise ValueError('Straight transfer endpoint/return contract changed')
    rows, passed = [], True
    for segment in record['segments']:
        start, end = segment['start_waypoint'], segment['end_waypoint']
        if not (0 <= start < end < len(plan.times)) or np.any(plan.phases[start:end] != segment['phase']):
            raise ValueError('Straight transfer phase bounds changed')
        if any(not np.allclose(plan.joints[i], segment[key], atol=1e-8, rtol=0)
               for i, key in ((start, 'start_joints_rad'), (end, 'end_joints_rad'))):
            raise ValueError('Straight transfer joint endpoint changed')
        poses = [np.asarray(segment[name], float) for name in ('start_pose_world_m', 'end_pose_world_m')]
        for i, pose in zip((start, end), poses):
            actual = robot.fk(plan.joints[i])
            if not np.allclose(actual, pose, atol=1e-8, rtol=0):
                raise ValueError('Straight transfer pose endpoint changed')
        rotation = Slerp([0., 1.], Rotation.from_matrix([p[:3, :3] for p in poses]))
        direction = poses[1][:3, 3]-poses[0][:3, 3]
        length2 = float(direction@direction)
        previous, progress_ok = -np.inf, True
        error, angle, samples = 0., 0., 0
        for interval in range(start, end):
            for t in plan.interval_times(interval, cfg.sweep_step_rad):
                fraction = _clock(float(np.clip((t-plan.times[start])/(plan.times[end]-plan.times[start]), 0., 1.)))
                actual = robot.fk(plan.sample(t)['q'], full=True)
                target = poses[0][:3, 3]+fraction*direction
                error = max(error, float(np.linalg.norm(actual[:3, 3]-target)*1000))
                angle = max(angle, float(np.degrees(np.linalg.norm(rotation_error(
                    rotation(fraction).as_matrix(), actual[:3, :3])))))
                if length2 > 1e-20:
                    progress = float((actual[:3, 3]-poses[0][:3, 3])@direction/length2)
                    tolerance = 1e-8
                    progress_ok &= -tolerance <= progress <= 1+tolerance and progress >= previous-tolerance
                    previous = progress
                samples += 1
        ok = (error <= cfg.interpolation_position_mm and angle <= cfg.interpolation_angle_deg and progress_ok)
        passed &= ok
        rows.append({'phase': segment['phase'], 'max_position_error_mm': error,
                     'max_orientation_error_deg': angle, 'monotonic_finite_segment': bool(progress_ok),
                     'samples': samples, 'passed': bool(ok)})
    result = {'version': TRANSFER_POLICY, 'passed': bool(passed), 'actual_time_interpolant': True,
              'verification_scope': 'sampled timed FK; not an exact continuous-line certificate', 'segments': rows}
    if not passed and raise_on_failure:
        raise RuntimeError('Straight transfer geometry audit failed: '+str(rows))
    return result


def load_transfer_record(directory, report, request):
    """Legacy plans keep their saved free curves; new plans require matching provenance."""
    if max(report.get('schema_version', 0), request.get('schema_version', 0)) < 8:
        if not any('transfer_policy' in row for row in (report, request)):
            return None
    if any(row.get('transfer_policy') != TRANSFER_POLICY for row in (report, request)):
        raise ValueError('Straight transfer policy changed; replan')
    path = Path(directory)/'transfer_reference.json'
    digest = fingerprint(path)['sha256']
    if any(row.get('transfer_reference_file', {}).get('sha256') != digest for row in (report, request)):
        raise ValueError('Straight transfer reference changed; replan')
    record = json.loads(path.read_text(encoding='utf-8'))
    validate_transfer_record(record)
    return record
