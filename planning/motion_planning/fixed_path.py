"""Fit a fixed joint cubic and allocate TOPPRA timing without changing its nodes."""
from copy import deepcopy
import numpy as np
from planning.infrastructure.fixed_spline import JointSpline
from planning.infrastructure.local_speed import LocalSpeedTrace
from planning.infrastructure.trajectory import Trajectory


TIMING_METHOD = 'fixed_joint_cubic_toppra'


def phase_durations(plan):
    dt = np.diff(plan.times)
    return {str(phase): float(dt[plan.phases == phase].sum()) for phase in np.unique(plan.phases)}


def build(joints, phases, teeth, robot, cfg, duration_s=None, *, fixed_trace=None):
    if duration_s is not None and (isinstance(duration_s, bool)
            or not np.isfinite(duration_s) or duration_s <= 0):
        raise ValueError('Target duration must be positive and finite')
    q, phases, teeth = np.asarray(joints, float), np.asarray(phases), np.asarray(teeth)
    trace_intervals = np.flatnonzero(phases == 'trace')
    if not len(trace_intervals) or not np.array_equal(trace_intervals, np.arange(trace_intervals[0], trace_intervals[-1]+1)):
        raise ValueError('Simple motion requires one continuous trace block')
    start = int(trace_intervals[0])
    end = int(trace_intervals[-1])+1
    trace = (JointSpline.build(q[start:end+1], robot, start) if fixed_trace is None
             else deepcopy(fixed_trace))
    if trace.start != start or not np.array_equal(trace.joints, q[start:end+1]):
        raise ValueError('Fixed spline nodes differ from the requested trace')
    c = np.stack([Trajectory._smoothstep_coefficients(a, b) for a, b in zip(q[:-1], q[1:])])
    dt = np.ones(len(phases))
    for i in np.r_[np.arange(start), np.arange(end, len(phases))]:
        delta = abs(q[i+1]-q[i])
        distance = np.linalg.norm(robot.fk(q[i+1])[:3, 3]-robot.fk(q[i])[:3, 3])
        dt[i] = max(.04, 1.875*delta.max()/cfg.joint_speed_rad_s,
                    np.sqrt(10/np.sqrt(3)*delta.max()/cfg.joint_accel_rad_s2),
                    1.875*np.degrees(delta.sum())/cfg.orientation_speed_deg_s,
                    1.875*distance/(cfg.transfer_speed_mm_s*.001))
        # Correct the free-space timing on its fixed curve, without refitting it.
        speed = 0.
        for u in np.linspace(0, 1, max(17, int(np.ceil(1.875*delta.max()/cfg.sweep_step_rad))+1)):
            position = q[i]+(10*u**3-15*u**4+6*u**5)*(q[i+1]-q[i])
            derivative = 30*u*u*(1-u)**2*(q[i+1]-q[i])/dt[i]
            speed = max(speed, np.linalg.norm(robot.jacobian(position)[:3]@derivative)*1000)
        dt[i] *= max(1., speed/cfg.transfer_speed_mm_s)*(1+1e-6)
    free_duration = float(dt[:start].sum()+dt[end:].sum())
    trace_duration = None if duration_s is None else duration_s-free_duration
    if trace_duration is not None and trace_duration <= 0:
        raise RuntimeError(f'Target leaves no cleaning time after {free_duration:.3f} s of entry/exit motion')
    print('TOPPRA: allocating local speed on the fixed joint spline', flush=True)
    trace = LocalSpeedTrace.retime(trace, robot, cfg)
    minimum = free_duration+trace.minimum_duration_s
    if duration_s is not None and minimum > duration_s+1e-8:
        raise RuntimeError(f'Certified fixed-path timing needs {minimum:.3f} s including entry/exit; '
                           f'target is {duration_s:g} s. Increase speed/rate limits or duration.')
    if duration_s is None:
        trace_duration = trace.minimum_duration_s
    print(f'Fixed-path timing: entry/exit {free_duration:.3f} s; cleaning '
          f'{trace_duration:.3f} s (minimum certified {trace.minimum_duration_s:.3f} s)', flush=True)
    trace_times = dt[:start].sum()+trace.fraction_at(trace.node_s)*trace_duration
    times = np.r_[np.r_[0., np.cumsum(dt[:start])][:-1], trace_times,
                  trace_times[-1]+np.cumsum(dt[end:])]
    mount = robot.model.body_pos[robot.model.body('robot_mount').id].copy()
    return Trajectory(times, q, phases, teeth, robot.joint_names, c, mount, trace)


def plan_pose_sequence(poses, robot, checker, cfg=None, *, joints=None, duration_s=None):
    """Small Python entry: ordered world TCP transforms -> audited smooth motion.

    Supply already selected continuous IK joints to avoid solving them again.
    The returned path starts at the first supplied pose; approach is separate.
    """
    from planning.config import Config
    from planning.infrastructure.robot import rotation_error
    from planning.infrastructure.audit import audit_motion
    from planning.pose_search.continuous import whole_path_search
    cfg = cfg or Config()
    poses = np.asarray(poses, float)
    if poses.ndim != 3 or poses.shape[1:] != (4, 4) or len(poses) < 2 or not np.isfinite(poses).all():
        raise ValueError('Expected at least two finite world TCP transforms, shape (n,4,4)')
    if not np.allclose(poses[:, 3], [0, 0, 0, 1]):
        raise ValueError('Invalid homogeneous pose matrices')
    rotations = poses[:, :3, :3]
    if (not np.allclose(rotations @ rotations.transpose(0, 2, 1), np.eye(3), atol=1e-7)
            or not np.allclose(np.linalg.det(rotations), 1., atol=1e-7)):
        raise ValueError('Pose rotations must be orthonormal with determinant +1')
    if joints is None:
        nodes = [[{'q': q, 'pose': pose} for q in robot.solutions(pose)] for pose in poses]
        def edge_valid(a, b):
            check = checker.edge(a['q'], b['q'], cfg.clearance_mm*.001, cfg.sweep_step_rad)
            return check.valid, check.pair
        def node_valid(node):
            check = checker.check(node['q'], cfg.clearance_mm*.001)
            return check.valid, check.pair
        selected = whole_path_search(nodes, cfg, edge_valid, robot.home, node_valid)
        joints = np.array([node['q'] for node in selected])
    joints = np.asarray(joints, float)
    if joints.shape != (len(poses), len(robot.joint_names)) or not np.isfinite(joints).all():
        raise ValueError('IK joints must match the input poses and robot joint order')
    for i, (pose, q) in enumerate(zip(poses, joints)):
        actual = robot.fk(q)
        if (np.linalg.norm(actual[:3, 3]-pose[:3, 3])*1000 > cfg.interpolation_position_mm
                or np.degrees(np.linalg.norm(rotation_error(pose[:3, :3], actual[:3, :3]))) > cfg.interpolation_angle_deg):
            raise ValueError(f'IK does not pass requested pose {i}')
    plan = build(joints, ['trace']*(len(poses)-1), [0]*(len(poses)-1), robot, cfg, duration_s)
    audit = audit_motion(plan, robot, checker, cfg)
    if audit['max_trace_tcp_speed_mm_s'] > cfg.speed_mm_s+1e-8:
        raise RuntimeError('Fixed trajectory exceeds the configured TCP speed limit')
    return plan, audit
