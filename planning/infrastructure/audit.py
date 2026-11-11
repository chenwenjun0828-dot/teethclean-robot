"""Independent replay of exported trace references and full motion timing."""
import numpy as np


def audit_trace(directory, trajectory, robot):
    from ..motion_planning.audit import audit_trace as audit
    return audit(directory, trajectory, robot)


def audit_motion(trajectory, robot, checker, cfg):
    """Validate the polynomial actually executed, including its interior extrema."""
    geometry_only = getattr(cfg, 'geometry_only', False)
    minimum, samples, max_speed, max_angular, max_free_speed = np.inf, 0, 0., 0., 0.
    import mujoco
    trace = trajectory.constant_trace
    if trace is not None:
        trace_duration = trajectory.times[trace.end]-trajectory.times[trace.start]
    jacp, jacr = np.zeros((3, robot.model.nv)), np.zeros((3, robot.model.nv))
    for i in range(len(trajectory.phases)):
        lo, hi = trajectory.position_bounds(i)
        if np.any(lo < robot.lower) or np.any(hi > robot.upper):
            raise RuntimeError('Continuous trajectory exceeds joint limits')
        for t in trajectory.interval_times(i, cfg.sweep_step_rad):
            state = trajectory.sample(t)
            # Geometry-only runs still replay the whole executed curve; they
            # just never ask the (absent) collision checker anything.
            if not geometry_only:
                check = checker.check(state['q'], cfg.clearance_mm*.001)
                minimum = min(minimum, check.clearance_m); samples += 1
                if not check.valid:
                    raise RuntimeError('Continuous trajectory collision: '+check.pair)
            robot.fk(state['q'], full=True)
            mujoco.mj_jacSite(robot.model, robot.verify_data, jacp, jacr, robot.site)
            angular = np.degrees(np.linalg.norm(jacr[:, robot.dadr]@state['dq']))
            max_angular = max(max_angular, angular)
            speed = float(np.linalg.norm(jacp[:, robot.dadr]@state['dq'])*1000)
            if state['phase'] in ('trace', 'surface_transfer'):
                max_speed = max(max_speed, speed)
            else:
                max_free_speed = max(max_free_speed, speed)
    limits = trajectory.limits()
    if (limits['max_joint_speed_rad_s'] > cfg.joint_speed_rad_s+1e-8
            or limits['max_joint_accel_rad_s2'] > cfg.joint_accel_rad_s2+1e-8
            or limits['tcp_angular_speed_bound_deg_s'] > cfg.orientation_speed_deg_s+1e-8):
        raise RuntimeError('Continuous trajectory rate limits exceeded')
    report = {'passed': True, 'actual_time_interpolant': True, 'samples': samples,
            'collision_checked': not geometry_only,
            'minimum_clearance_mm': None if geometry_only else float(minimum*1000),
            'max_trace_tcp_speed_mm_s': float(max_speed), 'max_tcp_angular_speed_deg_s': float(max_angular),
            'max_free_tcp_speed_mm_s': float(max_free_speed),
            'sweep_max_joint_step_rad': cfg.sweep_step_rad}
    if trace is not None and trace.version == 2:
        report['speed_profile'] = {'mode': 'local', 'solver': 'TOPPRA/ECOS',
                                  'path_length_mm': trace.length*1000,
                                  'minimum_certified_trace_duration_s': trace.minimum_duration_s,
                                  'trace_duration_s': trace_duration,
                                  'rate_bounds_method': 'Bernstein bounds on fixed Q(u) composed with C2 scalar clock'}
    return report
