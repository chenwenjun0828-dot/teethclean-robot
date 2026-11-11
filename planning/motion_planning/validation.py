"""Build and independently validate the initial timed planning result."""
from planning.infrastructure.trajectory import Trajectory
from planning.infrastructure.io import dump, fingerprint
from .audit import audit_trace, audit_motion

GEOMETRY_ONLY_STATUS = 'geometry_only_verified'


def validate(output, plan, report, robot, checker, cfg):
    """One exported curve audit and one physics run; no geometry repair loop.

    In geometry_only mode the exported curve, reachability and motion rates are
    audited, but no clearance is measured and no dynamics run is performed. The
    result is deliberately reported as not execution-ready.
    """
    from pathlib import Path
    from time import perf_counter
    import mujoco
    from control.position import execute
    from .fixed_path import phase_durations
    if plan is None:
        return None, report
    if plan.constant_trace is None or plan.constant_trace.version != 2:
        raise ValueError('New planning validation requires a TOPPRA fixed-path trajectory')
    output = Path(output)
    geometry_only = cfg.geometry_only
    try:
        report.update(execution_ready=False, fast_dynamics_verified=False, status='validating_fixed_path')
        dump(output/'planning_report.json', report)
        plan.save(output/'trajectory.npz')
        plan = Trajectory.load(output/'trajectory.npz')
        print('Checking fixed spline geometry and motion rates'
              + (' (collision checking disabled)' if geometry_only else ' and collision'), flush=True)
        trace = audit_trace(output, plan, robot)
        motion = audit_motion(plan, robot, checker, cfg)
        if (motion['max_trace_tcp_speed_mm_s'] > cfg.speed_mm_s+1e-8
                or motion['max_free_tcp_speed_mm_s'] > cfg.transfer_speed_mm_s+1e-8):
            raise RuntimeError('Fixed trajectory exceeds the configured TCP speed limit')
        report.update(trace_audit=trace, motion_audit=motion, motion=plan.limits(), duration_s=plan.duration,
                      phase_durations_s=phase_durations(plan),
                      trajectory_file=fingerprint(output/'trajectory.npz'),
                      planes_file=fingerprint(output/'tooth_planes.json'),
                      geometry_file=fingerprint(output/'geometry.json'),
                      all_selected_teeth_have_motion=all(r['retained_length_mm'] > 0 for r in report['per_tooth'].values()),
                      all_selected_meet_coverage=all(r['retained_length_mm']/max(r['source_length_mm'], 1e-12)
                          >= cfg.min_tooth_coverage for r in report['per_tooth'].values()))
        if geometry_only:
            report.update(check_mode=cfg.check_mode, collision_checking_enabled=False,
                          minimum_clearance_mm=None,
                          minimum_free_space_clearance_mm=None,
                          clearance_measured=False)
        else:
            report.update(check_mode=cfg.check_mode, collision_checking_enabled=True,
                          minimum_clearance_mm=motion['minimum_clearance_mm'],
                          minimum_free_space_clearance_mm=motion['minimum_clearance_mm'],
                          clearance_measured=True)
        if not report['all_selected_teeth_have_motion'] or not report['all_selected_meet_coverage']:
            raise RuntimeError('Fixed trajectory does not meet the selected teeth coverage requirement')
        if geometry_only:
            # Reachability only: never claim execution readiness without a
            # clearance certificate and a dynamics run.
            report.update(status=GEOMETRY_ONLY_STATUS, execution_ready=False,
                          fast_dynamics_verified=False, geometry_verified=True,
                          geometry_only_reason='Collision checking was disabled; this trajectory '
                                               'verifies pose/path feasibility only')
            dump(output/'planning_report.json', report)
            print('Geometry-only plan verified: reachable poses, connected IK branch and motion rates; '
                  'clearance and dynamics were not evaluated', flush=True)
            return plan, report
        dump(output/'planning_report.json', report)
        print(f'Checking actual dynamics once for {plan.duration:g} seconds of simulation', flush=True)
        data = mujoco.MjData(robot.model)
        data.qpos[:] = robot.initial_qpos
        mujoco.mj_forward(robot.model, data)
        started = perf_counter()
        execution = execute(robot.model, data, plan, checker, output, headless=True)
        if execution['status'] != 'completed' or execution['minimum_actual_clearance_mm'] <= 0:
            raise RuntimeError('Fixed-path dynamics did not complete with positive clearance')
        dump(output/'dynamics_validation.json', execution)
        report.update(execution_ready=True, status=report['path_kind'], fast_dynamics_verified=True,
                      fast_execution_summary=execution, dynamics_validation_wall_time_s=perf_counter()-started,
                      dynamics_validation_file=fingerprint(output/'dynamics_validation.json'),
                      execution_report_file=fingerprint(output/'execution_report.json'))
        dump(output/'planning_report.json', report)
        return plan, report
    except (ValueError, RuntimeError) as error:
        report.update(execution_ready=False, status='fixed_path_validation_failed', error=str(error))
        dump(output/'planning_report.json', report)
        print(f'Fixed-path plan rejected: {error}', flush=True)
        return None, report
