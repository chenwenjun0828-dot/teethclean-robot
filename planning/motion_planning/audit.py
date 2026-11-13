"""Independent full-MuJoCo FK verification of the saved timed trajectory."""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import minimize_scalar
from planning.infrastructure.io import dump
from planning.infrastructure.poses import unit, TCP_FRAME_ID
from planning.infrastructure.robot import rotation_error
from planning.infrastructure.audit import audit_motion
from ..pose_search.constraints import normal_angle, tool_rotation, axis_convention, gravity_frame
from ..config import Config, CONTINUOUS_PATH_KINDS


def audit_trace(directory, trajectory, robot, raise_on_failure=True):
    directory = Path(directory)
    report = json.loads((directory/'planning_report.json').read_text(encoding='utf-8'))
    config = Config.from_saved(report['config'])
    world = np.asarray(report['world_from_scan_m'])
    offset = report['config']['standoff_mm']*.001
    if report.get('axis_convention') != axis_convention(config) or report.get('tcp_frame_id') != TCP_FRAME_ID:
        raise ValueError('This audit requires the saved planning mode axis convention')
    maximum = {'position_error_mm': 0., 'normal_angle_error_deg': 0., 'frame_error_deg': 0., 'rotation_error_deg': 0.}
    rows = []
    maximum_regional_offsets = np.zeros(3)
    failed_intervals = set()
    if report.get('path_kind') not in CONTINUOUS_PATH_KINDS:
        raise ValueError('Only fixed-angle continuous plans are supported; replan')
    from .contract import load_sweep_reference, validate_sweep_contract
    sweep = load_sweep_reference(directory, report)
    contract = validate_sweep_contract(trajectory, report, sweep)
    for segment in report['motion_segments']:
        if segment['status'] != 'planned': continue
        u = np.asarray(segment['parameters'])
        position = lambda s: sweep.position(s, offset)
        tangent_at = lambda s: unit(sweep.evaluate(s, 1))
        normal_at = sweep.normal
        start, end = segment['trace_start_waypoint'], segment['trace_end_waypoint']
        if end-start+1 != len(u):
            raise ValueError('Trace reference mapping is inconsistent')
        for j in range(len(u)-1):
            interval = start+j
            for time in trajectory.interval_times(interval, report['config']['sweep_step_rad']):
                state = trajectory.sample(time)
                actual = robot.fk(state['q'], full=True)
                local = world[:3, :3].T@(actual[:3, 3]-world[:3, 3])
                # Match spatial progress, independent of nonuniform time parameterization.
                objective = lambda f: float(np.sum((position(u[j]+f*(u[j+1]-u[j]))-local)**2))
                fit = minimize_scalar(objective, bounds=(0., 1.), method='bounded', options={'xatol': 1e-10})
                fraction = min([0., float(fit.x), 1.], key=objective)
                parameter = u[j]+fraction*(u[j+1]-u[j])
                desired_p = world[:3, :3] @ position(parameter)+world[:3, 3]
                local_tangent = tangent_at(parameter)
                normal = normal_at(parameter)
                measured_angle = normal_angle(actual[:3, :3], world[:3, :3]@normal)
                roll0, roll1 = segment['rolls_deg'][j:j+2]
                roll = roll0+fraction*((roll1-roll0+180)%360-180)
                if config.planning_mode == 'simple':
                    target_r = world[:3, :3]@sweep.rotation(parameter)
                    deviation = abs(sweep.measured_offset(world[:3, :3].T@actual[:3, :3]))
                    maximum_regional_offsets = np.maximum(maximum_regional_offsets, deviation)
                    t_error = abs(measured_angle-normal_angle(target_r, world[:3, :3]@normal))
                    expected_x = target_r[:, 0]
                else:
                    gravity_scan = world[:3, :3].T@config.gravity_world
                    frame = gravity_frame(-normal, gravity_scan)
                    alpha, beta = np.deg2rad([config.tilt_deg, config.tilt_azimuth_deg])
                    expected_x = world[:3, :3]@frame@np.array([
                        np.cos(beta)*np.cos(alpha), np.sin(beta)*np.cos(alpha), -np.sin(alpha)])
                    target_r = world[:3, :3]@tool_rotation(
                        local_tangent, normal, config.tilt_deg, roll, config.tilt_azimuth_deg, gravity=gravity_scan)
                    t_error = abs(measured_angle-normal_angle(target_r, world[:3, :3]@normal))
                angle_error = float(np.degrees(np.arccos(np.clip(expected_x@actual[:3, 0], -1., 1.))))
                rotation_deg = float(np.degrees(np.linalg.norm(rotation_error(target_r, actual[:3, :3]))))
                position_error = float(np.linalg.norm(actual[:3, 3]-desired_p)*1000)
                values = [position_error, t_error, angle_error, rotation_deg]
                if (position_error > report['config']['interpolation_position_mm']+1e-6 or
                        max(t_error, angle_error, rotation_deg) > report['config']['interpolation_angle_deg']+1e-6):
                    failed_intervals.add(int(interval))
                for key, value in zip(maximum, values): maximum[key] = max(maximum[key], value)
                rows.append([sweep.tooth(parameter), segment['stroke_id'], parameter,
                             normal_angle(actual[:3, :3], world[:3, :3]@normal), *values])
    if not rows:
        raise ValueError('No trace references to audit')
    cfg = report['config']
    maximum['position_passed'] = maximum['position_error_mm'] <= cfg['interpolation_position_mm']+1e-6
    maximum['orientation_passed'] = max(maximum['normal_angle_error_deg'], maximum['frame_error_deg'],
                                      maximum['rotation_error_deg']) <= cfg['interpolation_angle_deg']+1e-6
    maximum['orientation_enforced'] = cfg.get('orientation_audit', 'strict') == 'strict'
    maximum['passed'] = maximum['position_passed'] and (
        maximum['orientation_passed'] or not maximum['orientation_enforced'])
    if config.planning_mode == 'simple':
        maximum['regional_angle_limit_deg'] = 5.
        maximum['maximum_regional_angle_offsets_deg'] = maximum_regional_offsets.tolist()
        maximum['regional_angle_bounds_passed'] = bool(np.all(maximum_regional_offsets <= 5.+1e-5))
        maximum['passed'] = maximum['passed'] and maximum['regional_angle_bounds_passed']
    angles = np.asarray(rows)[:, 3]
    maximum.update(actual_time_interpolant=True, samples=len(rows), motion=trajectory.limits(), full_fk=True,
                   planning_mode=config.planning_mode,
                   **contract,
                   failed_intervals=sorted(failed_intervals),
                   measured_normal_angle_min_deg=float(angles.min()), measured_normal_angle_max_deg=float(angles.max()),
                   measured_normal_angle_median_deg=float(np.median(angles)))
    dump(directory/'trace_audit.json', maximum)
    np.savetxt(directory/'trace_audit.csv', rows, delimiter=',', comments='',
               header='tooth,stroke,parameter,measured_normal_angle_deg,position_error_mm,normal_angle_error_deg,frame_error_deg,rotation_error_deg')
    if not maximum['passed'] and raise_on_failure:
        regional = (f'; regional angle offsets {maximum_regional_offsets.tolist()} / +/-5 deg'
                    if config.planning_mode == 'simple' else '')
        raise RuntimeError('Independent trace audit failed: '
                           f'position {maximum["position_error_mm"]:.4f} / {cfg["interpolation_position_mm"]:g} mm, '
                           f'rotation {maximum["rotation_error_deg"]:.3f} / {cfg["interpolation_angle_deg"]:g} deg{regional}; '
                           f'failed intervals {sorted(failed_intervals)[:12]}. Fixed curve was not refitted.')
    return maximum
