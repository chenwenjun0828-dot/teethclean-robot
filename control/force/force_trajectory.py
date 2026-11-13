"""Collision-enabled, fixed-jaw execution of an entire retained planning path.

This is simulation only. A private one-step rollout rejects commands whose next
sample predicts an unintended collision or force >= 2.5 N, before mj_step touches
the executed state. A stopped or timed-out path is never reported as completed.
"""
import json
from dataclasses import asdict
from pathlib import Path

import mujoco
import numpy as np
from scipy.interpolate import BSpline

from planning.infrastructure.io import dump, fingerprint
from planning.infrastructure.robot import Robot, rotation_error
from .contact_experiment import contacts, fixed_jaws, write_csv
from planning.execute import load_verified
from pipeline.simulation import create_simulation
from .compliance import ForceControlConfig, SensorContactForce, force_components, feedback_force
from .contact_path import ContactPath, ContactPathConfig
from planning.config import CONTINUOUS_PATH_KINDS


# Re-export these names for existing callers and tests.
from .force_servo import CartesianServo, apply_tool_roll
from .force_guard import PredictiveContactGuard
from .force_plot import plot


def length_tables(source, segments):
    """Arc-length lookup, including every retained segment (no path trimming)."""
    geo = json.loads((source/'geometry.json').read_text(encoding='utf-8'))
    strokes = {s['id']: s for s in geo['strokes']}
    tables = []
    sweep = None
    if any('reference_part_index' in s for s in segments):
        from planning.motion_planning.contract import load_sweep_reference
        saved = json.loads((source/'planning_report.json').read_text(encoding='utf-8'))
        sweep = load_sweep_reference(source, saved)
    for s in segments:
        start, end = s['parameters'][0], s['parameters'][-1]
        fraction = np.linspace(0., 1., max(1001, 10*len(s['parameters'])))
        if 'reference_part_index' in s:
            points = np.array([sweep.evaluate(float(s)) for s in start+fraction*(end-start)])
        else:
            record = geo['curves'][strokes[s['stroke_id']]['curve_id']]
            curve = BSpline(record['knots'], record['coefficients_m'], record['degree'])
            points = curve(start+fraction*(end-start))
        length = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]*1000
        tables.append((fraction, length))
    return tables


def segment_length_at(plan, segment, table, time):
    a, b = segment['trace_start_waypoint'], segment['trace_end_waypoint']
    # The saved parameters are aligned with source joint waypoints. Interpolation
    # between them is an arc-progress estimate, not an actual TCP path measurement.
    u = np.interp(time, plan.times[a:b+1], segment['parameters'])
    fraction = (u-segment['parameters'][0])/(segment['parameters'][-1]-segment['parameters'][0])
    return float(np.interp(fraction, *table))


def run(source, output, force_config=None, path_config=None, contact_margin_mm=.05, roll_corrections_deg=None,
        show_viewer=False, segment_speeds_m_s=None, contact_schedule='auto', contact_model=None, stage='regional', contact_material=None,
        previous_stage=None):
    """Persist a failed status as well as the signal log if an exception occurs."""
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError('Use a new empty output directory')
    panel = None
    try:
        if show_viewer:
            from .force_panel import ForcePanel
            panel = ForcePanel()
        return _run(source, output, force_config, path_config, contact_margin_mm, roll_corrections_deg, panel,
                    segment_speeds_m_s, contact_schedule, contact_model, stage, contact_material, previous_stage)
    except (Exception, KeyboardInterrupt) as error:
        report_file = output/'force_trajectory_report.json'
        output.mkdir(parents=True,exist_ok=True)
        if not report_file.exists():
            dump(report_file,dict(status='failed',force_control_passed=False,stage_passed=False,
                simulation_only=True,experiment_stage=stage,source=str(source),stop_reason=f'{type(error).__name__}: {error}'))
        if report_file.exists():
            report = json.loads(report_file.read_text(encoding='utf-8'))
            report.update(status='failed', force_control_passed=False,stage_passed=False,
                          stop_reason=f'{type(error).__name__}: {error}')
            dump(report_file, report)
        raise
    finally:
        if panel is not None:
            panel.close()


def _run(source, output, force_config=None, path_config=None, contact_margin_mm=.05, roll_corrections_deg=None,
         live_panel=None, segment_speeds_m_s=None, contact_schedule='auto', contact_model=None, stage='regional', contact_material=None,
         previous_stage=None):
    fc, cfg = force_config or ForceControlConfig(), path_config or ContactPathConfig()
    cfg.validate(fc)
    if not np.isfinite(contact_margin_mm) or not 0 <= contact_margin_mm <= .1:
        raise ValueError('Contact activation margin must lie in [0, 0.1] mm')
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError('Use a new empty output directory')
    _, _, plan, verification_checker = load_verified(source)
    saved = json.loads((source/'planning_report.json').read_text(encoding='utf-8'))
    segments = saved['motion_segments']
    continuous_source = saved.get('path_kind') in CONTINUOUS_PATH_KINDS
    if contact_schedule not in ('auto', 'regional', 'segments'):
        raise ValueError('Contact schedule must be auto, regional or segments')
    regional_contact = contact_schedule == 'regional' or (contact_schedule == 'auto' and continuous_source)
    if regional_contact and not continuous_source:
        raise ValueError('Regional contact requires a continuous regional source')
    if continuous_source and not regional_contact:
        from .sweep_source import adapt_sweep
        plan, segments = adapt_sweep(source, plan, saved)
    request = json.loads((source/'request.json').read_text(encoding='utf-8'))
    planes = json.loads((source/'tooth_planes.json').read_text(encoding='utf-8'))
    world = np.asarray(saved['world_from_scan_m'])
    normals = {int(k): world[:3, :3]@np.asarray(v['normal_scan']) for k, v in planes.items()}
    if regional_contact:
        if roll_corrections_deg or segment_speeds_m_s:
            raise ValueError('Regional contact retains the saved pose policy; per-segment overrides require segments mode')
        from .continuous_path import RegionalContactPath
        path = RegionalContactPath(source, plan, saved, world, fc, cfg, stage=stage)
        plan = path.plan
    else:
        if stage != 'regional':
            raise ValueError('hold/short stages require a continuous regional source')
        path = ContactPath(plan, segments, normals, fc, cfg, roll_corrections_deg, segment_speeds_m_s)
    if live_panel is not None:
        live_panel.set_config(fc, cfg, .001)
    tables = length_tables(source, path.segments)
    planned_length = sum(table[1][-1] for table in tables)
    jaw = request['inputs']['jaw']
    simulation_options = {'robot_mount_position_m': saved['robot_mount_position_m']}
    if saved.get('jaw_opening_deg') is not None:
        from pipeline.oral import jaw_overrides, load_jaw_settings
        simulation_options['jaw_settings'] = jaw_overrides(load_jaw_settings(), saved['jaw_opening_deg'])
    model, data = create_simulation(**simulation_options)
    model_info = None
    if contact_model is not None:
        from .contact_model import load_contact_model
        model, data, model_info = load_contact_model(contact_model, source, model, data, material_file=contact_material)
    fixed_jaws(model)
    if contact_model is not None and contact_margin_mm != 0:
        raise ValueError('Verified contact assets require zero activation margin; set --contact-margin-mm 0')
    if contact_model is None:
        for name in ('dental_mandibular_visual', 'dental_maxillary_visual', 'ur_end_scaler_geom'):
            g = model.geom(name).id
            model.geom_contype[g] = model.geom_conaffinity[g] = 1
            model.geom_margin[g] = contact_margin_mm*.001
    model.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    model.opt.timestep, model.opt.ccd_iterations, model.opt.ccd_tolerance = .001, 100, 1e-9
    robot = Robot(model, data)
    data.qpos[robot.qadr] = plan.joints[0]
    data.ctrl[:] = plan.joints[0]
    mujoco.mj_forward(model, data)
    observer = SensorContactForce(model)
    servo = CartesianServo(model, robot)
    geometry_guard = None
    previous_evidence = None
    if model_info and model_info.get('method') == 'local_surface':
        from .stage_gate import verify_previous
        if fc.force_component!='normal' or not regional_contact or cfg.max_trace_speed_m_s>.001:
            raise ValueError('Local stages require regional normal-force control with tangential speed <= 1 mm/s')
        if fc.target_force_N != (1. if stage=='regional' else .5):
            raise ValueError('Local hold/short require 0.5 N; regional requires 1 N')
        previous_evidence=verify_previous(stage,source,contact_model,previous_stage,fc,contact_material)
        from .scan_guard import OriginalScanGuard
        geometry_guard = OriginalScanGuard(verification_checker,source,model,robot,contact_model)
    guard = PredictiveContactGuard(model, robot, jaw, observer, cfg.predictive_force_N, saved['selected_teeth'],geometry_guard)
    jaw_ids = [model.body('dental_'+j).id for j in ('mandibular', 'maxillary')]
    fixed_pos, fixed_rot = data.xpos[jaw_ids].copy(), data.xmat[jaw_ids].copy()
    output.mkdir(parents=True, exist_ok=True)
    report = {'status': 'running', 'experiment': 'full_retained_gingival_force_trajectory',
              'simulation_only': True, 'jaw': jaw, 'physics_collisions_enabled': True, 'fixed_jaws': True,
              'force_control': asdict(fc), 'path_control': asdict(cfg),
              'segment_trace_speed_mm_s_one_based': {str(i+1): float(v*1000) for i, v in enumerate(path.trace_speeds_m_s)},
              'segment_roll_corrections_deg_one_based': {str(i+1): float(np.degrees(v)) for i, v in enumerate(path.roll_corrections) if v},
              'source': str(source), 'source_trajectory': fingerprint(source/'trajectory.npz'),
              'continuous_source_adapted': continuous_source,
              'source_jaw_opening_deg': saved.get('jaw_opening_deg'),
              'contact_schedule': ('One acquisition, one continuous regional trace including seams, then release and original retreat'
                                   if regional_contact else 'Acquire/trace/release on every retained source arc; original retreat retained'),
              'contact_schedule_mode': 'regional' if regional_contact else 'segments',
              'regulated_quantity': fc.force_component, 'experiment_stage': stage,
              'contact_model_manifest': model_info,
              'geometry_acceptance_scope': 'Local front work band only; not complete tooth-body contact capability',
              'online_original_mesh_guard': geometry_guard is not None,
              'previous_stage_evidence': previous_evidence,
              'clinical_or_hardware_validation': False,
              'source_planning_report': fingerprint(source/'planning_report.json'),
              'source_scene': request['scene'], 'source_robot_model': request['robot_model'],
              'source_oral_settings': request['oral_settings'],
              'code_files': [fingerprint(Path(__file__)), fingerprint(Path(__file__).with_name('contact_path.py')),
                             fingerprint(Path(__file__).with_name('compliance.py')),
                             fingerprint(Path(__file__).with_name('force_servo.py')),
                             fingerprint(Path(__file__).with_name('force_guard.py')),
                             fingerprint(Path(__file__).with_name('sweep_source.py')),
                             fingerprint(Path(__file__).with_name('continuous_path.py')),
                             fingerprint(Path(__file__).with_name('scan_guard.py')),
                             fingerprint(Path(__file__).with_name('stage_gate.py')),
                             fingerprint(Path(__file__).with_name('contact_model.py')),
                             fingerprint(Path(__file__).with_name('validation.py'))],
              'mujoco_version': mujoco.__version__, 'sample_rate_hz': 1000,
              'region': saved.get('region', 'outer_front'), 'selected_teeth': saved['selected_teeth'],
              'source_region_length_mm': saved.get('source_region_length_mm'),
              'source_retained_length_mm': saved['retained_length_mm'],
              'source_coverage_fraction': saved['coverage_fraction'],
              'retained_segment_count': len(path.segments), 'source_duration_s': plan.duration,
              'nominal_source_standoff_mm': saved['config']['standoff_mm'],
              'collision_model': ('Verified local inward-extruded work-band patches' if geometry_guard is not None
                                  else 'Verified convex components' if model_info else 'Legacy original mesh convex hulls'),
              'contact_margin_per_geom_mm': contact_margin_mm,
              'contact_response': 'original',
              'contact_parameters': {name: {'solimp': model.geom_solimp[model.geom(name).id].copy(),
                  'solref': model.geom_solref[model.geom(name).id].copy(),
                  'friction': model.geom_friction[model.geom(name).id].copy()}
                  for name in ('dental_mandibular_visual', 'dental_maxillary_visual', 'ur_end_scaler_geom')},
              'allowed_contact': 'Tool / selected jaw only, within 3 mm of TCP',
              'initialization': 'One initial home state; continuous transfers; no per-tooth resets',
              'prediction_policy': 'Check command application AND next sample in private data before committing; no executed samples discarded',
              'progress_metric': 'Arc length estimated from source waypoint parameters; separately count force-qualified progress'}
    dump(output/'force_trajectory_report.json', report)
    if model_info:
        dump(output/'contact_material_used.json',model_info['material_settings'])
    rows, hit_rows = [], []
    progress = np.zeros(len(path.segments))
    qualified = np.zeros_like(progress)
    previous_progress = np.zeros_like(progress)
    reason, predicted_rejection = '', None
    dt = model.opt.timestep
    last_stage = None
    from .motion_guard import MotionProgressGuard
    motion_guard = MotionProgressGuard()
    if live_panel is not None:
        live_panel.open(model, data, jaw)
    for _ in range(int(round(cfg.max_duration_s/dt))+1):
        mujoco.mj_forward(model, data)
        measured = observer.read(data)
        current_nominal = robot.fk(plan.sample(path.source_time)['q'])
        error = data.site_xpos[robot.site]-current_nominal[:3, 3]
        tangent_error = np.linalg.norm(error-path.normal*(error@path.normal))
        feedback_normal = path.normal.copy()
        try:
            feedback = feedback_force(measured, feedback_normal, fc.force_component)
        except ValueError as error:
            feedback = 0.
            path.fault_reason = str(error)
        state = path.update(feedback, tangent_error, dt, total_force_N=measured['force_N'])
        if (state['segment_index'], state['phase']) != last_stage:
            print(f"t={data.time:.3f}s tooth={state['tooth']} segment={state['segment_index']+1}/{len(path.segments)} {state['phase']}", flush=True)
            last_stage = (state['segment_index'], state['phase'])
        try:
            command, q, dq, ddq, nominal, target = servo.command(data, state, dt)
        except RuntimeError as error:
            reason = str(error)
            command, q, dq, ddq = data.ctrl.copy(), data.qpos[robot.qadr].copy(), np.zeros(6), np.zeros(6)
            nominal = target = robot.pose(data)
        hits, bad = contacts(model, data, robot, jaw, saved['selected_teeth'])
        geometry = geometry_guard.assess(data,state['phase']) if geometry_guard is not None else {'safe':True}
        if not geometry['safe']:
            reason='Original/local geometry guard: '+geometry['reason']
            report['geometry_stop_evidence']=geometry
        hit_rows.extend([{'time_s': float(data.time), **h} for h in hits])
        oracle = np.sum([[h[f'force_on_robot_world_{axis}_N'] for axis in 'xyz'] for h in hits
                         if any(n == 'ur_end_scaler_geom' or n.startswith('ur_force_tool_') for n in (h['geom1'], h['geom2']))], axis=0) if hits else np.zeros(3)
        tip = data.site_xpos[robot.site].copy()
        tcp_twist = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE, robot.site, tcp_twist, 0)
        if state['phase'] == 'acquire':
            try:
                motion_guard.update(float(tip@state['normal_world']), path.admittance.velocity, float(data.time))
            except RuntimeError as error:
                reason = str(error)
        else:
            motion_guard = MotionProgressGuard()
        tracking_error = float(np.linalg.norm(tip-target[:3, 3]))
        tangent = tip-nominal[:3, 3]
        tangent -= state['normal_world']*(tangent@state['normal_world'])
        tangent_error = float(np.linalg.norm(tangent))
        rotation_deg = float(np.degrees(np.linalg.norm(rotation_error(target[:3, :3], data.site_xmat[robot.site].reshape(3, 3)))))
        jaw_drift = float(np.max(np.linalg.norm(data.xpos[jaw_ids]-fixed_pos, axis=1)))
        jaw_rotation = float(np.max(abs(data.xmat[jaw_ids]-fixed_rot)))
        components = force_components(measured, feedback_normal)
        row = {'time_s': float(data.time), 'source_time_s': state['source_time_s'],
               'phase': state['phase'], 'segment': state['segment_index'], 'tooth': state['tooth'],
               'force_target_N': state['force_target_N'],
               'regulated_force_N': feedback, 'applied_regulated_force_N': feedback,
               **components,
               **dict(zip(['normal_world_x','normal_world_y','normal_world_z'], feedback_normal)),
               **dict(zip(['external_force_world_x_N','external_force_world_y_N','external_force_world_z_N'], measured['external_force_world_N'])), 'compensated_force_N': measured['force_N'],
               'contact_normal_sum_N': sum(max(0., h['normal_force_N']) for h in hits),
               'applied_command_force_N': measured['force_N'],
               'applied_command_contact_normal_sum_N': sum(max(0., h['normal_force_N']) for h in hits),
               'command_applied': False,
               'compensated_force_normal_N': components['normal_force_N'],
               'filtered_force_N': path.admittance.filtered_force,
               'normal_correction_mm': float(state['offset_world_m']@state['normal_world']*1000),
               'actual_normal_offset_from_scan_mm': float(saved['config']['standoff_mm']
                    + (tip-nominal[:3, 3])@state['normal_world']*1000),
               'actual_tcp_speed_mm_s': float(np.linalg.norm(tcp_twist[3:])*1000),
               **dict(zip(['actual_tcp_x_m', 'actual_tcp_y_m', 'actual_tcp_z_m'], tip)),
               'path_rate': state['path_rate'], 'tangent_error_mm': tangent_error*1000,
               'roll_correction_deg': float(np.degrees(state['roll_correction_rad'])),
               'position_error_mm': tracking_error*1000, 'orientation_error_deg': rotation_deg,
               'jaw_translation_drift_mm': jaw_drift*1000, 'jaw_rotation_matrix_drift': jaw_rotation,
               'unexpected_contact_count': len(bad), 'contact_count': len(hits),
               'sensor_solver_force_difference_N': float(np.linalg.norm(measured['external_force_world_N']-oracle)),
               'applied_sensor_solver_force_difference_N': float(np.linalg.norm(measured['external_force_world_N']-oracle)),
               'max_command_joint_speed_rad_s': float(np.max(abs(dq))),
               'max_command_joint_accel_rad_s2': float(np.max(abs(ddq))),
               **dict(zip(['Fx_N', 'Fy_N', 'Fz_N', 'Tx_Nm', 'Ty_Nm', 'Tz_Nm'], measured['raw_sensor_wrench'])),
               **dict(zip(['applied_Fx_N', 'applied_Fy_N', 'applied_Fz_N', 'applied_Tx_Nm', 'applied_Ty_Nm', 'applied_Tz_Nm'], measured['raw_sensor_wrench'])),
               **dict(zip(robot.joint_names, data.qpos[robot.qadr]))}
        rows.append(row)
        if live_panel is not None and not live_panel.is_running():
            reason = 'MuJoCo viewer closed by user'
        qualified_delta = 0.
        if state['phase'] == 'trace':
            i = state['segment_index']
            if regional_contact:
                parameters = path.segments[i]['parameters']
                fraction = (state['reference_parameter_m']-parameters[0])/(parameters[-1]-parameters[0])
                progress[i] = float(np.interp(fraction,*tables[i]))
            else:
                progress[i] = segment_length_at(plan, path.segments[i], tables[i], state['source_time_s'])
            delta = max(0., progress[i]-previous_progress[i])
            if feedback >= fc.contact_threshold_N and tangent_error <= cfg.max_tangent_error_m:
                qualified[i] += delta
                qualified_delta = delta
            previous_progress[i] = progress[i]
        if bad: reason = 'Unintended collision: ' + '; '.join(sorted(set(bad)))
        elif measured['force_N'] >= fc.force_stop_N or row['contact_normal_sum_N'] >= fc.force_stop_N:
            reason = 'Measured force reached 3 N/hard limit'
        elif state['fault_reason']: reason = state['fault_reason']
        elif jaw_drift > 1e-11 or jaw_rotation > 1e-10: reason = 'Fixed jaw moved'
        elif tracking_error > .002 or rotation_deg > 2.: reason = 'TCP tracking error exceeded bound'
        elif (np.max(abs(dq)) > cfg.max_joint_speed_rad_s
                or np.max(abs(ddq)) > cfg.max_joint_accel_rad_s2): reason = 'Command joint speed/acceleration exceeded bound'
        elif not np.isfinite(data.qpos).all() or any(w.number for w in data.warning): reason = 'Invalid dynamics'
        if reason or path.completed: break
        if data.time >= cfg.max_duration_s-1e-9:
            reason = 'Execution time budget exhausted; remaining path was not skipped'
            break
        prediction = guard.check(data, command, state['phase'],
            normal_world=state['normal_world'] if fc.force_component=='normal' else None)
        if not prediction['safe']:
            reason, predicted_rejection = prediction['reason'], prediction
            break
        data.ctrl[:] = command
        mujoco.mj_forward(model, data)
        applied = observer.read(data)
        applied_hits, _ = contacts(model, data, robot, jaw, saved['selected_teeth'])
        applied_oracle = np.sum([[h[f'force_on_robot_world_{axis}_N'] for axis in 'xyz'] for h in applied_hits
                                 if any(n == 'ur_end_scaler_geom' or n.startswith('ur_force_tool_') for n in (h['geom1'], h['geom2']))], axis=0) if applied_hits else np.zeros(3)
        applied_components = force_components(applied, feedback_normal)
        applied_feedback = feedback_force(applied, feedback_normal, fc.force_component)
        row.update(applied_command_force_N=applied['force_N'], command_applied=True,
                   applied_regulated_force_N=applied_feedback, applied_normal_force_N=applied_components['normal_force_N'],
                   applied_tangential_force_N=applied_components['tangential_force_N'],
                   **dict(zip(['applied_external_force_world_x_N','applied_external_force_world_y_N','applied_external_force_world_z_N'], applied['external_force_world_N'])),
                   applied_sensor_solver_force_difference_N=float(np.linalg.norm(applied['external_force_world_N']-applied_oracle)),
                   applied_command_contact_normal_sum_N=sum(max(0., h['normal_force_N']) for h in applied_hits),
                   **dict(zip(['applied_Fx_N', 'applied_Fy_N', 'applied_Fz_N', 'applied_Tx_Nm', 'applied_Ty_Nm', 'applied_Tz_Nm'], applied['raw_sensor_wrench'])))
        if state['phase'] == 'trace' and applied_feedback < fc.contact_threshold_N:
            qualified[state['segment_index']] -= qualified_delta
        if live_panel is not None:
            live_panel.update(data, row, progress.sum()/planned_length)
        mujoco.mj_step(model, data)
    trace = [r for r in rows if r['phase'] == 'trace']
    force = np.array([r['applied_regulated_force_N'] for r in trace])
    before_force = np.array([r['regulated_force_N'] for r in trace])
    all_force = np.array([[r['compensated_force_N'], r['applied_command_force_N']] for r in rows])
    completed = path.completed and not reason
    report.update(status='completed' if completed else 'stopped', stop_reason=reason,
                  predicted_rejected_step=predicted_rejection, recorded_duration_s=rows[-1]['time_s'],
                  samples=len(rows), trace_samples=len(trace),
                  scheduled_trace_length_mm=float(progress.sum()),
                  contact_qualified_trace_length_mm=float(qualified.sum()),
                  retained_arc_length_mm=float(planned_length),
                  retained_path_progress_fraction=float(progress.sum()/planned_length),
                  contact_qualified_progress_fraction=float(qualified.sum()/planned_length),
                  peak_compensated_force_N=float(all_force.max()),
                  peak_contact_normal_sum_N=max(max(r['contact_normal_sum_N'], r['applied_command_contact_normal_sum_N']) for r in rows),
                  max_tangent_error_mm=max(r['tangent_error_mm'] for r in trace) if trace else None,
                  max_position_error_mm=max(r['position_error_mm'] for r in rows),
                  max_orientation_error_deg=max(r['orientation_error_deg'] for r in rows),
                  max_actual_trace_tcp_speed_mm_s=max(r['actual_tcp_speed_mm_s'] for r in trace) if trace else None,
                  max_command_joint_speed_rad_s=max(r['max_command_joint_speed_rad_s'] for r in rows),
                  max_command_joint_accel_rad_s2=max(r['max_command_joint_accel_rad_s2'] for r in rows),
                  unexpected_contact_samples=sum(r['unexpected_contact_count'] > 0 for r in rows),
                  max_jaw_translation_drift_mm=max(r['jaw_translation_drift_mm'] for r in rows),
                  max_sensor_solver_difference_N=max(max(r['sensor_solver_force_difference_N'], r['applied_sensor_solver_force_difference_N']) for r in rows),
                  trace_force_mean_N=float(force.mean()) if force.size else None,
                  trace_force_rmse_N=float(np.sqrt(np.mean((force-fc.target_force_N)**2))) if force.size else None,
                  trace_fraction_within_tolerance=float(np.mean((abs(force-fc.target_force_N) <= cfg.force_tolerance_N)
                      & (abs(before_force-fc.target_force_N) <= cfg.force_tolerance_N))) if force.size else 0.,
                  trace_contact_loss_s=float(np.sum((force < fc.contact_threshold_N) | (before_force < fc.contact_threshold_N))*dt),
                  segments=[{'tooth': s['tooth'], 'stroke_id': s['stroke_id'],
                             'retained_length_mm': float(table[1][-1]),
                             'scheduled_length_mm': float(progress[i]),
                             'contact_qualified_length_mm': float(qualified[i])}
                            for i, (s, table) in enumerate(zip(path.segments, tables))])
    expected_length = planned_length if stage == 'regional' else (0. if stage == 'hold' else min(5., planned_length))
    report['stage_expected_length_mm'] = expected_length
    stage_coverage = 1. if stage == 'hold' else float(progress.sum()/max(expected_length, 1e-12))
    qualified_coverage = 1. if stage == 'hold' else float(qualified.sum()/max(expected_length, 1e-12))
    report['stage_progress_fraction'] = stage_coverage
    report['force_control_passed'] = bool(completed and stage_coverage >= .999
        and qualified_coverage >= .999 and report['trace_contact_loss_s'] == 0.
        and report['trace_fraction_within_tolerance'] >= .95 and all_force.max() < fc.force_stop_N
        and report['peak_contact_normal_sum_N'] < fc.force_stop_N
        and report['trace_force_rmse_N'] is not None and report['trace_force_rmse_N'] <= .1
        and report['max_tangent_error_mm'] <= .3 and report['max_orientation_error_deg'] <= 1.
        and report['unexpected_contact_samples'] == 0 and report['max_sensor_solver_difference_N'] < 1e-6)
    report.update(execution_completed=bool(completed),
                  force_limit_passed=bool(all_force.max() < fc.force_stop_N
                                         and report['peak_contact_normal_sum_N'] < fc.force_stop_N),
                  continuous_contact_passed=bool(completed and force.size and report['trace_contact_loss_s'] == 0.),
                  one_minute_completion_passed=bool(completed and rows[-1]['time_s'] <= 60.),
                  trace_force_min_N=float(min(force.min(), before_force.min())) if force.size else None,
                  trace_force_max_N=float(max(force.max(), before_force.max())) if force.size else None)
    report['force_tracking_passed'] = report['force_control_passed']
    write_csv(output/'trajectory_force.csv', rows)
    if hit_rows: write_csv(output/'trajectory_contacts.csv', hit_rows)
    dump(output/'force_trajectory_report.json', report)
    from .validation import audit
    independent = audit(output, checker=verification_checker)
    report.update(signal_audit_passed=bool(independent['signals_match_report'] and independent['source_fingerprints_match']),
                  scanned_surface_contact_passed=independent['scanned_surface_contact_passed'],
                  contact_trajectory_passed=independent['contact_trajectory_passed'],
                  force_control_passed=independent['contact_trajectory_passed'],
                  contact_validation_file=fingerprint(output/'contact_validation.json'))
    report['stage_passed'] = bool(report['force_control_passed'] and model_info is not None)
    report['contact_model_gate_passed'] = model_info is not None
    report['regional_force_control_passed'] = bool(stage == 'regional' and report['stage_passed'])
    report['force_control_passed'] = report['regional_force_control_passed']
    dump(output/'force_trajectory_report.json', report)
    if live_panel is not None:
        live_panel.finish(data, rows[-1], report, output)
    plot(output, rows, cfg)
    return report
