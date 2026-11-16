"""Independent saved-signal and scanned-triangle contact audit, not feedback."""
import csv
import json
from pathlib import Path

import fcl
import numpy as np

from planning.infrastructure.io import dump, fingerprint
from .contact_experiment import write_csv
from planning.motion_planning.execution import load_verified


def signal_metrics(rows, target_N, tolerance_N, contact_threshold_N, force_component='resultant'):
    if not rows:
        raise ValueError('Force validation requires recorded samples')
    time = np.array([float(r['time_s']) for r in rows])
    forces = np.array([[float(r['compensated_force_N']), float(r['applied_command_force_N'])] for r in rows])
    if not np.isfinite(np.r_[time, forces.ravel()]).all() or np.any(forces < 0):
        raise ValueError('Nonfinite or negative measured force')
    if len(time) > 1 and (np.any(np.diff(time) <= 0) or not np.allclose(np.diff(time), .001, atol=1e-9, rtol=0)):
        raise ValueError('Force log must retain every 1 ms executed sample in order')
    trace = np.array([r['phase'] == 'trace' for r in rows])
    feedback = forces
    if force_component == 'normal':
        normals = np.array([[float(r['normal_world_'+a]) for a in 'xyz'] for r in rows])
        if not np.allclose(np.linalg.norm(normals,axis=1),1.,atol=1e-6):
            raise ValueError('Force log contains non-unit surface normals')
        before = np.array([[float(r['external_force_world_'+a+'_N']) for a in 'xyz'] for r in rows])
        after = np.array([[float(r['applied_external_force_world_'+a+'_N']) for a in 'xyz'] for r in rows])
        if not np.allclose(np.c_[np.linalg.norm(before,axis=1),np.linalg.norm(after,axis=1)],
                           forces,atol=1e-9,rtol=0):
            raise ValueError('Recorded resultant forces disagree with the compensated vectors')
        signed = np.c_[(before*normals).sum(1),(after*normals).sum(1)]
        if not np.isfinite(signed).all() or np.any(signed < -.05):
            raise ValueError('Invalid or reversed signed normal force in recorded samples')
        feedback = np.maximum(signed,0.)
    values = feedback[trace]
    return {'samples': len(rows), 'trace_samples': int(trace.sum()),
            'recorded_duration_s': float(time[-1]), 'peak_force_N': float(forces.max()),
            'trace_mean_N': float(values[:, 1].mean()) if len(values) else None,
            'trace_rmse_N': float(np.sqrt(np.mean((values[:, 1]-target_N)**2))) if len(values) else None,
            'trace_min_N': float(values.min()) if len(values) else None,
            'trace_max_N': float(values.max()) if len(values) else None,
            'trace_contact_loss_s': float(np.any(values < contact_threshold_N, axis=1).sum()*.001),
            'trace_fraction_within_tolerance': float(np.all(abs(values-target_N) <= tolerance_N, axis=1).mean())
            if len(values) else 0.}


LINE_BAND_MM = 1.05
MIN_TRACE_ARC_MM = 1.5


def line_band_values(world_line, contacts, tips):
    """Apply the contact band and actual motion requirement to recorded positions."""
    from scipy.spatial import cKDTree
    distances = cKDTree(world_line).query(contacts)[0]*1000
    arc = float(np.linalg.norm(np.diff(tips,axis=0),axis=1).sum()*1000)
    return dict(contact_samples=len(contacts), max_contact_to_gingival_line_mm=float(distances.max()),
                line_band_mm=LINE_BAND_MM, min_trace_arc_mm=MIN_TRACE_ARC_MM,
                contact_samples_outside_1mm=int(np.sum(distances > LINE_BAND_MM)),
                actual_trace_tcp_arc_mm=arc,
                actual_trace_tcp_endpoint_displacement_mm=float(np.linalg.norm(tips[-1]-tips[0])*1000),
                passed=bool(np.all(distances <= LINE_BAND_MM) and arc >= MIN_TRACE_ARC_MM))


def line_band_metrics(output, source, rows):
    """Measure recorded tooth contacts against the original extracted gingival line."""
    from planning.motion_planning.contract import load_sweep_reference

    saved = json.loads((source/'planning_report.json').read_text(encoding='utf-8'))
    reference = load_sweep_reference(source, saved)
    parameters = np.linspace(0., reference.parts[0]['end'], 10001)
    scan_line = np.array([reference.evaluate(float(s)) for s in parameters])
    transform = np.asarray(saved['world_from_scan_m'], float)
    world_line = scan_line@transform[:3, :3].T+transform[:3, 3]
    trace = [row for row in rows if row['phase'] == 'trace']
    if not trace:
        return dict(passed=False, reason='No trace samples were recorded')
    first, last = float(trace[0]['time_s']), float(trace[-1]['time_s'])
    with (output/'trajectory_contacts.csv').open(encoding='utf-8-sig', newline='') as stream:
        hits = [hit for hit in csv.DictReader(stream)
                if first <= float(hit['time_s']) <= last and hit['allowed'] == 'True'
                and float(hit['normal_force_N']) > 0.]
    if not hits:
        raise ValueError('Line audit requires recorded target tooth contacts')
    contacts = np.array([[float(hit[k]) for k in ('x_m','y_m','z_m')] for hit in hits])
    tips = np.array([[float(row[k]) for k in ('actual_tcp_x_m','actual_tcp_y_m','actual_tcp_z_m')]
                     for row in trace])
    if not np.isfinite(contacts).all() or not np.isfinite(tips).all():
        raise ValueError('Line audit contains nonfinite contact or TCP positions')
    return dict(original_gingival_line_file=fingerprint(source/'sweep_reference.json'),
                contacts_file=fingerprint(output/'trajectory_contacts.csv'),
                sample_spacing_mm=float(np.linalg.norm(np.diff(world_line,axis=0),axis=1).max()*1000),
                **line_band_values(world_line, contacts, tips))


def audit(output, *, checker=None, scan_tolerance_mm=.1):
    """Replay actual joints against both original triangle meshes, without hulls."""
    output = Path(output)
    report = json.loads((output/'force_trajectory_report.json').read_text(encoding='utf-8'))
    source = Path(report['source'])
    if checker is None:
        _, _, _, checker = load_verified(source)
    robot, model = checker.robot, checker.model
    with (output/'trajectory_force.csv').open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    fc, pc = report['force_control'], report['path_control']
    signals = signal_metrics(rows, fc['target_force_N'], pc['force_tolerance_N'], fc['contact_threshold_N'],
                             'normal' if report.get('regulated_quantity') == 'normal' else 'resultant')
    signal_match = all(np.isclose(signals[a], report[b], atol=1e-9, rtol=0)
        if signals[a] is not None else report[b] is None
        for a, b in [('peak_force_N', 'peak_compensated_force_N'),
                     ('trace_rmse_N', 'trace_force_rmse_N'),
                     ('trace_contact_loss_s', 'trace_contact_loss_s'),
                     ('trace_fraction_within_tolerance', 'trace_fraction_within_tolerance')])
    inputs_match = (fingerprint(source/'trajectory.npz') == report['source_trajectory']
                    and fingerprint(source/'planning_report.json') == report['source_planning_report'])
    geom, _, tool = next(p for p in checker.parts if model.geom(p[0]).name == 'ur_end_scaler_geom')
    selected_jaw = dict(checker.teeth)[report['jaw']]
    require_tooth_label=report.get('contact_model_manifest') is not None
    tooth_labels=(np.load(source/'tooth_face_ids.npy',allow_pickle=False) if require_tooth_label else None)
    target_teeth=set(report.get('selected_teeth',[]))

    def replay(q, force, time, phase, predicted=False):
        pose = robot.fk(q)
        tool.setTransform(fcl.Transform(robot.data.geom_xmat[geom].reshape(3, 3), robot.data.geom_xpos[geom]))
        result = fcl.DistanceResult()
        gap = float(fcl.distance(tool, selected_jaw, fcl.DistanceRequest(enable_nearest_points=True), result))*1000
        near_tip = None
        penetration = 0.
        face_ids=[]
        if gap > 0:
            near_tip = float(np.linalg.norm(np.asarray(result.nearest_points[0])-pose[:3, 3])*1000)
            face_ids=[int(result.b2)]
        else:
            collision = fcl.CollisionResult()
            fcl.collide(tool, selected_jaw, fcl.CollisionRequest(num_max_contacts=100, enable_contact=True), collision)
            if collision.contacts:
                near_tip = float(max(np.linalg.norm(c.pos-pose[:3, 3])*1000 for c in collision.contacts))
                penetration = float(max(c.penetration_depth*1000 for c in collision.contacts))
                face_ids=[int(c.b2) for c in collision.contacts]
            else:
                penetration = float('inf')
            # FCL's negative mesh-distance sentinel is not a penetration depth.
            gap = 0.
        label_ok=(not require_tooth_label or bool(face_ids and all(
            0<=i<len(tooth_labels) and int(tooth_labels[i]) in target_teeth for i in face_ids)))
        return {'time_s': float(time), 'phase': phase, 'predicted_not_executed': predicted,
                'force_N': float(force), 'scan_mesh_tool_gap_mm': gap,
                'max_reported_penetration_mm': penetration,
                'contact_or_nearest_point_to_tcp_mm': near_tip,
                'target_tooth_label_consistent': label_ok,
                'surface_contact_consistent': bool(np.isfinite(gap) and gap <= scan_tolerance_mm
                                                    and penetration <= scan_tolerance_mm
                                                    and near_tip is not None and near_tip <= 3. and label_ok)}

    force = np.array([max(float(r['compensated_force_N']), float(r['applied_command_force_N'])) for r in rows])
    boundaries = {i for i in range(1, len(rows)) if rows[i]['phase'] != rows[i-1]['phase']}
    trace_indices = [i for i, r in enumerate(rows) if r['phase'] == 'trace']
    extra = {int(force.argmax()), len(rows)-1}
    if trace_indices:
        extra.add(min(trace_indices, key=lambda i: force[i]))
    positive = np.flatnonzero(force >= fc['contact_threshold_N'])
    if len(positive):
        extra.update((int(positive[0]), int(positive[-1])))
    indices = sorted(set(range(0, len(rows), 10)) | boundaries | extra)
    measured = [replay(np.array([float(rows[i][n]) for n in robot.joint_names]), force[i],
                       rows[i]['time_s'], rows[i]['phase']) for i in indices]
    prediction = report.get('predicted_rejected_step')
    predicted = None
    if prediction and prediction.get('joint_positions_rad') is not None:
        predicted = replay(np.asarray(prediction['joint_positions_rad']), prediction['force_N'],
                           prediction['time_s'], 'rejected_prediction', True)
    present = [r for r in measured if r['force_N'] >= fc['contact_threshold_N'] and r['phase'] == 'trace']
    surface_passed = bool(present and all(r['surface_contact_consistent'] for r in present))
    line_band = (line_band_metrics(output, source, rows) if
                 report.get('contact_model_manifest',{}).get('asset_scope') == 'first_tooth_line'
                 and report.get('experiment_stage') == 'short' else None)
    summary = {'simulation_only': True, 'method': 'FCL original tool / original scanned jaw triangle meshes',
               'sample_period_s': .01, 'scan_contact_tolerance_mm': scan_tolerance_mm,
               'extra_samples': 'phase boundaries, force extrema, first/last force-present sample',
               'input_csv': fingerprint(output/'trajectory_force.csv'),
               'audit_code': fingerprint(Path(__file__)),
               'signals': signals, 'signals_match_report': bool(signal_match),
               'source_fingerprints_match': bool(inputs_match),
               'replay_samples': len(measured), 'force_present_trace_replay_samples': len(present),
               'force_present_trace_scan_gap_range_mm': [min(r['scan_mesh_tool_gap_mm'] for r in present),
                                                        max(r['scan_mesh_tool_gap_mm'] for r in present)] if present else None,
               'scanned_surface_contact_passed': surface_passed,
               'line_band': line_band,
               'rejected_prediction': predicted,
               'contact_trajectory_passed': bool(report['force_tracking_passed'] and surface_passed
                                                  and signal_match and inputs_match
                                                  and (line_band is None or line_band['passed'])),
               'clinical_or_hardware_validation': False}
    summary['reason'] = ('No force-controlled trace samples' if not present else
                         'Force-producing convex-hull contacts do not all match localized scanned-surface contact'
                         if not surface_passed else 'Sampled scanned-surface consistency only; original convex-hull physics retained')
    dump(output/'contact_validation.json', summary)
    write_csv(output/'scan_contact_audit.csv', measured+([predicted] if predicted else []))
    return summary


def reaccept(output):
    """Recompute acceptance from an existing run; does not re-simulate."""
    output = Path(output).resolve()
    report = json.loads((output/'force_trajectory_report.json').read_text(encoding='utf-8'))
    independent = audit(output)
    report.update(signal_audit_passed=bool(independent['signals_match_report'] and independent['source_fingerprints_match']),
                  scanned_surface_contact_passed=independent['scanned_surface_contact_passed'],
                  contact_trajectory_passed=independent['contact_trajectory_passed'],
                  force_control_passed=independent['contact_trajectory_passed'],
                  contact_validation_file=fingerprint(output/'contact_validation.json'))
    report['stage_passed'] = bool(report['force_control_passed'] and report.get('contact_model_manifest') is not None)
    report['contact_model_gate_passed'] = report.get('contact_model_manifest') is not None
    report['regional_force_control_passed'] = bool(report.get('experiment_stage') == 'regional' and report['stage_passed'])
    report['force_control_passed'] = report['regional_force_control_passed']
    dump(output/'force_trajectory_report.json', report)
    return report


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description='Re-audit a saved force run without re-simulating')
    parser.add_argument('--run', type=Path, required=True, help='Existing force output directory')
    args = parser.parse_args(argv)
    report = reaccept(args.run)
    print(json.dumps({key: report.get(key) for key in
                      ('status', 'stage_passed', 'force_tracking_passed', 'contact_trajectory_passed',
                       'scanned_surface_contact_passed', 'signal_audit_passed', 'stop_reason')},
                     ensure_ascii=False))
    return 0 if report.get('stage_passed') else 2


if __name__ == '__main__':
    raise SystemExit(main())
