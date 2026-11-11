"""Independent saved-signal and scanned-triangle contact audit, not feedback."""
import csv
import json
from pathlib import Path

import fcl
import numpy as np

from planning.infrastructure.io import dump, fingerprint
from .contact_experiment import write_csv
from planning.motion_planning.execution import load_verified


def signal_metrics(rows, target_N, tolerance_N, contact_threshold_N):
    if not rows:
        raise ValueError('Force validation requires recorded samples')
    time = np.array([float(r['time_s']) for r in rows])
    forces = np.array([[float(r['compensated_force_N']), float(r['applied_command_force_N'])] for r in rows])
    if not np.isfinite(np.r_[time, forces.ravel()]).all() or np.any(forces < 0):
        raise ValueError('Nonfinite or negative measured force')
    if len(time) > 1 and (np.any(np.diff(time) <= 0) or not np.allclose(np.diff(time), .001, atol=1e-9, rtol=0)):
        raise ValueError('Force log must retain every 1 ms executed sample in order')
    trace = np.array([r['phase'] == 'trace' for r in rows])
    values = forces[trace]
    return {'samples': len(rows), 'trace_samples': int(trace.sum()),
            'recorded_duration_s': float(time[-1]), 'peak_force_N': float(forces.max()),
            'trace_mean_N': float(values[:, 1].mean()) if len(values) else None,
            'trace_rmse_N': float(np.sqrt(np.mean((values[:, 1]-target_N)**2))) if len(values) else None,
            'trace_min_N': float(values.min()) if len(values) else None,
            'trace_max_N': float(values.max()) if len(values) else None,
            'trace_contact_loss_s': float(np.any(values < contact_threshold_N, axis=1).sum()*.001),
            'trace_fraction_within_tolerance': float(np.all(abs(values-target_N) <= tolerance_N, axis=1).mean())
            if len(values) else 0.}


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
    signals = signal_metrics(rows, fc['target_force_N'], pc['force_tolerance_N'], fc['contact_threshold_N'])
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

    def replay(q, force, time, phase, predicted=False):
        pose = robot.fk(q)
        tool.setTransform(fcl.Transform(robot.data.geom_xmat[geom].reshape(3, 3), robot.data.geom_xpos[geom]))
        result = fcl.DistanceResult()
        gap = float(fcl.distance(tool, selected_jaw, fcl.DistanceRequest(enable_nearest_points=True), result))*1000
        near_tip = None
        penetration = 0.
        if gap > 0:
            near_tip = float(np.linalg.norm(np.asarray(result.nearest_points[0])-pose[:3, 3])*1000)
        else:
            collision = fcl.CollisionResult()
            fcl.collide(tool, selected_jaw, fcl.CollisionRequest(num_max_contacts=100, enable_contact=True), collision)
            if collision.contacts:
                near_tip = float(max(np.linalg.norm(c.pos-pose[:3, 3])*1000 for c in collision.contacts))
                penetration = float(max(c.penetration_depth*1000 for c in collision.contacts))
            else:
                penetration = float('inf')
            # FCL's negative mesh-distance sentinel is not a penetration depth.
            gap = 0.
        return {'time_s': float(time), 'phase': phase, 'predicted_not_executed': predicted,
                'force_N': float(force), 'scan_mesh_tool_gap_mm': gap,
                'max_reported_penetration_mm': penetration,
                'contact_or_nearest_point_to_tcp_mm': near_tip,
                'surface_contact_consistent': bool(np.isfinite(gap) and gap <= scan_tolerance_mm
                                                    and penetration <= scan_tolerance_mm
                                                    and near_tip is not None and near_tip <= 3.)}

    force = np.array([max(float(r['compensated_force_N']), float(r['applied_command_force_N'])) for r in rows])
    boundaries = {i for i in range(1, len(rows)) if rows[i]['phase'] != rows[i-1]['phase']}
    trace_indices = [i for i, r in enumerate(rows) if r['phase'] == 'trace']
    extra = {int(force.argmax()), len(rows)-1}
    if trace_indices:
        extra.add(min(trace_indices, key=lambda i: force[i]))
    positive = np.flatnonzero(force >= fc['contact_threshold_N'])
    if len(positive):
        extra.update((int(positive[0]), int(positive[-1])))
    indices = sorted(set(range(0, len(rows), 50)) | boundaries | extra)
    measured = [replay(np.array([float(rows[i][n]) for n in robot.joint_names]), force[i],
                       rows[i]['time_s'], rows[i]['phase']) for i in indices]
    prediction = report.get('predicted_rejected_step')
    predicted = None
    if prediction and prediction.get('joint_positions_rad') is not None:
        predicted = replay(np.asarray(prediction['joint_positions_rad']), prediction['force_N'],
                           prediction['time_s'], 'rejected_prediction', True)
    present = [r for r in measured if r['force_N'] >= fc['contact_threshold_N'] and r['phase'] == 'trace']
    surface_passed = bool(present and all(r['surface_contact_consistent'] for r in present))
    summary = {'simulation_only': True, 'method': 'FCL original tool / original scanned jaw triangle meshes',
               'sample_period_s': .05, 'scan_contact_tolerance_mm': scan_tolerance_mm,
               'extra_samples': 'phase boundaries, force extrema, first/last force-present sample',
               'input_csv': fingerprint(output/'trajectory_force.csv'),
               'audit_code': fingerprint(Path(__file__)),
               'signals': signals, 'signals_match_report': bool(signal_match),
               'source_fingerprints_match': bool(inputs_match),
               'replay_samples': len(measured), 'force_present_trace_replay_samples': len(present),
               'force_present_trace_scan_gap_range_mm': [min(r['scan_mesh_tool_gap_mm'] for r in present),
                                                        max(r['scan_mesh_tool_gap_mm'] for r in present)] if present else None,
               'scanned_surface_contact_passed': surface_passed,
               'rejected_prediction': predicted,
               'contact_trajectory_passed': bool(report['force_tracking_passed'] and surface_passed
                                                  and signal_match and inputs_match),
               'clinical_or_hardware_validation': False}
    summary['reason'] = ('No force-controlled trace samples' if not present else
                         'Force-producing convex-hull contacts do not all match localized scanned-surface contact'
                         if not surface_passed else 'Sampled scanned-surface consistency only; original convex-hull physics retained')
    dump(output/'contact_validation.json', summary)
    write_csv(output/'scan_contact_audit.csv', measured+([predicted] if predicted else []))
    return summary
