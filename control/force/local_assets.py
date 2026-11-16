"""Build the front work-band contact layer without closing a tooth mesh."""
import json
from pathlib import Path

import mujoco
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from planning.infrastructure.io import dump, fingerprint, ROOT
from . import local_surface as surface


def build(source, output, *, scope='regional', hold_face_id=None, line_candidate=None):
    from planning.motion_planning.execution import load_verified
    from planning.motion_planning.contract import load_sweep_reference
    from pipeline.simulation import DENTAL_MODEL_PATH
    from .contact_experiment import write_csv

    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists() and any(output.iterdir()): raise ValueError('Use a new empty contact asset directory')
    base_model,base_data,plan,checker=load_verified(source)
    saved = json.loads((source/'planning_report.json').read_text())
    request = json.loads((source/'request.json').read_text())
    if saved['region'] != 'outer_front': raise ValueError('This contact asset recipe is for outer_front')
    reference = load_sweep_reference(source, saved)
    if scope not in ('regional','first_tooth_hold','first_tooth_line'): raise ValueError('Unknown local asset scope')
    if hold_face_id is not None and scope!='first_tooth_hold':
        raise ValueError('Alternate hold face requires first_tooth_hold scope')
    if (line_candidate is None) != (scope!='first_tooth_line'):
        raise ValueError('first_tooth_line requires exactly one --line-candidate')
    first_tooth = int(reference.tooth(0.))
    if scope in ('first_tooth_hold','first_tooth_line') and first_tooth != 12:
        raise ValueError('Expected first contact on tooth 12')
    stations = [0.] if scope in ('first_tooth_hold','first_tooth_line') else reference.samples(.00025)
    work_points = np.array([reference.evaluate(s) for s in stations])*1000
    work_normals = np.array([reference.normal(s) for s in stations])
    scan_file = Path(request['inputs']['scan'])
    tool_file = ROOT/'assets/simulation/robot/assets/end_scaler.stl'
    label_file = source/'tooth_face_ids.npy'
    if hold_face_id is not None:
        original=o3d.io.read_triangle_mesh(str(scan_file))
        ov,of=np.asarray(original.vertices),np.asarray(original.triangles)
        original_labels=np.load(label_file,allow_pickle=False)
        if not 0<=hold_face_id<len(of) or int(original_labels[hold_face_id])!=first_tooth:
            raise ValueError('Alternate hold face must belong to the first planned tooth')
        work_points=ov[of[hold_face_id]].mean(0)[None]
        work_normals=surface.outward_normals(ov,of,[hold_face_id],original_labels)
    candidate = None
    if line_candidate is not None:
        from .line_envelope import verify_candidate
        line_candidate = Path(line_candidate).resolve()
        candidate = json.loads(line_candidate.read_text(encoding='utf-8'))
        original=o3d.io.read_triangle_mesh(str(scan_file))
        ov,of=np.asarray(original.vertices),np.asarray(original.triangles)
        original_labels=np.load(label_file,allow_pickle=False)
        line_ids,work_points,work_normals,line_q,line_distances=verify_candidate(
            candidate,source,scan_file,ov,of,original_labels,reference)
    output.mkdir(parents=True, exist_ok=True)
    info = dict(schema_version=1, method='local_surface', status='building', units='mm',
        simulation_only=True, jaw=request['inputs']['jaw'], region=saved['region'],
        asset_scope=scope, hold_face_id=hold_face_id,
        selected_teeth=[first_tooth] if scope in ('first_tooth_hold','first_tooth_line') else saved['selected_teeth'],
        source_trajectory_sha256=fingerprint(source/'trajectory.npz')['sha256'],
        source_plan_sha256=fingerprint(source/'planning_report.json')['sha256'],
        scene_sha256=fingerprint(DENTAL_MODEL_PATH)['sha256'],
        source_inputs=[dict(file=str(p.relative_to(ROOT)), sha256=fingerprint(p)['sha256'])
                       for p in (scan_file, tool_file, label_file)],
        material_status='uncalibrated_simulation_parameters', tool_contact_radius_mm=3.,
        recipe=dict(nominal_standoff_mm=float(saved['config']['standoff_mm']), normal_correction_mm=3.,
            tool_radius_mm=3., boundary_margin_mm=1., thickness_mm=.05, initial_patch_scale_mm=1.,
            minimum_patch_scale_mm=.25, patch_tolerance_mm=.05, final_tolerance_mm=.1),
        coverage='Local front work band only; original complete meshes remain mandatory runtime guards.',
        qa_scope='Sampled bidirectional working-surface distances and six approach rays; not a global Hausdorff certificate.',
        contact_policy=dict(surface.CONTACT_POLICY), parts=[], checks=[])
    if candidate is not None:
        info['line_candidate']=dict(file=str(line_candidate),sha256=fingerprint(line_candidate)['sha256'],
            source_face_ids=line_ids.tolist(),length_mm=float(candidate['length_mm']),
            maximum_gumline_distance_mm=float(line_distances.max()),roll_deg=30.)
    dump(output/'manifest.json', info)
    try:
        scan = o3d.io.read_triangle_mesh(str(scan_file))
        vertices, faces = np.asarray(scan.vertices).copy(), np.asarray(scan.triangles).copy()
        labels = np.load(label_file, allow_pickle=False)
        if labels.shape != (len(faces),): raise ValueError('Tooth labels do not match original scan triangles')
        radius = sum(info['recipe'][k] for k in ('nominal_standoff_mm','normal_correction_mm','tool_radius_mm','boundary_margin_mm'))
        # Subtract each triangle's radius so that a triangle crossing the band
        # boundary is included even if its centroid is outside the band.
        centers = vertices[faces].mean(1)
        triangle_radius = np.linalg.norm(vertices[faces]-centers[:, None], axis=2).max(1)
        dist, nearest = cKDTree(work_points).query(centers)
        selected = np.isin(labels, info['selected_teeth'])
        ids = np.flatnonzero((dist-triangle_radius <= radius) &
            (selected if scope in ('first_tooth_hold','first_tooth_line') else labels > 0))
        qa_ids = np.flatnonzero((dist <= radius-1.-triangle_radius) & selected)
        if not len(qa_ids): raise ValueError('No original front-region working triangles found')
        spec = mujoco.MjSpec.from_file(str(DENTAL_MODEL_PATH)); spec.compile()
        geom = spec.geom('ur_end_scaler_geom')
        rotation = np.empty(9); mujoco.mju_quat2Mat(rotation, geom.quat)
        tool = o3d.io.read_triangle_mesh(str(tool_file))
        tool.remove_duplicated_vertices()
        tv = np.asarray(tool.vertices)@rotation.reshape(3, 3).T+np.asarray(geom.pos)*1000
        tf = np.asarray(tool.triangles).copy()
        tip = np.asarray(spec.site('ur_scaler_tcp').pos)*1000
        if scope in ('first_tooth_hold','first_tooth_line'):
            raw_tf = tf.copy()
            tf, removed, deviation = surface.filtered_tool_faces(tv, raw_tf, tip)
            tool_original_face_ids=np.flatnonzero(~np.isin(np.arange(len(raw_tf)),removed))
            info['tool_filter'] = dict(removed_original_face_ids=removed.tolist(),
                                       maximum_source_deviation_mm=deviation)
        tc = tv[tf].mean(1)
        tr = np.linalg.norm(tv[tf]-tc[:, None], axis=2).max(1)
        tool_ids = np.flatnonzero(np.linalg.norm(tc-tip, axis=1)-tr <= 4.)
        tool_qa = np.flatnonzero(np.linalg.norm(tc-tip, axis=1)+tr <= 3.)
        evidence = output/'source_geometry.npz'
        np.savez_compressed(evidence, vertices=vertices, faces=faces, labels=labels, band_ids=ids,
            qa_ids=qa_ids, work_points=work_points, work_normals=work_normals, qa_normals=work_normals[nearest[qa_ids]],
            tool_vertices=tv, tool_faces=tf, tool_ids=tool_ids, tool_qa_ids=tool_qa, tip=tip,
            tool_original_faces=raw_tf if scope in ('first_tooth_hold','first_tooth_line') else tf,
            tool_original_face_ids=tool_original_face_ids if scope in ('first_tooth_hold','first_tooth_line') else np.arange(len(tf)))
        info['source_geometry_sha256'] = fingerprint(evidence)['sha256']
        info['band_radius_mm'] = radius
        envelope=None
        if scope=='first_tooth_hold':
            from .hold_envelope import from_plan
            envelope=from_plan(base_model,base_data,plan,checker,saved,tv,tf,tool_ids,work_normals[0],
                hold_face_id=hold_face_id,scan_vertices=vertices,scan_faces=faces,
                first_normal_scan=reference.normal(0.))
            info['hold_envelope']=dict(tip_mm=envelope['tip_mm'].tolist(),
                normal_scan=envelope['normal_scan'].tolist(),
                tool_rotation_scan=envelope['tool_rotation_scan'].tolist(),
                normal_travel_mm=3., allowance_mm=1.5, max_lateral_mm=1., max_rotation_deg=5.,
                target_q_rad=envelope['target_q_rad'].tolist(),
                first_clearance_q_rad=envelope['first_clearance_q_rad'].tolist(),
                target_clearance_q_rad=envelope['target_clearance_q_rad'].tolist())
        if scope=='first_tooth_line':
            from .line_envelope import from_candidate, prepare_motion
            first_clear,line_clear=prepare_motion(base_model,base_data,plan,checker,
                saved,reference,work_points,work_normals,line_q,
                np.asarray(candidate['q_contact_rad']),labels)
            envelope=from_candidate(base_model,base_data,checker.robot,
                base_model.body('dental_'+info['jaw']).id,base_model.body('ur_end_scaler_link').id,
                tv,tf,tool_ids,work_points,work_normals,line_q,
                info['recipe']['nominal_standoff_mm'])
            base_data.qpos[checker.robot.qadr]=line_q[0]
            mujoco.mj_forward(base_model,base_data)
            jaw_id=base_model.body('dental_'+info['jaw']).id
            tool_id=base_model.body('ur_end_scaler_link').id
            jaw_rotation=base_data.xmat[jaw_id].reshape(3,3)
            first_tip=(base_data.site_xpos[checker.robot.site]-base_data.xpos[jaw_id])@jaw_rotation*1000
            first_tool_rotation=jaw_rotation.T@base_data.xmat[tool_id].reshape(3,3)
            info['hold_envelope']=dict(tip_mm=first_tip.tolist(),normal_scan=work_normals[0].tolist(),
                tool_rotation_scan=first_tool_rotation.tolist(),normal_travel_mm=3.,allowance_mm=1.5,
                max_lateral_mm=1.,max_rotation_deg=5.,target_q_rad=line_q[0].tolist(),
                first_clearance_q_rad=first_clear.tolist(),target_clearance_q_rad=line_clear[0].tolist())
            arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(work_points,axis=0),axis=1))]
            info['line_trace']=dict(face_ids=line_ids.tolist(),point_scan_mm=work_points.tolist(),
                normal_scan=work_normals.tolist(),q_rad=line_q.tolist(),
                clearance_q_rad=line_clear.tolist(),arc_mm=arc.tolist(),length_mm=float(arc[-1]),
                roll_deg=30.,max_gumline_distance_mm=float(line_distances.max()))
        info['geometry_code'] = [fingerprint(Path(__file__)),fingerprint(Path(surface.__file__))]
        # The extrusion depth is bounded by the 0.1 mm first-contact gate: a
        # seam groove is exactly as deep as the extrusion, so a deep
        # extrusion cannot pass the gate however accurate the cap is.
        thickness = info['recipe']['thickness_mm']
        all_records, sets = [], []
        # Fail on an unusable real tip before spending time exporting thousands
        # of tooth patches. This uses the same recipe as the load-time rebuild.
        prepared_fields = {}
        for role, v, f, lab, band in (
            ('tool', tv, tf, np.zeros(len(tf), dtype=int), tool_ids),
            ('tooth', vertices, faces, labels, ids)):
            print(f'Preflight local {role}: {len(band)} original triangles', flush=True)
            prepared_fields[role] = surface.extrusion_fields(v, f, lab, band, thickness,
                                                            flush=role!='tool')
        for role, v, f, lab, band, qa, body, approach in (
            ('tooth', vertices, faces, labels, ids, qa_ids, 'dental_'+info['jaw'], work_normals[nearest[qa_ids]]),
            ('tool', tv, tf, np.zeros(len(tf), dtype=int), tool_ids, tool_qa, 'ur_end_scaler_link', None)):
            print(f'Building local {role}: {len(band)} original triangles', flush=True)
            # The tool tip is a thin shell: a fixed extrusion would punch
            # through it, so its depth follows the real local wall.  A thin
            # shell also cannot use a per-vertex extrusion field, because its
            # outer and inner walls carry opposite normals and the field
            # stops pointing outward, so it is extruded along the patch
            # normal instead.
            patches = surface.generate(v, f, lab, band, thickness_mm=thickness,
                                       flush=role!='tool', prepared=prepared_fields[role])
            sets.append((role, v, f, patches, qa))
            for patch in patches:
                i = len(info['parts']); name = f'{role}_{i:05d}.ply'
                mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(patch['vertices']),
                                               o3d.utility.Vector3iVector(patch['faces']))
                if not o3d.io.write_triangle_mesh(str(output/name), mesh): raise OSError(name)
                info['parts'].append(dict(file=name, sha256=fingerprint(output/name)['sha256'], body=body,
                    role=role, tooth=patch['tooth'] if role=='tooth' else None,
                    source_face_ids=patch['source_face_ids'].tolist(),
                    original_vertex_ids=patch['original_vertex_ids'].tolist(),
                    face_kinds=patch['face_kinds'].tolist(), normal_body=patch['normal'].tolist(),
                    body_from_source_mm=np.eye(4).tolist(), thickness_mm=patch['thickness_mm'],
                    scale_mm=patch['scale_mm'],
                    forward_error_mm=patch['forward_error_mm'], reverse_error_mm=patch['reverse_error_mm'],
                    maximum_error_mm=patch['maximum_error_mm'], samples=patch['samples']))
            check, records = surface.union_check(v, f, patches, qa, approach, band_ids=band,
                                                source_normals=prepared_fields[role]['normals'])
            if role=='tooth':
                gap, gap_records=surface.gap_check(v,f,lab,patches,band)
                if envelope is not None:
                    if scope=='first_tooth_line':
                        from .line_envelope import reachable_failures
                    else:
                        from .hold_envelope import reachable_failures
                    reachable,distances=reachable_failures(gap_records,envelope)
                    gap['excluded_unreachable_failures']=gap['failed_samples']-len(reachable)
                    gap['failed_samples']=len(reachable)
                    gap['geometry_passed']=not reachable
                    for record in gap_records:
                        record['hold_reachable']=None;record['hold_envelope_distance_mm']=None
                    for record,d in zip((r for r in gap_records if not r['geometry_passed']),distances):
                        record['hold_reachable']=d<=.1;record['hold_envelope_distance_mm']=d
                info['gap_check']=gap
                write_csv(output/'gap_errors.csv',gap_records)
            if envelope is not None and role=='tooth':
                if scope=='first_tooth_line':
                    from .line_envelope import reachable_failures
                else:
                    from .hold_envelope import reachable_failures
                reachable,distances=reachable_failures(records,envelope)
                check['excluded_unreachable_failures']=check['failed_samples']-len(reachable)
                check['failed_samples']=len(reachable)
                check['geometry_passed']=not reachable and not check['uncovered_normal_faces']
                for record in records:
                    record['hold_reachable']=None;record['hold_envelope_distance_mm']=None
                for record,d in zip((r for r in records if r['probe_valid'] and not r['geometry_passed']),distances):
                    record['hold_reachable']=d<=.1;record['hold_envelope_distance_mm']=d
            for record in records:
                record['role'] = role
                if envelope is not None:
                    record.setdefault('hold_reachable',None)
                    record.setdefault('hold_envelope_distance_mm',None)
            all_records.extend(records)
            if role == 'tool':
                info['checks'].append(dict(label='tool', **check))
            else:
                for tooth in info['selected_teeth']:
                    tooth_records = [r for r in records if int(labels[r['source_face']]) == tooth]
                    valid_records = [r for r in tooth_records if r['probe_valid']]
                    errors = [r['error_mm'] for r in valid_records if r['error_mm'] is not None
                              and r.get('hold_reachable') is not False]
                    info['checks'].append(dict(label=f'tooth_{tooth:02}', samples=len(tooth_records),
                        valid_samples=len(valid_records),
                        geometry_passed=bool(valid_records) and all(r['geometry_passed'] or
                            r.get('hold_reachable') is False for r in valid_records)
                            and not any(int(labels[i])==tooth for i in check['uncovered_normal_faces']),
                        uncovered_normal_faces=[i for i in check['uncovered_normal_faces'] if int(labels[i])==tooth],
                        maximum_error_mm=max(errors) if errors else None,
                        invalid_probes=len(tooth_records)-len(valid_records),
                        internal_or_missing_first_hits=sum(r['hit_kind']!='working' for r in valid_records)))
            dump(output/'manifest.json', info)
        write_csv(output/'first_contact_errors.csv', all_records)
        info['first_contact_errors_sha256'] = fingerprint(output/'first_contact_errors.csv')['sha256']
        _plots(output, sets, work_points, all_records)
        failed = [c['label'] for c in info['checks'] if not c['geometry_passed']]
        if not info['gap_check']['geometry_passed']: failed.append('open_or_label_boundaries')
        failures=[r for r in all_records if r['probe_valid'] and not r['geometry_passed']
                  and r.get('hold_reachable') is not False]
        dump(output/'geometry_failures.json',dict(status='failed' if failed else 'passed',
            failed_checks=failed, failed_first_hit_samples=len(failures),first_examples=failures[:100],
            invalid_probes=sum(1 for r in all_records if not r['probe_valid']),
            gap_check=info['gap_check'],
            dynamic_stages='Not permitted until all static geometry gates pass' if failed else 'Pending staged experiments'))
        info.update(status='failed_geometry_gate' if failed else 'verified', failed_parts=failed,
                    dynamic_validation='Pending staged hold and short experiments; static verification is not force acceptance.')
        dump(output/'manifest.json', info)
        return info
    except Exception as error:
        if isinstance(error, (surface.SurfaceTopologyError, surface.SurfaceWallError)):
            bad_ids = error.source_face_ids
            failure = dict(role=role, reason=str(error), source_face_ids=bad_ids.tolist(),
                original_tool_face_ids=(tool_original_face_ids[bad_ids].tolist()
                    if role=='tool' and scope in ('first_tooth_hold','first_tooth_line') else bad_ids.tolist()),
                original_triangles_mm=v[f[bad_ids]].tolist(),
                dynamic_stages='Not permitted: source geometry has no unambiguous inward extrusion')
            dump(output/'geometry_failures.json', failure)
            np.savez_compressed(output/'failed_source_faces.npz', vertices=v, faces=f,
                                source_face_ids=bad_ids)
            info['geometry_failure_file'] = 'geometry_failures.json'
        if isinstance(error, surface.SurfaceGeometryError):
            p = error.patch
            np.savez_compressed(output/'failed_patch.npz', **p)
            dump(output/'geometry_failure.json', dict(role=role, source_face_ids=p['source_face_ids'],
                original_triangles_mm=v[f[p['source_face_ids']]], scale_mm=error.scale,
                forward_error_mm=p['forward_error_mm'], reverse_error_mm=p['reverse_error_mm'],
                normal_body=p['normal'], limit_mm=.05,
                reason='Minimum-scale convex working face bridges beyond original patch; asset rejected',
                experiment_stages=dict(hold='not_run_geometry_failed',short='not_run_previous_gate_failed',regional='not_run_previous_gate_failed')))
            _plots(output, [(role,v,f,[p],p['source_face_ids'])], np.empty((0,3)), [])
            info['geometry_failure_file'] = 'geometry_failure.json'
        info.update(status='failed_geometry_gate' if isinstance(error, (surface.SurfaceTopologyError,surface.SurfaceWallError))
                    else 'failed', error=str(error))
        dump(output/'manifest.json', info)
        raise


def _plots(output, sets, work_points, records):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig = plt.figure(figsize=(12, 6))
    for i, (role, vertices, faces, patches, _) in enumerate(sets):
        ax = fig.add_subplot(1, 2, i+1, projection='3d')
        ids = np.concatenate([p['source_face_ids'] for p in patches])
        ax.add_collection3d(Poly3DCollection(vertices[faces[ids]], facecolor='gray', alpha=.3))
        for p in patches:
            ax.add_collection3d(Poly3DCollection(p['vertices'][p['faces'][p['face_kinds']=='working']],
                facecolor=plt.cm.tab20(len(p['source_face_ids']) % 20), alpha=.5, linewidth=.05))
        xyz = vertices[faces[ids]].reshape(-1, 3)
        ax.auto_scale_xyz(xyz[:, 0], xyz[:, 1], xyz[:, 2]); ax.set_title(role+' local layer (mm)')
        if role=='tooth': ax.plot(*work_points.T, color='red')
    fig.tight_layout(); fig.savefig(output/'contact_layer_overlay.png', dpi=160); plt.close(fig)
    fig, ax = plt.subplots(figsize=(11, 4))
    errors = [r['error_mm'] if r['error_mm'] is not None else np.nan for r in records]
    ax.plot(errors, '.', markersize=1); ax.axhline(.1, color='red', label='0.1 mm gate')
    bad = [i for i, r in enumerate(records) if r['probe_valid'] and not r['geometry_passed']]
    ax.scatter(bad, [.1]*len(bad), marker='x', color='orange', label='gate failure')
    skipped = [i for i, r in enumerate(records) if not r['probe_valid']]
    ax.scatter(skipped, [np.nan]*len(skipped), marker='.', color='gray', label='invalid probe (not counted)')
    ax.set(xlabel='sample / approach', ylabel='first-contact error (mm)'); ax.legend()
    fig.tight_layout(); fig.savefig(output/'first_contact_errors.png', dpi=160); plt.close(fig)
