"""Build the front work-band contact layer without closing a tooth mesh."""
import json
from pathlib import Path

import mujoco
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from planning.infrastructure.io import dump, fingerprint, ROOT
from . import local_surface as surface


def build(source, output):
    from planning.motion_planning.execution import load_verified
    from planning.motion_planning.contract import load_sweep_reference
    from pipeline.simulation import DENTAL_MODEL_PATH
    from .contact_experiment import write_csv

    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists() and any(output.iterdir()): raise ValueError('Use a new empty contact asset directory')
    load_verified(source)
    saved = json.loads((source/'planning_report.json').read_text())
    request = json.loads((source/'request.json').read_text())
    if saved['region'] != 'outer_front': raise ValueError('This contact asset recipe is for outer_front')
    reference = load_sweep_reference(source, saved)
    stations = reference.samples(.00025)
    work_points = np.array([reference.evaluate(s) for s in stations])*1000
    work_normals = np.array([reference.normal(s) for s in stations])
    scan_file = Path(request['inputs']['scan'])
    tool_file = ROOT/'assets/simulation/robot/assets/end_scaler.stl'
    label_file = source/'tooth_face_ids.npy'
    output.mkdir(parents=True, exist_ok=True)
    info = dict(schema_version=1, method='local_surface', status='building', units='mm',
        simulation_only=True, jaw=request['inputs']['jaw'], region=saved['region'], selected_teeth=saved['selected_teeth'],
        source_trajectory_sha256=fingerprint(source/'trajectory.npz')['sha256'],
        source_plan_sha256=fingerprint(source/'planning_report.json')['sha256'],
        scene_sha256=fingerprint(DENTAL_MODEL_PATH)['sha256'],
        source_inputs=[dict(file=str(p.relative_to(ROOT)), sha256=fingerprint(p)['sha256'])
                       for p in (scan_file, tool_file, label_file)],
        material_status='uncalibrated_simulation_parameters', tool_contact_radius_mm=3.,
        recipe=dict(nominal_standoff_mm=float(saved['config']['standoff_mm']), normal_correction_mm=3.,
            tool_radius_mm=3., boundary_margin_mm=1., thickness_mm=.5, initial_patch_scale_mm=1.,
            minimum_patch_scale_mm=.25, patch_tolerance_mm=.05, final_tolerance_mm=.1),
        coverage='Local front work band only; original complete meshes remain mandatory runtime guards.',
        qa_scope='Sampled bidirectional working-surface distances and six approach rays; not a global Hausdorff certificate.',
        parts=[], checks=[])
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
        selected = np.isin(labels, saved['selected_teeth'])
        ids = np.flatnonzero((dist-triangle_radius <= radius) & (labels > 0))
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
        tc = tv[tf].mean(1)
        tr = np.linalg.norm(tv[tf]-tc[:, None], axis=2).max(1)
        tool_ids = np.flatnonzero(np.linalg.norm(tc-tip, axis=1)-tr <= 4.)
        tool_qa = np.flatnonzero(np.linalg.norm(tc-tip, axis=1)+tr <= 3.)
        evidence = output/'source_geometry.npz'
        np.savez_compressed(evidence, vertices=vertices, faces=faces, labels=labels, band_ids=ids,
            qa_ids=qa_ids, work_points=work_points, work_normals=work_normals, qa_normals=work_normals[nearest[qa_ids]],
            tool_vertices=tv, tool_faces=tf, tool_ids=tool_ids, tool_qa_ids=tool_qa, tip=tip)
        info['source_geometry_sha256'] = fingerprint(evidence)['sha256']
        info['band_radius_mm'] = radius
        info['geometry_code'] = [fingerprint(Path(__file__)),fingerprint(Path(surface.__file__))]
        all_records, sets = [], []
        for role, v, f, lab, band, qa, body, approach in (
            ('tooth', vertices, faces, labels, ids, qa_ids, 'dental_'+info['jaw'], work_normals[nearest[qa_ids]]),
            ('tool', tv, tf, np.zeros(len(tf), dtype=int), tool_ids, tool_qa, 'ur_end_scaler_link', None)):
            print(f'Building local {role}: {len(band)} original triangles', flush=True)
            patches = surface.generate(v, f, lab, band)
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
                    body_from_source_mm=np.eye(4).tolist(), thickness_mm=.5, scale_mm=patch['scale_mm'],
                    forward_error_mm=patch['forward_error_mm'], reverse_error_mm=patch['reverse_error_mm'],
                    maximum_error_mm=patch['maximum_error_mm'], samples=patch['samples']))
            check, records = surface.union_check(v, f, patches, qa, approach)
            if role=='tooth':
                gap, gap_records=surface.gap_check(v,f,lab,patches,band)
                info['gap_check']=gap
                write_csv(output/'gap_errors.csv',gap_records)
            for record in records: record['role'] = role
            all_records.extend(records)
            if role == 'tool':
                info['checks'].append(dict(label='tool', **check))
            else:
                for tooth in saved['selected_teeth']:
                    tooth_records = [r for r in records if int(labels[r['source_face']]) == tooth]
                    valid = bool(tooth_records) and all(r['working_first_hit'] and r['error_mm'] is not None and r['error_mm'] <= .1 for r in tooth_records)
                    errors = [r['error_mm'] for r in tooth_records if r['error_mm'] is not None]
                    info['checks'].append(dict(label=f'tooth_{tooth:02}', samples=len(tooth_records),
                        geometry_passed=valid, maximum_error_mm=max(errors) if errors else None,
                        internal_or_missing_first_hits=sum(not r['working_first_hit'] for r in tooth_records)))
            dump(output/'manifest.json', info)
        write_csv(output/'first_contact_errors.csv', all_records)
        info['first_contact_errors_sha256'] = fingerprint(output/'first_contact_errors.csv')['sha256']
        _plots(output, sets, work_points, all_records)
        failed = [c['label'] for c in info['checks'] if not c['geometry_passed']]
        if not info['gap_check']['geometry_passed']: failed.append('open_or_label_boundaries')
        failures=[r for r in all_records if not r['working_first_hit'] or r['error_mm'] is None or r['error_mm']>.1]
        dump(output/'geometry_failures.json',dict(status='failed' if failed else 'passed',
            failed_checks=failed, failed_first_hit_samples=len(failures),first_examples=failures[:100],
            gap_check=info['gap_check'],
            dynamic_stages='Not permitted until all static geometry gates pass' if failed else 'Pending staged experiments'))
        info.update(status='failed_geometry_gate' if failed else 'verified', failed_parts=failed,
                    dynamic_validation='Pending staged hold and short experiments; static verification is not force acceptance.')
        dump(output/'manifest.json', info)
        return info
    except Exception as error:
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
        info.update(status='failed', error=str(error))
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
    bad = [i for i, r in enumerate(records) if not r['working_first_hit']]
    ax.scatter(bad, [.1]*len(bad), marker='x', color='orange', label='internal/missing first hit')
    ax.set(xlabel='sample / approach', ylabel='first-contact error (mm)'); ax.legend()
    fig.tight_layout(); fig.savefig(output/'first_contact_errors.png', dpi=160); plt.close(fig)
