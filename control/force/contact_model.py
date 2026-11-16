"""Load verified experimental contact geometry and explicit solver parameters."""
from pathlib import Path
import json

import mujoco
import numpy as np
import open3d as o3d

from planning.infrastructure.io import fingerprint, ROOT
from pipeline.simulation import DENTAL_MODEL_PATH


def load_contact_model(manifest_path, source, base_model, base_data, *, solref=(.02,1.),
                       solimp=(.9,.95,.001,.5,2.), friction=(1.,.005,.0001), material_file=None):
    path=Path(manifest_path).resolve()
    info=json.loads(path.read_text())
    if info.get('schema_version')!=1 or info.get('status')!='verified':
        raise ValueError('Contact assets have not passed the scanned-surface geometry gate')
    if info.get('units')!='mm':raise ValueError('Contact collision assets must use millimetres')
    required={f'tooth_{int(t):02d}' for t in info['selected_teeth']}|{'tool'}
    verified={c['label'] for c in info.get('checks',[]) if c.get('samples',0)>0 and c.get('geometry_passed')
              and c.get('maximum_error_mm') is not None and c['maximum_error_mm']<=.1}
    if not required<=verified:
        raise ValueError('Contact asset work-band measurements are incomplete or failed')
    for key,file in [('source_trajectory_sha256','trajectory.npz'),('source_plan_sha256','planning_report.json')]:
        if info[key]!=fingerprint(Path(source)/file)['sha256']:
            raise ValueError('Contact assets belong to a different or changed source plan')
    if info.get('scene_sha256')!=fingerprint(DENTAL_MODEL_PATH)['sha256']:
        raise ValueError('Contact asset source scene changed')
    if info.get('method') == 'local_surface':
        validate_local_assets(path, info, source)
    material=dict(status='uncalibrated',solref=list(solref),solimp=list(solimp),friction=list(friction),margin_mm=0.)
    material_provenance=None
    if material_file is not None:
        material=json.loads(Path(material_file).read_text())
        if not isinstance(material,dict):raise ValueError('Contact material must be a JSON object')
        material_provenance=fingerprint(material_file)
        solref,solimp,friction=material['solref'],material['solimp'],material['friction']
    if (material.get('status') not in ('uncalibrated','calibrated') or material.get('margin_mm')!=0
            or len(solref)!=2 or len(solimp)!=5 or len(friction)!=3
            or np.any(np.asarray(solref)<=0) or np.any(np.asarray(friction)<0)
            or not 0<solimp[0]<=solimp[1]<1 or solimp[2]<=0 or not 0<solimp[3]<1 or solimp[4]<1):
        raise ValueError('Invalid explicit contact material/solver parameters')
    for values in (solref,solimp,friction):
        if not np.isfinite(values).all():raise ValueError('Invalid contact parameters')
    spec=mujoco.MjSpec.from_file(str(DENTAL_MODEL_PATH));spec.compile()
    for name in ('dental_'+info['jaw']+'_visual','ur_end_scaler_geom'):
        geom=spec.geom(name);geom.contype=geom.conaffinity=0
    for i,part in enumerate(info['parts']):
        file=path.parent/part['file']
        if file.parent.resolve()!=path.parent or fingerprint(file)['sha256']!=part['sha256']:
            raise ValueError('Contact collision asset changed or escapes its manifest directory')
        mesh=o3d.io.read_triangle_mesh(str(file))
        mesh_name=f'force_mesh_{i:04d}'
        spec.add_mesh(name=mesh_name,uservert=(np.asarray(mesh.vertices)*.001).ravel().tolist(),
                      userface=np.asarray(mesh.triangles).ravel().tolist())
        name=(f'ur_force_tool_{i:04d}' if part['role']=='tool' else
              f'force_{info["jaw"]}_tooth_{part["tooth"]:02d}_{i:04d}')
        local = info.get('method') == 'local_surface'
        spec.body(part['body']).add_geom(name=name,type=mujoco.mjtGeom.mjGEOM_MESH,meshname=mesh_name,
            contype=(2 if part['role']=='tool' else 4) if local else 1,
            conaffinity=(4 if part['role']=='tool' else 2) if local else 1,
            mass=0.,group=3,rgba=[.4,.7,.9,.0],
            solref=solref,solimp=solimp,friction=friction,margin=0.,condim=3)
    model=spec.compile()
    if model.nq!=base_model.nq or model.nbody!=base_model.nbody:
        raise ValueError('Contact model changed the robot kinematic tree')
    for field in ('body_mass','body_inertia','body_ipos','body_iquat','sensor_pos' if hasattr(model,'sensor_pos') else 'site_pos',
                  'site_quat','sensor_adr','sensor_dim','sensor_objid'):
        if not np.allclose(getattr(model,field),getattr(base_model,field),rtol=0,atol=1e-12):
            raise ValueError('Contact model changed inertia or sensor coordinates: '+field)
    model.body_pos[:]=base_model.body_pos
    model.body_quat[:]=base_model.body_quat
    data=mujoco.MjData(model)
    data.qpos[:]=base_data.qpos;data.qvel[:]=base_data.qvel;data.ctrl[:]=base_data.ctrl
    mujoco.mj_forward(model,data)
    return model,data,dict(path=str(path),sha256=fingerprint(path)['sha256'],
        method=info.get('method','coacd'), geometry_status=info['status'],material_status=material['status'],
        contact_policy=info.get('contact_policy'), asset_scope=info.get('asset_scope','regional'),
        material_settings=material,material_provenance=material_provenance,
        solref=list(solref),solimp=list(solimp),friction=list(friction),margin_mm=0.,
        coverage=info['coverage'])


def validate_local_assets(path, info, source=None):
    """Recompute surface gates, rather than trusting a saved pass flag."""
    from . import local_surface as surface
    from .local_checks import validate_evidence, validate_evidence_sources
    evidence = path.parent/'source_geometry.npz'
    if fingerprint(evidence)['sha256'] != info.get('source_geometry_sha256'):
        raise ValueError('Local contact source geometry changed')
    for source_input in info.get('source_inputs', []):
        if fingerprint(ROOT/source_input['file'])['sha256'] != source_input['sha256']:
            raise ValueError('Original scan, labels or actual tool mesh changed')
    if len(info.get('source_inputs', [])) != 3: raise ValueError('Local contact provenance is incomplete')
    if info.get('contact_policy') != surface.CONTACT_POLICY:
        raise ValueError('Local contact policy changed; rebuild contact assets')
    with np.load(evidence, allow_pickle=False) as z:
        validate_evidence_sources(info,z)
        if source is not None:
            from planning.motion_planning.contract import load_sweep_reference
            saved=json.loads((Path(source)/'planning_report.json').read_text())
            reference=load_sweep_reference(source,saved)
            scope=info.get('asset_scope','regional')
            if scope in ('first_tooth_hold','first_tooth_line') and (info.get('selected_teeth') != [12] or reference.tooth(0.) != 12):
                raise ValueError('Single-tooth asset must belong to the first planned tooth 12')
            if scope not in ('regional','first_tooth_hold','first_tooth_line'):
                raise ValueError('Unknown local contact asset scope')
            face_id=info.get('hold_face_id')
            stations=[0.] if scope in ('first_tooth_hold','first_tooth_line') else reference.samples(.00025)
            points=np.array([reference.evaluate(s) for s in stations])*1000
            normals=np.array([reference.normal(s) for s in stations])
            candidate=None
            if scope=='first_tooth_line':
                from .line_envelope import verify_candidate
                saved_candidate=info.get('line_candidate',{})
                candidate_file=Path(saved_candidate['file']).resolve()
                if fingerprint(candidate_file)['sha256']!=saved_candidate['sha256']:
                    raise ValueError('Line candidate changed since contact asset build')
                candidate=json.loads(candidate_file.read_text(encoding='utf-8'))
                ids,points,normals,line_q,line_dist=verify_candidate(candidate,Path(source),
                    ROOT/info['source_inputs'][0]['file'],z['vertices'],z['faces'],z['labels'],reference)
                if (saved_candidate['source_face_ids']!=ids.tolist()
                        or not np.isclose(saved_candidate['length_mm'],candidate['length_mm'],atol=1e-6)
                        or not np.isclose(saved_candidate['maximum_gumline_distance_mm'],line_dist.max(),atol=1e-6)):
                    raise ValueError('Line candidate provenance disagrees with rebuilt work band')
            if face_id is not None:
                if scope!='first_tooth_hold' or not 0<=face_id<len(z['faces']) or int(z['labels'][face_id])!=12:
                    raise ValueError('Alternate hold face is outside planned first tooth')
                points=z['vertices'][z['faces'][face_id]].mean(0)[None]
                normals=surface.outward_normals(z['vertices'],z['faces'],[face_id],z['labels'])
            if (not np.allclose(points,z['work_points'],rtol=0,atol=1e-9)
                    or not np.allclose(normals,z['work_normals'],rtol=0,atol=1e-9)
                    or info['recipe']['nominal_standoff_mm']!=saved['config']['standoff_mm']):
                raise ValueError('Local contact work band differs from original saved curve')
        validate_evidence(info, z)
        envelope=None
        if info.get('asset_scope')=='first_tooth_hold':
            from .hold_envelope import from_plan
            from planning.motion_planning.execution import load_verified
            model,data,plan,checker=load_verified(source)
            envelope=from_plan(model,data,plan,checker,saved,z['tool_vertices'],
                z['tool_faces'],z['tool_ids'],z['work_normals'][0],
                hold_face_id=info.get('hold_face_id'),scan_vertices=z['vertices'],scan_faces=z['faces'],
                first_normal_scan=reference.normal(0.))
            stored=info.get('hold_envelope',{})
            if (stored.get('normal_travel_mm')!=3. or stored.get('allowance_mm')!=1.5
                    or stored.get('max_lateral_mm')!=1. or stored.get('max_rotation_deg')!=5.
                    or any(not np.allclose(stored.get(k),envelope[v],rtol=0,atol=1e-6)
                        for k,v in [('tip_mm','tip_mm'),('normal_scan','normal_scan'),
                                    ('tool_rotation_scan','tool_rotation_scan'),
                                    ('target_q_rad','target_q_rad'),
                                    ('first_clearance_q_rad','first_clearance_q_rad'),
                                    ('target_clearance_q_rad','target_clearance_q_rad')])):
                raise ValueError('Single-tooth hold reachability differs from saved plan')
        if info.get('asset_scope')=='first_tooth_line':
            from .line_envelope import prepare_motion, from_candidate
            from planning.motion_planning.execution import load_verified
            model,data,plan,checker=load_verified(source)
            first_clear,line_clear=prepare_motion(model,data,plan,checker,saved,reference,
                points,normals,line_q,np.asarray(candidate['q_contact_rad']),z['labels'])
            envelope=from_candidate(model,data,checker.robot,
                model.body('dental_'+info['jaw']).id,model.body('ur_end_scaler_link').id,
                z['tool_vertices'],z['tool_faces'],z['tool_ids'],points,normals,line_q,
                info['recipe']['nominal_standoff_mm'])
            trace=info.get('line_trace',{})
            arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
            if (trace.get('face_ids')!=ids.tolist() or trace.get('roll_deg')!=30.
                    or any(not np.allclose(trace.get(k),value,rtol=0,atol=1e-6) for k,value in [
                        ('point_scan_mm',points),('normal_scan',normals),('q_rad',line_q),
                        ('clearance_q_rad',line_clear),('arc_mm',arc)])
                    or not np.isclose(trace.get('length_mm'),arc[-1],atol=1e-6)
                    or not np.isclose(trace.get('max_gumline_distance_mm'),line_dist.max(),atol=1e-6)):
                raise ValueError('Line motion differs from saved original-scan candidate')
            data.qpos[checker.robot.qadr]=line_q[0]
            mujoco.mj_forward(model,data)
            jaw_id=model.body('dental_'+info['jaw']).id
            tool_id=model.body('ur_end_scaler_link').id
            jaw_rotation=data.xmat[jaw_id].reshape(3,3)
            stored=info.get('hold_envelope',{})
            expected=dict(tip_mm=(data.site_xpos[checker.robot.site]-data.xpos[jaw_id])@jaw_rotation*1000,
                normal_scan=normals[0],tool_rotation_scan=jaw_rotation.T@data.xmat[tool_id].reshape(3,3),
                target_q_rad=line_q[0],first_clearance_q_rad=first_clear,target_clearance_q_rad=line_clear[0])
            if (stored.get('normal_travel_mm')!=3. or stored.get('allowance_mm')!=1.5
                    or stored.get('max_lateral_mm')!=1. or stored.get('max_rotation_deg')!=5.
                    or any(not np.allclose(stored.get(k),v,rtol=0,atol=1e-6) for k,v in expected.items())):
                raise ValueError('Line entry and hold envelope differs from saved original scan')
        groups = {'tooth': [], 'tool': []}
        thickness = info['recipe']['thickness_mm']
        # The flush seam depends on a field shared by every patch of a role,
        # so the field is rebuilt here rather than trusted from the manifest.
        fields = {}
        for role, prefix in (('tooth', ''), ('tool', 'tool_')):
            v, f = z[prefix+'vertices'], z[prefix+'faces']
            band = z['tool_ids' if role=='tool' else 'band_ids']
            labels = np.zeros(len(f), dtype=int) if role=='tool' else z['labels']
            fields[role] = surface.extrusion_fields(v, f, labels, band, thickness,
                                                   flush=role!='tool')
        for part in info['parts']:
            file = path.parent/part['file']
            if file.parent.resolve() != path.parent or fingerprint(file)['sha256'] != part['sha256']:
                raise ValueError('Local contact patch changed')
            role = part['role']; prefix = 'tool_' if role=='tool' else ''
            v, f = z[prefix+'vertices'], z[prefix+'faces']
            ids = np.asarray(part['source_face_ids'], dtype=int)
            prepared = fields[role]
            expected = surface.convex_patch(v, f, ids, thickness,
                outward=prepared['normals'][ids], field=prepared['field'],
                depth_field=prepared['depth_field'])
            mesh = o3d.io.read_triangle_mesh(str(file))
            if (not np.array_equal(np.asarray(mesh.triangles), expected['faces'])
                    or not np.allclose(np.asarray(mesh.vertices), expected['vertices'],rtol=0,atol=1e-9)
                    or part['face_kinds'] != expected['face_kinds'].tolist()
                    or not np.isclose(part['thickness_mm'], expected['thickness_mm'],rtol=0,atol=1e-12)
                    or part['original_vertex_ids'] != expected['original_vertex_ids'].tolist()
                    or not np.allclose(part['normal_body'],expected['normal'],rtol=0,atol=1e-9)
                    or expected['maximum_error_mm'] > .05):
                raise ValueError('Local layer disagrees with original working surface or inward extrusion')
            groups[role].append(expected)
        for role, patches in groups.items():
            prefix = 'tool_' if role=='tool' else ''
            band = z['tool_ids' if role=='tool' else 'band_ids']
            check, records = surface.union_check(z[prefix+'vertices'], z[prefix+'faces'], patches,
                z['tool_qa_ids' if role=='tool' else 'qa_ids'], None if role=='tool' else z['qa_normals'],
                band_ids=band, source_normals=fields[role]['normals'])
            if envelope is not None and role=='tooth':
                if info.get('asset_scope')=='first_tooth_line':
                    from .line_envelope import reachable_failures
                else:
                    from .hold_envelope import reachable_failures
                reachable,_=reachable_failures(records,envelope)
                check['geometry_passed']=not reachable and not check['uncovered_normal_faces']
            if not check['geometry_passed']: raise ValueError('Recomputed local first-contact geometry gate failed')
            if role=='tooth':
                check,records=surface.gap_check(z['vertices'],z['faces'],z['labels'],patches,z['band_ids'])
                if envelope is not None:
                    if info.get('asset_scope')=='first_tooth_line':
                        from .line_envelope import reachable_failures
                    else:
                        from .hold_envelope import reachable_failures
                    reachable,_=reachable_failures(records,envelope)
                    check['geometry_passed']=not reachable
                if not check['geometry_passed']: raise ValueError('Local layer fills an original gap or label boundary')
