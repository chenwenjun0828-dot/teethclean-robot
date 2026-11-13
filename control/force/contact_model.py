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
    for source in info.get('source_inputs', []):
        if fingerprint(ROOT/source['file'])['sha256'] != source['sha256']:
            raise ValueError('Original scan, labels or actual tool mesh changed')
    if len(info.get('source_inputs', [])) != 3: raise ValueError('Local contact provenance is incomplete')
    with np.load(evidence, allow_pickle=False) as z:
        validate_evidence_sources(info,z)
        if source is not None:
            from planning.motion_planning.contract import load_sweep_reference
            saved=json.loads((Path(source)/'planning_report.json').read_text())
            reference=load_sweep_reference(source,saved)
            stations=reference.samples(.00025)
            points=np.array([reference.evaluate(s) for s in stations])*1000
            normals=np.array([reference.normal(s) for s in stations])
            if (not np.allclose(points,z['work_points'],rtol=0,atol=1e-9)
                    or not np.allclose(normals,z['work_normals'],rtol=0,atol=1e-9)
                    or info['recipe']['nominal_standoff_mm']!=saved['config']['standoff_mm']):
                raise ValueError('Local contact work band differs from original saved curve')
        validate_evidence(info, z)
        groups = {'tooth': [], 'tool': []}
        for part in info['parts']:
            file = path.parent/part['file']
            if file.parent.resolve() != path.parent or fingerprint(file)['sha256'] != part['sha256']:
                raise ValueError('Local contact patch changed')
            role = part['role']; prefix = 'tool_' if role=='tool' else ''
            v, f = z[prefix+'vertices'], z[prefix+'faces']
            ids = np.asarray(part['source_face_ids'], dtype=int)
            expected = surface.convex_patch(v, f, ids)
            mesh = o3d.io.read_triangle_mesh(str(file))
            if (not np.array_equal(np.asarray(mesh.triangles), expected['faces'])
                    or not np.allclose(np.asarray(mesh.vertices), expected['vertices'],rtol=0,atol=1e-9)
                    or part['face_kinds'] != expected['face_kinds'].tolist()
                    or expected['maximum_error_mm'] > .05):
                raise ValueError('Local layer disagrees with original working surface or inward extrusion')
            groups[role].append(expected)
        for role, patches in groups.items():
            prefix = 'tool_' if role=='tool' else ''
            check, _ = surface.union_check(z[prefix+'vertices'], z[prefix+'faces'], patches,
                z['tool_qa_ids' if role=='tool' else 'qa_ids'], None if role=='tool' else z['qa_normals'])
            if not check['geometry_passed']: raise ValueError('Recomputed local first-contact geometry gate failed')
            if role=='tooth':
                check,_=surface.gap_check(z['vertices'],z['faces'],z['labels'],patches,z['band_ids'])
                if not check['geometry_passed']: raise ValueError('Local layer fills an original gap or label boundary')
