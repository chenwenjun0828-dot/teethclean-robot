"""Integrity of coverage, triangle ownership, and the original scan evidence."""
import numpy as np
from scipy.spatial import cKDTree
from .local_surface import adjacency


def validate_evidence(info, z):
    recipe = info['recipe']
    expected = dict(normal_correction_mm=3.,tool_radius_mm=3.,boundary_margin_mm=1.,thickness_mm=.5,
                    initial_patch_scale_mm=1.,minimum_patch_scale_mm=.25,patch_tolerance_mm=.05,final_tolerance_mm=.1)
    if any(recipe[k] != v for k,v in expected.items()): raise ValueError('Local contact recipe changed')
    radius = sum(recipe[k] for k in ('nominal_standoff_mm','normal_correction_mm','tool_radius_mm','boundary_margin_mm'))
    tri = z['vertices'][z['faces']]; centers = tri.mean(1)
    tr = np.linalg.norm(tri-centers[:,None],axis=2).max(1)
    distance = cKDTree(z['work_points']).query(centers)[0]
    required = set(np.flatnonzero((distance-tr <= radius) & (z['labels']>0)))
    if required != set(z['band_ids'].tolist()): raise ValueError('Work-band boundary is not covered')
    qa=set(np.flatnonzero((distance<=radius-1.-tr)&np.isin(z['labels'],info['selected_teeth'])))
    if not qa or qa!=set(z['qa_ids'].tolist()): raise ValueError('Operational ray coverage is incomplete')
    tool=z['tool_vertices'][z['tool_faces']]; center=tool.mean(1)
    extent=np.linalg.norm(tool-center[:,None],axis=2).max(1)
    distance=np.linalg.norm(center-z['tip'],axis=1)
    if (set(np.flatnonzero(distance-extent<=4.))!=set(z['tool_ids'].tolist())
            or set(np.flatnonzero(distance+extent<=3.))!=set(z['tool_qa_ids'].tolist())):
        raise ValueError('Actual tool tip coverage is incomplete')
    for role in ('tooth','tool'):
        f = z['tool_faces' if role=='tool' else 'faces']
        labels = np.zeros(len(f),dtype=int) if role=='tool' else z['labels']
        neighbors = adjacency(f, labels)
        used = []
        for part in (p for p in info['parts'] if p['role']==role):
            ids = set(part['source_face_ids']); used.extend(ids)
            if not ids or min(ids)<0 or max(ids)>=len(f): raise ValueError('Invalid source triangle index')
            if role=='tooth' and any(int(labels[i])!=part['tooth'] for i in ids):
                raise ValueError('A patch crosses tooth labels')
            visited=set(); todo=[min(ids)]
            while todo:
                i=todo.pop()
                if i in visited: continue
                visited.add(i); todo.extend((neighbors[i]&ids)-visited)
            if visited != ids: raise ValueError('A patch bridges a gap or disconnected surface')
        needed = set(z['tool_ids' if role=='tool' else 'band_ids'].tolist())
        if len(used)!=len(set(used)) or set(used)!=needed: raise ValueError('Local patch coverage is missing or duplicated')


def validate_evidence_sources(info,z):
    import mujoco
    import open3d as o3d
    from planning.infrastructure.io import ROOT
    from pipeline.simulation import DENTAL_MODEL_PATH
    scan_file,tool_file,label_file=[ROOT/item['file'] for item in info['source_inputs']]
    scan=o3d.io.read_triangle_mesh(str(scan_file))
    if (not np.array_equal(np.asarray(scan.vertices),z['vertices'])
            or not np.array_equal(np.asarray(scan.triangles),z['faces'])
            or not np.array_equal(np.load(label_file,allow_pickle=False),z['labels'])):
        raise ValueError('Saved contact evidence disagrees with the actual original scan/labels')
    tool=o3d.io.read_triangle_mesh(str(tool_file)); tool.remove_duplicated_vertices()
    spec=mujoco.MjSpec.from_file(str(DENTAL_MODEL_PATH));spec.compile()
    geom=spec.geom('ur_end_scaler_geom'); rotation=np.empty(9);mujoco.mju_quat2Mat(rotation,geom.quat)
    vertices=np.asarray(tool.vertices)@rotation.reshape(3,3).T+np.asarray(geom.pos)*1000
    if (not np.array_equal(np.asarray(tool.triangles),z['tool_faces'])
            or not np.allclose(vertices,z['tool_vertices'],rtol=0,atol=1e-9)
            or not np.allclose(np.asarray(spec.site('ur_scaler_tcp').pos)*1000,z['tip'],rtol=0,atol=1e-9)):
        raise ValueError('Saved contact evidence disagrees with actual tool geometry/transform')
