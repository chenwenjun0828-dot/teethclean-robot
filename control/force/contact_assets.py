"""Offline convex contact assets with measured surface-error gates (millimetres)."""
from pathlib import Path
import argparse
import json
import os
import multiprocessing
import tempfile

import numpy as np
import open3d as o3d

from planning.infrastructure.io import dump, fingerprint


def _decomposition_worker(input_file,output_file,threshold):
    import coacd
    if hasattr(coacd,'set_log_level'):coacd.set_log_level('warn')
    with np.load(input_file,allow_pickle=False) as arrays:
        vertices,faces,closed=arrays['vertices'],arrays['faces'],bool(arrays['watertight'])
    parts=coacd.run_coacd(coacd.Mesh(vertices,faces),threshold=threshold,real_metric=True,
        seed=0,preprocess_mode='off' if closed else 'auto',preprocess_resolution=100,
        resolution=1000,mcts_iterations=20,mcts_nodes=10,merge=False)
    values={'count':np.array(len(parts))}
    for i,(v,f) in enumerate(parts):values[f'v{i}']=v;values[f'f{i}']=f
    np.savez_compressed(output_file,**values)


def decompose_with_budget(closed,threshold_mm,seconds=120.):
    """A native decomposition cannot monopolize the caller indefinitely."""
    if not np.isfinite(seconds) or seconds<=0:raise ValueError('Mesh time budget must be positive')
    with tempfile.TemporaryDirectory(prefix='dental_contact_') as directory:
        input_file,output_file=Path(directory)/'input.npz',Path(directory)/'output.npz'
        np.savez_compressed(input_file,vertices=np.asarray(closed.vertices),faces=np.asarray(closed.triangles),
                            watertight=np.array(closed.is_watertight()))
        process=multiprocessing.get_context('spawn').Process(target=_decomposition_worker,
            args=(str(input_file),str(output_file),threshold_mm))
        process.start()
        try:
            process.join(seconds)
            if process.is_alive():
                raise TimeoutError(f'Convex decomposition exceeded {seconds:g} s for one mesh; contact geometry is not verified')
            if process.exitcode!=0 or not output_file.exists():
                raise ValueError('Convex decomposition worker failed; contact geometry is not verified')
            with np.load(output_file,allow_pickle=False) as arrays:
                return [(arrays[f'v{i}'].copy(),arrays[f'f{i}'].copy()) for i in range(int(arrays['count']))]
        finally:
            if process.is_alive():process.terminate();process.join()


def mesh(vertices, faces):
    return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices),
                                   o3d.utility.Vector3iVector(faces))


def close_boundaries(original):
    """Cap existing boundary loops without moving or smoothing original faces."""
    result=o3d.geometry.TriangleMesh(original)
    result.remove_duplicated_vertices();result.remove_degenerate_triangles()
    if not result.orient_triangles():
        raise ValueError('Tooth surface is not consistently orientable')
    vertices=np.asarray(result.vertices).tolist()
    faces=np.asarray(result.triangles).tolist()
    # Segmentation cuts can leave separate face fans touching at one vertex.
    # Split those vertices without changing any triangle's spatial position.
    incident={}
    for i,face in enumerate(faces):
        for v in face:incident.setdefault(v,[]).append(i)
    for vertex,indices in incident.items():
        neighbors={i:set() for i in indices};spokes={}
        for i in indices:
            for v in faces[i]:
                if v!=vertex:spokes.setdefault(v,[]).append(i)
        for group in spokes.values():
            for i in group:neighbors[i].update(set(group)-{i})
        unseen=set(indices);first=True
        while unseen:
            todo=[min(unseen)];fan=set()
            while todo:
                i=todo.pop()
                if i in fan:continue
                fan.add(i);todo.extend(neighbors[i]-fan)
            unseen-=fan
            if first:first=False;continue
            replacement=len(vertices);vertices.append(list(vertices[vertex]))
            for i in fan:faces[i]=[replacement if v==vertex else v for v in faces[i]]
    edges={}
    for face in faces:
        for a,b in zip(face,face[1:]+face[:1]):
            edges.setdefault(tuple(sorted((a,b))),[]).append((a,b))
    boundary=[value[0] for value in edges.values() if len(value)==1]
    if any(len(value)>2 for value in edges.values()):
        raise ValueError('Non-manifold tooth mesh cannot be capped without surface repair')
    forward={a:b for a,b in boundary}
    if len(forward)!=len(boundary) or len(set(forward.values()))!=len(boundary):
        raise ValueError('Boundary is not a collection of simple oriented loops')
    while forward:
        first=next(iter(forward));loop=[first];current=first
        while True:
            current=forward.pop(current)
            if current==first:break
            loop.append(current)
        # A new cap vertex avoids reusing any interior edge of the original patch.
        # Non-planar rims can produce intersecting caps; those are explicitly
        # repaired by CoACD preprocessing and still face the original-surface gate.
        center=len(vertices)
        vertices.append(np.mean(np.asarray(vertices)[loop],axis=0).tolist())
        for a,b in zip(loop,loop[1:]+loop[:1]):faces.append([b,a,center])
    closed=mesh(vertices,faces)
    if not closed.is_edge_manifold(allow_boundary_edges=False) or not closed.is_vertex_manifold():
        raise ValueError('Boundary caps did not produce closed manifold topology')
    triangles=np.asarray(closed.vertices)[np.asarray(closed.triangles)]
    volume=np.einsum('ij,ij->i',triangles[:,0],np.cross(triangles[:,1],triangles[:,2])).sum()/6
    if volume<0:closed.triangles=o3d.utility.Vector3iVector(np.asarray(closed.triangles)[:,::-1])
    return closed


def first_hit_error(original, pieces, samples, band_radius_mm=5.):
    """Measure first ray contacts near the reference, not CoACD's tuning threshold.

    Two-sided surface-normal rays check the accessible hull union boundary.
    This is sampled geometry validation, not a global Hausdorff certificate.
    """
    from scipy.spatial import cKDTree
    original.compute_triangle_normals()
    vertices, faces = np.asarray(original.vertices), np.asarray(original.triangles)
    centers = vertices[faces].mean(axis=1)
    ids = np.flatnonzero(cKDTree(samples).query(centers)[0] <= band_radius_mm)
    if not len(ids):
        raise ValueError('No original surface triangles inside the contact work band')
    # Bound diagnostics while retaining deterministic, spatially distributed samples.
    ids = ids[::max(1, len(ids)//12000)]
    points = centers[ids]
    normals = np.asarray(original.triangle_normals)[ids]
    scene = o3d.t.geometry.RaycastingScene(nthreads=1)
    for part in pieces:
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(part))
    errors = []
    for sign in (1., -1.):
        directions = normals * sign
        rays = np.c_[points + directions*.5, -directions].astype(np.float32)
        hits = scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy()
        errors.append(np.abs(hits-.5))
    # Open surface patches may be inaccessible from the inward side; the
    # outward normals are the production contact direction and must all hit.
    outward = errors[0]
    return dict(samples=len(points), geometry_passed=bool(np.isfinite(outward).all() and outward.max() <= .1),
                maximum_error_mm=float(outward.max()) if np.isfinite(outward).all() else None,
                p95_error_mm=float(np.percentile(outward,95)),
                missing_rays=int(np.sum(~np.isfinite(outward))),
                inward_maximum_error_mm=(float(errors[1].max()) if np.isfinite(errors[1]).all() else None))


def build(source, output, tolerance_mm=.1, threshold_mm=.05, max_mesh_seconds=120., method='local_surface'):
    """Decompose single teeth and the real tool; never modify the source plan."""
    if method == 'local_surface':
        from .local_assets import build as local_build
        return local_build(source, output)
    if method != 'coacd': raise ValueError('Unknown contact asset method')
    import mujoco
    from planning.motion_planning.execution import load_verified
    from planning.motion_planning.contract import load_sweep_reference
    from pipeline.simulation import DENTAL_MODEL_PATH

    if not 0 < threshold_mm <= tolerance_mm <= .1:
        raise ValueError('Require 0 < decomposition threshold <= surface tolerance <= 0.1 mm')
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError('Use a new empty contact asset directory')
    model, data, _, _ = load_verified(source)
    saved = json.loads((source/'planning_report.json').read_text())
    request = json.loads((source/'request.json').read_text())
    reference = load_sweep_reference(source,saved)
    stations = reference.samples(.00025)
    work_points = np.array([reference.evaluate(s) for s in stations])*1000
    output.mkdir(parents=True,exist_ok=True)
    manifest = dict(schema_version=1, method='coacd', status='building', simulation_only=True,
        source_trajectory_sha256=fingerprint(source/'trajectory.npz')['sha256'],
        source_plan_sha256=fingerprint(source/'planning_report.json')['sha256'],
        jaw=request['inputs']['jaw'], region=saved['region'], selected_teeth=saved['selected_teeth'],
        material_status='uncalibrated_simulation_parameters', scene_sha256=fingerprint(DENTAL_MODEL_PATH)['sha256'],
        units='mm', threshold_mm=threshold_mm, surface_tolerance_mm=tolerance_mm,max_mesh_seconds=max_mesh_seconds,
        tool_contact_radius_mm=3., parts=[], checks=[],
        decomposition_parameters=dict(seed=0,preprocess_resolution=100,resolution=1000,mcts_iterations=20,mcts_nodes=10,merge=False),
        coverage='All labelled teeth; unlabelled scan faces are checked by original-mesh audit, not certified physical tissue.')
    dump(output/'manifest.json',manifest)
    try:
        # The mesh coordinates below are in their corresponding rigid body frames.
        inputs=[]
        for path in sorted((source/'teeth').glob('tooth_*_scan_mm.ply')):
            tooth=int(path.name.split('_')[1])
            original=o3d.io.read_triangle_mesh(str(path))
            inputs.append((f'tooth_{tooth:02d}', original, f'dental_{manifest["jaw"]}', tooth))
        if not inputs:
            raise ValueError('Source plan contains no exported single-tooth meshes')
        root=Path(__file__).resolve().parents[2]
        tool_path=root/'assets/simulation/robot/assets/end_scaler.stl'
        tool=o3d.io.read_triangle_mesh(str(tool_path))
        # Original STL local frame -> actual end_scaler_link frame, including geom placement.
        spec=mujoco.MjSpec.from_file(str(DENTAL_MODEL_PATH));spec.compile()
        geom=spec.geom('ur_end_scaler_geom')
        rotation=np.empty(9);mujoco.mju_quat2Mat(rotation,geom.quat)
        tool.vertices=o3d.utility.Vector3dVector(np.asarray(tool.vertices)@rotation.reshape(3,3).T+np.asarray(geom.pos)*1000)
        inputs.append(('tool',tool,'ur_end_scaler_link',None))
        for label, original, body, tooth in inputs:
            print(f'Decomposing {label}: {len(original.triangles)} faces',flush=True)
            original.remove_duplicated_vertices();original.remove_degenerate_triangles()
            closed=close_boundaries(original)
            parts=decompose_with_budget(closed,threshold_mm,max_mesh_seconds)
            hulls=[]
            for i,(vertices,faces) in enumerate(parts):
                hull=mesh(vertices,faces);hulls.append(hull)
                name=f'{label}_{i:04d}.ply'
                if not o3d.io.write_triangle_mesh(str(output/name),hull):
                    raise OSError(f'Could not write {name}')
                manifest['parts'].append(dict(file=name,sha256=fingerprint(output/name)['sha256'],
                    body=body,tooth=tooth,role='tool' if label=='tool' else 'tooth'))
            if not hulls:
                raise ValueError(f'No convex parts generated for {label}')
            if label=='tool':
                tip=np.array(spec.site('ur_scaler_tcp').pos)*1000
                samples=np.array([tip])
                check=first_hit_error(original,hulls,samples,3.)
            elif tooth in saved['selected_teeth']:
                check=first_hit_error(original,hulls,work_points)
            else:
                check=dict(samples=0,maximum_error_mm=None,scope='non-target tooth; runtime forbidden contact only')
            manifest['checks'].append(dict(label=label,**check))
            if check['samples']>0 and (not check.get('geometry_passed') or check['maximum_error_mm']>tolerance_mm):
                manifest.update(status='failed_geometry_gate',failed_parts=[label])
                dump(output/'manifest.json',manifest)
                return manifest
            dump(output/'manifest.json',manifest)
        failed=[c['label'] for c in manifest['checks'] if c['samples'] > 0
                and (not c.get('geometry_passed') or c['maximum_error_mm'] > tolerance_mm)]
        manifest.update(status='failed_geometry_gate' if failed else 'verified',failed_parts=failed)
        # Never serialize Infinity as evidence of a valid numeric measurement.
        for c in manifest['checks']:
            for key,value in list(c.items()):
                if isinstance(value,float) and not np.isfinite(value):c[key]=None
        dump(output/'manifest.json',manifest)
        return manifest
    except Exception as error:
        manifest.update(status='failed',error=str(error))
        for c in manifest['checks']:
            for key,value in list(c.items()):
                if isinstance(value,float) and not np.isfinite(value):c[key]=None
        dump(output/'manifest.json',manifest)
        raise


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--threshold-mm',type=float,default=.05)
    parser.add_argument('--method',choices=('local_surface','coacd'),default='local_surface')
    parser.add_argument('--max-mesh-seconds',type=float,default=120.)
    args=parser.parse_args(argv)
    try:
        report=build(args.source,args.out,threshold_mm=args.threshold_mm,max_mesh_seconds=args.max_mesh_seconds,method=args.method)
        print('Contact asset status:',report['status'])
        return 0 if report['status']=='verified' else 2
    except (ValueError,RuntimeError,OSError) as error:
        print('Contact asset build failed:',error)
        return 1


if __name__=='__main__':
    raise SystemExit(main())
