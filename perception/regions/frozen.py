"""Adapt the collaborator's ordered six-region snapshot to the common contract."""
from dataclasses import asdict
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.interpolate import BSpline

from perception.regional_paths import load_regional_paths
from perception.tooth_segmentation import load_prediction, clean_instances
from .config import REGION_ORDER
from .geometry import Geometry, Stroke, fit_curve
from .surface import surface_plane
from .io import fingerprint, unit
from .continuous import SweepPath
from .config import continuous_path_kind


def build_regional_geometry(dataset, inputs, cfg):
    dataset = load_regional_paths(dataset) if isinstance(dataset, (str, Path)) else dataset
    if dataset.jaw != inputs.jaw or fingerprint(inputs.scan)['sha256'] != dataset.metadata['mesh_sha256']:
        raise ValueError('Regional paths do not belong to the requested scan/jaw')
    name = cfg.region
    chain = next((c for c in dataset.chains if c.region == name), None)
    if chain is None:
        raise ValueError(f'No prepared source for region {cfg.region}')
    mesh = o3d.io.read_triangle_mesh(str(inputs.scan))
    v, f = np.asarray(mesh.vertices)*.001, np.asarray(mesh.triangles)
    raw, _, _ = load_prediction(inputs.prediction, inputs.scan, v*1000, f)
    labels, instances = clean_instances(v*1000, f, raw)
    triangles = v[f]
    centers = triangles.mean(axis=1)
    cross = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    lengths = np.linalg.norm(cross, axis=1)
    face_normals = cross/np.maximum(lengths[:, None], 1e-20)
    cloud_ids = np.flatnonzero((labels > 0) & (lengths > 1e-14))
    points, tangents, parameters, curve = fit_curve(
        chain.scan_points_mm*.001, cfg.fit_tolerance_mm*.001, cfg.spacing_mm*.001)
    distance, nearest = cKDTree(centers[cloud_ids]).query(points)
    assigned = labels[cloud_ids[nearest]]
    scene = o3d.t.geometry.RaycastingScene(nthreads=1)
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    source_distances = scene.compute_distance(o3d.core.Tensor(chain.scan_points_mm.astype(np.float32))).numpy()
    if source_distances.max() > .35:
        raise ValueError('Frozen regional source is more than 0.35 mm from its scan surface')
    nearest_mesh = scene.compute_closest_points(o3d.core.Tensor((points*1000).astype(np.float32)))['primitive_ids'].numpy().astype(int)
    normals = face_normals[nearest_mesh]
    arch_points = np.vstack([c.scan_points_mm[:, [0, 2]] for c in dataset.chains])*.001
    arch_fit = np.linalg.lstsq(np.c_[2*arch_points, np.ones(len(arch_points))],
                              np.sum(arch_points**2, axis=1), rcond=None)[0]
    planes = {}
    for tooth in dict.fromkeys(map(int, assigned)):
        line = points[assigned == tooth]
        radial = line.mean(axis=0)[[0, 2]]-arch_fit[:2]
        normal = unit([radial[0], 0., radial[1]])
        if name.startswith('inner'):
            normal = -normal
        nearby = cKDTree(line).query(centers)[0]
        plane = {'valid': False, 'reason': 'insufficient_region_side_surface_support'}
        support = np.array([], int)
        for alignment in (.5, .75, .85):
            for band in sorted({cfg.plane_band_mm, 3., 2., 1.5, .75}, reverse=True):
                support = np.flatnonzero((labels == tooth) & (lengths > 1e-14)
                                        & (nearby <= min(cfg.plane_band_mm, band)*.001)
                                        & (face_normals@normal > alignment))
                try:
                    plane = surface_plane(centers[support], normal, cfg.plane_rms_limit_mm*.001, lengths[support])
                except ValueError:
                    continue
                if plane['valid']:
                    break
            if plane['valid']:
                break
        plane.update(support_face_ids=support, side='inner' if name.startswith('inner') else 'outer')
        planes[tooth] = plane
    cuts = np.r_[0, 1+np.flatnonzero(np.diff(assigned) != 0), len(points)]
    strokes = []
    for a, b in zip(cuts[:-1], cuts[1:]):
        if b-a < 2:
            continue
        strokes.append(Stroke(len(strokes), 0, int(assigned[a]), cfg.region,
                              points[a:b], tangents[a:b], parameters[a:b], np.arange(a, b)))
    teeth = list(dict.fromkeys(s.tooth for s in strokes))
    if not strokes:
        raise ValueError('No regional source intervals remain')
    report = {'schema_version': 1, 'module': 'perception', 'jaw': inputs.jaw, 'units': 'm',
              'workflow': 'frozen_six_regions_preserving_source_order',
              'region': cfg.region, 'source_region': name, 'region_names': list(REGION_ORDER),
              'regions': {cfg.region: teeth}, 'selected_teeth': teeth,
              'preserve_stroke_order': True, 'config': asdict(cfg), 'curves': [curve],
              'instances': instances, 'source_gaps_preserved': True, 'omitted_source_chains': [],
              'source_metadata': dataset.metadata,
              'source_surface_max_distance_mm': float(source_distances.max()),
              'assignment_max_distance_mm': float(distance.max()*1000),
              'assignment_beyond_support_count': int((distance > cfg.assignment_max_distance_mm*.001).sum()),
              'tooth_id_convention': 'Nearest CrossTooth instance for orientation support; not certified FDI',
              'inputs': {'scan': fingerprint(inputs.scan),
                         'prediction': fingerprint(inputs.prediction/'prediction.npz')},
              'strokes': [{'id': s.id, 'curve_id': s.curve_id, 'tooth': s.tooth,
                           'region': s.region, 'length_mm': s.length_mm,
                           'samples': len(s.points)} for s in strokes]}
    if dataset.metadata.get('dataset_dir'):
        folder = Path(dataset.metadata['dataset_dir'])
        report['inputs'].update({name: fingerprint(folder/name) for name in (
            'metadata.json', 'regional_chain_candidates_camera_mm.csv', 'T_camera_scan.txt')})
    lookup = {'points_scan_m': points, 'tooth_id': assigned, 'distance_mm': distance*1000,
              'nearest_source_face_id': cloud_ids[nearest], 'curve_id': np.zeros(len(points), int),
              'cloud_scan_m': centers[cloud_ids], 'cloud_tooth_id': labels[cloud_ids]}
    return Geometry(strokes, planes, report, v, f, labels), lookup


def continuous_regional_reference(geometry, cfg, options):
    """Keep the supplied regional curve intact, including every tooth boundary."""
    strokes = geometry.strokes
    curve_record = geometry.report['curves'][0]
    curve = BSpline(curve_record['knots'], curve_record['coefficients_m'], 3, extrapolate=False)
    grid = np.linspace(0., 1., 10001)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(curve(grid), axis=0), axis=1))]
    trim = min(options.trim_mm*.001, max(0., (arc[-1]-cfg.min_stroke_mm*.001)/2))
    retained_start, retained_end = np.interp([trim, arc[-1]-trim], arc, grid)
    boundaries = np.r_[0., [(a.parameters[-1]+b.parameters[0])/2
                             for a, b in zip(strokes[:-1], strokes[1:])], 1.]
    normals = []
    for stroke in strokes:
        plane = geometry.planes[stroke.tooth]
        if not plane['valid']:
            raise ValueError(f'Invalid tooth plane: {stroke.tooth}')
        normals.append(unit(plane['normal_scan']))
    boundary_normals = [normals[0]]+[unit(a+b) for a, b in zip(normals[:-1], normals[1:])]+[normals[-1]]
    parts, totals, retained = [], {}, {}
    for i, stroke in enumerate(strokes):
        total = float(np.diff(np.interp(boundaries[i:i+2], grid, arc))[0]*1000)
        totals[stroke.tooth] = totals.get(stroke.tooth, 0.)+total
        a, b = max(boundaries[i], retained_start), min(boundaries[i+1], retained_end)
        if b <= a:
            continue
        length = float(np.diff(np.interp([a, b], grid, arc))[0])
        start = parts[-1]['end'] if parts else 0.
        span = float((b-a)*arc[-1])
        parts.append({'kind': 'source', 'stroke_id': stroke.id, 'curve_id': 0,
                      'u0': float(a), 'u1': float(b), 'start': start, 'end': start+span,
                      'knots': curve_record['knots'], 'coefficients_m': curve_record['coefficients_m'],
                      'normal0': boundary_normals[i], 'normal1': boundary_normals[i+1],
                      'tooth0': stroke.tooth, 'tooth1': stroke.tooth})
        retained[stroke.tooth] = retained.get(stroke.tooth, 0.)+length*1000
    for tooth, total in totals.items():
        if retained.get(tooth, 0.)/total < cfg.min_tooth_coverage:
            raise ValueError(f'Insufficient retained source coverage for tooth {tooth}')
    total_length, retained_length = sum(totals.values()), sum(retained.values())
    return SweepPath({'schema_version': 1, 'kind': continuous_path_kind(cfg.region),
        'region': cfg.region, 'planning_mode': cfg.planning_mode,
        'teeth': list(totals), 'parts': parts, 'options': asdict(options),
        'trim_mm': options.trim_mm, 'trim_scope': 'region_endpoints_only',
        'source_trims': [{'trim_start_mm': trim*1000, 'trim_end_mm': trim*1000}],
        'omitted_stroke_ids': [], 'geometry_owner': 'perception',
        'source_region_length_mm': total_length, 'retained_length_mm': retained_length,
        'trimmed_or_omitted_length_mm': total_length-retained_length,
        'synthetic_bridge_length_mm': 0., 'sweep_length_mm': retained_length,
        'per_tooth': {str(t): {'source_length_mm': totals[t], 'retained_length_mm': retained[t],
                             'coverage_fraction': retained[t]/totals[t]} for t in totals}})
