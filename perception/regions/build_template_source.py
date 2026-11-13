"""Create a review-required six-region source on another scan's actual surface."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.spatial import cKDTree

from perception.gingival_line import read_topology
from perception.regional_paths import load_regional_paths, REGION_ORDER
from .io import ROOT, fingerprint


def surface_path(vertices, edges, guide_xz, gingival_distance, anchors):
    """Join template anchors over mesh edges; never use straight gap connectors."""
    guide_distance = cKDTree(guide_xz).query(vertices[:, [0, 2]])[0]
    length = np.linalg.norm(vertices[edges[:, 1]] - vertices[edges[:, 0]], axis=1)
    penalty = 1 + (guide_distance / 1.5)**2 + (gingival_distance / .6)**2
    weights = length * (penalty[edges[:, 0]] + penalty[edges[:, 1]]) / 2
    graph = coo_matrix((np.r_[weights, weights],
        (np.r_[edges[:, 0], edges[:, 1]], np.r_[edges[:, 1], edges[:, 0]])),
        shape=(len(vertices), len(vertices))).tocsr()
    path = [int(anchors[0])]
    for target in anchors[1:]:
        target = int(target)
        if target == path[-1]:
            continue
        distance, predecessor = dijkstra(graph, directed=False, indices=path[-1], return_predecessors=True)
        if not np.isfinite(distance[target]):
            raise ValueError('Upper surface is disconnected between selected template anchors')
        segment = [target]
        while segment[-1] != path[-1]:
            segment.append(int(predecessor[segment[-1]]))
        path.extend(segment[-2::-1])
    # No repeated-vertex excursions: retain a simple surface route from start to end.
    simple, positions = [], {}
    for vertex in path:
        if vertex in positions:
            cut = positions[vertex] + 1
            for removed in simple[cut:]:
                del positions[removed]
            del simple[cut:]
        else:
            positions[vertex] = len(simple)
            simple.append(vertex)
    path = simple
    points = vertices[path]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    keep = [0]
    for i in range(1, len(points)-1):
        if arc[i] - arc[keep[-1]] >= .6:
            keep.append(i)
    keep.append(len(points)-1)
    return points[keep], np.array(path), arc[-1]


def build_source(jaw, template_dir, registration_matrix, output):
    if jaw != 'maxillary':
        raise ValueError('This reviewed-template transfer currently targets maxillary scans only')
    template = load_regional_paths(template_dir)
    if template.jaw != 'mandibular':
        raise ValueError('Expected the mandibular six-region template')
    scan_path = ROOT / f'assets/source_cad/teeth/{jaw}_scan.ply'
    topology_path = ROOT / f'assets/perception/{jaw}/gingival_line_segments.obj'
    gum, _ = read_topology(topology_path)
    original_gum_count = len(gum)
    y_band = np.percentile(gum[:, 1], [5, 95])
    gum = gum[(gum[:, 1] >= y_band[0]) & (gum[:, 1] <= y_band[1])]
    mesh = o3d.io.read_triangle_mesh(str(scan_path))
    vertices, faces = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
    edges = np.unique(np.sort(np.vstack([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1), axis=0)
    gum_distance = cKDTree(gum).query(vertices)[0]
    gum_vertex = cKDTree(vertices).query(gum)[1]
    template_points = np.vstack([c.scan_points_mm for c in template.chains])[:, [0, 2]]
    lower_bounds = np.array([template_points.min(axis=0), template_points.max(axis=0)])
    upper_bounds = np.percentile(gum[:, [0, 2]], [1, 99], axis=0)
    scale = np.diff(upper_bounds, axis=0)[0] / np.diff(lower_bounds, axis=0)[0]
    transform = np.load(registration_matrix).copy()
    transform[:3, 3] *= 1000.
    rows, reports = [], []
    for order, chain in enumerate(template.chains, 1):
        guide = (chain.scan_points_mm[:, [0, 2]] - lower_bounds[0]) * scale + upper_bounds[0]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(guide, axis=0), axis=1))]
        samples = np.linspace(0., arc[-1], max(3, int(np.ceil(arc[-1] / 8.)) + 1))
        anchors_xz = np.column_stack([np.interp(samples, arc, guide[:, axis]) for axis in (0, 1)])
        nearest_gum = cKDTree(gum[:, [0, 2]]).query(anchors_xz)[1]
        points, path_ids, surface_length = surface_path(vertices, edges, guide, gum_distance, gum_vertex[nearest_gum])
        if len(points) < 4:
            raise ValueError(f'Insufficient points for {chain.region}')
        camera = points @ transform[:3, :3].T + transform[:3, 3]
        arc_source = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        for i, (point, station) in enumerate(zip(camera, arc_source)):
            rows.append([order, chain.region, i, *point, station, True, False])
        reports.append(dict(region=chain.region, source_points=len(points), surface_edge_path_length_mm=surface_length,
            source_polyline_length_mm=float(arc_source[-1]), anchors_scan_mm=vertices[gum_vertex[nearest_gum]].tolist(),
            max_gingival_distance_mm=float(gum_distance[path_ids].max()),
            max_source_step_mm=float(np.linalg.norm(np.diff(points, axis=0), axis=1).max()),
            source_on_mesh_vertices=True))
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise ValueError('Use an empty new source directory')
    output.mkdir(parents=True, exist_ok=True)
    np.savetxt(output / 'T_camera_scan.txt', transform, header='scan_mm -> depth_camera_mm; serialization placement only')
    with (output / 'regional_chain_candidates_camera_mm.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['chain_order', 'region', 'point_order', 'x_mm', 'y_mm', 'z_mm',
                         'arc_length_mm', 'simulation_only', 'robot_execution_allowed'])
        writer.writerows(rows)
    report = dict(method='mandibular six-region template transferred in XZ, anchors snapped to maxillary gingival topology, mesh-edge geodesics',
        new_source_is_candidate=True, region_boundaries_approved=False,
        contains_generated_surface_paths=True, contains_straight_gap_connectors=False,
        inputs=[fingerprint(scan_path), fingerprint(topology_path), fingerprint(Path(template_dir) / 'metadata.json'),
                fingerprint(registration_matrix)], regions=reports,
        xz_template_scale=scale.tolist(), lower_bounds_xz_mm=lower_bounds.tolist(), upper_bounds_xz_mm=upper_bounds.tolist(),
        anchor_y_band_mm=y_band.tolist(), original_gingival_points=original_gum_count,
        anchor_gingival_points=len(gum), loop_erased=True,
        limitations=['Region boundaries are template proposals, not anatomical certification.',
                     'Sources use generated mesh-edge routes near the gingival line, not only original extracted line edges.',
                     'No robot execution or contact validation.'])
    (output / 'stitch_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    names = ['regional_chain_candidates_camera_mm.csv', 'T_camera_scan.txt', 'stitch_report.json']
    metadata = dict(schema_version=1, jaw=jaw, frame='d405_camera', unit='mm',
        transform_name='T_camera_scan', transform_translation_unit='mm',
        mesh_path=str(scan_path.relative_to(ROOT)).replace('\\', '/'), mesh_sha256=fingerprint(scan_path)['sha256'],
        files_sha256={name: hashlib.sha256((output / name).read_bytes()).hexdigest() for name in names},
        region_order=list(REGION_ORDER), expected_points=len(rows),
        source_version='template_geodesic_candidate_v1', source_snapshot='maxillary_surface_candidate',
        region_boundaries_approved=False, robot_execution_allowed=False,
        usage='Review-required maxillary six-region source; no robot execution.',
        provenance_note='See stitch_report.json. Current registration only serializes points; loader removes it before any new placement.')
    (output / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    load_regional_paths(output)
    for item in reports:
        print(item['region'], item['source_points'], round(item['source_polyline_length_mm'], 3), 'mm',
              'max gum distance', round(item['max_gingival_distance_mm'], 3), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jaw', default='maxillary')
    parser.add_argument('--template', type=Path, default=ROOT / 'assets/regional_paths/mandibular')
    parser.add_argument('--registration-matrix', type=Path, required=True, help='Scan-to-depth-camera matrix in metres')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    build_source(args.jaw, args.template, args.registration_matrix, args.out)


if __name__ == '__main__':
    main()
