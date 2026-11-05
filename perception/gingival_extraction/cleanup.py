"""Remove unsupported color regions before extracting gingival contours.

Long scan-rim artifacts are connected curves, so point-cloud density filtering
cannot distinguish them from real gingival margins. Instead, assess the mesh
color regions that generate those curves using surface area and scan boundaries.
"""
from __future__ import annotations

import numpy as np
import open3d as o3d
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components, dijkstra


def clean_color_regions(
    mesh: o3d.geometry.TriangleMesh,
    scores: np.ndarray,
    threshold: float,
    min_tooth_area_mm2: float = 10.0,
    min_gum_area_mm2: float = 2.0,
    boundary_band_mm: float = 3.0,
) -> tuple[np.ndarray, dict]:
    """Remove tiny tooth patches, scan-rim strips and small enclosed gum specks.

Reject a tooth-like component when its surface area is too small or at least
half of its vertices lie in the mesh's open-boundary band. Unlike cropping all
points near an edge, this preserves genuine margins on a supported crown even
if part of that crown touches the scan boundary. Closed components have no
boundary distance limit. Small gum islands are filled only when surrounded by
retained tooth regions. Distances are measured along mesh edges, in millimetres.
"""
    vertices, faces = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
    scores = np.asarray(scores, dtype=float)
    if scores.shape != (len(vertices),) or not np.isfinite(scores).all():
        raise ValueError('Cleanup requires a finite gum score for every mesh vertex')
    if not 0 < threshold < 1:
        raise ValueError('Cleanup requires a threshold between 0 and 1')
    if min(min_tooth_area_mm2, min_gum_area_mm2, boundary_band_mm) < 0:
        raise ValueError('Cleanup area and boundary limits must be nonnegative')

    edges, occurrences = np.unique(np.sort(np.concatenate(
        (faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]])), axis=1),
        axis=0, return_counts=True)
    tooth = scores < threshold
    same_class = tooth[edges[:, 0]] == tooth[edges[:, 1]]
    region_edges = edges[same_class]
    graph = coo_matrix((np.ones(len(region_edges)),
                        (region_edges[:, 0], region_edges[:, 1])),
                       shape=(len(vertices), len(vertices))).tocsr()
    count, labels = connected_components(graph, directed=False)

    face_area = np.linalg.norm(np.cross(vertices[faces[:, 1]]-vertices[faces[:, 0]],
                                         vertices[faces[:, 2]]-vertices[faces[:, 0]]), axis=1) / 2
    vertex_area = np.bincount(faces.ravel(), weights=np.repeat(face_area / 3, 3),
                              minlength=len(vertices))
    areas = np.bincount(labels, weights=vertex_area, minlength=count)
    boundary = np.unique(edges[occurrences == 1])
    boundary_distance = np.full(len(vertices), np.inf)
    if len(boundary):
        lengths = np.linalg.norm(vertices[edges[:, 0]]-vertices[edges[:, 1]], axis=1)
        metric = coo_matrix((np.tile(lengths, 2),
                             (np.r_[edges[:, 0], edges[:, 1]], np.r_[edges[:, 1], edges[:, 0]])),
                            shape=(len(vertices), len(vertices))).tocsr()
        boundary_distance = dijkstra(metric, directed=False, indices=boundary, min_only=True)

    ordered = np.argsort(labels, kind='stable')
    regions = np.split(ordered, np.flatnonzero(np.diff(labels[ordered])) + 1)
    cleaned = scores.copy()
    supported = set()
    removed_teeth = []
    for indices in regions:
        if not tooth[indices[0]]:
            continue
        component = int(labels[indices[0]])
        median_distance = float(np.median(boundary_distance[indices]))
        reason = None
        if areas[component] < min_tooth_area_mm2:
            reason = 'small_tooth_color_island'
        elif median_distance < boundary_band_mm:
            reason = 'scan_boundary_color_strip'
        if reason is None:
            supported.add(component)
        else:
            cleaned[indices] = 1.0
            removed_teeth.append({
                'reason': reason, 'vertices': len(indices),
                'area_mm2': float(areas[component]),
                'median_boundary_distance_mm': median_distance if np.isfinite(median_distance) else None,
                'centroid_mm': vertices[indices].mean(axis=0).tolist(),
            })
    if not supported:
        raise ValueError('No supported tooth region remains after gingival contour cleanup')

    adjacent = [set() for _ in range(count)]
    for a, b in edges[~same_class]:
        ca, cb = int(labels[a]), int(labels[b])
        adjacent[ca].add(cb)
        adjacent[cb].add(ca)
    filled_gum = 0
    for indices in regions:
        component = int(labels[indices[0]])
        if (not tooth[indices[0]] and areas[component] < min_gum_area_mm2
                and adjacent[component] and adjacent[component].issubset(supported)):
            cleaned[indices] = 0.0
            filled_gum += 1

    return cleaned, {
        'enabled': True,
        'method': 'mesh_color_component_area_and_geodesic_boundary_support',
        'min_tooth_area_mm2': min_tooth_area_mm2,
        'min_gum_area_mm2': min_gum_area_mm2,
        'boundary_band_mm': boundary_band_mm,
        'retained_tooth_components': len(supported),
        'removed_tooth_components': len(removed_teeth),
        'removed_tooth_regions': removed_teeth,
        'filled_small_gum_islands': filled_gum,
        'changed_vertices': int(np.count_nonzero((cleaned < threshold) != tooth)),
    }
