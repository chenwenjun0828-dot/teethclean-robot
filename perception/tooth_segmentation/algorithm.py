"""Verified CrossTooth predictions and topology-supported per-tooth gingival arcs.

Network classes are local instance identifiers, never certified FDI numbers.
"""
from pathlib import Path
import hashlib
import json
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components, dijkstra


def load_prediction(directory, scan_path, vertices_mm, faces):
    """Require identical input bytes AND original vertex/face numbering."""
    import open3d as o3d
    directory = Path(directory)
    summary = json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    digest = hashlib.sha256(Path(scan_path).read_bytes()).hexdigest()
    if summary['input_sha256'] != digest:
        raise ValueError('CrossTooth input SHA256 differs from the current scan')
    mesh = o3d.io.read_triangle_mesh(str(directory/'segmentation.ply'))
    if (np.asarray(mesh.vertices).shape != np.shape(vertices_mm) or
            not np.allclose(mesh.vertices, vertices_mm, rtol=0, atol=1e-5) or
            not np.array_equal(mesh.triangles, faces)):
        raise ValueError('CrossTooth original vertex/face numbering differs from the scan')
    with np.load(directory/'prediction.npz', allow_pickle=False) as data:
        labels = data['face_labels'].astype(int)
        probabilities = data['face_probabilities'].copy()
    if (labels.shape != (len(faces),) or probabilities.shape != (len(faces), 19) or
            not np.isfinite(probabilities).all() or np.any(probabilities < 0) or
            not np.allclose(probabilities.sum(axis=1), 1, atol=1e-4) or
            np.any((labels < 0) | (labels > 16))):
        raise ValueError('Invalid CrossTooth face labels/probabilities')
    expected = probabilities.argmax(axis=1); expected[expected >= 17] = 0
    if not np.array_equal(expected, labels):
        raise ValueError('CrossTooth labels are inconsistent with probability argmax')
    return labels, probabilities, summary


def face_neighbours(faces):
    edges = np.sort(faces[:, [[0, 1], [1, 2], [2, 0]]].reshape(-1, 2), axis=1)
    owners = np.repeat(np.arange(len(faces)), 3)
    order = np.lexsort((edges[:, 1], edges[:, 0])); edges, owners = edges[order], owners[order]
    starts = np.r_[0, 1+np.flatnonzero(np.any(edges[1:] != edges[:-1], axis=1))]
    counts = np.diff(np.r_[starts, len(edges)])
    starts = starts[counts == 2]  # no open-rim/nonmanifold adjacency
    return np.column_stack((owners[starts], owners[starts+1]))


def clean_instances(vertices_mm, faces, labels, min_faces=100, min_area_mm2=5.):
    """Keep each class's largest area-connected patch; preserve raw labels separately."""
    v, f, labels = np.asarray(vertices_mm), np.asarray(faces), np.asarray(labels)
    tri = v[f]
    areas = np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1)/2
    neighbours = face_neighbours(f)
    same = neighbours[(labels[neighbours[:, 0]] == labels[neighbours[:, 1]]) &
                      (labels[neighbours[:, 0]] > 0) &
                      (areas[neighbours] > 1e-10).all(axis=1)]
    graph = coo_matrix((np.ones(len(same)), (same[:, 0], same[:, 1])), shape=(len(f), len(f))).tocsr()
    _, components = connected_components(graph, directed=False)
    clean = np.zeros(len(f), int); records = []
    for label in sorted(set(labels)-{0}):
        ids = np.flatnonzero(labels == label)
        component_area = np.bincount(components[ids], weights=areas[ids])
        biggest = component_area.argmax()
        kept = ids[(components[ids] == biggest) & (areas[ids] > 1e-10)]
        accepted = len(kept) >= min_faces and areas[kept].sum() >= min_area_mm2
        if accepted:
            clean[kept] = label
        records.append(dict(tooth_id=int(label), raw_faces=len(ids), kept_faces=len(kept) if accepted else 0,
                            largest_component_area_mm2=float(areas[kept].sum()), accepted=bool(accepted),
                            reason='' if accepted else 'small_predicted_class'))
    return clean, records


def monotone_assignment(probabilities, tooth_ids, point_angles, class_angles):
    """Minimum negative-log likelihood, ordered nonempty contiguous tooth intervals.

No tooth-count prior is imposed: the accepted model classes supply the states.
Angle order is for assignment only; curve order comes from original edges.
"""
    scores = np.asarray(probabilities, float)
    ids = np.asarray(tooth_ids, int)
    n, k = scores.shape
    if (k != len(ids) or n < k or not k or len(set(ids)) != k or
            not np.isfinite(scores).all() or np.any(scores < 0) or
            not np.isfinite(point_angles).all() or not np.isfinite(class_angles).all()):
        raise ValueError('Invalid ordered tooth assignment input')
    order = np.argsort(-np.asarray(point_angles), kind='stable')
    classes = np.argsort(-np.asarray(class_angles), kind='stable')
    norm = scores/np.maximum(scores.sum(axis=1, keepdims=True), 1e-30)
    cost = -np.log(np.maximum(norm[order][:, classes], 1e-8))
    previous = np.full(k, np.inf); previous[0] = cost[0, 0]
    advance = np.zeros((n, k), bool)
    for i in range(1, n):
        shifted = np.r_[np.inf, previous[:-1]]
        advance[i] = shifted < previous
        previous = np.minimum(previous, shifted)+cost[i]
    labels = np.empty(n, int); state = k-1
    for i in range(n-1, -1, -1):
        labels[order[i]] = ids[classes[state]]
        if advance[i, state]:
            state -= 1
    confidence = np.array([norm[i, np.flatnonzero(ids == label)[0]] for i, label in enumerate(labels)])
    return labels, confidence, ids[classes]


def propagate_on_tooth_surface(vertices_mm, faces, clean_labels, tooth_face_mask, *,
                               anchor_faces=None, anchor_labels=None, boundary_band_mm=1.2):
    """Geodesic extension of model cores to the existing tooth/gum interface.

Background, open rims and nonmanifold edges cannot be crossed. An unseeded
component remains unassigned instead of receiving a Euclidean nearby tooth.
"""
    centers = np.asarray(vertices_mm)[faces].mean(axis=1)
    pairs = face_neighbours(faces)
    pairs = pairs[np.asarray(tooth_face_mask)[pairs].all(axis=1)]
    costs = np.linalg.norm(centers[pairs[:, 0]]-centers[pairs[:, 1]], axis=1)
    costs = np.maximum(costs, 1e-10)
    graph = coo_matrix((costs, (pairs[:, 0], pairs[:, 1])), shape=(len(faces), len(faces))).tocsr()
    seed_labels = np.asarray(clean_labels).copy()
    if anchor_faces is not None:
        af, al = np.asarray(anchor_faces, int), np.asarray(anchor_labels, int)
        if (af.shape != al.shape or af.ndim != 1 or not len(af) or
                np.any((af < 0) | (af >= len(faces))) or np.any(al <= 0) or
                not tooth_face_mask[af].all() or boundary_band_mm <= 0):
            raise ValueError('Invalid tooth-side gingival anchors')
        distance = dijkstra(graph, directed=False, indices=np.unique(af), min_only=True,
                            limit=boundary_band_mm)
        seed_labels[np.isfinite(distance)] = 0
        # Shared triangles at an interproximal cut use deterministic majority.
        for face in np.unique(af):
            votes = np.bincount(al[af == face])
            seed_labels[face] = votes.argmax()
    seeds = np.flatnonzero((seed_labels > 0) & tooth_face_mask)
    if not len(seeds):
        raise ValueError('No model tooth cores on the existing tooth-side mesh')
    distance, _, source = dijkstra(graph, directed=False, indices=seeds, min_only=True,
                                  return_predecessors=True)
    assigned = np.zeros(len(faces), int)
    good = np.isfinite(distance) & tooth_face_mask & (source >= 0)
    assigned[good] = seed_labels[source[good]]
    return assigned


def principal_arc(points_mm, edges, assigned, tooth_id, angles):
    """Longest graph-supported diameter path, without bridging fragments or branches.

Double-sweep is exact on trees, a lower-bound heuristic on cyclic graphs.
Any cycles/branches and omitted support are reported explicitly.
"""
    p, e = np.asarray(points_mm), np.asarray(edges, int)
    members = np.flatnonzero(np.asarray(assigned) == tooth_id)
    keep = np.zeros(len(p), bool); keep[members] = True
    e = e[keep[e].all(axis=1)]
    e = np.unique(np.sort(e[e[:, 0] != e[:, 1]], axis=1), axis=0)
    length = np.linalg.norm(p[e[:, 0]]-p[e[:, 1]], axis=1)
    e, length = e[length > 1e-10], length[length > 1e-10]
    graph = coo_matrix((length, (e[:, 0], e[:, 1])), shape=(len(p), len(p))).tocsr()
    graph = graph+graph.T
    _, comp = connected_components(graph, directed=False)
    best, best_length = np.empty(0, int), 0.
    for component in np.unique(comp[members]):
        group = members[comp[members] == component]
        if len(group) < 2:
            continue
        distance = dijkstra(graph, directed=False, indices=int(group[0]))
        a = int(group[np.argmax(distance[group])])
        distance, prev = dijkstra(graph, directed=False, indices=a, return_predecessors=True)
        b = int(group[np.argmax(distance[group])])
        if distance[b] > best_length:
            path = [b]
            while path[-1] != a:
                path.append(int(prev[path[-1]]))
            best, best_length = np.array(path), float(distance[b])
    if len(best) and angles[best[0]] < angles[best[-1]]:
        best = best[::-1]
    total = float(length.sum())
    degrees = np.diff(graph.indptr)[members]
    report = dict(tooth_id=int(tooth_id), assigned_points=len(members), main_arc_points=len(best),
                  main_arc_length_mm=best_length, source_edge_length_mm=total,
                  main_arc_coverage=best_length/total if total else 0.,
                  fragment_count=len(np.unique(comp[members])), branch_points=int((degrees > 2).sum()),
                  graph_has_cycles=bool(len(e) > len(members)-len(np.unique(comp[members]))),
                  omitted_point_indices=np.setdiff1d(members, best).tolist(),
                  gaps_bridged=False)
    return best, report
