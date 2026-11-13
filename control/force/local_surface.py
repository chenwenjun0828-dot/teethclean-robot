"""Connected, inward extruded scan patches. All geometric quantities are mm.

The original triangles are evidence, never a surface-repair input. Convex hulls
are allowed only after comparison with that evidence in both directions.
"""
from collections import defaultdict

import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull


class SurfaceGeometryError(ValueError):
    def __init__(self, patch, scale):
        self.patch, self.scale = patch, scale
        super().__init__(f'Local patch failed at {scale:g} mm: {patch["maximum_error_mm"]:g} mm; source faces {patch["source_face_ids"].tolist()}')


def triangle_samples(triangles, spacing=.2):
    points = []
    for triangle in triangles:
        n = max(2, int(np.ceil(np.linalg.norm(triangle-np.roll(triangle, 1, axis=0), axis=1).max()/spacing)))
        # Include vertices and edges, as well as the face interior.
        uv = np.array([(i/n, j/n) for i in range(n+1) for j in range(n+1-i)])
        points.append(triangle[0]+uv[:, :1]*(triangle[1]-triangle[0])+uv[:, 1:]*(triangle[2]-triangle[0]))
    return np.concatenate(points) if points else np.empty((0, 3))


def scene(vertices, faces):
    result = o3d.t.geometry.RaycastingScene(nthreads=1)
    result.add_triangles(o3d.core.Tensor(np.asarray(vertices, np.float32)),
                         o3d.core.Tensor(np.asarray(faces, np.uint32)))
    return result


def distance_to(surface, points):
    return surface.compute_distance(o3d.core.Tensor(np.asarray(points, np.float32))).numpy()


def adjacency(faces, labels):
    edges = defaultdict(list)
    for i, face in enumerate(faces):
        for a, b in zip(face, np.roll(face, 1)):
            edges[tuple(sorted((int(a), int(b))))].append(i)
    neighbors = [set() for _ in faces]
    for ids in edges.values():
        # A non-manifold edge is a boundary, never a shortcut across a gap.
        if len(ids) == 2 and labels[ids[0]] == labels[ids[1]]:
            a, b = ids
            neighbors[a].add(b); neighbors[b].add(a)
    return neighbors


def connected_groups(vertices, faces, labels, ids, scale_mm, neighbors=None):
    neighbors = neighbors or adjacency(faces, labels)
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-30)
    pending = set(map(int, ids))
    while pending:
        seed = min(pending); pending.remove(seed)
        group = [seed]; todo = sorted(neighbors[seed] & pending)
        low, high = triangles[seed].min(0), triangles[seed].max(0)
        while todo:
            i = todo.pop()
            if i not in pending: continue
            lo, hi = np.minimum(low, triangles[i].min(0)), np.maximum(high, triangles[i].max(0))
            if np.linalg.norm(hi-lo) > scale_mm or normals[i]@normals[seed] < np.cos(np.deg2rad(25)):
                continue
            # Adjacency alone is insufficient: an L-shaped connected patch has
            # a convex hull that fills its missing corner. Keep its projected
            # footprint convex while growing, before doing any thickening.
            candidate = triangles[group+[i]]
            n = normals[seed]
            u = np.cross(n,np.eye(3)[np.argmin(abs(n))]); u /= np.linalg.norm(u)
            basis = np.array([u,np.cross(n,u)]).T
            projected = candidate@basis
            area = np.abs(np.cross(projected[:,1]-projected[:,0],projected[:,2]-projected[:,0])).sum()/2
            hull_area = ConvexHull(projected.reshape(-1,2)).volume
            if hull_area > area+max(1e-10,area*1e-6): continue
            pending.remove(i); group.append(i); low, high = lo, hi
            todo.extend(sorted(neighbors[i] & pending))
        yield np.array(sorted(group), dtype=int)


def convex_patch(vertices, faces, ids, thickness_mm=.5):
    original_ids, inverse = np.unique(faces[ids], return_inverse=True)
    top = vertices[original_ids].copy()
    original_faces = inverse.reshape(-1, 3)
    t = top[original_faces]
    cross = np.cross(t[:, 1]-t[:, 0], t[:, 2]-t[:, 0])
    normal = cross.sum(0); normal /= np.linalg.norm(normal)
    points = np.concatenate([top, top-thickness_mm*normal])
    hull = ConvexHull(points)
    hull_faces = hull.simplices.copy()
    # scipy simplices have no winding guarantee. MuJoCo needs outward winding.
    for i, face in enumerate(hull_faces):
        a, b, c = points[face]
        if np.cross(b-a, c-a)@hull.equations[i, :3] < 0: hull_faces[i] = face[::-1]
    work = np.all(hull_faces < len(top), axis=1) & (hull.equations[:, :3]@normal > .2)
    bottom = np.all(hull_faces >= len(top), axis=1)
    kinds = np.where(work, 'working', np.where(bottom, 'bottom', 'side'))
    if not work.any(): raise ValueError('Patch has no outward working surface')
    a = triangle_samples(t)
    b = triangle_samples(points[hull_faces[work]])
    forward = distance_to(scene(points, hull_faces[work]), a)
    reverse = distance_to(scene(top, original_faces), b)
    return dict(vertices=points, faces=hull_faces, face_kinds=kinds,
                original_vertex_ids=original_ids, original_faces=original_faces,
                normal=normal, source_face_ids=ids.copy(),
                forward_error_mm=float(forward.max()), reverse_error_mm=float(reverse.max()),
                maximum_error_mm=float(max(forward.max(), reverse.max())),
                samples=len(a)+len(b))


def generate(vertices, faces, labels, ids, initial_scale_mm=1., minimum_scale_mm=.25,
             tolerance_mm=.05, thickness_mm=.5):
    """Refine connected patches; a single triangle remains an exact prism."""
    vertices, faces, labels = np.asarray(vertices), np.asarray(faces), np.asarray(labels)
    neighbors = adjacency(faces, labels)
    result = []

    def refine(group, scale):
        patch = convex_patch(vertices, faces, group, thickness_mm)
        if patch['maximum_error_mm'] <= tolerance_mm:
            patch['scale_mm'] = scale
            patch['tooth'] = int(labels[group[0]])
            result.append(patch); return
        if scale <= minimum_scale_mm+1e-12 or len(group) == 1:
            raise SurfaceGeometryError(patch, scale)
        smaller = max(minimum_scale_mm, scale/2)
        for child in connected_groups(vertices, faces, labels, group, smaller, neighbors): refine(child, smaller)

    for group in connected_groups(vertices, faces, labels, ids, initial_scale_mm, neighbors): refine(group, initial_scale_mm)
    return result


def union_check(vertices, faces, patches, face_ids, approach_normals=None):
    """First-hit comparisons with the original scan, including oblique rays.

    Only exposed working triangles count as surface hits. Side/bottom first
    hits explicitly fail. The caller supplies the operational band, with the
    extra coverage halo reserved for oblique approach and the actual tip.
    """
    vertices, faces = np.asarray(vertices), np.asarray(faces)
    original = scene(vertices, faces)
    pv, pf, kinds, offset = [], [], [], 0
    for patch in patches:
        pv.append(patch['vertices']); pf.append(patch['faces']+offset)
        kinds.extend(patch['face_kinds']); offset += len(patch['vertices'])
    layer = scene(np.concatenate(pv), np.concatenate(pf))
    kinds = np.asarray(kinds)
    tri = vertices[faces[face_ids]]
    points = tri.mean(1)
    normals = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-30)
    approaches = normals if approach_normals is None else np.asarray(approach_normals)
    approaches = np.where((approaches*normals).sum(1)[:, None] < 0, -approaches, approaches)
    # Explicit seam/boundary probes, slightly inside each incident original
    # triangle to avoid undefined edge ownership in float32 ray casting.
    # Both sides of each triangle edge are represented by incident triangles.
    edge_points = np.stack([(tri[:,a]+tri[:,b])/2 for a,b in ((0,1),(1,2),(2,0))],axis=1)
    edge_points = edge_points*.98+points[:,None]*.02
    count = len(points)
    points = np.concatenate([points,edge_points.reshape(-1,3)])
    normals = np.concatenate([normals,np.repeat(normals,3,axis=0)])
    approaches = np.concatenate([approaches,np.repeat(approaches,3,axis=0)])
    sample_faces = np.r_[face_ids,np.repeat(face_ids,3)]
    tangent = np.cross(approaches, np.eye(3)[np.argmin(abs(approaches), axis=1)])
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    bitangent = np.cross(approaches, tangent)
    directions = [normals, approaches]
    for axis in (tangent, bitangent):
        for sign in (-1, 1):
            d = approaches+sign*np.tan(np.deg2rad(15))*axis
            directions.append(d/np.linalg.norm(d, axis=1)[:, None])
    records = []
    for k, direction in enumerate(directions):
        rays = o3d.core.Tensor(np.c_[points+direction*.75, -direction].astype(np.float32))
        oh, lh = original.cast_rays(rays), layer.cast_rays(rays)
        a, b = oh['t_hit'].numpy(), lh['t_hit'].numpy()
        hit_ids = lh['primitive_ids'].numpy().astype(np.int64)
        finite = np.isfinite(a) & np.isfinite(b)
        work = np.zeros(len(points), dtype=bool)
        work[finite] = kinds[hit_ids[finite]] == 'working'
        error = np.full(len(a),np.inf)
        error[finite] = abs(a[finite]-b[finite])
        for i in range(len(points)):
            records.append(dict(source_face=int(sample_faces[i]), direction=k,
                sample_kind='face' if i<count else 'seam_or_boundary',
                x_mm=float(points[i, 0]), y_mm=float(points[i, 1]), z_mm=float(points[i, 2]),
                error_mm=float(error[i]) if np.isfinite(error[i]) else None,
                original_hit_mm=float(a[i]) if np.isfinite(a[i]) else None,
                layer_hit_mm=float(b[i]) if np.isfinite(b[i]) else None,
                layer_facet=int(hit_ids[i]) if finite[i] else None,
                hit_kind=str(kinds[hit_ids[i]]) if finite[i] else 'missing',
                approach_x=float(direction[i,0]),approach_y=float(direction[i,1]),approach_z=float(direction[i,2]),
                working_first_hit=bool(work[i])))
    valid = all(r['working_first_hit'] and r['error_mm'] is not None and r['error_mm'] <= .1 for r in records)
    finite_errors = [r['error_mm'] for r in records if r['error_mm'] is not None]
    return dict(samples=len(records), geometry_passed=valid,
                maximum_error_mm=max(finite_errors) if finite_errors else None,
                internal_or_missing_first_hits=sum(not r['working_first_hit'] for r in records)), records


def gap_check(vertices, faces, labels, patches, band_ids):
    """Probe 0.05 mm outside open/label boundaries for newly filled gaps."""
    vertices, faces, labels = np.asarray(vertices), np.asarray(faces), np.asarray(labels)
    band=set(map(int,band_ids)); edges=defaultdict(list)
    for i,face in enumerate(faces):
        for a,b in zip(face,np.roll(face,1)): edges[tuple(sorted((int(a),int(b))))].append(i)
    points, directions, source_ids=[],[],[]
    for (a,b),ids in edges.items():
        if len(ids)==2 and labels[ids[0]]==labels[ids[1]]: continue
        for i in ids:
            if i not in band: continue
            t=vertices[faces[i]]; normal=np.cross(t[1]-t[0],t[2]-t[0]); normal/=np.linalg.norm(normal)
            edge=(vertices[a]+vertices[b])/2
            outward=edge-t.mean(0); outward-=normal*(outward@normal)
            if np.linalg.norm(outward)<1e-12: continue
            points.append(edge+outward/np.linalg.norm(outward)*.05)
            directions.append(normal); source_ids.append(i)
    if not points: return dict(samples=0,geometry_passed=True,scope='No open or tooth-label boundary in band'),[]
    points,directions=np.asarray(points),np.asarray(directions)
    original=scene(vertices,faces)
    pv,pf,kinds,offset=[],[],[],0
    for p in patches:
        pv.append(p['vertices']); pf.append(p['faces']+offset); kinds.extend(p['face_kinds']); offset+=len(p['vertices'])
    layer=scene(np.concatenate(pv),np.concatenate(pf)); kinds=np.asarray(kinds)
    rays=o3d.core.Tensor(np.c_[points+.75*directions,-directions].astype(np.float32))
    a,b=original.cast_rays(rays),layer.cast_rays(rays)
    at,bt=a['t_hit'].numpy(),b['t_hit'].numpy(); ids=b['primitive_ids'].numpy()
    records=[]
    for i in range(len(points)):
        original_present=np.isfinite(at[i]) and at[i]<=1.5
        layer_present=np.isfinite(bt[i]) and bt[i]<=1.5
        kind=str(kinds[ids[i]]) if layer_present else 'missing'
        error=float(abs(at[i]-bt[i])) if original_present and layer_present else None
        passed=(not original_present and not layer_present) or (original_present and layer_present and kind=='working' and error<=.1)
        records.append(dict(source_face=source_ids[i],sample_kind='outside_open_or_label_edge',
            x_mm=float(points[i,0]),y_mm=float(points[i,1]),z_mm=float(points[i,2]),
            original_hit_mm=float(at[i]) if original_present else None,
            layer_hit_mm=float(bt[i]) if layer_present else None,error_mm=error,hit_kind=kind,geometry_passed=bool(passed)))
    return dict(samples=len(records),geometry_passed=all(r['geometry_passed'] for r in records),
                failed_samples=sum(not r['geometry_passed'] for r in records),probe_outside_edge_mm=.05,
                ray_depth_interval_mm=[0.,1.5]),records
