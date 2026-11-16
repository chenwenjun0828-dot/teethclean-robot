"""Connected, inward extruded scan patches. All geometric quantities are mm.

The original triangles are evidence, never a surface-repair input. Convex hulls
are allowed only after comparison with that evidence in both directions.
"""
from collections import defaultdict

import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull, cKDTree
from scipy.spatial import QhullError


class SurfaceGeometryError(ValueError):
    def __init__(self, patch, scale):
        self.patch, self.scale = patch, scale
        super().__init__(f'Local patch failed at {scale:g} mm: {patch["maximum_error_mm"]:g} mm; source faces {patch["source_face_ids"].tolist()}')


class SurfaceTopologyError(ValueError):
    def __init__(self, ids):
        self.source_face_ids = np.asarray(ids, int)
        super().__init__(f'Closed source component has zero thickness/volume: faces {ids[:8]}')


class SurfaceWallError(ValueError):
    def __init__(self, ids):
        self.source_face_ids=np.asarray(ids,int)
        super().__init__(f'Cannot establish tool wall thickness: faces {self.source_face_ids[:8].tolist()}')


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


def connected_components(faces, labels):
    """Connected groups of faces, never crossing a label boundary."""
    comp = _component_labels(faces, labels)[1]
    order = np.argsort(comp, kind='stable')
    sorted_comp = comp[order]
    boundaries = np.flatnonzero(np.diff(sorted_comp))+1
    groups = np.split(order, boundaries)
    return [np.sort(g) for g in groups if len(g)]


def _component_labels(faces, labels):
    """A label per face: connected component of the same-label adjacency.

    The adjacency is built once with a sparse graph, so the cost is linear in
    the number of faces.  Recomputing it per query made the geometry gate
    quadratic in the size of the scan.
    """
    import scipy.sparse as sparse
    from scipy.sparse.csgraph import connected_components
    faces = np.asarray(faces)
    labels = np.asarray(labels)
    counts = defaultdict(list)
    for i, face in enumerate(faces):
        for a, b in zip(face, np.roll(face, 1)):
            counts[tuple(sorted((int(a), int(b))))].append(i)
    rows, cols = [], []
    for group in counts.values():
        if len(group) == 2 and labels[group[0]] == labels[group[1]]:
            rows += group; cols += group[::-1]
    data = np.ones(len(rows))
    graph = sparse.coo_matrix((data, (rows, cols)), shape=(len(faces), len(faces))).tocsr()
    return connected_components(graph, directed=False)


def outward_normals(vertices, faces, ids, labels=None, escape_mm=4.):
    """Orient normals across manifold edges without changing source triangles.

    Closed components use positive volume to select the exterior. Open scan
    components retain their seed winding: their volume is not an orientation
    certificate and changes under translation. Non-manifold edges are barriers.
    """
    vertices, faces = np.asarray(vertices), np.asarray(faces)
    ids = np.asarray(ids, int)
    t = vertices[faces]
    normals = np.cross(t[:, 1]-t[:, 0], t[:, 2]-t[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1), 1e-30)[:, None]
    if labels is None:
        labels = np.zeros(len(faces), int)
    edges = defaultdict(list)
    for i, face in enumerate(faces):
        for a, b in zip(face, np.roll(face, -1)):
            edges[tuple(sorted((int(a), int(b))))].append((i, 1 if a < b else -1))
    neighbors = [[] for _ in faces]
    for edge in edges.values():
        if len(edge) != 2: continue
        (a, da), (b, db) = edge
        if labels[a] != labels[b]: continue
        parity = -da*db
        neighbors[a].append((b, parity)); neighbors[b].append((a, parity))
    signs = np.zeros(len(faces), int)
    for seed in range(len(faces)):
        if signs[seed]: continue
        signs[seed] = 1; todo = [seed]; group = []
        while todo:
            a = todo.pop(); group.append(a)
            for b, parity in neighbors[a]:
                expected = signs[a]*parity
                if signs[b] and signs[b] != expected:
                    raise ValueError('Source surface is not consistently orientable')
                if not signs[b]: signs[b] = expected; todo.append(b)
        if _is_closed(faces, group):
            tri = t[group]-t[seed, 0]
            volume = np.sum(signs[group]*np.einsum('ij,ij->i', tri[:, 0],
                                                  np.cross(tri[:, 1], tri[:, 2])))/6.
            if abs(volume) < 1e-12:
                if np.intersect1d(group, ids).size:
                    raise SurfaceTopologyError(group)
            if volume < 0: signs[group] *= -1
    return normals[ids]*signs[ids, None]


def _is_closed(faces, group):
    """True when every edge of the group is shared by exactly two faces."""
    counts = defaultdict(int)
    for i in group:
        for a, b in zip(faces[i], np.roll(faces[i], 1)):
            counts[tuple(sorted((int(a), int(b))))] += 1
    return all(count == 2 for count in counts.values())


def vertex_extrusion_field(vertices, faces, ids, labels=None, iterations=20, smoothing=.7,
                           normals=None):
    """A per-vertex extrusion direction, consistent across shared edges.

    Two patches that share an edge must extrude that edge to the SAME point,
    otherwise their side walls are two different walls and the gap between
    them is a groove that a grazing approach ray falls into.  A field defined
    on the vertices satisfies this by construction: every patch that touches
    vertex ``v`` moves ``v`` to ``v - thickness * field[v]``.

    The field starts as the sum of the incident outward face normals.  It is
    then relaxed over the shared edges so that neighbouring vertices agree
    closely and the shared wall quad stays nearly planar.

    The relaxation is ANCHORED to the initial field.  A plain diffusion
    would converge to the harmonic solution of the mesh, which for a closed
    surface is the zero field: the outward normals cancel and the extrusion
    direction stops being outward at all.
    """
    vertices, faces = np.asarray(vertices), np.asarray(faces)
    ids = np.asarray(ids, int)
    outward = outward_normals(vertices, faces, ids, labels) if normals is None else normals
    anchor = np.zeros_like(vertices, dtype=float)
    for position, face in enumerate(faces[ids]):
        for k in face:
            anchor[k] += outward[position]
    norm = np.linalg.norm(anchor, axis=1)
    usable = norm > 1e-9
    anchor[usable] /= norm[usable, None]
    anchor[~usable] = np.array([0., 0., 1.])
    field = anchor.copy()
    edges = set()
    for face in faces[ids]:
        for a, b in zip(face, np.roll(face, 1)):
            edges.add(tuple(sorted((int(a), int(b)))))
    edges = np.array(sorted(edges), dtype=int)
    for _ in range(iterations):
        acc = np.zeros_like(field); cnt = np.zeros(len(vertices))
        np.add.at(acc, edges[:, 0], field[edges[:, 1]])
        np.add.at(acc, edges[:, 1], field[edges[:, 0]])
        np.add.at(cnt, edges[:, 0], 1); np.add.at(cnt, edges[:, 1], 1)
        shared = cnt > 0
        updated = field.copy()
        # The anchor keeps the field from drifting to the harmonic solution.
        updated[shared] = ((1-smoothing)*anchor[shared]
                            + smoothing*(acc[shared]/cnt[shared, None]))
        norm = np.linalg.norm(updated, axis=1)
        usable = norm > 1e-9
        updated[usable] /= norm[usable, None]
        updated[~usable] = anchor[~usable]
        field = updated
    return field


def vertex_depth_field(vertices, faces, ids, face_depth):
    """Per-vertex extrusion depth, consistent across shared edges.

    The flush property of the seam depends on the bottom points being shared,
    which means the DEPTH must also be a function of the vertex and not of
    the patch: a patch that is thin because of a local wall must not push its
    neighbour's shared edge somewhere else.

    A vertex takes the MINIMUM depth of the faces that touch it.  Taking the
    maximum would let a vertex next to a thin wall be extruded by the depth
    of a thick neighbour, and the thin wall would then be pierced.
    """
    vertices, faces = np.asarray(vertices), np.asarray(faces)
    ids = np.asarray(ids, int)
    face_depth = np.asarray(face_depth, float)
    depth = np.full(len(vertices), np.inf)
    for position, face in enumerate(faces[ids]):
        for k in face:
            depth[k] = min(depth[k], float(face_depth[position]))
    depth[~np.isfinite(depth)] = float(face_depth.max()) if len(face_depth) else 0.
    return depth


def local_wall_thickness(vertices, faces, ids, normals=None, limit_mm=.5):
    """Distance from each face to the opposing wall, measured perpendicular.

    A thin shell has an opposing wall a fraction of a millimetre away.  A
    fixed extrusion would punch through it, so the tool tip must be thickened
    along its real wall, not along a nominal value.

    The wall is measured by casting a ray inward from just below the face
    and adding the small offset, so the result is the perpendicular
    thickness rather than a diagonal between two face centroids.
    """
    vertices, faces = np.asarray(vertices), np.asarray(faces)
    ids = np.asarray(ids, int)
    if normals is None:
        normals = outward_normals(vertices, faces, ids)
    centers = vertices[faces].mean(1)
    whole = scene(vertices, faces)
    offset = 1e-5
    origins = centers[ids]-normals*offset
    rays = o3d.core.Tensor(np.c_[origins, -normals].astype(np.float32))
    hit = whole.cast_rays(rays)['t_hit'].numpy()
    if not np.isfinite(hit).all():
        raise SurfaceWallError(ids[~np.isfinite(hit)])
    return np.minimum(limit_mm, .4*(hit+offset))


def extrusion_fields(vertices, faces, labels, ids, thickness_mm=.05,
                     wall_thickness_mm=None, flush=True):
    """Shared build/reload recipe, including a normal indexed by source face."""
    normals = outward_normals(vertices, faces, np.arange(len(faces)), labels)
    field = (vertex_extrusion_field(vertices, faces, ids, labels, normals=normals[ids])
             if flush else None)
    if wall_thickness_mm is None:
        wall_thickness_mm = (np.full(len(ids), thickness_mm) if flush else
            local_wall_thickness(vertices, faces, ids, normals[ids], limit_mm=thickness_mm))
    depth = vertex_depth_field(vertices, faces, ids, np.minimum(wall_thickness_mm, thickness_mm))
    return dict(field=field, depth_field=depth, normals=normals)


def _exposed_cap_facets(hull_faces, top, hull_equations, tolerance=1e-7):
    """The facets of a prism that bound the original top vertices.

    The exposed cap is the set of prism facets whose plane SUPPORTS the top
    point set: the whole top set lies on the inner side of the plane.  A
    facet qualifies when it uses only top vertices and every top vertex is
    on the inner side of its plane.

    Comparing vertex sets against the top set's own hull does not work: the
    prism and the top set triangulate the same cap differently, so the
    vertex sets differ even though the facets coincide.  Testing the
    supporting plane is triangulation independent.
    """
    n_top = len(top)
    only_top = np.all(hull_faces < n_top, axis=1)
    work = np.zeros(len(hull_faces), bool)
    for i in np.flatnonzero(only_top):
        # scipy's plane equation is inward: the hull interior is
        # plane[:3].x + plane[3] <= 0.
        plane = hull_equations[i]
        values = top @ plane[:3] + plane[3]
        if float(values.max()) <= tolerance:
            work[i] = True
    return work


def convex_patch(vertices, faces, ids, thickness_mm=.05, normal=None, outward=None,
                 field=None, depth_field=None):
    """One convex prism: the patch's vertices on top, extruded inward.

    The cap is the convex hull of the original vertices, so the exposed
    surface is a chord of the scanned surface.  The extrusion is strictly
    inward, so the prism never protrudes above the original surface.

    With ``field`` the extrusion direction is read per VERTEX instead of per
    patch.  Patches that share an edge then share the edge's bottom points,
    so their side walls are coincident and the seam between them is flush:
    a grazing approach ray cannot fall into a groove.
    """
    original_ids, inverse = np.unique(faces[ids], return_inverse=True)
    top = vertices[original_ids].copy()
    original_faces = inverse.reshape(-1, 3)
    t = top[original_faces]
    cross = np.cross(t[:, 1]-t[:, 0], t[:, 2]-t[:, 0])
    # The patch normal still decides which way the prism is extruded and
    # which hull facets face free space.
    normal = cross.sum(0) if normal is None else np.asarray(normal, float)
    if np.linalg.norm(normal) < 1e-30:
        normal = np.array([0., 0., 1.])
    normal = normal/np.linalg.norm(normal)
    if outward is not None and float(normal @ outward.sum(0)) < 0:
        normal = -normal
    if field is None:
        # Without a per-vertex field the prism is extruded along the patch
        # normal, but the depth may still vary per vertex: a thin shell must
        # not be thickened past its own wall.
        if depth_field is None:
            depth = np.full(len(original_ids), thickness_mm)
        else:
            depth = depth_field[original_ids]
        points = np.concatenate([top, top-depth[:, None]*normal])
        actual_depth = float(depth.max()) if len(depth) else float(thickness_mm)
    else:
        depth = (depth_field[original_ids] if depth_field is not None
                 else np.full(len(original_ids), thickness_mm))
        points = np.concatenate([top, top-depth[:, None]*field[original_ids]])
        actual_depth = float(depth.max()) if len(depth) else float(thickness_mm)
    if thickness_mm <= 0 and (field is None or not np.any(actual_depth > 0)):
        # A zero-depth prism has no volume, so its hull would collapse.  The
        # patch is then simply the original triangles.
        return dict(vertices=top.copy(), faces=original_faces.copy(),
                    face_kinds=np.array(['working']*len(original_faces)),
                    original_vertex_ids=original_ids, original_faces=original_faces,
                    normal=normal, source_face_ids=ids.copy(), thickness_mm=0.,
                    forward_error_mm=0., reverse_error_mm=0., maximum_error_mm=0.,
                    samples=0)
    hull = ConvexHull(points, qhull_options='QJ')
    hull_faces = hull.simplices.copy()
    # scipy simplices have no winding guarantee. MuJoCo needs outward winding.
    for i, face in enumerate(hull_faces):
        a, b, c = points[face]
        if np.cross(b-a, c-a)@hull.equations[i, :3] < 0: hull_faces[i] = face[::-1]
    bottom = np.all(hull_faces >= len(top), axis=1)
    # The exposed working surface is the top set's own convex hull: the
    # facets of the extruded prism that also bound the original vertices.
    # Testing against the patch normal instead would leave holes in the
    # cap, and a ray arriving at such a hole hits the extrusion floor
    # rather than the surface.
    work = _exposed_cap_facets(hull_faces, top, hull.equations)
    kinds = np.where(work, 'working', np.where(bottom, 'bottom', 'side'))
    if not work.any(): raise ValueError('Patch has no outward working surface')
    a = triangle_samples(t)
    b = triangle_samples(points[hull_faces[work]])
    forward = distance_to(scene(points, hull_faces[work]), a)
    reverse = distance_to(scene(top, original_faces), b)
    return dict(vertices=points, faces=hull_faces, face_kinds=kinds,
                original_vertex_ids=original_ids, original_faces=original_faces,
                normal=normal, source_face_ids=ids.copy(),
                thickness_mm=actual_depth,
                forward_error_mm=float(forward.max()), reverse_error_mm=float(reverse.max()),
                maximum_error_mm=float(max(forward.max(), reverse.max())),
                samples=len(a)+len(b))


def generate(vertices, faces, labels, ids, initial_scale_mm=1., minimum_scale_mm=.25,
             tolerance_mm=.05, thickness_mm=.05, wall_thickness_mm=None, flush=True,
             prepared=None):
    """Refine connected patches; a single triangle remains an exact prism.

    ``wall_thickness_mm`` overrides the extrusion depth per patch, so a thin
    tool shell is never thickened past its own wall.

    ``flush`` extrudes along a per-vertex field, so patches that share an
    edge also share that edge's bottom points.  Their side walls are then
    coincident and the seam between them carries no groove for a grazing
    approach ray to fall into.

    A THIN SHELL cannot use a per-vertex field: its outer and inner walls
    carry opposite normals, so the incident normals cancel and the field
    stops pointing outward.  Such a role must be built with ``flush=False``,
    which extrudes along the patch normal instead, with the depth still
    bounded by the local wall.
    """
    vertices, faces, labels = np.asarray(vertices), np.asarray(faces), np.asarray(labels)
    neighbors = adjacency(faces, labels)
    ids = np.asarray(ids, int)
    prepared = prepared or extrusion_fields(vertices, faces, labels, ids, thickness_mm,
                                            wall_thickness_mm, flush)
    field, depth_field = prepared['field'], prepared['depth_field']
    result = []

    def refine(group, scale):
        patch = convex_patch(vertices, faces, group, thickness_mm, outward=prepared['normals'][group],
                             field=field, depth_field=depth_field)
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


CONTACT_POLICY = dict(version=3, distance_tolerance_mm=.1, normal_tolerance_deg=None,
                      tool_wall_fraction=.4)


def filtered_tool_faces(vertices, faces, tip):
    """Remove only the known, isolated, oppositely wound zero-volume tip pair.

    The original STL remains the independent collision reference. The pair's
    distance to the retained surface is measured again for every build/load.
    """
    faces = np.asarray(faces)
    pair = np.array([2660, 2889])
    if (len(faces) <= pair.max() or set(faces[pair[0]]) != set(faces[pair[1]])
            or np.dot(np.cross(*(vertices[faces[pair[0],1:]]-vertices[faces[pair[0],0]])),
                      np.cross(*(vertices[faces[pair[1],1:]]-vertices[faces[pair[1],0]]))) >= 0):
        raise ValueError('Expected opposite tool-tip face pair changed')
    if np.linalg.norm(vertices[faces[pair[0]]].mean(0)-tip) > .05:
        raise ValueError('Opposite face pair is no longer at the tool tip')
    # Face 2392 has no measurable inward wall, but its entire small surface
    # lies within 0.00011 mm of adjacent original faces. Retain the original
    # mesh for independent collision checks and omit this redundant proxy face.
    if len(faces)<=2392: raise ValueError('Tool tip source face changed')
    removed=np.array([2392,2660,2889])
    keep = np.ones(len(faces), bool); keep[removed] = False
    error = float(distance_to(scene(vertices, faces[keep]),
        triangle_samples(vertices[faces[removed]], spacing=.002)).max())
    if error > .0015: raise ValueError('Omitting redundant tip faces changes the surface beyond 0.0015 mm')
    return faces[keep], removed, error


def contact_metrics(original, points, contact_normals, source_normals,
                    tolerance_mm=.1, normal_tolerance_deg=None, reference=None):
    """Check source position and retain the contact angle as diagnostic evidence."""
    points = np.asarray(points, float).reshape(-1, 3)
    contact_normals = np.asarray(contact_normals, float).reshape(-1, 3)
    if reference is None:
        nearest = original.compute_closest_points(o3d.core.Tensor(points.astype(np.float32)))
        reference = nearest['points'].numpy(), nearest['primitive_ids'].numpy().astype(int)
    nearest_points, ids = reference
    ids = np.asarray(ids,int)
    delta = points-nearest_points
    normal = np.asarray(source_normals)[ids]
    lengths = np.linalg.norm(contact_normals, axis=1)
    unit = contact_normals/np.maximum(lengths[:, None], 1e-30)
    angle = np.degrees(np.arccos(np.clip(np.einsum('ij,ij->i', unit, normal), -1., 1.)))
    distance = np.linalg.norm(delta, axis=1)
    signed = np.einsum('ij,ij->i', delta, normal)
    angle_ok = np.ones(len(points),bool) if normal_tolerance_deg is None else angle <= normal_tolerance_deg
    passed = ((distance <= tolerance_mm) & angle_ok
              & (lengths > 1e-9) & np.isfinite(points).all(1)
              & (np.linalg.norm(normal, axis=1) > .9))
    return dict(source_faces=ids, distance_mm=distance, signed_distance_mm=signed,
                normal_error_deg=angle, geometry_passed=passed)


def closest_on_triangles(points, triangles):
    """Float64 closest points for a small patch's source triangles.

    Float32 mesh queries can choose a perpendicular neighbour near a skinny
    tool triangle, producing a spurious 90 degree contact-normal error.
    """
    points, triangles = np.asarray(points,float), np.asarray(triangles,float)
    a,b,c = triangles[:,0],triangles[:,1],triangles[:,2]
    normal = np.cross(b-a,c-a)
    n2 = np.einsum('ij,ij->i',normal,normal)
    if np.any(n2 <= 1e-30): raise ValueError('Degenerate source contact triangle')
    query = points[:,None,:]
    projected = query-(np.sum((query-a)*normal,axis=2)/n2)[:,:,None]*normal
    inside = np.ones(projected.shape[:2],bool)
    for start,end in ((a,b),(b,c),(c,a)):
        inside &= np.sum(np.cross(end-start,projected-start)*normal,axis=2)>=-1e-12*n2
    best = projected.copy()
    distance2 = np.where(inside,np.sum((query-projected)**2,axis=2),np.inf)
    for start,end in ((a,b),(b,c),(c,a)):
        edge = end-start
        t = np.clip(np.sum((query-start)*edge,axis=2)/np.sum(edge*edge,axis=1),0.,1.)
        candidate = start+t[:,:,None]*edge
        d2 = np.sum((query-candidate)**2,axis=2)
        take = d2<distance2
        best[take] = candidate[take]; distance2 = np.minimum(distance2,d2)
    ids = np.argmin(distance2,axis=1)
    return best[np.arange(len(points)),ids], ids


def union_check(vertices, faces, patches, face_ids, approach_normals=None, band_ids=None,
                tolerance_mm=.1, standoff_mm=3., source_normals=None):
    """Compare first-hit positions and report normals; never certify missing coverage.

    Occlusion is distinct from an origin behind the reference surface. Shorter
    probes are tried when a distant origin cannot see the intended sample.
    Open scans use local sidedness, not an unsupported solid-occupancy claim.
    """
    vertices, faces = np.asarray(vertices), np.asarray(faces)
    face_ids = np.asarray(face_ids, int)
    if not len(face_ids) or not patches:
        return dict(samples=0, valid_samples=0, geometry_passed=False,
                    maximum_error_mm=None, invalid_probes=0, failed_samples=0,
                    uncovered_normal_faces=face_ids.tolist()), []
    original = scene(vertices, faces)
    pv, pf, kinds, offset = [], [], [], 0
    for patch in patches:
        pv.append(patch['vertices']); pf.append(patch['faces']+offset)
        kinds.extend(patch['face_kinds']); offset += len(patch['vertices'])
    layer = scene(np.concatenate(pv), np.concatenate(pf))
    kinds = np.asarray(kinds)
    if source_normals is None:
        source_normals = outward_normals(vertices, faces, np.arange(len(faces)))
    tri = vertices[faces[face_ids]]
    centers = tri.mean(1)
    outward = source_normals[face_ids]
    approaches = outward if approach_normals is None else np.asarray(approach_normals, float)
    if approaches.shape != outward.shape or not np.isfinite(approaches).all():
        raise ValueError('Approach normals must be finite vectors for every QA face')
    if np.any(np.linalg.norm(approaches, axis=1) < 1e-9):
        raise ValueError('Approach normal has zero length')
    approaches = approaches/np.linalg.norm(approaches, axis=1)[:, None]
    edges = np.stack([(tri[:,a]+tri[:,b])/2 for a,b in ((0,1),(1,2),(2,0))], axis=1)
    points = np.concatenate([centers, (edges*.98+centers[:,None]*.02).reshape(-1,3)])
    outward = np.concatenate([outward, np.repeat(outward,3,axis=0)])
    approaches = np.concatenate([approaches, np.repeat(approaches,3,axis=0)])
    sample_faces = np.r_[face_ids, np.repeat(face_ids,3)]
    tangent = np.cross(approaches, np.eye(3)[np.argmin(abs(approaches), axis=1)])
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    bitangent = np.cross(approaches, tangent)
    directions = [outward, approaches]
    for axis in (tangent, bitangent):
        for sign in (-1,1):
            d = approaches+sign*np.tan(np.deg2rad(15))*axis
            directions.append(d/np.linalg.norm(d,axis=1)[:,None])
    band_mask = np.zeros(len(faces), bool)
    band_mask[np.asarray(face_ids if band_ids is None else band_ids, int)] = True
    records = []; covered = set()
    for k, direction in enumerate(directions):
        usable = np.einsum('ij,ij->i', direction, outward) > 1e-6
        origins = points+direction*standoff_mm
        stands = np.full(len(points), standoff_mm)
        reasons = np.full(len(points), 'approach_into_surface', dtype=object)
        valid = np.zeros(len(points), bool)
        # Only failed probes are retried. No source geometry is changed.
        for stand in dict.fromkeys([standoff_mm, min(standoff_mm,.75),
                                    min(standoff_mm,.25), min(standoff_mm,.05)]):
            retry = np.flatnonzero(usable & ~valid)
            if not len(retry): break
            candidate = points[retry]+direction[retry]*stand
            rays = o3d.core.Tensor(np.c_[candidate,-direction[retry]].astype(np.float32))
            hit = original.cast_rays(rays)
            travel = hit['t_hit'].numpy()
            ref = np.clip(hit['primitive_ids'].numpy().astype(np.int64),0,len(faces)-1)
            nearest = original.compute_closest_points(o3d.core.Tensor(candidate.astype(np.float32)))
            near_ids = nearest['primitive_ids'].numpy().astype(int)
            signed = np.einsum('ij,ij->i', candidate-nearest['points'].numpy(),source_normals[near_ids])
            finite = np.isfinite(travel)
            local_reason = np.where(~finite,'missing_reference',
                np.where(signed < -1e-4,'origin_behind_surface',
                np.where(~band_mask[ref],'reference_outside_work_band',
                np.where(abs(travel-stand)>1e-3,'occluded_before_sample',''))))
            origins[retry] = candidate; stands[retry] = stand
            reasons[retry] = local_reason; valid[retry] = local_reason == ''
        rays = o3d.core.Tensor(np.c_[origins,-direction].astype(np.float32))
        oh, lh = original.cast_rays(rays), layer.cast_rays(rays)
        a, b = oh['t_hit'].numpy(), lh['t_hit'].numpy()
        hit_ids = lh['primitive_ids'].numpy().astype(np.int64)
        finite = np.isfinite(b)
        judged = finite & valid
        distance = np.full(len(points), np.nan); angle = distance.copy(); signed = distance.copy()
        error = distance.copy(); passed = np.zeros(len(points), bool)
        if judged.any():
            contacts = origins[judged]-direction[judged]*b[judged,None]
            reference_points = origins[judged]-direction[judged]*a[judged,None]
            metric = contact_metrics(original, contacts, lh['primitive_normals'].numpy()[judged],
                source_normals, tolerance_mm,
                reference=(reference_points,oh['primitive_ids'].numpy()[judged].astype(int)))
            distance[judged] = metric['distance_mm']; angle[judged] = metric['normal_error_deg']
            signed[judged] = metric['signed_distance_mm']
            error[judged] = np.maximum(metric['distance_mm'],abs(b[judged]-a[judged]))
            passed[judged] = metric['geometry_passed'] & (error[judged] <= tolerance_mm)
        if k == 0:
            covered.update(sample_faces[:len(centers)][valid[:len(centers)]].tolist())
        for i in range(len(points)):
            records.append(dict(source_face=int(sample_faces[i]), direction=k,
                sample_kind='face' if i<len(centers) else 'seam_or_boundary',
                x_mm=float(points[i,0]), y_mm=float(points[i,1]), z_mm=float(points[i,2]),
                error_mm=float(error[i]) if judged[i] else None,
                surface_distance_mm=float(distance[i]) if judged[i] else None,
                signed_distance_mm=float(signed[i]) if judged[i] else None,
                normal_error_deg=float(angle[i]) if judged[i] else None,
                original_hit_mm=float(a[i]) if np.isfinite(a[i]) else None,
                layer_hit_mm=float(b[i]) if finite[i] else None,
                layer_facet=int(hit_ids[i]) if finite[i] else None,
                hit_kind=str(kinds[hit_ids[i]]) if finite[i] else 'missing',
                standoff_mm=float(stands[i]),
                approach_x=float(direction[i,0]), approach_y=float(direction[i,1]), approach_z=float(direction[i,2]),
                probe_valid=bool(valid[i]), probe_invalid_reason=str(reasons[i]),
                geometry_passed=bool(passed[i])))
    valid_records = [r for r in records if r['probe_valid']]
    failures = [r for r in valid_records if not r['geometry_passed']]
    errors = [r['error_mm'] for r in valid_records if r['error_mm'] is not None]
    uncovered = sorted(set(face_ids.tolist())-covered)
    return dict(samples=len(records), valid_samples=len(valid_records),
                geometry_passed=bool(valid_records) and not failures and not uncovered,
                maximum_error_mm=max(errors) if errors else None,
                maximum_normal_error_deg=max((r['normal_error_deg'] for r in valid_records
                    if r['normal_error_deg'] is not None), default=None),
                uncovered_normal_faces=uncovered,
                invalid_probes=len(records)-len(valid_records),
                invalid_origin_behind_surface=sum(r['probe_invalid_reason']=='origin_behind_surface' for r in records),
                invalid_occluded=sum(r['probe_invalid_reason']=='occluded_before_sample' for r in records),
                invalid_reference_outside_work_band=sum(r['probe_invalid_reason']=='reference_outside_work_band' for r in records),
                failed_samples=len(failures),
                internal_or_missing_first_hits=sum(r['hit_kind']!='working' for r in valid_records)), records


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
    # The probe travels along the outward direction of the boundary face, so
    # it starts in free space and asks whether the layer reaches into the
    # open gap that the original scan leaves empty.
    # All boundary faces are resolved in ONE call: one call per face was
    # quadratic in the size of the scan.
    unique_ids = np.unique(np.asarray(source_ids, int))
    unique_normals = outward_normals(vertices, faces, unique_ids, labels)
    lookup = {int(i): n for i, n in zip(unique_ids, unique_normals)}
    probe_directions = np.array([lookup[int(i)] for i in source_ids])
    origins=points+probe_directions*.75
    rays=o3d.core.Tensor(np.c_[origins,-probe_directions].astype(np.float32))
    a,b=original.cast_rays(rays),layer.cast_rays(rays)
    at,bt=a['t_hit'].numpy(),b['t_hit'].numpy(); ids=b['primitive_ids'].numpy()
    records=[]
    layer_present=np.isfinite(bt)&(bt<=1.5)
    kinds_hit=np.where(layer_present,kinds[np.clip(ids,0,len(kinds)-1)],'missing')
    # A small nearest-surface distance does not permit filling an open gap:
    # the original ray itself must hit a surface at the same location.
    errors=np.full(len(points),np.nan)
    if layer_present.any():
        contacts=origins[layer_present]-probe_directions[layer_present]*bt[layer_present,None]
        errors[layer_present]=distance_to(original,contacts)
    for i in range(len(points)):
        kind=str(kinds_hit[i])
        error=None if np.isnan(errors[i]) else float(errors[i])
        original_present=np.isfinite(at[i]) and at[i]<=1.5
        passed=(not layer_present[i]) or (original_present and error is not None
                                         and error<=.1 and abs(at[i]-bt[i])<=.1)
        records.append(dict(source_face=source_ids[i],sample_kind='outside_open_or_label_edge',
            x_mm=float(points[i,0]),y_mm=float(points[i,1]),z_mm=float(points[i,2]),
            original_hit_mm=float(at[i]) if np.isfinite(at[i]) and at[i]<=1.5 else None,
            layer_hit_mm=float(bt[i]) if layer_present[i] else None,error_mm=error,hit_kind=kind,geometry_passed=bool(passed)))
    return dict(samples=len(records),geometry_passed=all(r['geometry_passed'] for r in records),
                failed_samples=sum(not r['geometry_passed'] for r in records),probe_outside_edge_mm=.05,
                ray_depth_interval_mm=[0.,1.5]),records
