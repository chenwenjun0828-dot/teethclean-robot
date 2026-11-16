"""Conservative contact certificates for original, oriented open mesh faces.

This backend certifies a *local oriented heightfield*, never a solid interior.
Every working triangle must project wholly into a regular, consistently wound
patch. Adaptively local patches exclude only geometry provably outside the
complete tool-to-support segments. Boundary exclusion and domain clipping prove
coverage of triangle interiors, including triangles perpendicular to the patch.
Folds, holes, open edges under the tool and conflicting sheets are unverifiable.
Reported signed gaps are measured along a common local direction; their
negative magnitude conservatively bounds Euclidean penetration into that patch.
No FCL penetration depth, finite contact list or sampled surface is a proof.
"""
from dataclasses import asdict, dataclass
from collections import OrderedDict
import hashlib
import json
from numbers import Real

import numpy as np
import open3d as o3d
import fcl


SEPARATION_BAND_M = .002
SUPPORT_BANDS_M = tuple(SEPARATION_BAND_M/2**i for i in range(6))
MAX_CHART_SUBDIVISIONS = 2
GEOMETRY_ROUNDOFF_M = 1e-12


@dataclass(frozen=True)
class ContactPolicy:
    contact_tolerance_mm: float
    working_segment_radius_mm: float = 1.
    nonworking_clearance_mm: float = 0.

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not np.isfinite(value):
                raise ValueError(f'{name} must be a finite number')
            if value < 0 or (name == 'working_segment_radius_mm' and value == 0):
                raise ValueError(f'{name} must be ' + ('positive' if name == 'working_segment_radius_mm' else 'nonnegative'))
            object.__setattr__(self, name, float(value))


def array_hash(*arrays):
    digest = hashlib.sha256()
    for array in arrays:
        array = np.ascontiguousarray(array)
        digest.update(str(array.dtype).encode())
        digest.update(json.dumps(array.shape).encode())
        digest.update(array.tobytes())
    return digest.hexdigest()


def partition_working_faces(vertices, faces, tcp_from_geom, policy):
    """Return loaded face IDs; mixed sphere-boundary faces stay forbidden."""
    vertices = np.asarray(vertices, float)
    faces = np.asarray(faces, np.int64)
    transform = np.asarray(tcp_from_geom, float)
    local = vertices @ transform[:3, :3].T + transform[:3, 3]
    within = np.linalg.norm(local[faces], axis=2) <= policy.working_segment_radius_mm*.001
    working = within.all(axis=1)
    if not working.any():
        raise ValueError('Working-segment partition is empty; no loaded face has all vertices inside the TCP sphere')
    mixed = within.any(axis=1) & ~working
    return local, np.flatnonzero(working), np.flatnonzero(~working), np.flatnonzero(mixed)


@dataclass(frozen=True)
class WorkingGap:
    verified: bool
    gap_m: float | None
    reason: str = ''


def _cross2(a, b):
    return a[..., 0]*b[..., 1]-a[..., 1]*b[..., 0]


def _point_in_triangles(points, triangles, margin=0.):
    edges = np.roll(triangles, -1, axis=1)-triangles
    values = _cross2(edges[None], points[:, None, None]-triangles[None])
    return np.all(values >= margin, axis=-1)


def _segments_meet(a, b, c, d, eps):
    # Inclusive: tangency to a patch boundary is deliberately ambiguous.
    if np.any(np.maximum(np.minimum(a, b), np.minimum(c, d)) >
              np.minimum(np.maximum(a, b), np.maximum(c, d))+eps):
        return False
    cross = np.array([_cross2(b-a, c-a), _cross2(b-a, d-a),
                      _cross2(d-c, a-c), _cross2(d-c, b-c)])
    tol = eps*max(np.linalg.norm(b-a), np.linalg.norm(d-c), eps)
    return (min(cross[:2]) <= tol and max(cross[:2]) >= -tol and
            min(cross[2:]) <= tol and max(cross[2:]) >= -tol)


def _intersection_area(subject, clip):
    polygon = list(subject)
    for a, b in zip(clip, np.roll(clip, -1, axis=0)):
        output = []
        for p, q in zip(polygon, polygon[1:]+polygon[:1]):
            dp, dq = _cross2(b-a, p-a), _cross2(b-a, q-a)
            if dp >= 0:
                output.append(p)
            if (dp >= 0) != (dq >= 0):
                output.append(p+(q-p)*(dp/(dp-dq)))
        polygon = output
        if not polygon:
            return 0.
    polygon = np.asarray(polygon)
    return abs(float(np.sum(_cross2(polygon, np.roll(polygon, -1, axis=0)))))*.5


def _clip_domain(flat_tool, flat_jaw):
    """Exact linear clipping in tool barycentric coordinates, even edge-on."""
    polygon = [np.array([0., 0.]), np.array([1., 0.]), np.array([0., 1.])]
    directions = flat_tool[1:]-flat_tool[0]
    for a, b in zip(flat_jaw, np.roll(flat_jaw, -1, axis=0)):
        edge = b-a
        constant = _cross2(edge, flat_tool[0]-a)
        coefficients = _cross2(edge, directions)
        output = []
        for p, q in zip(polygon, polygon[1:]+polygon[:1]):
            dp, dq = constant+coefficients@p, constant+coefficients@q
            if dp >= 0:
                output.append(p)
            if (dp >= 0) != (dq >= 0):
                output.append(p+(q-p)*(dp/(dp-dq)))
        polygon = output
        if not polygon:
            return np.empty((0, 2))
    return np.asarray(polygon)


def _projected_overlap(triangle, candidates):
    """Conservative 2D triangle SAT, retaining edge-on and tangent pairs."""
    result = np.ones(len(candidates), dtype=bool)
    for source in (candidates, np.broadcast_to(triangle, candidates.shape)):
        edges = np.roll(source, -1, axis=1)-source
        axes = np.stack((-edges[..., 1], edges[..., 0]), axis=-1)
        own = np.einsum('nej,nvj->nev', axes, candidates)
        other = np.einsum('nej,vj->nev', axes, triangle)
        margin = GEOMETRY_ROUNDOFF_M*np.linalg.norm(axes, axis=-1)
        separated = ((own.max(axis=-1) < other.min(axis=-1)-margin) |
                     (other.max(axis=-1) < own.min(axis=-1)-margin))
        result &= ~separated.any(axis=1)
    return result


def _triangle_distances(triangle, candidates):
    """Unsigned original-triangle distances, used only to exclude remote faces."""
    def object_for(vertices):
        shape = fcl.BVHModel()
        shape.beginModel(3, 1)
        shape.addSubModel(np.asarray(vertices, np.float64), np.array([[0, 1, 2]], np.int32))
        shape.endModel()
        return fcl.CollisionObject(shape)

    origin = triangle[0]
    tool = object_for(triangle-origin)
    distances = []
    for face in candidates:
        distance = float(fcl.distance(tool, object_for(face-origin),
                                      fcl.DistanceRequest(), fcl.DistanceResult()))
        # FCL's negative marker denotes intersection, never penetration depth.
        distances.append(max(0., distance) if np.isfinite(distance) else float('nan'))
    return np.array(distances)


class HeightfieldContactSurface:
    """World-space original jaw mesh, retaining loaded winding and topology."""

    def __init__(self, vertices, faces):
        # Own immutable inputs so cached topology, bounds and patch proofs
        # cannot become stale if the caller later changes its mesh arrays.
        self.vertices = np.array(vertices, dtype=float, copy=True)
        self.faces = np.array(faces, dtype=np.int64, copy=True)
        self.triangles = self.vertices[self.faces]
        self.low = self.triangles.min(axis=1)
        self.high = self.triangles.max(axis=1)
        self.mesh_low = self.low.min(axis=0)
        self.mesh_high = self.high.max(axis=0)
        raw = np.cross(self.triangles[:, 1]-self.triangles[:, 0],
                       self.triangles[:, 2]-self.triangles[:, 0])
        lengths = np.linalg.norm(raw, axis=1)
        self.normals = raw/np.maximum(lengths[:, None], 1e-300)
        edges = np.concatenate([self.faces[:, [0, 1]], self.faces[:, [1, 2]], self.faces[:, [2, 0]]])
        owners = np.tile(np.arange(len(self.faces)), 3)
        _, inverse, counts = np.unique(np.sort(edges, axis=1), axis=0, return_inverse=True, return_counts=True)
        winding = np.bincount(inverse, weights=np.where(edges[:, 0] < edges[:, 1], 1, -1))
        bad = (counts > 2) | ((counts == 2) & (winding != 0))
        self.bad = lengths <= 1e-18
        self.bad[owners[bad[inverse]]] = True
        for array in (self.vertices, self.faces, self.triangles, self.low, self.high,
                      self.mesh_low, self.mesh_high, self.normals, self.bad):
            array.setflags(write=False)
        self._patch_cache = OrderedDict()
        self._patch_cache_size = 16
        self._seed_scene = None

    def _prepare_patch(self, ids, direction=None):
        """Work-independent proof; no pose or working gap is cached here."""
        if self.bad[ids].any():
            return WorkingGap(False, None, 'ambiguous_topology_or_winding')
        patch = self.triangles[ids]
        normal = self.normals[ids].sum(axis=0) if direction is None else direction.copy()
        if np.linalg.norm(normal) < 1e-12:
            return WorkingGap(False, None, 'conflicting_local_orientation')
        normal /= np.linalg.norm(normal)
        origin = patch[0, 0]
        alignment = self.normals[ids] @ normal
        if np.any(alignment <= .25):
            return WorkingGap(False, None, 'folded_or_conflicting_local_support')
        # Choose a right-handed planar basis, keeping the loaded normal sign.
        u = patch[0, 1]-origin
        u -= normal*(u @ normal); u /= np.linalg.norm(u)
        basis = np.column_stack((u, np.cross(normal, u)))
        flat_patch = (patch-origin) @ basis
        # Index-based manifold tests alone do not detect duplicate geometric
        # sheets or a planar patch that overlaps itself with different IDs.
        patch_low, patch_high = flat_patch.min(axis=1), flat_patch.max(axis=1)
        for i, triangle in enumerate(flat_patch):
            overlaps = np.flatnonzero(np.all(patch_low[i+1:] < patch_high[i], axis=1) &
                                      np.all(patch_high[i+1:] > patch_low[i], axis=1))+i+1
            for j in overlaps:
                # Nondegenerate triangles have three distinct vertex indices;
                # the 3x3 equality count is their exact intersection cardinality.
                if np.count_nonzero(self.faces[ids[i], :, None] == self.faces[ids[j], None, :]) >= 2:
                    continue  # Consistently wound adjacent projections lie on opposite sides.
                if _intersection_area(triangle, flat_patch[j]) > 1e-24:
                    return WorkingGap(False, None, 'overlapping_local_faces')
        faces = self.faces[ids]
        edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
        unique, counts = np.unique(np.sort(edges, axis=1), axis=0, return_counts=True)
        boundary = (self.vertices[unique[counts == 1]]-origin) @ basis
        return patch, origin, alignment, basis, flat_patch, patch_low, patch_high, boundary

    def verify(self, working_triangles):
        triangles = np.asarray(working_triangles, float)
        if not np.isfinite(triangles).all() or not len(triangles):
            return WorkingGap(False, None, 'invalid_working_geometry')
        low = triangles.min(axis=(0, 1)); high = triangles.max(axis=(0, 1))
        box_gap = np.linalg.norm(np.maximum(0., np.maximum(self.mesh_low-high, low-self.mesh_high)))
        if box_gap > SEPARATION_BAND_M:
            # A whole-jaw AABB proof excludes contact, with no signed-gap claim.
            return WorkingGap(True, None, 'aabb_separated_no_contact_permission')
        nearby = np.linalg.norm(np.maximum(0., np.maximum(self.low-high, low-self.high)), axis=1)
        failure = None
        for band in SUPPORT_BANDS_M:
            ids = np.flatnonzero(nearby <= band)
            if not len(ids):
                break
            result = self._verify_patch(triangles, ids, band)
            if result.verified:
                return result
            if failure is None:
                failure = result
        if failure is None:
            return WorkingGap(False, None, 'no_local_orientation_support')
        return self._verify_triangle_charts(triangles, np.flatnonzero(nearby <= SEPARATION_BAND_M))

    def _verify_triangle_charts(self, triangles, nearby_ids, depth=0):
        if self._seed_scene is None:
            # Float32 proximity chooses directions only. All acceptance proofs
            # below still use the original float64 geometry and full faces.
            self._seed_origin = (self.mesh_low+self.mesh_high)*.5
            self._seed_scene = o3d.t.geometry.RaycastingScene()
            self._seed_scene.add_triangles(
                o3d.core.Tensor((self.vertices-self._seed_origin).astype(np.float32)),
                o3d.core.Tensor(self.faces.astype(np.uint32)))
        samples = np.concatenate((triangles.mean(axis=1)[:, None], triangles), axis=1)
        seeds = self._seed_scene.compute_closest_points(o3d.core.Tensor(
            (samples.reshape(-1, 3)-self._seed_origin).astype(np.float32)))['primitive_ids'].numpy().reshape(-1, 4)
        minimum = float('inf')
        for triangle, seed in zip(triangles, seeds):
            if self.bad[seed].any():
                return WorkingGap(False, None, 'ambiguous_topology_or_winding')
            candidates = np.vstack((self.normals[seed].sum(axis=0), self.normals[np.unique(seed)]))
            failure, accepted = None, None
            directions = []
            for candidate in candidates:
                length = np.linalg.norm(candidate)
                if length < 1e-12:
                    continue
                direction = candidate/length
                if any(np.linalg.norm(direction-previous) < 1e-12 for previous in directions):
                    continue
                directions.append(direction)
                result = self._verify_triangle_chart(triangle, nearby_ids, direction)
                if result.verified:
                    accepted = result
                    break
                if failure is None:
                    failure = result
            if accepted is None:
                if depth < MAX_CHART_SUBDIVISIONS:
                    a, b, c = triangle
                    ab, bc, ca = (triangle+np.roll(triangle, -1, axis=0))*.5
                    children = np.array([[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]])
                    # This partitions the entire original face. Each child
                    # still requires the full analytic coverage/gap proof.
                    refined = self._verify_triangle_charts(children, nearby_ids, depth+1)
                    if refined.verified:
                        accepted = refined
                if accepted is None:
                    return failure or WorkingGap(False, None, 'conflicting_local_orientation')
            minimum = min(minimum, accepted.gap_m)
        return WorkingGap(True, minimum, 'conservative_oriented_triangle_chart_directional_gap')

    def _verify_triangle_chart(self, triangle, nearby_ids, direction):
        axis = np.eye(3)[np.argmin(np.abs(direction))]
        u = np.cross(direction, axis); u /= np.linalg.norm(u)
        basis = np.column_stack((u, np.cross(direction, u)))
        low, high = triangle.min(axis=0), triangle.max(axis=0)
        distance = np.linalg.norm(np.maximum(0., np.maximum(
            self.low[nearby_ids]-high, low-self.high[nearby_ids])), axis=1)
        ids = nearby_ids[distance <= SEPARATION_BAND_M]
        flat_work = (triangle-triangle[0]) @ basis
        flat_patch = (self.triangles[ids]-triangle[0]) @ basis
        possible = (np.all(flat_patch.min(axis=1) <= flat_work.max(axis=0)+GEOMETRY_ROUNDOFF_M, axis=1) &
                    np.all(flat_patch.max(axis=1)+GEOMETRY_ROUNDOFF_M >= flat_work.min(axis=0), axis=1))
        ids, flat_patch = ids[possible], flat_patch[possible]
        ids = ids[_projected_overlap(flat_work, flat_patch)]
        distance = np.linalg.norm(np.maximum(0., np.maximum(self.low[ids]-high, low-self.high[ids])), axis=1)
        failure = None
        for exact in (False, True):
            if exact:
                distance = _triangle_distances(triangle, self.triangles[ids])
                if not np.isfinite(distance).all():
                    return WorkingGap(False, None, 'invalid_triangle_exclusion_distance')
                # Face sets change only at these radii. Keep the largest safe
                # radius in every interval, guarding unsigned FCL roundoff.
                bands = sorted({SEPARATION_BAND_M, *(float(d-2*GEOMETRY_ROUNDOFF_M)
                    for d in distance if 2*GEOMETRY_ROUNDOFF_M < d <= SEPARATION_BAND_M)}, reverse=True)
            else:
                bands = SUPPORT_BANDS_M
            for band in bands:
                selected = ids[distance <= band+(GEOMETRY_ROUNDOFF_M if exact else 0.)]
                if not len(selected):
                    break
                result = self._verify_patch(triangle[None], selected, band, direction)
                if result.verified:
                    return result
                if failure is None:
                    failure = result
        return failure or WorkingGap(False, None, 'no_local_orientation_support')

    def _verify_patch(self, triangles, ids, band, direction=None):
        key = (ids.shape, ids.tobytes(), None if direction is None else direction.tobytes())
        if key in self._patch_cache:
            prepared = self._patch_cache[key]
            self._patch_cache.move_to_end(key)
        else:
            prepared = self._prepare_patch(ids, direction)
            self._patch_cache[key] = prepared
            if len(self._patch_cache) > self._patch_cache_size:
                self._patch_cache.popitem(last=False)
        if isinstance(prepared, WorkingGap):
            return prepared
        patch, origin, alignment, basis, flat_patch, patch_low, patch_high, boundary = prepared
        flat_work = (triangles-origin) @ basis
        points = flat_work.reshape(-1, 2)
        # Vertices alone are insufficient: the boundary test below also rules
        # out concavities, holes and missing coverage in triangle interiors.
        if not _point_in_triangles(points, flat_patch, -1e-24).any(axis=1).all():
            return WorkingGap(False, None, 'projection_outside_local_patch')
        work_low, work_high = flat_work.min(axis=1), flat_work.max(axis=1)
        for segment in boundary:
            # Expand by the exact roundoff used by the existing segment test;
            # only provably disjoint pairs are removed before those predicates.
            candidates = np.flatnonzero(np.all(work_low <= segment.max(axis=0)+GEOMETRY_ROUNDOFF_M, axis=1) &
                                        np.all(work_high+GEOMETRY_ROUNDOFF_M >= segment.min(axis=0), axis=1))
            for triangle in flat_work[candidates]:
                # A hole boundary wholly inside a tool triangle must be caught,
                # even when it intersects none of that triangle's three edges.
                area = _cross2(triangle[1]-triangle[0], triangle[2]-triangle[0])
                ordered = triangle if area >= 0 else triangle[[0, 2, 1]]
                if abs(area) > 1e-24 and _point_in_triangles(segment, ordered[None]).any():
                    return WorkingGap(False, None, 'projection_crosses_patch_boundary')
                if any(_segments_meet(segment[0], segment[1], triangle[i], triangle[(i+1) % 3],
                                      GEOMETRY_ROUNDOFF_M) for i in range(3)):
                    return WorkingGap(False, None, 'projection_crosses_patch_boundary')
        minimum = float('inf')
        maximum_magnitude = 0.
        for triangle, flat_triangle in zip(triangles, flat_work):
            covered_area = 0.
            low2, high2 = flat_triangle.min(axis=0), flat_triangle.max(axis=0)
            candidates = np.flatnonzero(np.all(patch_low <= high2, axis=1) &
                                        np.all(patch_high >= low2, axis=1))
            for i in candidates:
                domain = _clip_domain(flat_triangle, flat_patch[i])
                if not len(domain):
                    continue
                covered_area += abs(float(np.sum(_cross2(domain, np.roll(domain, -1, axis=0)))))*.5
                points3 = triangle[0]+domain @ (triangle[1:]-triangle[0])
                gaps = (points3-patch[i, 0]) @ self.normals[ids[i]] / alignment[i]
                minimum = min(minimum, float(gaps.min()))
                maximum_magnitude = max(maximum_magnitude, float(np.abs(gaps).max()))
            # Lambda triangle has area 1/2. Degenerate projected triangles can
            # duplicate domains on a shared internal edge; continuous adjacent
            # facets agree there. Boundary exclusion above already proves union
            # coverage, and this catches clipping arithmetic/query failures.
            if covered_area < .5-1e-10:
                return WorkingGap(False, None, 'incomplete_clipped_surface_coverage')
        if not np.isfinite(minimum):
            return WorkingGap(False, None, 'invalid_directional_gap')
        # Every complete tool-to-support segment must stay in the band used
        # to select jaw faces. An omitted face has a strictly larger AABB
        # distance from the working mesh, an FCL triangle-distance exclusion,
        # or a disjoint chart projection, so cannot meet any segment.
        # Affine extrema over the clipped domains bound all interior points.
        if maximum_magnitude > band:
            return WorkingGap(False, None, 'support_segments_outside_local_band')
        # On each clipped barycentric polygon directional separation is affine;
        # extrema are attained at polygon vertices, including newly introduced
        # intersections inside a working face. No sampling bound is assumed.
        return WorkingGap(True, minimum, 'conservative_oriented_heightfield_directional_gap')
