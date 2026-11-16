"""FCL triangle-mesh tooth clearance and robot checks; no whole-jaw hulls."""
from dataclasses import asdict, dataclass
import numpy as np
import mujoco
import fcl
import open3d as o3d
from .io import Inputs, body_transform
from .contact import (ContactPolicy, HeightfieldContactSurface, WorkingGap,
                      SEPARATION_BAND_M, GEOMETRY_ROUNDOFF_M, array_hash, partition_working_faces)


def bvh(vertices, faces):
    model = fcl.BVHModel()
    model.beginModel(len(vertices), len(faces))
    model.addSubModel(np.asarray(vertices, np.float64), np.asarray(faces, np.int32))
    model.endModel()
    return model


def mesh_from_geom(model, geom):
    mesh_id = model.geom_dataid[geom]
    a, n = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
    b, count = model.mesh_faceadr[mesh_id], model.mesh_facenum[mesh_id]
    return model.mesh_vert[a:a+n].astype(float), model.mesh_face[b:b+count].copy()


@dataclass
class Check:
    valid: bool
    clearance_m: float
    pair: str
    working_gap_m: float | None = None
    contact_verified: bool = False


class CollisionChecker:
    mode = 'full'

    def __init__(self, robot, jaw_transforms=None, scan_paths=None, contact_policy=None):
        self.robot = robot; self.model = robot.model
        if contact_policy is not None and not isinstance(contact_policy, ContactPolicy):
            raise TypeError('contact_policy must be a ContactPolicy or None')
        self.contact_policy = contact_policy
        self.contact_manifest = None
        robot.fk(robot.home)
        self.teeth = []
        self.tooth_bounds = {}
        self.meshes = []
        self.dental_transforms = {}
        self.contact_surfaces = {}
        jaw_manifest = {}
        for jaw in ('mandibular', 'maxillary'):
            scan = (scan_paths or {}).get(jaw, Inputs.defaults(jaw).scan)
            mesh = o3d.io.read_triangle_mesh(str(scan))
            transform = (jaw_transforms or {}).get(jaw, body_transform(robot.model, robot.data, jaw))
            self.dental_transforms[jaw] = transform.copy()
            self.meshes.append(bvh(np.asarray(mesh.vertices)*.001, np.asarray(mesh.triangles)))
            self.teeth.append((jaw, fcl.CollisionObject(self.meshes[-1], fcl.Transform(transform[:3, :3], transform[:3, 3]))))
            world_vertices = np.asarray(mesh.vertices)*.001 @ transform[:3, :3].T+transform[:3, 3]
            self.tooth_bounds[jaw] = (world_vertices.min(axis=0), world_vertices.max(axis=0))
            if contact_policy is not None:
                self.contact_surfaces[jaw] = HeightfieldContactSurface(world_vertices, np.asarray(mesh.triangles))
                jaw_manifest[jaw] = {'loaded_mesh_sha256': array_hash(
                    np.asarray(mesh.vertices, np.float64)*.001, np.asarray(mesh.triangles, np.int64)),
                    'world_from_scan_m': transform.tolist()}
        self.parts = []
        self.part_bounds = {}
        self.working_geom = None
        if contact_policy is not None:
            tool = self.model.geom('ur_end_scaler_geom').id
            vertices, faces = mesh_from_geom(self.model, tool)
            tcp = robot.pose(robot.data)
            geom_transform = np.eye(4)
            geom_transform[:3, :3] = robot.data.geom_xmat[tool].reshape(3, 3)
            geom_transform[:3, 3] = robot.data.geom_xpos[tool]
            tcp_from_geom = np.linalg.inv(tcp) @ geom_transform
            # On the actual tool both frames share a body. Body-local transforms
            # avoid FK roundoff changing provenance when a saved plan is loaded.
            if self.model.geom_bodyid[tool] == self.model.site_bodyid[robot.site]:
                geom_rotation, site_rotation = np.empty(9), np.empty(9)
                mujoco.mju_quat2Mat(geom_rotation, self.model.geom_quat[tool])
                mujoco.mju_quat2Mat(site_rotation, self.model.site_quat[robot.site])
                site_rotation = site_rotation.reshape(3, 3)
                tcp_from_geom[:3, :3] = site_rotation.T @ geom_rotation.reshape(3, 3)
                tcp_from_geom[:3, 3] = site_rotation.T @ (self.model.geom_pos[tool]-self.model.site_pos[robot.site])
            local, working, nonworking, mixed = partition_working_faces(vertices, faces, tcp_from_geom, contact_policy)
            self.working_geom = tool
            self.working_vertices = vertices
            self.working_faces = faces[working]
            self._nonworking_faces = faces[nonworking]
            self.contact_manifest = {
                'version': 1, 'policy': asdict(contact_policy),
                'partition_rule': 'all_three_loaded_face_vertices_inside_tcp_sphere',
                'geom': 'ur_end_scaler_geom', 'working_face_indices': working.tolist(),
                'nonworking_face_indices': nonworking.tolist(), 'mixed_boundary_face_indices': mixed.tolist(),
                'loaded_mesh_sha256': array_hash(vertices, faces.astype(np.int64)),
                'tcp_from_geom_m': np.round(tcp_from_geom, 12).tolist(),
                'tcp_vertices_sha256': array_hash(np.round(local, 12)), 'jaws': jaw_manifest,
                'verification_backend': 'whole_triangle_oriented_heightfield_v3',
                'verification_scope': 'Original winding supplies local sign only; exact projected whole-triangle '
                    'coverage within adaptive regular piecewise-planar heightfields, with per-working-triangle '
                    'charts and bounded analytic face subdivision for curved neighborhoods. '
                    'Entire tool-to-support segments stay inside the selection '
                    'band; omitted faces have disjoint projections or a larger AABB/original-triangle '
                    'FCL separation. FCL unsigned distances only exclude faces and never supply contact sign. '
                    'Conflicting, folded, boundary or unsupported '
                    'near geometry is contact_unverifiable. Whole-jaw AABB or all-vertex separating-plane gap greater than 2 mm '
                    'excludes contact without permission and reports no signed gap. No global solid sign, '
                    'FCL depth, contact-list truncation or sampled-surface certificate. Signed gaps are '
                    'conservative local directional bounds, not Euclidean nearest signed distances.',
            }
        for geom in range(self.model.ngeom):
            body = int(self.model.geom_bodyid[geom])
            if not (self.model.body(body).name or '').startswith('ur_'):
                continue
            if not (self.model.geom_contype[geom] or self.model.geom_conaffinity[geom]):
                continue
            kind = self.model.geom_type[geom]; size = self.model.geom_size[geom]
            if kind == mujoco.mjtGeom.mjGEOM_MESH:
                vertices, faces = mesh_from_geom(self.model, geom)
                if geom == self.working_geom:
                    faces = self._nonworking_faces
                    if not len(faces):
                        continue
                shape = bvh(vertices, faces)
                used = vertices[np.unique(faces)] if geom == self.working_geom else vertices
                low, high = used.min(axis=0), used.max(axis=0)
            elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
                shape = fcl.Capsule(size[0], 2*size[1])
            elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
                shape = fcl.Cylinder(size[0], 2*size[1])
            elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
                shape = fcl.Sphere(size[0])
            elif kind == mujoco.mjtGeom.mjGEOM_BOX:
                shape = fcl.Box(*(2*size))
            else:
                raise ValueError(f'Unsupported moving collision geometry: {kind}')
            if kind != mujoco.mjtGeom.mjGEOM_MESH:
                if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
                    extent = np.array([size[0], size[0], size[0]+size[1]])
                elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
                    extent = np.array([size[0], size[0], size[1]])
                elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
                    extent = np.full(3, size[0])
                else: extent = size.copy()
                low, high = -extent, extent
            self.part_bounds[geom] = np.array([[x, y, z] for x in (low[0], high[0])
                for y in (low[1], high[1]) for z in (low[2], high[2])])
            self.meshes.append(shape)
            self.parts.append((geom, body, fcl.CollisionObject(shape)))
        self.parts.sort(key=lambda part: 0 if self.model.geom(part[0]).name=='ur_end_scaler_geom' else 1)
        self.calls = 0
        self.cache = {}
        tool = self.model.geom('ur_end_scaler_geom').id
        v, _ = mesh_from_geom(self.model, tool)
        world = v @ robot.data.geom_xmat[tool].reshape(3, 3).T + robot.data.geom_xpos[tool]
        tcp = robot.pose(robot.data)
        self.tool_center_tcp = (world.mean(axis=0)-tcp[:3, 3]) @ tcp[:3, :3]
        # The exact tooth BVHs replace MuJoCo's single convex jaw proxies in this
        # private model. Runtime control calls THIS checker as well as mj_contact.
        for jaw in ('mandibular', 'maxillary'):
            geom = self.model.geom('dental_'+jaw+'_visual').id
            self.model.geom_contype[geom] = 0
            self.model.geom_conaffinity[geom] = 0

    def check(self, q, clearance_m=0.):
        if self.contact_policy is not None:
            return self._check_contact_policy(q)
        key = np.asarray(q, float).tobytes()
        if key in self.cache:
            distance, pair = self.cache[key]
            return Check(distance > clearance_m, distance, pair)
        self.robot.fk(q, full=True)
        data = self.robot.verify_data
        self.calls += 1
        for contact in data.contact[:data.ncon]:
            names = [self.model.body(int(self.model.geom_bodyid[g])).name or '' for g in (contact.geom1, contact.geom2)]
            if any(n.startswith('ur_') for n in names):
                return Check(False, -1., ' / '.join(names))
        nearest, closest = float('inf'), ''
        for geom, body, obj in self.parts:
            obj.setTransform(fcl.Transform(data.geom_xmat[geom].reshape(3, 3), data.geom_xpos[geom]))
            corners = self.part_bounds[geom] @ data.geom_xmat[geom].reshape(3, 3).T+data.geom_xpos[geom]
            low, high = corners.min(axis=0), corners.max(axis=0)
            for jaw, tooth in self.teeth:
                tooth_low, tooth_high = self.tooth_bounds[jaw]
                bound = np.linalg.norm(np.maximum(0., np.maximum(tooth_low-high, low-tooth_high)))
                if bound > nearest:
                    continue
                # A separated bounding box supplies a conservative clearance.
                # Exact mesh queries are only necessary inside a 2 mm band.
                if bound > .002:
                    nearest, closest = bound, (self.model.geom(geom).name or self.model.body(body).name)+' / '+jaw
                    continue
                if fcl.collide(obj, tooth, fcl.CollisionRequest(num_max_contacts=1), fcl.CollisionResult()):
                    pair = (self.model.geom(geom).name or self.model.body(body).name)+' / '+jaw
                    self.cache[key] = (-1., pair)
                    return Check(False, -1., pair)
                distance = float(fcl.distance(obj, tooth, fcl.DistanceRequest(), fcl.DistanceResult()))
                if distance < nearest:
                    nearest, closest = distance, (self.model.geom(geom).name or self.model.body(body).name)+' / '+jaw
                if distance <= 0:
                    self.cache[key] = (nearest, closest)
                    return Check(False, nearest, closest)
        self.cache[key] = (nearest, closest)
        return Check(nearest > clearance_m, nearest, closest)

    def _check_contact_policy(self, q):
        """Check all forbidden pairs before granting any working permission."""
        key = np.asarray(q, float).tobytes()
        if key in self.cache:
            return self.cache[key]
        self.robot.fk(q, full=True)
        data = self.robot.verify_data
        self.calls += 1
        for contact in data.contact[:data.ncon]:
            names = [self.model.body(int(self.model.geom_bodyid[g])).name or '' for g in (contact.geom1, contact.geom2)]
            if any(name.startswith('ur_') for name in names):
                result = Check(False, -1., ' / '.join(names))
                self.cache[key] = result
                return result
        nearest, closest = float('inf'), ''
        threshold = self.contact_policy.nonworking_clearance_mm*.001
        for geom, body, obj in self.parts:
            rotation = data.geom_xmat[geom].reshape(3, 3)
            position = data.geom_xpos[geom]
            obj.setTransform(fcl.Transform(rotation, position))
            corners = self.part_bounds[geom] @ rotation.T+position
            low, high = corners.min(axis=0), corners.max(axis=0)
            for jaw, tooth in self.teeth:
                tooth_low, tooth_high = self.tooth_bounds[jaw]
                bound = np.linalg.norm(np.maximum(0., np.maximum(tooth_low-high, low-tooth_high)))
                if bound > nearest:
                    continue
                pair = (self.model.geom(geom).name or self.model.body(body).name)+' / '+jaw
                if bound > max(.002, threshold):
                    nearest, closest = bound, pair
                    continue
                if fcl.collide(obj, tooth, fcl.CollisionRequest(num_max_contacts=1), fcl.CollisionResult()):
                    result = Check(False, -1., pair)
                    self.cache[key] = result
                    return result
                distance = float(fcl.distance(obj, tooth, fcl.DistanceRequest(), fcl.DistanceResult()))
                if not np.isfinite(distance):
                    result = Check(False, float('nan'), 'contact_unverifiable: '+pair)
                    self.cache[key] = result
                    return result
                if distance < nearest:
                    nearest, closest = distance, pair
                if distance <= threshold:
                    result = Check(False, nearest, closest)
                    self.cache[key] = result
                    return result
        geom = self.working_geom
        world = self.working_vertices @ data.geom_xmat[geom].reshape(3, 3).T+data.geom_xpos[geom]
        minimum = None
        for jaw, surface in self.contact_surfaces.items():
            gap = surface.verify(world[self.working_faces])
            if not gap.verified:
                if not hasattr(self, '_working_contact_object'):
                    self._working_contact_shape = bvh(self.working_vertices, self.working_faces)
                    self._working_contact_object = fcl.CollisionObject(self._working_contact_shape)
                self._working_contact_object.setTransform(fcl.Transform(
                    data.geom_xmat[geom].reshape(3, 3), data.geom_xpos[geom]))
                tooth = next(obj for name, obj in self.teeth if name == jaw)
                query = fcl.DistanceResult()
                separation = float(fcl.distance(self._working_contact_object, tooth,
                                               fcl.DistanceRequest(enable_nearest_points=True), query))
                if np.isfinite(separation) and separation > SEPARATION_BAND_M+GEOMETRY_ROUNDOFF_M:
                    # Unsigned mesh separation alone cannot exclude enclosure.
                    # A plane separating ALL original vertices also excludes
                    # the convex hull, so cannot permit a deeply enclosed tip.
                    direction = np.asarray(query.nearest_points[1])-query.nearest_points[0]
                    length = np.linalg.norm(direction)
                    if np.isfinite(length) and length > 0:
                        direction /= length
                        working = world[self.working_faces].reshape(-1, 3)
                        origin = working[0]
                        a = (working-origin) @ direction
                        b = (surface.vertices-origin) @ direction
                        bound = max(b.min()-a.max(), a.min()-b.max())
                        if bound > SEPARATION_BAND_M+GEOMETRY_ROUNDOFF_M:
                            gap = WorkingGap(True, None, 'separating_plane_no_contact_permission')
            pair = 'ur_end_scaler_geom:working / '+jaw
            if not gap.verified:
                result = Check(False, nearest, 'contact_unverifiable: '+pair+' ('+gap.reason+')', minimum, False)
                self.cache[key] = result
                return result
            if gap.gap_m is not None:
                minimum = gap.gap_m if minimum is None else min(minimum, gap.gap_m)
                tolerance = self.contact_policy.contact_tolerance_mm*.001
                # Affine clipping/normal division introduces several ulps at
                # the exact boundary. This is float comparison roundoff only;
                # keep the raw gap and do not add a geometric safety tolerance.
                roundoff = 32*np.finfo(float).eps*max(abs(gap.gap_m), tolerance)
                if gap.gap_m < -tolerance-roundoff:
                    result = Check(False, nearest, pair, minimum, True)
                    self.cache[key] = result
                    return result
        result = Check(nearest > threshold, nearest, closest, minimum, True)
        self.cache[key] = result
        return result

    def edge(self, a, b, clearance_m, step_rad=.004):
        """Validate the actual straight joint segment used by the time law.

        Finite-resolution sweep, including both endpoints. This is recorded as
        a sampled check, not continuous collision detection or a solid-volume
        certificate for open oral scans.
        """
        count = max(2, int(np.ceil(np.max(abs(np.asarray(b)-a))/step_rad))+1)
        minimum = float('inf')
        working_minimum = None
        contact_verified = self.contact_policy is not None
        for u in np.linspace(0, 1, count):
            check = self.check((1-u)*a+u*b, clearance_m)
            minimum = min(minimum, check.clearance_m)
            if check.working_gap_m is not None:
                working_minimum = (check.working_gap_m if working_minimum is None else
                                   min(working_minimum, check.working_gap_m))
            contact_verified = contact_verified and check.contact_verified
            if not check.valid:
                return Check(False, minimum, check.pair, working_minimum, contact_verified)
        return Check(True, minimum, '', working_minimum, contact_verified)


class NoCollisionChecker:
    """Geometry-only stand-in: same call surface, no collision query at all.

    A plan built with this checker only answers whether the requested poses and
    joint paths are reachable along one connected IK branch. Every check is
    valid by definition, so the reported clearance is not a measurement and
    must not be read as one.
    """

    mode = 'geometry_only'

    def __init__(self, robot):
        self.robot = robot
        self.model = robot.model
        self.calls = 0
        self.dental_transforms = {}

    def check(self, q, clearance_m=0.):
        self.calls += 1
        return Check(True, float('nan'), 'collision_check_disabled')

    def edge(self, a, b, clearance_m, step_rad=.004):
        self.calls += 1
        return Check(True, float('nan'), 'collision_check_disabled')
