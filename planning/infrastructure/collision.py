"""FCL triangle-mesh tooth clearance and robot checks; no whole-jaw hulls."""
from dataclasses import dataclass
import numpy as np
import mujoco
import fcl
import open3d as o3d
from .io import Inputs, body_transform


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


class CollisionChecker:
    mode = 'full'

    def __init__(self, robot, jaw_transforms=None, scan_paths=None):
        self.robot = robot; self.model = robot.model
        robot.fk(robot.home)
        self.teeth = []
        self.tooth_bounds = {}
        self.meshes = []
        self.dental_transforms = {}
        for jaw in ('mandibular', 'maxillary'):
            scan = (scan_paths or {}).get(jaw, Inputs.defaults(jaw).scan)
            mesh = o3d.io.read_triangle_mesh(str(scan))
            transform = (jaw_transforms or {}).get(jaw, body_transform(robot.model, robot.data, jaw))
            self.dental_transforms[jaw] = transform.copy()
            self.meshes.append(bvh(np.asarray(mesh.vertices)*.001, np.asarray(mesh.triangles)))
            self.teeth.append((jaw, fcl.CollisionObject(self.meshes[-1], fcl.Transform(transform[:3, :3], transform[:3, 3]))))
            world_vertices = np.asarray(mesh.vertices)*.001 @ transform[:3, :3].T+transform[:3, 3]
            self.tooth_bounds[jaw] = (world_vertices.min(axis=0), world_vertices.max(axis=0))
        self.parts = []
        self.part_bounds = {}
        for geom in range(self.model.ngeom):
            body = int(self.model.geom_bodyid[geom])
            if not (self.model.body(body).name or '').startswith('ur_'):
                continue
            if not (self.model.geom_contype[geom] or self.model.geom_conaffinity[geom]):
                continue
            kind = self.model.geom_type[geom]; size = self.model.geom_size[geom]
            if kind == mujoco.mjtGeom.mjGEOM_MESH:
                vertices, faces = mesh_from_geom(self.model, geom)
                shape = bvh(vertices, faces)
                low, high = vertices.min(axis=0), vertices.max(axis=0)
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

    def edge(self, a, b, clearance_m, step_rad=.004):
        """Validate the actual straight joint segment used by the time law.

        Finite-resolution sweep, including both endpoints. This is recorded as
        a sampled check, not continuous collision detection or a solid-volume
        certificate for open oral scans.
        """
        count = max(2, int(np.ceil(np.max(abs(np.asarray(b)-a))/step_rad))+1)
        minimum = float('inf')
        for u in np.linspace(0, 1, count):
            check = self.check((1-u)*a+u*b, clearance_m)
            minimum = min(minimum, check.clearance_m)
            if not check.valid:
                return Check(False, minimum, check.pair)
        return Check(True, minimum, '')


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
