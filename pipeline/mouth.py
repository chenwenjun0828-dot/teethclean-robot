"""White face mask with a true elliptical aperture, in the oral-scan frame.

All public dimensions are millimetres. The mask is visual scenery: its mesh
must never be used as a convex collision proxy, which would close the hole.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / 'config' / 'mouth_opening.json'
SEGMENTS = 128
FACE_HALF_WIDTH = 0.065
FACE_HALF_HEIGHT = 0.075
THICKNESS = 0.003
CURVE_DEPTH = 0.012
RIM_RADIUS = 0.0015


@dataclass(frozen=True)
class MouthSettings:
    enabled: bool = True
    width_mm: float = 45.
    height_mm: float = 80.
    center_mm: tuple[float, float, float] = (0., 10., 45.)

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('Mouth enabled must be true or false')
        for name, maximum in (('width_mm', 110.), ('height_mm', 130.)):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
                raise ValueError(f'Mouth {name} must be a finite number')
            if not 10. <= value <= maximum:
                raise ValueError(f'Mouth {name} must be between 10 and {maximum:g} mm')
        center = np.asarray(self.center_mm, dtype=float)
        if center.shape != (3,) or not np.all(np.isfinite(center)):
            raise ValueError('Mouth center_mm must contain three finite coordinates')
        object.__setattr__(self, 'center_mm', tuple(float(x) for x in center))


def load_settings(path: Path = CONFIG_PATH) -> MouthSettings:
    values = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    return MouthSettings(**values)


def save_settings(settings: MouthSettings, path: Path = CONFIG_PATH) -> None:
    """Atomically save the configuration used by create_simulation."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(asdict(settings), indent=2, allow_nan=False) + '\n',
                         encoding='utf-8')
    temporary.replace(path)


def ellipse(settings: MouthSettings):
    theta = np.arange(SEGMENTS) * (2. * np.pi / SEGMENTS)
    a, b = settings.width_mm / 2000., settings.height_mm / 2000.
    points = np.column_stack((a*np.cos(theta), b*np.sin(theta), np.zeros(SEGMENTS)))
    normals = np.column_stack((np.cos(theta)/a, np.sin(theta)/b, np.zeros(SEGMENTS)))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    return points, normals


def face_geometry(settings: MouthSettings):
    """Closed curved rectangular shell; triangles never span the aperture."""
    opening, normals = ellipse(settings)
    inner = opening + 2.*RIM_RADIUS*normals
    theta = np.arange(SEGMENTS) * (2.*np.pi/SEGMENTS)
    directions = np.column_stack((np.cos(theta), np.sin(theta)))
    extent = np.minimum(FACE_HALF_WIDTH/np.maximum(np.abs(directions[:, 0]), 1e-15),
                        FACE_HALF_HEIGHT/np.maximum(np.abs(directions[:, 1]), 1e-15))
    outer = np.column_stack((directions*extent[:, None], np.zeros(SEGMENTS)))
    outer[:, 2] = -CURVE_DEPTH*(outer[:, 0]/FACE_HALF_WIDTH)**2
    back = np.array([0., 0., THICKNESS])
    vertices = np.concatenate((inner, outer, inner-back, outer-back))
    faces = []
    for i in range(SEGMENTS):
        j = (i+1) % SEGMENTS
        # Front, back, aperture wall and outer edge, with outward winding.
        for a, b, c, d in ((i, SEGMENTS+i, SEGMENTS+j, j),
                            (2*SEGMENTS+i, 2*SEGMENTS+j, 3*SEGMENTS+j, 3*SEGMENTS+i),
                            (i, j, 2*SEGMENTS+j, 2*SEGMENTS+i),
                            (SEGMENTS+i, 3*SEGMENTS+i, 3*SEGMENTS+j, SEGMENTS+j)):
            faces.extend(((a, b, c), (a, c, d)))
    return vertices, np.asarray(faces, dtype=int)


def _numbers(values):
    return ' '.join(f'{x:.12g}' for x in np.asarray(values).ravel())


def write_builtin_assets(directory: Path = ROOT / 'models') -> None:
    """Generate the checked-in MJCF includes, also usable in a plain viewer."""
    settings = MouthSettings()
    vertices, faces = face_geometry(settings)
    assets = ET.Element('mujocoinclude')
    ET.SubElement(assets, 'mesh', name='mouth_face_mesh', vertex=_numbers(vertices),
                  face=_numbers(faces), smoothnormal='false')
    content = ET.Element('mujocoinclude')
    body = ET.SubElement(content, 'body', name='mouth_opening',
                         pos=_numbers(np.asarray(settings.center_mm)/1000.))
    ET.SubElement(body, 'geom', name='mouth_face', type='mesh', mesh='mouth_face_mesh',
                  rgba='0.94 0.95 0.96 1', contype='0', conaffinity='0', group='2')
    points, normals = ellipse(settings)
    centers = points + RIM_RADIUS*normals
    for i in range(SEGMENTS):
        ET.SubElement(body, 'geom', name=f'mouth_rim_{i:03d}', type='capsule',
                      fromto=_numbers([centers[i], centers[(i+1) % SEGMENTS]]),
                      size=str(RIM_RADIUS), rgba='0.99 0.99 0.99 1',
                      contype='0', conaffinity='0', group='2')
    for name, tree in (('mouth_opening_assets.xml', assets), ('mouth_opening_body.xml', content)):
        ET.indent(tree, space='  ')
        ET.ElementTree(tree).write(directory/name, encoding='utf-8', xml_declaration=False)


class MouthOpening:
    """Update an existing mask without rebuilding the robot or its state.

    After update, call viewer.update_mesh(mesh_id) OUTSIDE viewer.lock(), or
    mujoco.mjr_uploadMesh(model, renderer_context, mesh_id) for an offscreen view.
    """

    def __init__(self, model, prefix='dental_'):
        self.model = model
        self.body_id = model.body(prefix+'mouth_opening').id
        self.face_id = model.geom(prefix+'mouth_face').id
        self.mesh_id = model.mesh(prefix+'mouth_face_mesh').id
        self.rim_ids = [model.geom(prefix+f'mouth_rim_{i:03d}').id for i in range(SEGMENTS)]
        mesh = self.mesh_id
        self.vertex_start = int(model.mesh_vertadr[mesh])
        self.vertex_count = int(model.mesh_vertnum[mesh])
        if self.vertex_count != 4*SEGMENTS:
            raise ValueError('Unexpected mouth mesh topology; regenerate built-in mouth assets')
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, model.mesh_quat[mesh])
        self.mesh_rotation = rotation.reshape(3, 3)
        self.mesh_position = model.mesh_pos[mesh].copy()

    def update(self, settings: MouthSettings, data=None):
        m = self.model
        vertices, _ = face_geometry(settings)
        # The compiler centres and rotates mesh vertices into an inertia frame.
        # Keep that frame and the face geom's pose; transform new vertices back.
        transformed = (vertices-self.mesh_position) @ self.mesh_rotation
        start, count = self.vertex_start, self.vertex_count
        m.mesh_vert[start:start+count] = transformed
        face_start = int(m.mesh_faceadr[self.mesh_id])
        face_count = int(m.mesh_facenum[self.mesh_id])
        faces = m.mesh_face[face_start:face_start+face_count]
        normal_indices = m.mesh_facenormal[face_start:face_start+face_count]
        triangle_normals = np.cross(transformed[faces[:, 1]]-transformed[faces[:, 0]],
                                    transformed[faces[:, 2]]-transformed[faces[:, 0]])
        normal_start = int(m.mesh_normaladr[self.mesh_id])
        normal_count = int(m.mesh_normalnum[self.mesh_id])
        normals = np.zeros((normal_count, 3))
        for corner in range(3):
            np.add.at(normals, normal_indices[:, corner], triangle_normals)
        normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-15)
        m.mesh_normal[normal_start:normal_start+normal_count] = normals
        m.body_pos[self.body_id] = np.asarray(settings.center_mm)/1000.
        opening, directions = ellipse(settings)
        centers = opening + RIM_RADIUS*directions
        for i, geom_id in enumerate(self.rim_ids):
            a, b = centers[i], centers[(i+1) % SEGMENTS]
            delta = b-a
            m.geom_pos[geom_id] = (a+b)/2.
            mujoco.mju_quatZ2Vec(m.geom_quat[geom_id], delta)
            m.geom_size[geom_id, :2] = (RIM_RADIUS, np.linalg.norm(delta)/2.)
            m.geom_rbound[geom_id] = RIM_RADIUS+np.linalg.norm(delta)/2.
        ids = [self.face_id, *self.rim_ids]
        m.geom_rgba[ids, 3] = 1. if settings.enabled else 0.
        self.settings = settings
        if data is not None:
            mujoco.mj_forward(m, data)


def with_overrides(settings, width_mm=None, height_mm=None, forward_mm=None, enabled=None):
    values = {}
    if width_mm is not None:
        values['width_mm'] = width_mm
    if height_mm is not None:
        values['height_mm'] = height_mm
    if forward_mm is not None:
        values['center_mm'] = (*settings.center_mm[:2], forward_mm)
    if enabled is not None:
        values['enabled'] = enabled
    return replace(settings, **values)
