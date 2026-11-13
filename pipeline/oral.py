"""口腔调整：参数读写、椭圆口缘、下颌开合及统一预览。

运行 python -m pipeline.oral 打开预览；参数集中在 config/oral_settings.py。
尺寸和坐标单位为毫米，角度单位为度。面罩仅用于显示及相机遮挡。
"""
from __future__ import annotations

import argparse
import ast
from dataclasses import asdict, dataclass, replace
import json
from pathlib import Path
from pprint import pformat
from queue import Empty, SimpleQueue
import sys
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / 'config' / 'oral_settings.py'


# 统一配置读写

def _assignment(source, section):
    matches = [node.value for node in ast.parse(source).body
               if isinstance(node, ast.Assign)
               and any(isinstance(target, ast.Name) and target.id == section
                       for target in node.targets)]
    if len(matches) != 1 or not isinstance(matches[0], ast.Dict):
        raise ValueError(f'Configuration must contain one literal {section} dictionary')
    return matches[0]

def load_section(section, path=CONFIG_PATH):
    path = Path(path)
    source = path.read_text(encoding='utf-8-sig')
    # Explicit legacy JSON paths remain available to callers with old files.
    if path.suffix.lower() == '.json':
        return json.loads(source)
    return ast.literal_eval(_assignment(source, section))

def save_section(section, values, path=CONFIG_PATH):
    """Atomically update one group, retaining other parameters and comments."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.suffix.lower() == '.json':
        source = json.dumps(values, indent=2, allow_nan=False) + '\n'
    else:
        source = path.read_text(encoding='utf-8-sig')
        node = _assignment(source, section)
        current = ast.literal_eval(node)
        lines = source.splitlines(keepends=True)

        def offset(line, column):
            # AST column offsets are UTF-8 bytes, including for Chinese text.
            return (sum(map(len, lines[:line-1])) +
                    len(lines[line-1].encode('utf-8')[:column].decode('utf-8')))

        replacements = []
        if current.keys() == values.keys():
            for key, value in zip(node.keys, node.values):
                replacements.append((offset(value.lineno, value.col_offset),
                                     offset(value.end_lineno, value.end_col_offset),
                                     repr(values[ast.literal_eval(key)])))
        else:
            replacements.append((offset(node.lineno, node.col_offset),
                                 offset(node.end_lineno, node.end_col_offset),
                                 pformat(values, sort_dicts=False)))
        for start, end, value in sorted(replacements, reverse=True):
            source = source[:start] + value + source[end:]
        # Verify the generated configuration before replacing the original.
        ast.literal_eval(_assignment(source, section))
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(source, encoding='utf-8')
    temporary.replace(path)


# 椭圆口缘几何与调整

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
    distance_mm: float | None = None

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
        if self.distance_mm is not None:
            value = self.distance_mm
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not np.isfinite(value) or value < 0.):
                raise ValueError('Mouth distance_mm must be a finite nonnegative number')

def load_mouth_settings(path: Path = CONFIG_PATH) -> MouthSettings:
    return MouthSettings(**load_section('MOUTH', path))

def save_mouth_settings(settings: MouthSettings, path: Path = CONFIG_PATH) -> None:
    """Save mouth parameters without changing the saved jaw opening."""
    save_section('MOUTH', asdict(settings), path)

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
        self.prefix = prefix
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
        self.tooth_front_mm = self._tooth_front_mm(prefix)

    def _tooth_front_mm(self, prefix):
        """Frontmost vertex of both jaws along the mouth parent's local +Z."""
        m = self.model
        root_id = int(m.body_parentid[self.body_id])
        fronts = []
        for jaw in ('mandibular', 'maxillary'):
            geom_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, prefix+jaw+'_visual')
            if geom_id < 0:
                continue
            mesh_id = int(m.geom_dataid[geom_id])
            start, count = int(m.mesh_vertadr[mesh_id]), int(m.mesh_vertnum[mesh_id])
            points = m.mesh_vert[start:start+count].astype(float)
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, m.geom_quat[geom_id])
            points = points @ rotation.reshape(3, 3).T + m.geom_pos[geom_id]
            body_id = int(m.geom_bodyid[geom_id])
            while body_id != root_id:
                if body_id == 0:
                    raise ValueError('Mouth and jaw meshes must share the teeth coordinate frame')
                mujoco.mju_quat2Mat(rotation, m.body_quat[body_id])
                points = points @ rotation.reshape(3, 3).T + m.body_pos[body_id]
                body_id = int(m.body_parentid[body_id])
            fronts.append(float(points[:, 2].max())*1000.)
        return max(fronts) if fronts else None

    def center_mm(self, settings: MouthSettings):
        self.tooth_front_mm = self._tooth_front_mm(self.prefix)
        center = np.array(settings.center_mm)
        if settings.distance_mm is not None:
            if self.tooth_front_mm is None:
                raise ValueError('Mouth distance requires at least one jaw mesh in the scene')
            center[2] = self.tooth_front_mm + settings.distance_mm
        return center

    def distance_mm(self, settings: MouthSettings):
        center = self.center_mm(settings)
        if self.tooth_front_mm is None:
            return None
        return float(center[2]-self.tooth_front_mm)

    def refresh_position(self, settings: MouthSettings, data=None):
        """Reapply tooth-relative placement after a jaw pose change."""
        self.model.body_pos[self.body_id] = self.center_mm(settings)/1000.
        if data is not None:
            mujoco.mj_forward(self.model, data)

    def update(self, settings: MouthSettings, data=None):
        m = self.model
        center = self.center_mm(settings)
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
        m.body_pos[self.body_id] = center/1000.
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

def mouth_overrides(settings, width_mm=None, height_mm=None, forward_mm=None, enabled=None,
                   distance_mm=None):
    if forward_mm is not None and distance_mm is not None:
        raise ValueError('Specify mouth distance or absolute forward position, not both')
    values = {}
    if width_mm is not None:
        values['width_mm'] = width_mm
    if height_mm is not None:
        values['height_mm'] = height_mm
    if forward_mm is not None:
        values['center_mm'] = (*settings.center_mm[:2], forward_mm)
        values['distance_mm'] = None
    if distance_mm is not None:
        values['distance_mm'] = distance_mm
    if enabled is not None:
        values['enabled'] = enabled
    return replace(settings, **values)


# 下颌开合

MIN_OPENING_DEG = -10.

MAX_OPENING_DEG = 40.

@dataclass(frozen=True)
class JawSettings:
    opening_deg: float = 0.
    hinge_center_mm: tuple[float, float, float] = (0., 10., -40.)

    def __post_init__(self):
        angle = self.opening_deg
        if (isinstance(angle, bool) or not isinstance(angle, (int, float))
                or not np.isfinite(angle)
                or not MIN_OPENING_DEG <= angle <= MAX_OPENING_DEG):
            raise ValueError('Jaw opening_deg must be a finite number between -10 and 40 degrees')
        try:
            center = np.asarray(self.hinge_center_mm, dtype=float)
        except (TypeError, ValueError) as error:
            raise ValueError('Jaw hinge_center_mm must contain three finite coordinates') from error
        if center.shape != (3,) or not np.all(np.isfinite(center)):
            raise ValueError('Jaw hinge_center_mm must contain three finite coordinates')
        object.__setattr__(self, 'hinge_center_mm', tuple(float(x) for x in center))

def load_jaw_settings(path: Path = CONFIG_PATH) -> JawSettings:
    return JawSettings(**load_section('JAW', path))

def save_jaw_settings(settings: JawSettings, path: Path = CONFIG_PATH) -> None:
    """Save jaw parameters without changing the saved mouth dimensions."""
    save_section('JAW', asdict(settings), path)

def jaw_overrides(settings: JawSettings, opening_deg=None) -> JawSettings:
    return settings if opening_deg is None else replace(settings, opening_deg=opening_deg)

class JawOpening:
    """Rotate a fixed lower jaw, preserving the upper jaw and robot state.

    Construct once while the model is in its reference pose, before applying
    any opening. Reuse this controller for subsequent absolute angle updates.
    Call under viewer.lock() when a passive viewer is attached.
    """

    def __init__(self, model, prefix='dental_'):
        self.model = model
        self.body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, prefix+'mandibular')
        upper_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, prefix+'maxillary')
        if self.body_id < 0 or upper_id < 0:
            raise ValueError('Jaw opening requires mandibular and maxillary bodies')
        if model.body_parentid[self.body_id] != model.body_parentid[upper_id]:
            raise ValueError('Jaw bodies must share a parent teeth frame')
        if model.body_jntnum[self.body_id] != 0:
            raise ValueError('Jaw opening requires a fixed mandibular body')
        self.reference_position = model.body_pos[self.body_id].copy()
        self.reference_quaternion = model.body_quat[self.body_id].copy()
        # mj_setConst needs scratch data; never reset the caller's robot state.
        self.scratch = mujoco.MjData(model)

    def update(self, settings: JawSettings, data=None):
        angle = np.deg2rad(settings.opening_deg)
        quaternion = np.array([np.cos(angle/2.), np.sin(angle/2.), 0., 0.])
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, quaternion)
        hinge = np.asarray(settings.hinge_center_mm)/1000.
        self.model.body_pos[self.body_id] = (
            hinge + rotation.reshape(3, 3) @ (self.reference_position-hinge))
        mujoco.mju_mulQuat(self.model.body_quat[self.body_id], quaternion,
                           self.reference_quaternion)
        # Rebuild constants for the moved static collision geometry as well.
        mujoco.mj_setConst(self.model, self.scratch)
        self.settings = settings
        if data is not None:
            mujoco.mj_forward(self.model, data)


# 预览相机与图片导出

def closeup_camera(camera, data):
    root = data.body('dental_teeth_root')
    rotation = root.xmat.reshape(3, 3)
    target = root.xpos + rotation @ np.array([0., .010, .025])
    direction = rotation[:, 2]
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = target
    camera.distance = .28
    # MuJoCo azimuth/elevation describe the direction from eye to target.
    camera.azimuth = np.degrees(np.arctan2(direction[1], direction[0])) + 180.
    camera.elevation = -np.degrees(np.arcsin(direction[2]))

def render_preview(model, data, path):
    from PIL import Image
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(model, height=720, width=960) as renderer:
        camera = mujoco.MjvCamera()
        closeup_camera(camera, data)
        renderer.update_scene(data, camera=camera)
        Image.fromarray(renderer.render()).save(path)


def main(argv=None):
    """Adjust mouth dimensions and jaw opening together, without robot motion."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--width-mm', type=float, help='Clear opening width, 10-110 mm')
    parser.add_argument('--height-mm', type=float, help='Clear opening height, 10-130 mm')
    parser.add_argument('--opening-deg', type=float, help='Lower-jaw angle, -10 to 40 degrees')
    position = parser.add_mutually_exclusive_group()
    position.add_argument('--forward-mm', type=float, help='Absolute mask position along scan +Z')
    position.add_argument('--distance-mm', type=float, help='Gap from frontmost tooth to mask, mm (>=0)')
    parser.add_argument('--no-mouth', action='store_true', help='Hide the mask')
    parser.add_argument('--save', action='store_true', help='Save both groups to the unified config')
    parser.add_argument('--render', type=Path, help='Save a PNG and exit without a viewer')
    args = parser.parse_args(argv)
    try:
        mouth_settings = mouth_overrides(load_mouth_settings(), args.width_mm, args.height_mm,
                                         args.forward_mm, enabled=False if args.no_mouth else None,
                                         distance_mm=args.distance_mm)
        jaw_settings = jaw_overrides(load_jaw_settings(), args.opening_deg)
        # Import locally so simulation can use this module without a cycle.
        from pipeline.simulation import create_simulation
        model, data = create_simulation(mouth_settings=mouth_settings,
                                         jaw_settings=replace(jaw_settings, opening_deg=0.))
        jaw, mouth = JawOpening(model), MouthOpening(model)

        def apply_settings():
            jaw.update(jaw_settings, data)
            mouth.update(mouth_settings, data)

        def save_current():
            save_mouth_settings(mouth_settings)
            save_jaw_settings(jaw_settings)
            print(f'Saved mouth and jaw: {CONFIG_PATH}')

        def print_current():
            print(f'Mouth {mouth_settings.width_mm:g} x {mouth_settings.height_mm:g} mm; '
                  f'distance {mouth.distance_mm(mouth_settings):.2f} mm; '
                  f'jaw {jaw_settings.opening_deg:g} degrees; visible={mouth_settings.enabled}')

        apply_settings()
        model.geom_rgba[model.geom('dental_dental_chair_visual').id, 3] = 0.
        print_current()
        if args.save:
            save_current()
        if args.render:
            render_preview(model, data, args.render)
            print(f'Preview: {args.render.resolve()}')
            return 0

        from mujoco import viewer
        pending = SimpleQueue()
        print('J/L: width -/+ 2 mm; K/I: height -/+ 2 mm; U/O: distance -/+ 2 mm')
        print('Z/X: jaw close/open 1 degree; arrows: mask position; S: save both; R: reset both')
        print('M: show/hide mask; C: front view. Close to exit; changes persist only after S.')
        with viewer.launch_passive(model, data, key_callback=pending.put) as window:
            with window.lock():
                closeup_camera(window.cam, data)
            while window.is_running():
                while True:
                    try:
                        key = pending.get_nowait()
                    except Empty:
                        break
                    try:
                        if key in (ord('J'), ord('L')):
                            mouth_settings = replace(mouth_settings, width_mm=mouth_settings.width_mm +
                                                     (-2 if key == ord('J') else 2))
                        elif key in (ord('K'), ord('I')):
                            mouth_settings = replace(mouth_settings, height_mm=mouth_settings.height_mm +
                                                     (-2 if key == ord('K') else 2))
                        elif key in (ord('U'), ord('O')):
                            distance = mouth.distance_mm(mouth_settings) + (-2 if key == ord('U') else 2)
                            mouth_settings = replace(mouth_settings, distance_mm=max(0., distance))
                        elif key in (ord('Z'), ord('X')):
                            jaw_settings = replace(jaw_settings, opening_deg=jaw_settings.opening_deg +
                                                   (-1 if key == ord('Z') else 1))
                        elif key in (262, 263, 264, 265):
                            center = list(mouth_settings.center_mm)
                            axis, amount = {262: (0, -2), 263: (0, 2),
                                            264: (1, -2), 265: (1, 2)}[key]
                            center[axis] += amount
                            mouth_settings = replace(mouth_settings, center_mm=center)
                        elif key == ord('R'):
                            mouth_settings = MouthSettings()
                            jaw_settings = replace(jaw_settings, opening_deg=0.)
                        elif key == ord('M'):
                            mouth_settings = replace(mouth_settings, enabled=not mouth_settings.enabled)
                        elif key == ord('S'):
                            save_current()
                            continue
                        elif key == ord('C'):
                            with window.lock():
                                closeup_camera(window.cam, data)
                            continue
                        else:
                            continue
                        with window.lock():
                            apply_settings()
                        # update_mesh takes its own lock; do not nest it in window.lock().
                        window.update_mesh(mouth.mesh_id)
                        print_current()
                    except ValueError as error:
                        print(error)
                window.sync()
                time.sleep(.02)
        return 0
    except (ValueError, OSError) as error:
        print(f'Oral preview failed: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
