"""MuJoCo scene setup and lightweight trajectory drawing for this pipeline."""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DENTAL_MODEL_PATH = PROJECT_ROOT / 'assets' / 'simulation' / 'scene_ur7e_dental.xml'


def body_transform(model, data, body_name):
    """Return a MuJoCo body pose as a world-from-body transform."""
    body = model.body(body_name).id
    transform = np.eye(4)
    transform[:3, :3] = data.xmat[body].reshape(3, 3)
    transform[:3, 3] = data.xpos[body]
    return transform


def set_body_world_transform(model, data, body_name, transform):
    """Set a fixed body's local pose so its world pose equals ``transform``."""
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError(f'{body_name} transform must be a finite 4x4 matrix')
    parent = int(model.body_parentid[model.body(body_name).id])
    parent_transform = np.eye(4) if parent == 0 else body_transform(model, data, model.body(parent).name)
    local = np.linalg.inv(parent_transform) @ transform
    rotation = local[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-7)
            or not np.allclose(local[3], [0., 0., 0., 1.], atol=1e-8)):
        raise ValueError(f'{body_name} transform is not a rigid pose')
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, rotation.reshape(-1))
    body = model.body(body_name).id
    model.body_pos[body] = local[:3, 3]
    model.body_quat[body] = quaternion


def apply_dental_transforms(model, data, jaw_transforms_m):
    """Place the static jaw bodies from scan poses in the MuJoCo world."""
    if jaw_transforms_m is None:
        return
    transforms = {str(jaw): np.asarray(value, dtype=float)
                  for jaw, value in jaw_transforms_m.items()}
    unknown = set(transforms) - {'mandibular', 'maxillary'}
    if unknown:
        raise ValueError(f'Unknown dental transform(s): {sorted(unknown)}')
    if not transforms:
        return
    # Both jaw bodies share teeth_root. Use one jaw as the root pose, then set
    # each child local pose so lower/upper can retain independent measured poses.
    mujoco.mj_forward(model, data)
    root_pose = transforms.get('mandibular', transforms.get('maxillary'))
    set_body_world_transform(model, data, 'dental_teeth_root', root_pose)
    mujoco.mj_forward(model, data)
    for jaw, transform in transforms.items():
        set_body_world_transform(model, data, f'dental_{jaw}', transform)
        mujoco.mj_forward(model, data)


def create_simulation(model_path: Path = DENTAL_MODEL_PATH, mouth_settings=None,
                      jaw_settings=None, robot_mount_position_m=None,
                      tooth_transforms_m=None):
    path = Path(model_path).resolve()
    if not path.is_file():
        raise FileNotFoundError(f'MuJoCo model not found: {path}')
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'home')
    if home_id < 0:
        home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'ur_home')
    if home_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_id)
    else:
        mujoco.mj_resetData(model, data)
    # Opening must precede the mask's tooth-relative distance calculation.
    has_lower = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'dental_mandibular') >= 0
    has_upper = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'dental_maxillary') >= 0
    if (has_lower and has_upper) or jaw_settings is not None:
        from pipeline.oral import JawOpening, load_jaw_settings
        JawOpening(model).update(load_jaw_settings() if jaw_settings is None else jaw_settings)
    # Apply the saved dimensions to this model only. Other/custom MJCF models
    # can still use the generic loader without requiring a dental face mask.
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'dental_mouth_opening') >= 0:
        from pipeline.oral import MouthOpening, load_mouth_settings
        MouthOpening(model).update(load_mouth_settings() if mouth_settings is None else mouth_settings)
    if robot_mount_position_m is not None:
        mount_id = model.body('robot_mount').id
        position = np.asarray(robot_mount_position_m, float)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('robot_mount_position_m must be a finite 3-vector')
        model.body_pos[mount_id] = position
    apply_dental_transforms(model, data, tooth_transforms_m)
    mujoco.mj_forward(model, data)
    return model, data


def add_trajectory_line(scene, start, end, color, width=2.0) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE,
                        np.ones(3), np.zeros(3), np.eye(3).ravel(),
                        np.asarray(color, dtype=np.float32))
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, width, start, end)
    scene.ngeom += 1


def draw_tcp_axes(scene, data, site_id, length_m=0.012) -> None:
    origin = data.site_xpos[site_id]
    rotation = data.site_xmat[site_id].reshape(3, 3)
    add_trajectory_line(scene, origin, origin + length_m * rotation[:, 0],
                        (1., 0.08, 0.06, 1.), 5.)
    add_trajectory_line(scene, origin, origin + length_m * rotation[:, 1],
                        (1., 0.94, 0.04, 1.), 5.)
