"""MuJoCo scene setup and lightweight trajectory drawing for this pipeline."""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DENTAL_MODEL_PATH = PROJECT_ROOT / 'models' / 'scene_ur7e_dental.xml'


def create_simulation(model_path: Path = DENTAL_MODEL_PATH, mouth_settings=None):
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
    # Apply the saved dimensions to this model only. Other/custom MJCF models
    # can still use the generic loader without requiring a dental face mask.
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'dental_mouth_opening') >= 0:
        from pipeline.mouth import MouthOpening, load_settings
        MouthOpening(model).update(load_settings() if mouth_settings is None else mouth_settings)
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
