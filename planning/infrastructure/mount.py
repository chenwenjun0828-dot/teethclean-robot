"""Apply an explicitly selected simulation mount position."""
import numpy as np


def apply_mount_position(model, position_m, data=None):
    """Apply a runtime robot mount translation without changing the XML asset."""
    import mujoco
    position = np.asarray(position_m, float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError('robot_mount_position_m must be a finite 3-vector')
    model.body_pos[model.body('robot_mount').id] = position
    if data is not None:
        mujoco.mj_forward(model, data)
