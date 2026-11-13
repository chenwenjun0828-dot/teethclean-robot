"""MuJoCo adapter implementing the common hardware protocol."""

from __future__ import annotations

import numpy as np
import mujoco

from .protocol import HardwareState


class MujocoHardwareBackend:
    def __init__(self, model, data, joint_ids=None, site_name='ur_scaler_tcp'):
        self.model, self.data = model, data
        if joint_ids is None:
            joint_ids = (model.actuator_trnid[:, 0] if model.nu
                         else np.arange(model.nq))
        self.joint_ids = np.asarray(joint_ids, int)
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        self.connected = False
        self.stopped = False

    def connect(self):
        if self.site_id < 0:
            raise ValueError(f'MuJoCo site not found: {self.site_id}')
        self.connected = True
        self.stopped = False

    def disconnect(self):
        self.connected = False

    def _require_connected(self):
        if not self.connected:
            raise RuntimeError('MuJoCo backend is not connected')
        if self.stopped:
            raise RuntimeError('MuJoCo backend is stopped')

    def read_state(self):
        self._require_connected()
        pose = np.eye(4)
        pose[:3, :3] = self.data.site_xmat[self.site_id].reshape(3, 3)
        pose[:3, 3] = self.data.site_xpos[self.site_id]
        q = self.data.qpos[self.joint_ids].copy()
        dq = self.data.qvel[self.joint_ids].copy()
        return HardwareState(float(self.data.time), q, dq, pose,
                             np.zeros(6, dtype=float), True)

    def read_force(self):
        return self.read_state().wrench_N_m

    def command_joint_position(self, position_rad):
        self._require_connected()
        value = np.asarray(position_rad, float)
        if value.shape != self.joint_ids.shape:
            raise ValueError('joint position shape does not match backend')
        self.data.ctrl[:len(value)] = value
        mujoco.mj_step(self.model, self.data)

    def command_joint_velocity(self, velocity_rad_s):
        raise NotImplementedError('MuJoCo position actuators do not expose velocity commands')

    def emergency_stop(self, reason):
        self.stopped = True
        self.data.ctrl[:] = 0.
