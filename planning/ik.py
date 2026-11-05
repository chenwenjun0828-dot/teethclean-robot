"""Six-joint TCP inverse kinematics."""
from __future__ import annotations
import math
import mujoco
import numpy as np

def rotation_error(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    """World-axis rotation logarithm, including the 180-degree case."""
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, np.ascontiguousarray(target @ current.T).ravel())
    if quaternion[0] < 0:
        quaternion *= -1
    norm = np.linalg.norm(quaternion[1:])
    if norm < 1e-12:
        return 2*quaternion[1:]
    return quaternion[1:] * (2*math.atan2(norm, quaternion[0])/norm)


class FixedPoseIK:
    """Solve one six-dimensional pose; consecutive calls may change orientation."""
    def __init__(self, model, site_name='ur_scaler_tcp'):
        self.model = model
        self.data = mujoco.MjData(model)
        self.site_id = model.site(site_name).id
        # These trajectories explicitly support the existing scalar hinge servo model.
        if model.nq != 6 or model.nv != 6 or model.nu != 6:
            raise ValueError('Gingival tracking requires the six-hinge UR model')
        if (not np.array_equal(model.actuator_trnid[:, 0], np.arange(6))
                or not np.array_equal(model.jnt_qposadr, np.arange(6))
                or not np.allclose(model.actuator_gear[:, 0], 1)):
            raise ValueError('Expected one unit-gear position actuator per joint')
        self.lower = model.jnt_range[:, 0]+1e-4
        self.upper = model.jnt_range[:, 1]-1e-4
        self.jacp = np.zeros((3, 6))
        self.jacr = np.zeros((3, 6))

    def solve(self, position, rotation, seed, iterations=240):
        q = np.clip(np.asarray(seed, float), self.lower, self.upper)
        damping = 1e-6

        def residual(candidate):
            self.data.qpos[:] = candidate
            self.data.qvel[:] = 0
            mujoco.mj_forward(self.model, self.data)
            ep = position-self.data.site_xpos[self.site_id]
            er = rotation_error(rotation, self.data.site_xmat[self.site_id].reshape(3, 3))
            return ep, er, np.r_[ep, 0.15*er]

        for _ in range(iterations):
            ep, er, error = residual(q)
            if np.linalg.norm(ep) < 2e-5 and np.linalg.norm(er) < 2e-4:
                return q.copy()
            mujoco.mj_jacSite(self.model, self.data, self.jacp, self.jacr, self.site_id)
            jac = np.vstack((self.jacp, 0.15*self.jacr))
            delta = jac.T @ np.linalg.solve(jac @ jac.T+damping*np.eye(6), error)
            # Scale the whole step: component clipping can reverse its descent direction.
            delta *= min(1., 0.12/max(np.max(np.abs(delta)), 1e-12))
            for scale in (1., .5, .25, .125, .0625):
                candidate = np.clip(q+scale*delta, self.lower, self.upper)
                _, _, next_error = residual(candidate)
                if np.dot(next_error, next_error) < np.dot(error, error):
                    q = candidate
                    damping = max(1e-8, damping*.5)
                    break
            else:
                damping = min(1., damping*10.)
        ep, er, _ = residual(q)
        raise ValueError(f'IK target unreachable: position residual {np.linalg.norm(ep)*1000:.2f} mm, '
                         f'orientation residual {np.rad2deg(np.linalg.norm(er)):.3f} deg')

