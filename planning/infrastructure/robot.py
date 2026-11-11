"""Six-axis robot adapter with exact UR branches and independent FK checks."""
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def rotation_error(target, actual):
    return Rotation.from_matrix(target @ actual.T).as_rotvec()


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class Robot:
    def __init__(self, model, data):
        self.model = model
        names = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
        self.joint_names = ['ur_'+n for n in names]
        self.joints = np.array([model.joint(n).id for n in self.joint_names])
        self.qadr = model.jnt_qposadr[self.joints]
        self.dadr = model.jnt_dofadr[self.joints]
        self.lower, self.upper = model.jnt_range[self.joints].T.copy()
        self.lower += 1e-6; self.upper -= 1e-6
        self.site = model.site('ur_scaler_tcp').id
        self.initial_qpos = data.qpos.copy()
        self.home = data.qpos[self.qadr].copy()
        self.data = mujoco.MjData(model)
        self.verify_data = mujoco.MjData(model)
        self.base = model.body('ur_base').id
        self.shoulder = model.body('ur_shoulder_link').id
        self.wrist = model.body('ur_wrist_3_link').id
        self.fk(np.zeros(6))
        self.base_rotation = self.data.xmat[self.base].reshape(3, 3).copy()
        self.origin = self.data.xpos[self.shoulder].copy()
        wrist = np.eye(4)
        wrist[:3, :3] = self.data.xmat[self.wrist].reshape(3, 3)
        wrist[:3, 3] = self.data.xpos[self.wrist]
        self.tcp_to_wrist = np.linalg.inv(np.linalg.inv(wrist) @ self.pose(self.data))
        body_names = ['shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']
        ids = [model.body('ur_'+name).id for name in body_names]
        expected = [[0, 0, .163], [0, .138, 0], [0, -.131, .425], [0, 0, .392], [0, .127, 0], [0, 0, .1]]
        axes = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 0, 1], [0, 1, 0]]
        self.analytic = (np.allclose(model.body_pos[ids], expected, atol=1e-10)
                         and np.allclose(model.jnt_axis[self.joints], axes)
                         and np.allclose(model.jnt_pos[self.joints], 0))
        for i, body in enumerate(ids):
            mat = np.empty(9)
            mujoco.mju_quat2Mat(mat, model.body_quat[body])
            self.analytic &= np.allclose(mat.reshape(3, 3), ry(np.pi/2) if i in (1, 3) else np.eye(3))

    def pose(self, data):
        pose = np.eye(4)
        pose[:3, :3] = data.site_xmat[self.site].reshape(3, 3)
        pose[:3, 3] = data.site_xpos[self.site]
        return pose

    def fk(self, q, full=False):
        data = self.verify_data if full else self.data
        data.qpos[:] = self.initial_qpos
        data.qpos[self.qadr] = q
        data.qvel[:] = 0
        if full:
            mujoco.mj_forward(self.model, data)
        else:
            mujoco.mj_kinematics(self.model, data)
            mujoco.mj_comPos(self.model, data)
        return self.pose(data)

    def jacobian(self, q):
        """World TCP translational/rotational Jacobian in trajectory joint order."""
        self.fk(q)
        linear, angular = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, linear, angular, self.site)
        return np.vstack((linear[:, self.dadr], angular[:, self.dadr]))

    def _branches(self, target):
        wrist = target @ self.tcp_to_wrist
        p = wrist[:3, 3]-self.origin
        local = self.base_rotation.T @ p
        radius = np.linalg.norm(local[:2])
        if radius < .134:
            return []
        theta = np.arctan2(local[1], local[0]); alpha = np.arcsin(.134/radius)
        solutions = []
        for q1 in (theta-alpha, theta-np.pi+alpha):
            r1 = self.base_rotation @ rz(q1)
            y1 = r1[:, 1]
            cross = np.cross(y1, wrist[:3, 1]); length = np.linalg.norm(cross)
            if length < 1e-8:
                continue
            for sign in (-1., 1.):
                axis = sign*cross/length
                arm = p-.134*y1-.100*axis
                cosine = (arm@arm-.425**2-.392**2)/(2*.425*.392)
                if abs(cosine) > 1.+1e-10:
                    continue
                v, a = r1.T @ arm, r1.T @ axis
                phi = np.arctan2(a[0], a[2])
                orient = (r1 @ ry(phi)).T @ wrist[:3, :3]
                q5 = np.arctan2(-orient[0, 1], orient[1, 1])
                q6 = np.arctan2(-orient[2, 0], orient[2, 2])
                for q3 in (np.arccos(np.clip(cosine, -1, 1)), -np.arccos(np.clip(cosine, -1, 1))):
                    q2 = np.arctan2(-v[2], v[0])-np.arctan2(.392*np.sin(q3), .425+.392*np.cos(q3))
                    q4 = phi-np.pi-q2-q3
                    solutions.append((np.array([q1, q2, q3, q4, q5, q6])+np.pi) % (2*np.pi)-np.pi)
        return solutions

    def solutions(self, target, seed=None):
        seed = self.home if seed is None else np.asarray(seed)
        candidates = self._branches(target) if self.analytic else []
        if not candidates:
            # A bounded numerical fallback also handles degenerate analytic branches.
            if self.analytic:
                # Outside the exact analytic workspace: do not spend minutes on retries.
                return []
            def residual(q):
                actual = self.fk(q)
                return np.r_[actual[:3, 3]-target[:3, 3], .1*rotation_error(target[:3, :3], actual[:3, :3])]
            for start in (seed, self.home, np.zeros(6)):
                result = least_squares(residual, np.clip(start, self.lower, self.upper),
                                       bounds=(self.lower, self.upper), max_nfev=120,
                                       ftol=1e-11, xtol=1e-11, gtol=1e-11)
                candidates.append(result.x)
        result = []
        for q in candidates:
            q = q + 2*np.pi*np.round((seed-q)/(2*np.pi))
            for j in range(6):
                while q[j] > self.upper[j]: q[j] -= 2*np.pi
                while q[j] < self.lower[j]: q[j] += 2*np.pi
            if np.any(q < self.lower) or np.any(q > self.upper):
                continue
            actual = self.fk(q, full=True)
            if (np.linalg.norm(actual[:3, 3]-target[:3, 3]) < .00005 and
                    np.linalg.norm(rotation_error(target[:3, :3], actual[:3, :3])) < np.deg2rad(.05)):
                if not any(np.max(abs(q-old)) < 1e-5 for old in result):
                    result.append(q.copy())
        return sorted(result, key=lambda q: np.linalg.norm(q-seed))
