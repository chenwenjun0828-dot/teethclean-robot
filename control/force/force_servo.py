"""Cartesian position servo with bounded normal and axial corrections."""
import mujoco
import numpy as np
from planning.infrastructure.robot import rotation_error


def apply_tool_roll(pose, radians):
    """Rotate only about tool +X: preserve TCP position and 15-degree shaft tilt."""
    result = pose.copy()
    c, s = np.cos(radians), np.sin(radians)
    result[:3, :3] = pose[:3, :3]@np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])
    return result


class CartesianServo:
    """Keep nominal tangential position/orientation while applying normal offset."""
    def __init__(self, model, robot):
        self.model, self.robot = model, robot
        self.last_q, self.last_dq = None, np.zeros(6)
        self.gain = model.actuator_gainprm[:, 0].copy()
        self.damping = -model.actuator_biasprm[:, 2].copy()
        if (model.nu != 6 or np.any(self.gain <= 0)
                or not np.allclose(model.actuator_biasprm[:, 1], -self.gain)
                or not np.array_equal(model.actuator_trnid[:, 0], robot.joints)):
            raise ValueError('Requires six position actuators in robot joint order')

    def command(self, data, state, dt):
        m, robot = self.model, self.robot
        nominal = robot.fk(state['q']).copy()
        target = apply_tool_roll(nominal, state.get('roll_correction_rad', 0.))
        target[:3, 3] += state['offset_world_m']
        q = state['q'].copy()
        # Re-linearize twice: unlike a single Jacobian offset, this also removes
        # the orientation/position residual when the tooth normal changes.
        for _ in range(3):
            actual = robot.fk(q)
            error = np.r_[target[:3, 3]-actual[:3, 3], rotation_error(target[:3, :3], actual[:3, :3])]
            if np.linalg.norm(error) < 1e-9:
                break
            jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
            mujoco.mj_jacSite(m, robot.data, jp, jr, robot.site)
            jac = np.vstack([jp[:, robot.dadr], jr[:, robot.dadr]])
            q += jac.T @ np.linalg.solve(jac@jac.T+1e-10*np.eye(6), error)
        solved = robot.fk(q)
        if (np.linalg.norm(solved[:3, 3]-target[:3, 3]) > 1e-5
                or np.linalg.norm(rotation_error(target[:3, :3], solved[:3, :3])) > 1e-4
                or np.any(q < robot.lower) or np.any(q > robot.upper)
                or np.max(abs(q-state['q'])) > (.7 if state.get('roll_correction_rad', 0.) else .15)):
            raise RuntimeError('Force correction failed bounded inverse kinematics')
        dq = state['dq'].copy() if self.last_q is None else (q-self.last_q)/dt
        ddq = state['ddq'].copy() if self.last_q is None else (dq-self.last_dq)/dt
        inertia = np.empty((m.nv, m.nv))
        if hasattr(data, 'qM'):
            mujoco.mj_fullM(m, inertia, data.qM)
        else:
            mujoco.mj_fullM(m, data, inertia)
        torque = inertia[np.ix_(robot.dadr, robot.dadr)]@state['ddq'] + data.qfrc_bias[robot.dadr]
        ctrl = q+(torque+self.damping*dq)/self.gain
        if np.any(ctrl < m.actuator_ctrlrange[:, 0]) or np.any(ctrl > m.actuator_ctrlrange[:, 1]):
            raise RuntimeError('Force servo actuator command saturated')
        self.last_q, self.last_dq = q.copy(), dq.copy()
        return ctrl, q, dq, ddq, nominal, target
