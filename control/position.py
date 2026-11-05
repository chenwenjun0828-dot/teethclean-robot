"""MuJoCo position-servo tracking and runtime checks."""
from __future__ import annotations
import mujoco
import numpy as np
from planning.validation import robot_contacts
from planning.ik import rotation_error
from planning.trajectory import TrackingPlan

def step_checked(model, data, steps=10):
    """Check every physics substep so a transient contact cannot hide in a batch."""
    for _ in range(steps):
        mujoco.mj_step(model, data)
        contacts = robot_contacts(model, data)
        if contacts:
            raise RuntimeError('Robot contact during physics substep: '+contacts[0])
        if any(warning.number for warning in data.warning):
            raise RuntimeError('MuJoCo reported a simulation warning')
    mujoco.mj_forward(model, data)
    contacts = robot_contacts(model, data)
    if contacts:
        raise RuntimeError('Robot contact after physics substep: '+contacts[0])


class GingivalTCPController:
    def __init__(self, model, plan: TrackingPlan):
        self.model, self.plan = model, plan
        self.start_time = None
        self.gain = model.actuator_gainprm[:, 0].copy()
        self.damping = -model.actuator_biasprm[:, 2].copy()
        if np.any(self.gain <= 0) or not np.allclose(model.actuator_biasprm[:, 1], -self.gain):
            raise ValueError('Tracking requires affine position servos with positive gains')

    def update(self, data):
        if self.start_time is None:
            self.start_time = data.time
        time_s = data.time-self.start_time
        position, q, phase = self.plan.sample(time_s)
        contacts = robot_contacts(self.model, data)
        if contacts:
            raise RuntimeError('Robot contact during execution: '+contacts[0])
        error = np.linalg.norm(data.site_xpos[self.plan.site_id]-position)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError('Non-finite robot state')
        if time_s > 0.2 and error > 0.005:
            raise RuntimeError(f'TCP tracking error exceeds 5 mm: {error*1000:.2f} mm')
        if phase == 'trace':
            angle_deg = np.rad2deg(np.linalg.norm(rotation_error(
                self.plan.sample_rotation(time_s),
                data.site_xmat[self.plan.site_id].reshape(3, 3))))
            if angle_deg > 5.:
                raise RuntimeError(f'TCP orientation tracking error exceeds 5 degrees: {angle_deg:.2f}')
        dt = 0.002
        _, q_next, _ = self.plan.sample(time_s+dt)
        velocity = (q_next-q)/dt
        control = q+(data.qfrc_bias+self.damping*velocity)/self.gain
        data.ctrl[:] = np.clip(control, self.model.actuator_ctrlrange[:, 0], self.model.actuator_ctrlrange[:, 1])
        return position, phase

