"""One-step physical contact guard; independent predicted state only."""
import mujoco
import numpy as np
from .contact_experiment import contacts


class PredictiveContactGuard:
    """Do not commit a simulated step that already predicts a rejected state."""
    def __init__(self, model, robot, jaw, observer, force_limit):
        self.model, self.robot, self.jaw = model, robot, jaw
        self.observer, self.force_limit = observer, force_limit
        self.trial = mujoco.MjData(model)

    def check(self, data, command, phase):
        mujoco.mj_copyData(self.trial, self.model, data)
        self.trial.ctrl[:] = command
        # Same pre-step refresh and integration order as the committed execution.
        mujoco.mj_forward(self.model, self.trial)
        immediate = self._assess(phase)
        if not immediate['safe']:
            return self._result(immediate, 'command_application')
        mujoco.mj_step(self.model, self.trial)
        mujoco.mj_forward(self.model, self.trial)
        return dict(self._result(self._assess(phase), 'next_step'),
                    command_application_force_N=immediate['force_N'])

    def _result(self, assessment, stage):
        return dict(assessment, rejected_at=stage, time_s=float(self.trial.time),
                    joint_positions_rad=self.trial.qpos[self.robot.qadr].copy()
                    if hasattr(self.robot, 'qadr') else None,
                    tcp_world_m=self.trial.site_xpos[self.robot.site].copy())

    def _assess(self, phase):
        force = self.observer.read(self.trial)['force_N']
        hits, bad = contacts(self.model, self.trial, self.robot, self.jaw)
        normal_sum = sum(max(0., h['normal_force_N']) for h in hits)
        reason = ''
        if bad:
            reason = 'Predicted unintended collision: ' + '; '.join(sorted(set(bad)))
        elif not np.isfinite(force) or force >= self.force_limit or normal_sum >= self.force_limit:
            reason = f'Predicted contact force reaches {self.force_limit:g} N guard'
        elif phase == 'transfer' and force >= .05:
            reason = 'Predicted contact during free transfer'
        elif not np.isfinite(self.trial.qpos).all() or any(w.number for w in self.trial.warning):
            reason = 'Predicted invalid dynamics'
        return {'safe': not reason, 'reason': reason, 'force_N': force,
                'contact_normal_sum_N': normal_sum, 'contacts': hits}
