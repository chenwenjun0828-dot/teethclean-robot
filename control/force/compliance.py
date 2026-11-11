"""Sensor-feedback normal admittance for the position-actuated MuJoCo robot.

The regulated quantity is compensated contact force magnitude (N). Compliance
changes only the normal position; nominal tangential motion/orientation remain.
The observer's inverse-dynamics compensation is simulation/model dependent.
"""
import copy
from dataclasses import dataclass, asdict

import mujoco
import numpy as np


@dataclass(frozen=True)
class ForceControlConfig:
    target_force_N: float = 1.
    force_stop_N: float = 3.
    virtual_mass_kg: float = 5.
    damping_Ns_m: float = 300.
    filter_time_s: float = .005
    target_ramp_s: float = .4
    contact_threshold_N: float = .05
    max_normal_speed_m_s: float = .002
    max_normal_accel_m_s2: float = .05
    max_displacement_m: float = .003

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be positive and finite')
        if not self.contact_threshold_N < self.target_force_N < self.force_stop_N:
            raise ValueError('Require contact threshold < target force < force stop limit')


class SensorContactForce:
    """Subtract same-state no-contact inverse dynamics from the actual sensor.

MuJoCo's force sensor reports parent-on-tool force. Consequently the external
contact force is reference minus measured, rotated from sensor to world axes.
No contact force solver output is used as feedback here.
"""
    def __init__(self, model):
        self.model = model
        self.reference_model = copy.copy(model)
        self.reference_model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self.reference = mujoco.MjData(self.reference_model)
        self.site = model.site('ur_scaler_force_site').id
        self.addresses = []
        for name in ('ur_scaler_force', 'ur_scaler_torque'):
            sensor = model.sensor(name).id
            if model.sensor_dim[sensor] != 3 or model.sensor_objid[sensor] != self.site:
                raise ValueError('Expected colocated three-axis force/torque sensors')
            self.addresses.append(int(model.sensor_adr[sensor]))

    def read(self, data):
        reference = self.reference
        reference.qpos[:] = data.qpos
        reference.qvel[:] = data.qvel
        reference.qacc[:] = data.qacc
        reference.time = data.time
        reference.xfrc_applied[:] = 0.
        reference.qfrc_applied[:] = 0.
        mujoco.mj_inverse(self.reference_model, reference)
        mujoco.mj_sensorAcc(self.reference_model, reference)
        raw = np.concatenate([data.sensordata[a:a+3] for a in self.addresses]).copy()
        unloaded = np.concatenate([reference.sensordata[a:a+3] for a in self.addresses])
        rotation = data.site_xmat[self.site].reshape(3, 3)
        external_force = rotation @ (unloaded[:3] - raw[:3])
        external_torque = rotation @ (unloaded[3:] - raw[3:])
        if not np.isfinite(np.r_[raw, external_force, external_torque]).all():
            raise RuntimeError('Nonfinite force sensor/compensation result')
        return {'raw_sensor_wrench': raw, 'unloaded_sensor_wrench': unloaded.copy(),
                'external_force_world_N': external_force,
                'external_torque_world_Nm': external_torque,
                'force_N': float(np.linalg.norm(external_force))}


class NormalAdmittance:
    """M dv/dt + B v = measured - desired; positive motion is outward."""
    def __init__(self, config):
        self.config = config
        self.displacement = 0.
        self.velocity = 0.
        self.filtered_force = 0.

    def step(self, measured_force, target_force, dt):
        c = self.config
        if not np.isfinite([measured_force, target_force, dt]).all() or dt <= 0 or dt > .02:
            raise ValueError('Invalid force sample or controller time step (max 20 ms)')
        if measured_force < 0 or not 0 <= target_force <= c.target_force_N:
            raise ValueError('Invalid force magnitude or target')
        alpha = -np.expm1(-dt / c.filter_time_s)
        self.filtered_force += alpha * (measured_force - self.filtered_force)
        equilibrium_velocity = (self.filtered_force - target_force) / c.damping_Ns_m
        decay = np.exp(-c.damping_Ns_m * dt / c.virtual_mass_kg)
        velocity = equilibrium_velocity + (self.velocity - equilibrium_velocity) * decay
        velocity = np.clip(velocity, self.velocity-c.max_normal_accel_m_s2*dt,
                            self.velocity+c.max_normal_accel_m_s2*dt)
        self.velocity = float(np.clip(velocity, -c.max_normal_speed_m_s, c.max_normal_speed_m_s))
        proposed = self.displacement + self.velocity * dt
        self.displacement = float(np.clip(proposed, -c.max_displacement_m, c.max_displacement_m))
        if self.displacement != proposed:
            self.velocity = 0.  # anti-windup at travel limit
        return self.displacement


def smoothstep(u):
    u = np.clip(u, 0., 1.)
    return u**3 * (10. - 15.*u + 6.*u*u)


class CompliantTCPController:
    """Normal force outer loop and the existing position-servo inner loop.

The caller owns collision/fixed-jaw checks and must stop on fault_reason.
Supported force phases: approach, contact_hold, trace; retract fades the
normal correction out over its interval without jumping to the zero target.
"""
    def __init__(self, model, robot, plan, normal_world, config=None):
        self.model, self.robot, self.plan = model, robot, plan
        self.config = config or ForceControlConfig()
        normal = np.asarray(normal_world, float)
        if normal.shape != (3,) or not np.isfinite(normal).all() or np.linalg.norm(normal) < 1e-10:
            raise ValueError('Expected nonzero finite world normal')
        self.normal = normal / np.linalg.norm(normal)
        self.observer = SensorContactForce(model)
        self.admittance = NormalAdmittance(self.config)
        self.contact_start = None
        self.anchor = None
        self.last_time = None
        self.last_q = None
        self.last_dq = np.zeros(6)
        self.release = None
        self.last_normal_command = None
        self.fault_reason = ''
        self.gain = model.actuator_gainprm[:, 0].copy()
        self.damping = -model.actuator_biasprm[:, 2].copy()
        if (model.nu != 6 or np.any(self.gain <= 0)
                or not np.allclose(model.actuator_biasprm[:, 1], -self.gain)
                or not np.array_equal(model.actuator_trnid[:, 0], robot.joints)):
            raise ValueError('Requires the six configured position actuators in robot joint order')

    def update(self, data, state):
        c, robot = self.config, self.robot
        time = float(data.time)
        dt = self.model.opt.timestep if self.last_time is None else time - self.last_time
        if not 0 < dt <= .02:
            raise ValueError('Force controller requires increasing time, step <= 20 ms')
        measurement = self.observer.read(data)
        force = measurement['force_N']
        if force > c.force_stop_N:
            self.fault_reason = f'Compensated force {force:.3f} N exceeds {c.force_stop_N:g} N stop limit'
        nominal = robot.fk(state['q']).copy()
        jacp, jacr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, robot.data, jacp, jacr, robot.site)
        jac = np.vstack([jacp[:, robot.dadr], jacr[:, robot.dadr]])
        nominal_normal = float(nominal[:3, 3] @ self.normal)
        force_phase = state['phase'] in ('approach', 'contact_hold', 'trace')
        newly_engaged = False
        if force_phase and self.contact_start is None and force >= c.contact_threshold_N:
            self.contact_start, self.anchor = time, nominal_normal
            self.admittance.filtered_force = force
            self.admittance.velocity = float(np.clip(self.normal @ jacp[:, robot.dadr] @ state['dq'],
                                                     -c.max_normal_speed_m_s, c.max_normal_speed_m_s))
            newly_engaged = True
        normal_command, target = nominal_normal, 0.
        mode = 'position'
        if force_phase and self.contact_start is not None:
            target = c.target_force_N * smoothstep((time-self.contact_start) / c.target_ramp_s)
            if not newly_engaged:
                self.admittance.step(force, target, dt)
            normal_command = self.anchor + self.admittance.displacement
            mode = 'force'
            if abs(self.admittance.displacement) >= c.max_displacement_m - 1e-12:
                self.fault_reason = 'Normal compliance travel limit reached'
        elif self.contact_start is not None:
            if self.release is None:
                end = float(self.plan.times[state['interval']+1])
                self.release = (time, end, self.last_normal_command-nominal_normal)
            begin, end, offset = self.release
            normal_command += offset * (1. - smoothstep((time-begin) / max(end-begin, dt)))
            mode = 'release'
        correction = normal_command - nominal_normal
        # Damped differential IK preserves nominal orientation and tangent.
        dq_offset = jac.T @ np.linalg.solve(jac @ jac.T + 1e-10*np.eye(6),
                                            np.r_[self.normal*correction, np.zeros(3)])
        q = state['q'] + dq_offset
        dq = state['dq'].copy() if self.last_q is None else (q-self.last_q) / dt
        ddq = state['ddq'].copy() if self.last_q is None else (dq-self.last_dq) / dt
        if np.any(q < robot.lower) or np.any(q > robot.upper) or np.max(abs(dq_offset)) > .15:
            self.fault_reason = 'Compliance IK exceeds joint/travel bounds'
        inertia = np.empty((self.model.nv, self.model.nv))
        if hasattr(data, 'qM'):
            mujoco.mj_fullM(self.model, inertia, data.qM)
        else:
            mujoco.mj_fullM(self.model, data, inertia)
        # Do not feed force-driven finite-difference acceleration back as torque:
        # it amplifies contact onset/stick-slip into a second acceleration loop.
        # The nominal path keeps its smooth inertial feedforward; compliance is
        # realized through bounded position/velocity commands and servo damping.
        torque = inertia[np.ix_(robot.dadr, robot.dadr)] @ state['ddq'] + data.qfrc_bias[robot.dadr]
        command = q + (torque+self.damping*dq) / self.gain
        data.ctrl[:] = np.clip(command, self.model.actuator_ctrlrange[:, 0], self.model.actuator_ctrlrange[:, 1])
        self.last_time, self.last_q, self.last_dq = time, q.copy(), dq.copy()
        self.last_normal_command = normal_command
        return {'q': q, 'dq': dq, 'ddq': ddq, 'force_control_state': mode,
                'force_target_N': float(target), 'feedback_force_N': force,
                'filtered_force_N': self.admittance.filtered_force,
                'normal_correction_mm': correction*1000,
                'admittance_velocity_mm_s': self.admittance.velocity*1000,
                'compensated_force_normal_N': float(measurement['external_force_world_N'] @ self.normal),
                'fault_reason': self.fault_reason}
