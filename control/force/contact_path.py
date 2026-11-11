"""Continuous contact-path scheduling and bounded normal compliance.

The source trajectory remains immutable. Each retained trace is visited in order,
with an explicit acquisition and release; transfers never teleport the robot.
"""
from dataclasses import dataclass

import numpy as np

from .compliance import ForceControlConfig, NormalAdmittance, smoothstep


@dataclass(frozen=True)
class ContactPathConfig:
    clearance_m: float = .002
    release_s: float = 1.
    acquisition_timeout_s: float = 8.
    stable_contact_s: float = .15
    force_tolerance_N: float = .2
    soft_force_N: float = 2.
    predictive_force_N: float = 2.5
    max_duration_s: float = 60.
    max_trace_speed_m_s: float = .002
    max_tangent_error_m: float = .0003
    rate_filter_s: float = .04
    max_joint_speed_rad_s: float = 2.2
    max_joint_accel_rad_s2: float = 12.

    def validate(self, force):
        for name, value in vars(self).items():
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be positive and finite')
        if not force.target_force_N < self.soft_force_N < self.predictive_force_N < force.force_stop_N <= 3.:
            raise ValueError('Require target < soft guard < predictive guard < hard limit <= 3 N')
        if self.clearance_m >= force.max_displacement_m:
            raise ValueError('Clearance must fit inside the compliance travel bound')
        if self.force_tolerance_N >= force.target_force_N - force.contact_threshold_N:
            raise ValueError('Stable force band must exclude contact loss')


class RateGovernor:
    """Two cascaded low-pass stages make pausing/resuming a smooth time warp."""
    def __init__(self, tau):
        self.tau, self.internal, self.rate = tau, 0., 0.

    def step(self, goal, dt):
        old = self.rate
        alpha = -np.expm1(-dt / self.tau)
        self.internal += alpha * (np.clip(goal, 0., 1.) - self.internal)
        self.rate += alpha * (self.internal - self.rate)
        return self.rate, (self.rate - old) / dt


class ContactPath:
    """One state machine for ALL source traces and their connecting movements."""
    def __init__(self, plan, segments, normals, force=None, config=None, roll_corrections_deg=None,
                 segment_speeds_m_s=None):
        self.plan = plan
        self.force = force or ForceControlConfig()
        self.config = config or ContactPathConfig()
        self.config.validate(self.force)
        self.segments = [dict(s) for s in segments if s['status'] == 'planned']
        if not self.segments:
            raise ValueError('No retained traces')
        self.trace_speeds_m_s = np.full(len(self.segments), self.config.max_trace_speed_m_s)
        for index, speed in (segment_speeds_m_s or {}).items():
            if (isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(self.segments)
                    or isinstance(speed, bool) or not np.isfinite(speed) or speed <= 0):
                raise ValueError('Segment speed requires a valid segment index and positive finite speed')
            self.trace_speeds_m_s[index] = speed
        self.roll_corrections = np.zeros(len(self.segments))
        for index, angle in (roll_corrections_deg or {}).items():
            if (isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(self.segments)
                    or not np.isfinite(angle) or abs(angle) > 30.):
                raise ValueError('Roll correction requires a valid segment index and angle within +/-30 degrees')
            self.roll_corrections[index] = np.radians(angle)
        marked = np.zeros(len(plan.phases), bool)
        previous_end = 0
        for s in self.segments:
            a, b = s['trace_start_waypoint'], s['trace_end_waypoint']
            if not (previous_end <= a < b < len(plan.times)) or marked[a:b].any():
                raise ValueError('Overlapping or unordered trace segments')
            if not np.all(plan.phases[a:b] == 'trace') or not np.all(plan.teeth[a:b] == s['tooth']):
                raise ValueError('Trace metadata does not match trajectory')
            if not np.isfinite(s['length_mm']) or s['length_mm'] <= 0:
                raise ValueError('Invalid trace length')
            marked[a:b] = True
            previous_end = b
        if not np.array_equal(marked, plan.phases == 'trace'):
            raise ValueError('Every source trace must be retained, without omissions')
        self.exit_begin = float(plan.times[previous_end])
        self.has_exit = previous_end < len(plan.times)-1
        self.normals = {}
        for s in self.segments:
            n = np.asarray(normals[s['tooth']], float)
            if n.shape != (3,) or not np.isfinite(n).all() or np.linalg.norm(n) < 1e-8:
                raise ValueError('Invalid tooth normal')
            self.normals[s['tooth']] = n / np.linalg.norm(n)
        self.index, self.stage = 0, 'transfer'
        self.elapsed, self.progress, self.stable = 0., 0., 0.
        self.transfer_begin = 0.
        self.transfer_offset = np.zeros(3)
        self.admittance = NormalAdmittance(self.force)
        self.admittance.displacement = self.config.clearance_m
        self.governor = RateGovernor(self.config.rate_filter_s)
        self.source_time, self.completed = 0., False
        self.fault_reason = ''
        self.release_start = self.config.clearance_m
        self.durations = []
        self.free_transfers = []
        previous_end = 0.
        for index, s in enumerate(self.segments):
            a, b = self.plan.times[[s['trace_start_waypoint'], s['trace_end_waypoint']]]
            phases = self.plan.phases[(self.plan.times[:-1] >= previous_end) & (self.plan.times[:-1] < a)]
            self.free_transfers.append(bool(np.any(phases == 'transfer')))
            self.durations.append((self._duration(previous_end, a, .2),
                self._duration(a, b, 1.875*s['length_mm']*.001/self.trace_speeds_m_s[index])))
            previous_end = b
        self.exit_duration = self._duration(self.exit_begin, plan.duration, .2) if self.has_exit else 0.

    def _duration(self, begin, end, minimum):
        duration = max(float(end-begin), minimum)
        vmax, amax = 0., 0.
        # Include samples from every source interval: a uniform time grid can
        # miss the very short ingress/retreat intervals in the 58-second plan.
        times = []
        for lo, hi in zip(self.plan.times[:-1], self.plan.times[1:]):
            if hi > begin and lo < end:
                times.extend(np.linspace(max(lo, begin), min(hi, end), 17))
        fractions = (np.asarray(times)-begin)/max(end-begin, 1e-12)
        low, high = np.zeros_like(fractions), np.ones_like(fractions)
        for _ in range(40):
            mid = .5*(low+high)
            mask = smoothstep(mid) < fractions
            low = np.where(mask, mid, low)
            high = np.where(mask, high, mid)
        for u in np.unique(np.r_[np.linspace(0., 1., 257), .5*(low+high)]):
            state = self.plan.sample(begin+smoothstep(u)*(end-begin))
            rate = 30*u*u*(1-u)**2*(end-begin)/duration
            acceleration = 60*u*(1-u)*(1-2*u)*(end-begin)/duration**2
            vmax = max(vmax, float(np.max(abs(state['dq']*rate))))
            amax = max(amax, float(np.max(abs(state['ddq']*rate**2+state['dq']*acceleration))))
        # Leave room for normal corrections and contact-driven rate changes.
        return duration*max(1., vmax/(.75*self.config.max_joint_speed_rad_s),
                            np.sqrt(amax/(.6*self.config.max_joint_accel_rad_s2)))

    @property
    def segment(self):
        return self.segments[self.index]

    @property
    def normal(self):
        return self.normals[self.segment['tooth']]

    def _transition(self, stage):
        self.stage, self.elapsed, self.progress, self.stable = stage, 0., 0., 0.
        self.governor = RateGovernor(self.config.rate_filter_s)

    def _sample(self, source_time, rate=0., acceleration=0.):
        state = self.plan.sample(source_time)
        state['ddq'] = state['ddq'] * rate**2 + state['dq'] * acceleration
        state['dq'] = state['dq'] * rate
        return state

    def update(self, force_N, tangent_error_m, dt):
        if not np.isfinite([force_N, tangent_error_m, dt]).all() or force_N < 0 or not 0 < dt <= .02:
            raise ValueError('Invalid feedback or sample interval')
        c, fc, s = self.config, self.force, self.segment
        a, b = self.plan.times[[s['trace_start_waypoint'], s['trace_end_waypoint']]]
        normal = self.normal.copy()
        target, rate, acceleration, advance = 0., 0., 0., False
        stage, index = self.stage, self.index
        roll = self.roll_corrections[index]
        offset = normal * self.admittance.displacement
        if force_N >= fc.force_stop_N:
            self.fault_reason = 'Measured contact force reached hard limit'
        if stage == 'exit':
            u = min(self.elapsed/self.exit_duration, 1.)
            blend = smoothstep(u)
            span = self.plan.duration-self.exit_begin
            self.source_time = self.exit_begin+blend*span
            rate = 30*u*u*(1-u)**2*span/self.exit_duration
            acceleration = 60*u*(1-u)*(1-2*u)*span/self.exit_duration**2
            # Remove the correction with source retreat progress, so an early
            # fade cannot move the tip inward before the nominal path withdraws.
            offset = (1-blend)*self.transfer_offset
            advance = u >= 1.
        elif stage == 'transfer':
            duration = self.durations[index][0]
            u = min(self.elapsed / duration, 1.)
            blend = smoothstep(u)
            rate = 30*u*u*(1-u)**2 * (a-self.transfer_begin)/duration
            acceleration = 60*u*(1-u)*(1-2*u) * (a-self.transfer_begin)/duration**2
            self.source_time = self.transfer_begin + blend*(a-self.transfer_begin)
            if self.free_transfers[index]:
                # After leaving the mouth, remove the normal correction before
                # large free-space joint rotations; restore it before ingress.
                offset = ((1-smoothstep(u/.2))*self.transfer_offset
                          + smoothstep((u-.8)/.2)*c.clearance_m*normal)
            else:
                offset = (1-blend)*self.transfer_offset + blend*c.clearance_m*normal
            previous_roll = self.roll_corrections[index-1] if index else 0.
            roll = (1-blend)*previous_roll + blend*roll
            advance = u >= 1.
        elif stage in ('acquire', 'trace'):
            target = fc.target_force_N * (smoothstep(self.elapsed/fc.target_ramp_s) if stage == 'acquire' else 1.)
            if force_N >= c.soft_force_N:
                target = 0.  # unload before reaching the predictive/hard limits
            self.admittance.step(force_N, target, dt)
            offset = normal * self.admittance.displacement
            if abs(self.admittance.displacement) >= fc.max_displacement_m-1e-12:
                self.fault_reason = 'Contact acquisition/tracking exhausted normal travel'
            stable = abs(force_N-fc.target_force_N) <= c.force_tolerance_N and tangent_error_m <= c.max_tangent_error_m
            if stage == 'acquire':
                self.source_time = float(a)
                self.stable = self.stable+dt if stable and self.elapsed >= fc.target_ramp_s else 0.
                advance = self.stable >= c.stable_contact_s
                if self.elapsed >= c.acquisition_timeout_s:
                    self.fault_reason = 'Contact could not settle in acquisition timeout'
            else:
                duration = self.durations[index][1]
                # Stop advancing when unloaded, overloaded, or lagging. Normal
                # compliance remains active throughout the pause/reacquisition.
                goal = 1. if stable else 0.
                speed, speed_dot = self.governor.step(goal, dt)
                self.progress = min(self.progress + speed*dt, duration)
                u = self.progress/duration
                self.source_time = float(a + smoothstep(u)*(b-a))
                dp, ddp = 30*u*u*(1-u)**2, 60*u*(1-u)*(1-2*u)
                rate = float((b-a)/duration * dp*speed)
                acceleration = float((b-a)*(ddp*speed**2/duration**2+dp*speed_dot/duration))
                advance = self.progress >= duration
        elif stage == 'release':
            self.source_time = float(b)
            x = self.release_start + smoothstep(self.elapsed/c.release_s)*(c.clearance_m-self.release_start)
            offset = normal*x
            advance = self.elapsed >= c.release_s and force_N < fc.contact_threshold_N
            if self.elapsed >= c.release_s+c.acquisition_timeout_s:
                self.fault_reason = 'Contact did not release; transfer forbidden'
        state = self._sample(self.source_time, rate, acceleration)
        state.update(phase='transfer' if stage == 'exit' else stage, tooth=s['tooth'], segment_index=index,
                     offset_world_m=offset, normal_world=normal, force_target_N=float(target),
                     roll_correction_rad=float(roll),
                     path_rate=rate, source_time_s=self.source_time,
                     fault_reason=self.fault_reason)
        self.elapsed += dt
        if advance and not self.fault_reason:
            if stage == 'exit':
                self.completed = True
            elif stage == 'transfer':
                self.admittance = NormalAdmittance(fc)
                self.admittance.displacement = c.clearance_m
                self.admittance.filtered_force = force_N
                self._transition('acquire')
            elif stage == 'acquire':
                self._transition('trace')
            elif stage == 'trace':
                self.release_start = self.admittance.displacement
                self._transition('release')
            elif stage == 'release':
                if index == len(self.segments)-1:
                    if self.has_exit:
                        self.transfer_offset = offset.copy()
                        self._transition('exit')
                    else:
                        self.completed = True
                else:
                    self.transfer_begin = float(b)
                    self.transfer_offset = offset.copy()
                    self.index += 1
                    self._transition('transfer')
        return state
