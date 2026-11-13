"""One acquisition and force-controlled trace of an immutable regional sweep."""
import numpy as np

from .contact_path import ContactPath
from .sweep_source import ContactSource
from planning.motion_planning.contract import load_sweep_reference


class RegionalContactPath(ContactPath):
    def __init__(self, source, original, report, world, force, config, stage='regional'):
        if stage not in ('hold', 'short', 'regional'):
            raise ValueError('Unknown force experiment stage')
        self.experiment_stage = stage
        self.reference = load_sweep_reference(source, report)
        blocks = [s for s in report['motion_segments'] if s['status'] == 'planned']
        if len(blocks) != 1:
            raise ValueError('Regional contact requires exactly one continuous sweep')
        self.block = blocks[0]
        first, last = self.block['trace_start_waypoint'], self.block['trace_end_waypoint']
        self.source_parameters = np.asarray(self.block['parameters'])
        self.world_rotation = np.asarray(world)[:3, :3]
        representative = self.reference.tooth(0.)
        teeth = original.teeth.copy()
        teeth[first:last] = representative
        adapted = ContactSource(original, original.phases.copy(), teeth)
        grid = np.unique(np.r_[np.linspace(0., self.reference.end, 10001), self.reference.ends])
        points = np.array([self.reference.evaluate(s) for s in grid])
        length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()*1000)
        segment = dict(self.block, tooth=representative, stroke_id=-1,
                       reference_part_index=-1, length_mm=length)
        normal = self.world_rotation@self.reference.normal(0.)
        super().__init__(adapted, [segment], {representative: normal}, force, config)
        self.stage_stop_time = float(original.times[last])
        self.hold_elapsed = 0.
        self.probe_exit_start = None
        if stage == 'short':
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
            end_parameter = float(np.interp(min(.005, arc[-1]), arc, grid))
            times = np.linspace(original.times[first], original.times[last], 10001)
            parameters = [self.parameter_at(t) for t in times]
            self.stage_stop_time = float(np.interp(end_parameter, parameters, times))
            self.durations[0] = (self.durations[0][0], self._duration(
                original.times[first], self.stage_stop_time,
                1.875*min(.005, arc[-1])/config.max_trace_speed_m_s))
        elif stage == 'hold':
            self.stage_stop_time = float(original.times[first])

    def parameter_at(self, source_time):
        a, b = self.block['trace_start_waypoint'], self.block['trace_end_waypoint']
        original = self.plan.original
        clock = original.constant_trace
        if clock is not None and hasattr(clock, 'clock'):
            fraction = np.clip((source_time-original.times[a])/(original.times[b]-original.times[a]), 0., 1.)
            return float(np.interp(clock.clock(fraction), clock.knots, self.source_parameters))
        return float(np.interp(source_time, original.times[a:b+1], self.source_parameters))

    @property
    def normal(self):
        return self.world_rotation@self.reference.normal(self.parameter_at(self.source_time))

    def update(self, force_N, tangent_error_m, dt, total_force_N=None):
        if self.stage == 'probe_exit':
            # Return along the visited prefix at clearance; never jump to the full-path endpoint.
            duration = self.probe_exit_duration
            u = min(self.elapsed/duration, 1.)
            from .compliance import smoothstep
            span = self.probe_exit_start
            time = span*(1-smoothstep(u))
            rate = -30*u*u*(1-u)**2*span/duration
            acceleration = -60*u*(1-u)*(1-2*u)*span/duration**2
            state = self._sample(time, rate, acceleration)
            parameter = self.parameter_at(time)
            normal = self.world_rotation@self.reference.normal(parameter)
            first_time = self.plan.times[self.block['trace_start_waypoint']]
            fade = smoothstep(min(time/max(first_time, 1e-9), 1.))
            state.update(phase='transfer', tooth=self.reference.tooth(parameter), segment_index=0,
                offset_world_m=normal*self.config.clearance_m*fade, normal_world=normal,
                force_target_N=0., roll_correction_rad=0., path_rate=rate,
                source_time_s=float(time), fault_reason=self.fault_reason, reference_parameter_m=parameter)
            self.source_time = float(time)
            self.elapsed += dt
            self.completed = u >= 1.
            return state
        previous_stage = self.stage
        # Hold at the first contact point rather than advancing the nominal curve.
        if self.experiment_stage == 'hold' and self.stage == 'trace':
            self.progress = 0.
        state = super().update(force_N, tangent_error_m, dt, total_force_N)
        if self.experiment_stage == 'hold' and previous_stage == 'trace':
            self.source_time = float(self.plan.times[self.block['trace_start_waypoint']])
            state.update(self._sample(self.source_time))
            state['phase'] = 'trace'
            state['source_time_s'] = self.source_time
            state['path_rate'] = 0.
            stable = abs(force_N-self.force.target_force_N) <= self.config.force_tolerance_N
            self.hold_elapsed = self.hold_elapsed+dt if stable else 0.
            if self.hold_elapsed >= 5.:
                self.release_start = self.admittance.displacement
                self._transition('release')
        if self.experiment_stage == 'short' and previous_stage == 'trace' and self.source_time >= self.stage_stop_time:
            self.source_time = self.stage_stop_time
            state.update(self._sample(self.source_time))
            state.update(phase='trace', source_time_s=self.source_time, path_rate=0.)
            self.release_start = self.admittance.displacement
            self._transition('release')
        if self.experiment_stage != 'regional' and previous_stage == 'release':
            # Base release normally samples the full endpoint. Pin it to the tested prefix.
            stop_time = (float(self.plan.times[self.block['trace_start_waypoint']])
                         if self.experiment_stage == 'hold' else self.stage_stop_time)
            self.source_time = stop_time
            state.update(self._sample(stop_time))
            state.update(phase='release', source_time_s=stop_time, path_rate=0.)
            if self.stage == 'exit' or self.completed:
                self.completed = False
                self.probe_exit_start = stop_time
                self.probe_exit_duration = self._duration(0., stop_time, 1.)
                self._transition('probe_exit')
        parameter = self.parameter_at(state['source_time_s'])
        normal = self.world_rotation@self.reference.normal(parameter)
        magnitude = np.linalg.norm(state['offset_world_m'])
        if state['offset_world_m']@state['normal_world'] < 0:
            magnitude = -magnitude
        if state['phase'] not in ('transfer',):
            state['offset_world_m'] = normal*magnitude
            state['normal_world'] = normal
        state.update(tooth=self.reference.tooth(parameter), reference_parameter_m=parameter)
        return state
