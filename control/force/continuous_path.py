"""One acquisition and force-controlled trace of an immutable regional sweep."""
import numpy as np

from .contact_path import ContactPath
from .sweep_source import ContactSource
from planning.motion_planning.contract import load_sweep_reference


class RegionalContactPath(ContactPath):
    def __init__(self, source, original, report, world, force, config):
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

    def update(self, force_N, tangent_error_m, dt):
        state = super().update(force_N, tangent_error_m, dt)
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
