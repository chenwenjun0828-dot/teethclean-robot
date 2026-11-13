"""Expose saved continuous source arcs to the contact scheduler without refitting."""
import numpy as np
from planning.motion_planning.contract import load_sweep_reference


class ContactSource:
    def __init__(self, original, phases, teeth):
        self.original = original
        self.times, self.joints = original.times, original.joints
        self.phases, self.teeth = phases, teeth
        self.duration = original.duration

    def sample(self, time):
        state = self.original.sample(time).copy()
        interval = state['interval']
        state.update(phase=str(self.phases[interval]), tooth=int(self.teeth[interval]))
        return state


def adapt_sweep(source, plan, report):
    """Retain every source arc and bridge; acquire/release only at source boundaries."""
    path = load_sweep_reference(source, report)
    block, = [s for s in report['motion_segments'] if s['status'] == 'planned']
    parameters = np.asarray(block['parameters'])
    origin = block['trace_start_waypoint']
    phases, teeth = plan.phases.astype(object), plan.teeth.copy()
    segments = []
    for index, part in enumerate(path.parts):
        bounds = []
        for boundary in (part['start'], part['end']):
            matches = np.flatnonzero(np.isclose(parameters, boundary, rtol=0., atol=1e-12))
            if len(matches) != 1:
                raise ValueError('Continuous contact adaptation requires every source/bridge seam')
            bounds.append(int(matches[0]))
        first, last = bounds
        a, b = origin+first, origin+last
        if part['kind'] == 'bridge':
            phases[a:b] = 'surface_transfer'
            continue
        tooth = int(part['tooth0'])
        teeth[a:b] = tooth
        grid = np.linspace(part['start'], part['end'], 1001)
        points = np.array([path.evaluate(s) for s in grid])
        length_mm = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()*1000)
        segments.append({'status': 'planned', 'tooth': tooth, 'stroke_id': part['stroke_id'],
                         'reference_part_index': index, 'trace_start_waypoint': a,
                         'trace_end_waypoint': b, 'length_mm': length_mm,
                         'parameters': parameters[first:last+1].tolist()})
    if not segments:
        raise ValueError('Continuous reference has no retained source arcs')
    return ContactSource(plan, phases, teeth), segments
