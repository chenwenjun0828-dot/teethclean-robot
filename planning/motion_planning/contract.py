"""Structural and provenance checks for a single selected-region sweep."""
import json
from pathlib import Path
import numpy as np
from planning.infrastructure.io import fingerprint
from perception.regions.continuous import SweepPath
from ..config import Config, SweepOptions, continuous_path_kind


def load_sweep_reference(directory, report):
    file = Path(directory)/'sweep_reference.json'
    if fingerprint(file)['sha256'] != report.get('sweep_reference_file', {}).get('sha256'):
        raise ValueError('Continuous sweep reference changed since planning; replan')
    record = json.loads(file.read_text(encoding='utf-8'))
    if report.get('geometry_owner') == 'perception':
        perception_file = Path(directory)/'perception_reference.json'
        if fingerprint(perception_file)['sha256'] != report.get('perception_reference_file', {}).get('sha256'):
            raise ValueError('Perception continuous line changed since planning; replan')
        original = json.loads(perception_file.read_text(encoding='utf-8'))
        for key in ('parts', 'region', 'teeth', 'per_tooth', 'trim_mm', 'source_region_length_mm',
                    'retained_length_mm', 'synthetic_bridge_length_mm', 'sweep_length_mm'):
            if record.get(key) != original.get(key):
                raise ValueError('Planned geometry differs from perception continuous line; replan')
    region = report.get('region', 'outer_front')
    if (record.get('kind') != continuous_path_kind(region) or record.get('schema_version') != 1
            or record.get('region', 'outer_front') != region):
        raise ValueError('Unsupported continuous sweep reference')
    reference_options = SweepOptions.from_saved(record.get('options', {}))
    report_options = SweepOptions.from_saved(report.get('sweep_options', {}))
    if reference_options.fixed_roll_deg != report_options.fixed_roll_deg:
        raise ValueError('Continuous reference/report fixed roll changed; replan')
    config = Config.from_saved(report['config'])
    if config.planning_mode == 'simple':
        from ..simple import SimplePath
        path = SimplePath(record, config)
    else:
        path = SweepPath(record)
    validate_planning_mode(report, path)
    if config.planning_mode == 'simple' and 'world_from_scan_m' in report:
        gravity_scan = np.asarray(report['world_from_scan_m'])[:3, :3].T@config.gravity_world
        if not np.allclose(path.gravity_scan, gravity_scan, atol=1e-10):
            raise ValueError('Regional reference gravity differs from the saved world transform; replan')
    return path


def validate_planning_mode(report, path):
    mode = Config.from_saved(report['config']).planning_mode
    if (report.get('planning_mode', 'continuous') != mode
            or path.record.get('planning_mode', 'continuous') != mode):
        raise ValueError('Planning mode differs from the saved reference/configuration; replan')
    if mode == 'simple':
        from ..simple import SimplePath
        if not isinstance(path, SimplePath):
            raise ValueError('Simple planning requires its saved regional orientation reference')
    return mode


def validate_sweep_contract(trajectory, report, path):
    validate_planning_mode(report, path)
    region = report.get('region', 'outer_front')
    kind = continuous_path_kind(region)
    if (report.get('path_kind') != kind or path.record.get('kind') != kind
            or path.record.get('region', 'outer_front') != region
            or report['config'].get('region', 'outer_front') != region):
        raise ValueError('Continuous sweep region differs from its reference/configuration')
    segments = report.get('motion_segments', [])
    if len(segments) != 1 or segments[0].get('status') != 'planned':
        raise ValueError('Continuous sweep requires exactly one complete motion segment')
    segment = segments[0]
    if segment.get('reference_kind') != kind:
        raise ValueError('Missing continuous motion reference')
    start, end = segment['trace_start_waypoint'], segment['trace_end_waypoint']
    if not (0 <= start < end <= len(trajectory.joints)-1):
        raise ValueError('Continuous sweep has invalid waypoint bounds')
    if (np.any(trajectory.phases[start:end] != 'trace') or np.any(trajectory.phases[:start] == 'trace')
            or np.any(~np.isin(trajectory.phases[end:], ['retreat', 'transfer']))):
        raise ValueError('Retreat/transfer/re-approach or extra trace inside continuous sweep')
    u, rolls = np.asarray(segment['parameters']), np.asarray(segment['rolls_deg'])
    if (u.shape != (end-start+1,) or rolls.shape != u.shape or not np.isfinite(u).all()
            or not np.isfinite(rolls).all() or np.any(np.diff(u) <= 0)
            or not np.isclose(u[0], 0., atol=1e-12) or not np.isclose(u[-1], path.end, atol=1e-12)):
        raise ValueError('Incomplete or inconsistent continuous sweep reference mapping')
    fixed_roll = SweepOptions.from_saved(report.get('sweep_options', {})).fixed_roll_deg
    expected_rolls = (np.array([path.base_angles[2]+path.angle_offset(s)[2] for s in u])
                      if report['config'].get('planning_mode') == 'simple' else fixed_roll)
    if np.any(abs((rolls-expected_rolls+180)%360-180) > 1e-6):
        raise ValueError('Target roll differs from the requested fixed roll')
    expected = np.array([path.tooth((a+b)/2) for a, b in zip(u[:-1], u[1:])])
    if not np.array_equal(trajectory.teeth[start:end], expected):
        raise ValueError('Continuous sweep tooth labels changed')
    selected = list(map(int, path.record.get('teeth', path.record['per_tooth'])))
    teeth = set(selected)
    if (not teeth or len(teeth) != len(selected)
            or list(map(int, segment['teeth'])) != selected
            or set(map(int, path.record['per_tooth'])) != teeth
            or set(expected) != teeth
            or list(map(int, report.get('selected_teeth', selected))) != selected):
        raise ValueError('Continuous sweep does not visit exactly the selected region teeth')
    for tooth in teeth:
        row = path.record['per_tooth'][str(tooth)]
        if row['retained_length_mm']/row['source_length_mm'] < report['config']['min_tooth_coverage']:
            raise ValueError('Continuous sweep source coverage below minimum')
    result = {'continuous_trace_blocks': 1, 'trace_start_waypoint': start, 'trace_end_waypoint': end,
              'retreats_inside_sweep': 0, 'transfers_inside_sweep': 0,
              'region': region, 'selected_teeth': selected, 'all_selected_teeth': True}
    return result
