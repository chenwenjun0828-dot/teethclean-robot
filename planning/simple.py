"""Regional and local surface frames with bounded angle corrections."""
from itertools import product
import numpy as np
from .infrastructure.poses import unit
from perception.regions.continuous import SweepPath
from .pose_search.constraints import gravity_frame, tool_rotation, regional_angle_offsets


ANGLE_LIMIT_DEG = 5.
POLICY = 'regional_gravity_fixed_z_y_tool_x_y_down_v3'
LOCAL_SURFACE_POLICY = 'local_surface_gravity_fixed_z_y_tool_x_y_down_v1'


def representative_source(geometry, options):
    strokes = geometry.strokes
    distance = sum(s.length_mm for s in strokes)/2
    for stroke in strokes:
        if distance <= stroke.length_mm:
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(stroke.points, axis=0), axis=1))*1000]
            i = int(np.argmin(abs(arc-distance)))
            normal = unit(geometry.planes[stroke.tooth]['normal_scan'])
            tangent = unit(stroke.tangents[i])
            return {'selection': 'midpoint_of_cumulative_principal_source_length',
                    'point_scan_m': stroke.points[i].tolist(), 'tangent_scan': tangent.tolist(),
                    'normal_scan': normal.tolist(), 'tooth': int(stroke.tooth),
                    'stroke_id': int(stroke.id), 'sample_index': i}
        distance -= stroke.length_mm
    raise ValueError('No representative regional source point')


class BoundedOrientationPath(SweepPath):
    def __init__(self, record, cfg):
        super().__init__(record)
        saved = record.get(self.orientation_key, {})
        if saved.get('policy') != self.policy or saved.get('angle_limit_deg') != ANGLE_LIMIT_DEG:
            raise ValueError('Bounded orientation reference is missing or retired; replan')
        self.gravity_scan = unit(saved['gravity_scan'])
        if not np.allclose(saved['gravity_world'], cfg.gravity_world, atol=1e-12):
            raise ValueError('Orientation reference gravity differs from the configuration; replan')
        self.base_angles = np.asarray(saved['base_angles_deg'], float)
        expected = [cfg.tilt_deg, cfg.tilt_azimuth_deg, record['options']['fixed_roll_deg']]
        if self.base_angles.shape != (3,) or not np.array_equal(self.base_angles, expected):
            raise ValueError('Orientation reference angles differ from the requested three angles; replan')
        self.set_corrections(saved.get('parameters_m'), saved.get('angle_offsets_deg'))

    @property
    def orientation(self):
        return self.record[self.orientation_key]

    def set_corrections(self, parameters, offsets):
        parameters, offsets = np.asarray(parameters, float), np.asarray(offsets, float)
        if (parameters.ndim != 1 or len(parameters) < 2 or offsets.shape != (len(parameters), 3)
                or not np.isfinite(parameters).all() or not np.isfinite(offsets).all()
                or np.any(np.diff(parameters) <= 0) or parameters[0] != 0.
                or not np.isclose(parameters[-1], self.end, atol=1e-12)
                or np.any(abs(offsets) > ANGLE_LIMIT_DEG)):
            raise ValueError('Invalid reference angle corrections: each angle must stay within +/-5 degrees')
        self.orientation_parameters = parameters.copy()
        self.offsets = offsets.copy()
        self.orientation.update(parameters_m=parameters.tolist(), angle_offsets_deg=offsets.tolist())

    def angle_offset(self, s):
        self.part_index(s)
        i = min(max(int(np.searchsorted(self.orientation_parameters, s, side='right'))-1, 0), len(self.offsets)-2)
        a, b = self.orientation_parameters[i:i+2]
        f = np.clip((s-a)/(b-a), 0., 1.)
        w = f**3*(10-15*f+6*f*f)
        return (1-w)*self.offsets[i]+w*self.offsets[i+1]

    def rotation_for_offset(self, offset, s=0.):
        angle, azimuth, flip = (self.base_angles+np.asarray(offset)+180)%360-180
        return tool_rotation(None, self.orientation_normal(s), angle, flip, azimuth, gravity=self.gravity_scan)

    def rotation(self, s):
        return self.rotation_for_offset(self.angle_offset(s), s)

    def measured_offset(self, rotation_scan, s=0.):
        """Recover reference-frame angles from FK, including wrapped azimuth/roll."""
        return regional_angle_offsets(rotation_scan, self.frame_at(s), self.base_angles)


class SimplePath(BoundedOrientationPath):
    orientation_key = 'regional_orientation'
    policy = POLICY

    def __init__(self, record, cfg):
        super().__init__(record, cfg)
        self.anchor = self.orientation['anchor']
        self.tangent = unit(self.anchor['tangent_scan'])
        self.anchor_normal = unit(self.anchor['normal_scan'])
        self.frame = gravity_frame(-self.anchor_normal, self.gravity_scan)

    def orientation_normal(self, s):
        return self.anchor_normal

    def frame_at(self, s):
        return self.frame


class LocalSurfacePath(BoundedOrientationPath):
    orientation_key = 'local_surface_orientation'
    policy = LOCAL_SURFACE_POLICY

    def orientation_normal(self, s):
        return self.normal(s)

    def frame_at(self, s):
        return gravity_frame(-self.normal(s), self.gravity_scan)


def orient_reference(reference, geometry, cfg, options, *, world=None):
    """Attach an orientation policy without changing perception's geometry."""
    from copy import deepcopy
    path = SweepPath(deepcopy(reference.record))
    path.record['options']['fixed_roll_deg'] = options.fixed_roll_deg
    world = np.eye(4) if world is None else np.asarray(world)
    path_type = LocalSurfacePath if cfg.planning_mode == 'local_surface' else SimplePath
    orientation = {
        'policy': path_type.policy,
        'gravity_scan': (world[:3, :3].T@cfg.gravity_world).tolist(),
        'gravity_world': list(cfg.gravity_world),
        'angle_names': ['fixed_z_angle', 'fixed_y_azimuth', 'tool_x_flip'],
        'base_angles_deg': [cfg.tilt_deg, cfg.tilt_azimuth_deg, options.fixed_roll_deg],
        'angle_limit_deg': ANGLE_LIMIT_DEG, 'parameters_m': [0., path.end],
        'angle_offsets_deg': [[0., 0., 0.], [0., 0., 0.]], 'search_mode': 'nominal_fixed'}
    if cfg.planning_mode == 'simple':
        orientation['anchor'] = representative_source(geometry, options)
    path.record.update(planning_mode=cfg.planning_mode)
    path.record[path_type.orientation_key] = orientation
    return path_type(path.record, cfg)


def target_at(path, s, roll_deg, world, cfg):
    from .infrastructure.io import validate_transform
    world = validate_transform(world)
    if not np.allclose(path.gravity_scan, world[:3, :3].T@cfg.gravity_world, atol=1e-10):
        raise ValueError('Orientation reference gravity differs from the world transform; rebuild the reference')
    pose = np.eye(4)
    pose[:3, :3] = world[:3, :3]@path.rotation(s)
    pose[:3, 3] = world[:3, :3]@path.position(s, cfg.standoff_mm*.001)+world[:3, 3]
    return pose


def solve_sweep(path, parameters, robot, checker, world, cfg):
    from dataclasses import replace
    from .pose_search.continuous import whole_path_search, SweepFailure
    if not np.allclose(path.gravity_scan, world[:3, :3].T@cfg.gravity_world, atol=1e-10):
        raise ValueError('Orientation reference gravity differs from the world transform; rebuild the reference')
    grid = np.array(sorted(product((-5., -2.5, 0., 2.5, 5.), repeat=3),
                           key=lambda v: (np.dot(v, v), v)))
    label = 'Local surface' if cfg.planning_mode == 'local_surface' else 'Regional'
    positions = [world[:3, :3]@path.position(s, cfg.standoff_mm*.001)+world[:3, 3] for s in parameters]

    def bank(i, delta, collision=False, first_only=False):
        pose = np.eye(4)
        pose[:3, :3] = world[:3, :3]@path.rotation_for_offset(delta, parameters[i])
        pose[:3, 3] = positions[i]
        result = []
        for q in robot.solutions(pose):
            if collision and not cfg.geometry_only and not checker.check(q, cfg.clearance_mm*.001).valid:
                continue
            result.append({'q': q, 'pose': pose, 'roll': float(path.base_angles[2]+delta[2]),
                           'angle_offset': np.array(delta), 'pose_penalty': .002*float(delta@delta), 'layer': i})
            if first_only:
                break
        return result

    def node_valid(node):
        check = checker.check(node['q'], cfg.clearance_mm*.001)
        if not check.valid:
            layer = node['layer']
            if not any(checker.check(candidate['q'], cfg.clearance_mm*.001).valid for candidate in nodes[layer]):
                raise SweepFailure('no_collision_free_ik', layer=layer, parameter=float(parameters[layer]),
                                   pair=check.pair)
        return check.valid, check.pair

    def edge_valid(a, b):
        if cfg.geometry_only:
            return True, ''
        check = checker.edge(a['q'], b['q'], cfg.clearance_mm*.001, cfg.sweep_step_rad)
        return check.valid, check.pair

    probes = np.unique(np.linspace(0, len(parameters)-1, min(17, len(parameters))).astype(int))
    failures = []
    for index, delta in enumerate(grid):
        if index % 25 == 0:
            print(f'{label} constant-angle candidates {index+1}/{len(grid)}', flush=True)
        failed_probe = next((int(i) for i in probes
                             if not bank(int(i), delta, collision=True, first_only=True)), None)
        if failed_probe is not None:
            failures.append({'reason': 'no_valid_ik_at_probe', 'layer': failed_probe,
                             'parameter_m': float(parameters[failed_probe]),
                             'angle_offset_deg': delta.tolist()})
            probes = np.r_[failed_probe, probes[probes != failed_probe]]
            continue
        nodes = [bank(i, delta) for i in range(len(parameters))]
        try:
            selected = whole_path_search(nodes, cfg, edge_valid, robot.home,
                                         None if cfg.geometry_only else node_valid)
        except SweepFailure as error:
            failures.append(error.details)
            layer = error.details.get('layer')
            if layer is not None:
                probes = np.unique(np.r_[int(layer), probes])
            continue
        path.set_corrections([0., path.end], [delta, delta])
        path.orientation.update(
            search_mode='nominal_fixed' if not np.any(delta) else 'adjusted_fixed',
            candidates_tested=index+1, grid_step_deg=2.5)
        print(f'{label} angle correction {delta.tolist()} degrees', flush=True)
        return selected

    if not cfg.local_angle_search:
        print(f'No common {label.lower()} angle correction; local angle search disabled', flush=True)
        raise SweepFailure('no_common_angle_correction_within_5_deg', local_angle_search=False,
                           fixed_angle_candidates_tested=len(grid), fixed_attempt_failures=failures[-3:])
    print(f'No common {label.lower()} angle correction on the bounded grid; checking local corrections', flush=True)
    nodes, previous = [], np.zeros(3)
    for i in range(len(parameters)):
        order = np.argsort(np.sum((grid-previous)**2, axis=1)+.25*np.sum(grid**2, axis=1), kind='stable')
        candidates, orientations = [], 0
        for j in order:
            feasible = bank(i, grid[j], collision=True)
            if not feasible:
                continue
            if not orientations:
                previous = grid[j]
            candidates.extend(feasible)
            orientations += 1
            if orientations == 3:
                break
        # Retain axis alternatives so the graph can anticipate a later obstacle.
        alternatives = np.r_[np.zeros((1, 3)), np.eye(3)*2.5, -np.eye(3)*2.5,
                             np.eye(3)*ANGLE_LIMIT_DEG, -np.eye(3)*ANGLE_LIMIT_DEG]
        for delta in alternatives:
            if not any(np.array_equal(n['angle_offset'], delta) for n in candidates):
                candidates.extend(bank(i, delta, collision=True))
        if not candidates:
            reason = ('no_local_surface_pose_within_5_deg' if cfg.planning_mode == 'local_surface'
                      else 'no_regional_pose_within_5_deg')
            raise SweepFailure(reason, layer=i, parameter=float(parameters[i]),
                               tooth=path.tooth(parameters[i]), grid_step_deg=2.5,
                               fixed_attempt_failures=failures[-3:])
        nodes.append(candidates)
        if i % 100 == 0:
            print(f'{label} bounded candidates {i+1}/{len(parameters)}', flush=True)
    local_cfg = (replace(cfg, max_orientation_step_deg=min(5., cfg.max_orientation_step_deg))
                 if cfg.planning_mode == 'simple' else cfg)
    selected = whole_path_search(nodes, local_cfg,
                                 edge_valid, robot.home, None if cfg.geometry_only else node_valid)
    path.set_corrections(parameters, [node['angle_offset'] for node in selected])
    path.orientation.update(search_mode='bounded_local', grid_step_deg=2.5,
                            orientations_per_layer_max=16)
    return selected
