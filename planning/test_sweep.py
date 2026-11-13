"""Behavioral checks for continuous cross-tooth geometry, search and audit."""
from dataclasses import asdict, replace
from pathlib import Path
import tempfile
import unittest
import numpy as np
from perception.regions.geometry import Geometry, Stroke
from planning.infrastructure.io import dump, fingerprint
from planning.infrastructure.trajectory import Trajectory
from planning.infrastructure.poses import TCP_FRAME_ID
from .config import Config
from planning.pose_search.constraints import AXIS_CONVENTION, tool_rotation
from .config import SweepOptions
from perception.regions.frozen import continuous_regional_reference
from .pose_search.continuous import whole_path_search, SweepFailure
from .motion_planning.contract import validate_sweep_contract, load_sweep_reference
from planning.motion_planning.audit import audit_trace


def straight_geometry():
    strokes, planes = [], {}
    for i in range(4):
        parameters = np.linspace(i/4, (i+1)/4, 21)
        points = np.c_[.024*parameters, np.zeros((21, 2))]
        tangents = np.tile([1., 0., 0.], (len(points), 1))
        strokes.append(Stroke(i, 0, i+1, 'outer_front', points, tangents, parameters, np.arange(len(points))))
        a = .12*i
        planes[i+1] = {'valid': True, 'normal_scan': [0., np.cos(a), np.sin(a)]}
    curve = {'knots': [0]*4+[1]*4,
             'coefficients_m': np.c_[np.linspace(0., .024, 4), np.zeros((4, 2))]}
    return Geometry(strokes, planes, {'regions': {'outer_front': [1, 2, 3, 4]},
                    'curves': [curve], 'preserve_stroke_order': True},
                    np.empty((0, 3)), np.empty((0, 3)), np.empty(0))


def build_reference(geometry, cfg, options, trim_mm=None, *, world=None):
    """Construct a single-region perception fixture, then apply orientation."""
    if trim_mm is not None:
        options = replace(options, trim_mm=trim_mm)
    path = continuous_regional_reference(geometry, cfg, options)
    if cfg.planning_mode == 'simple':
        from .simple import orient_reference
        path = orient_reference(path, geometry, cfg, options, world=world)
    return path


def prepared_region(geometry=None, cfg=None, inputs=None):
    """A perception output for tests of downstream planning and orchestration."""
    from perception.regions import PreparedRegion
    from perception.regions.io import Inputs
    geometry = straight_geometry() if geometry is None else geometry
    cfg = Config(planning_mode='continuous') if cfg is None else cfg
    geometry.report.update(region=cfg.region, config=asdict(cfg), strokes=[
        {'id': s.id, 'curve_id': s.curve_id, 'tooth': s.tooth, 'region': s.region,
         'length_mm': s.length_mm, 'samples': len(s.points)} for s in geometry.strokes])
    path = build_reference(geometry, cfg, SweepOptions(), 1.)
    return PreparedRegion(inputs or Inputs.defaults('mandibular'), geometry, {}, path)


class SweepGeometryTests(unittest.TestCase):
    def test_shared_cubic_preserves_derivatives_at_tooth_boundaries(self):
        geometry = straight_geometry()
        geometry.report['curves'][0]['coefficients_m'][:, 1] = [0, .0008, -.0005, .0004]
        path = build_reference(geometry, Config(planning_mode='continuous'), SweepOptions(), 1.)
        self.assertTrue(all(p['kind'] == 'source' for p in path.parts))
        for seam in path.ends[:-1]:
            for derivative, atol in [(0, 1e-8), (1, 1e-6), (2, .01)]:
                np.testing.assert_allclose(path.evaluate(seam-1e-10, derivative),
                                           path.evaluate(seam+1e-10, derivative), atol=atol)
            np.testing.assert_allclose(path.normal(seam-1e-10), path.normal(seam+1e-10), atol=1e-8)

    def test_only_region_endpoints_are_trimmed_and_coverage_is_honest(self):
        path = build_reference(straight_geometry(), Config(planning_mode='continuous'), SweepOptions(), 1.)
        self.assertAlmostEqual(path.record['retained_length_mm'], 22., places=5)
        self.assertAlmostEqual(path.record['trimmed_or_omitted_length_mm'], 2., places=5)
        self.assertEqual(path.record['synthetic_bridge_length_mm'], 0.)
        self.assertEqual(path.record['trim_scope'], 'region_endpoints_only')
        for tooth in ('2', '3'):
            self.assertAlmostEqual(path.record['per_tooth'][tooth]['coverage_fraction'], 1.)

    def test_synthetic_bridge_artifacts_are_rejected(self):
        from perception.regions.continuous import SweepPath
        path = build_reference(straight_geometry(), Config(planning_mode='continuous'), SweepOptions())
        path.record['parts'][0]['kind'] = 'bridge'
        with self.assertRaisesRegex(ValueError, 'source-preserving'):
            SweepPath(path.record)

    def test_invalid_options_and_missing_plane_fail(self):
        for kwargs in ({'trim_mm': 0}, {'fixed_roll_deg': None}, {'trim_mm': np.nan}):
            with self.assertRaises(ValueError):
                SweepOptions(**kwargs)
        geometry = straight_geometry()
        geometry.planes[4]['valid'] = False
        with self.assertRaisesRegex(ValueError, 'Invalid tooth plane'):
            build_reference(geometry, Config(planning_mode='continuous'), SweepOptions())


def node(q):
    return {'q': np.array([q]), 'pose': np.eye(4), 'roll': 0.}


class SweepSearchTests(unittest.TestCase):
    def test_relaxed_joint_step_still_rejects_colliding_connections(self):
        nodes = [[node(0.)], [node(.2)], [node(.4)]]
        with self.assertRaises(SweepFailure):
            whole_path_search(nodes, Config(planning_mode='continuous'), lambda a, b: (True, ''), np.zeros(1))
        cfg = Config(planning_mode='continuous', joint_step_rad=.25)
        selected = whole_path_search(nodes, cfg, lambda a, b: (True, ''), np.zeros(1))
        self.assertEqual(len(selected), 3)
        with self.assertRaises(SweepFailure) as failed:
            whole_path_search(nodes, cfg, lambda a, b: (a['q'][0] > 0., 'tooth collision'), np.zeros(1))
        self.assertEqual(failed.exception.details['blocked_edge_count'], 1)
        self.assertEqual(failed.exception.details['rejected_edges'][0]['reason'], 'tooth collision')

    def test_disconnected_graph_cannot_be_returned_as_two_runs(self):
        for nodes in ([[node(0)], [], [node(.1)]], [[node(0)], [node(1)], [node(1.1)]]):
            with self.assertRaises(SweepFailure):
                whole_path_search(nodes, Config(planning_mode='continuous'), lambda a, b: (True, ''), np.zeros(1))

    def test_alternate_branch_avoids_blocked_connection_without_splitting(self):
        nodes = [[node(0)], [node(.01), node(.1)], [node(.11)]]
        selected = whole_path_search(nodes, Config(planning_mode='continuous'),
                    lambda a, b: (not (a['q'][0] == .01 and b['q'][0] == .11), 'obstacle'), np.zeros(1))
        self.assertEqual([n['q'][0] for n in selected], [0, .1, .11])

    def test_lazy_node_collision_cannot_reach_output(self):
        nodes = [[node(0)], [node(.01), node(.1)], [node(.11)]]
        selected = whole_path_search(nodes, Config(planning_mode='continuous'), lambda a, b: (True, ''), np.zeros(1),
                                     lambda n: (n['q'][0] != .01, 'tooth'))
        self.assertEqual([n['q'][0] for n in selected], [0, .1, .11])
        with self.assertRaises(SweepFailure):
            whole_path_search(nodes, Config(planning_mode='continuous'), lambda a, b: (True, ''), np.zeros(1),
                              lambda n: (n['q'][0] not in (.01, .1), 'tooth'))


class SweepAuditTests(unittest.TestCase):
    def setup_reference(self, directory, cfg=None):
        cfg = cfg or Config(planning_mode='continuous')
        path = build_reference(straight_geometry(), cfg, SweepOptions(), 1.)
        params = np.unique(np.r_[np.linspace(0, path.end, 30), path.ends])
        teeth = [path.tooth((a+b)/2) for a, b in zip(params[:-1], params[1:])]
        trajectory = Trajectory(np.linspace(0, 10, len(params)), params[:, None],
                                ['trace']*(len(params)-1), teeth, ['progress'])
        dump(directory/'sweep_reference.json', path.record)
        report = {'path_kind': 'continuous_outer_front_sweep', 'axis_convention': AXIS_CONVENTION,
                  'tcp_frame_id': TCP_FRAME_ID, 'world_from_scan_m': np.eye(4), 'config': asdict(cfg),
                  'sweep_options': asdict(SweepOptions()),
                  'sweep_reference_file': fingerprint(directory/'sweep_reference.json'),
                  'motion_segments': [{'status': 'planned', 'reference_kind': 'continuous_outer_front_sweep',
                      'stroke_id': -1, 'tooth': 0, 'teeth': [1, 2, 3, 4], 'parameters': params,
                      'rolls_deg': np.zeros(len(params)), 'trace_start_waypoint': 0,
                      'trace_end_waypoint': len(params)-1}]}
        dump(directory/'planning_report.json', report)
        dump(directory/'geometry.json', {'strokes': []})
        dump(directory/'tooth_planes.json', {})
        return path, trajectory, report

    def test_full_fk_audit_checks_the_whole_region_and_varying_normal(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            path, trajectory, report = self.setup_reference(directory)
            class Robot:
                wrong = False
                def fk(self, q, full=False):
                    s = q[0]; pose = np.eye(4)
                    normal = path.normal(s)
                    if self.wrong and s > path.end/2: normal = np.array([0., 1., 0.])
                    pose[:3, :3] = tool_rotation(path.evaluate(s, 1), normal)
                    pose[:3, 3] = path.position(s, .0015)
                    return pose
            robot = Robot()
            result = audit_trace(directory, trajectory, robot)
            self.assertTrue(result['passed'])
            self.assertEqual(result['continuous_trace_blocks'], 1)
            robot.wrong = True
            with self.assertRaisesRegex(RuntimeError, 'Independent trace audit'):
                audit_trace(directory, trajectory, robot)

    def test_no_transfer_hidden_inside_sweep_and_modified_reference_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            path, trajectory, report = self.setup_reference(directory)
            validate_sweep_contract(trajectory, report, path)
            trajectory.phases[3] = 'retreat'
            with self.assertRaisesRegex(ValueError, 'Retreat/transfer'):
                validate_sweep_contract(trajectory, report, path)
            (directory/'sweep_reference.json').write_text('{}')
            with self.assertRaisesRegex(ValueError, 'reference changed'):
                load_sweep_reference(directory, report)


if __name__ == '__main__': unittest.main()
