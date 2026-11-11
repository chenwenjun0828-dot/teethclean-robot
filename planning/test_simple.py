"""Regional orientation invariance, collision recovery, bounds and replay."""
from contextlib import redirect_stdout
from copy import deepcopy
from dataclasses import asdict, replace
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from .config import Config, SweepOptions
from .infrastructure.io import dump, fingerprint
from .infrastructure.poses import TCP_FRAME_ID
from .infrastructure.trajectory import Trajectory
from .motion_planning.audit import audit_trace
from .motion_planning.contract import load_sweep_reference
from .pose_search.constraints import SIMPLE_AXIS_CONVENTION, target_at
from .pose_search.continuous import SweepFailure
from .simple import SimplePath, representative_source, solve_sweep
from .test_sweep import straight_geometry, build_reference


class SimplePlanningTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Config(planning_mode='simple', tilt_deg=35., tilt_azimuth_deg=-20.)
        self.options = SweepOptions(fixed_roll_deg=-120.)
        self.geometry = straight_geometry()
        self.path = build_reference(self.geometry, self.cfg, self.options, 1.)

    def test_one_orientation_across_all_teeth_and_changing_normals(self):
        rotations = np.array([self.path.rotation(s) for s in np.linspace(0, self.path.end, 101)])
        np.testing.assert_allclose(rotations, np.broadcast_to(rotations[0], rotations.shape), atol=1e-12)
        self.assertGreater(np.linalg.norm(self.path.normal(0)-self.path.normal(self.path.end)), .1)
        np.testing.assert_allclose(self.path.measured_offset(rotations[0]), 0., atol=1e-10)

    def test_representative_is_real_source_and_independent_of_trims(self):
        anchor = representative_source(self.geometry, self.options)
        stroke = next(s for s in self.geometry.strokes if s.id == anchor['stroke_id'])
        np.testing.assert_array_equal(anchor['point_scan_m'], stroke.points[anchor['sample_index']])
        other = build_reference(self.geometry, self.cfg, self.options, 2.)
        self.assertEqual(other.anchor, self.path.anchor)
        self.assertNotIn('bridge', anchor['selection'])

    def test_side_regions_use_their_own_source_anchor(self):
        from .test_region_pose import regional_geometry
        for region in ('outer_left', 'outer_right'):
            with self.subTest(region=region):
                cfg = replace(self.cfg, region=region)
                path = build_reference(regional_geometry(region), cfg, self.options, 1.)
                self.assertIn(path.anchor['tooth'], path.record['teeth'])
                self.assertEqual(path.record['region'], region)
                np.testing.assert_allclose(path.rotation(0.), path.rotation(path.end), atol=1e-12)

    def test_three_angles_and_world_transform_are_used(self):
        world = np.eye(4)
        world[:3, :3] = Rotation.from_euler('z', 20., degrees=True).as_matrix()
        world[:3, 3] = [.2, -.4, .6]
        for s in (0., self.path.end/2, self.path.end):
            local = target_at(self.path, s, -120., np.eye(4), self.cfg)
            np.testing.assert_allclose(target_at(self.path, s, -120., world, self.cfg), world@local)
        for name in ('tilt_deg', 'tilt_azimuth_deg'):
            other = build_reference(self.geometry, replace(self.cfg, **{name: getattr(self.cfg, name)+3.}), self.options, 1.)
            self.assertGreater(np.linalg.norm(other.rotation(0)-self.path.rotation(0)), .01)
        other = build_reference(self.geometry, self.cfg, SweepOptions(fixed_roll_deg=-115.), 1.)
        self.assertGreater(np.linalg.norm(other.rotation(0)-self.path.rotation(0)), .01)

    def test_curving_source_tangents_do_not_rotate_the_tool(self):
        from scipy.interpolate import BSpline
        geometry = straight_geometry()
        for stroke, record in zip(geometry.strokes, geometry.report['curves']):
            record['coefficients_m'][:, 1] = [0., .0005, -.0003, .0002]
            curve = BSpline(record['knots'], record['coefficients_m'], 3)
            stroke.points = curve(stroke.parameters)
            derivative = curve(stroke.parameters, nu=1)
            stroke.tangents = derivative/np.linalg.norm(derivative, axis=1)[:, None]
        path = build_reference(geometry, self.cfg, self.options, 1.)
        for s in np.linspace(0., path.end, 51):
            np.testing.assert_allclose(path.rotation(s), path.rotation(0.), atol=1e-12)

    def test_regional_frame_uses_registered_world_gravity(self):
        from .pose_search.constraints import tool_rotation
        world = np.eye(4)
        world[:3, :3] = Rotation.from_euler('xyz', [20., -30., 15.], degrees=True).as_matrix()
        path = build_reference(self.geometry, self.cfg, self.options, 1., world=world)
        target = target_at(path, 0., -120., world, self.cfg)
        expected = tool_rotation([1, 0, 0], world[:3, :3]@path.anchor_normal, 35., -120., -20.)
        np.testing.assert_allclose(target[:3, :3], expected, atol=1e-12)
        with self.assertRaisesRegex(ValueError, 'gravity differs'):
            target_at(path, 0., -120., np.eye(4), self.cfg)

    def test_smooth_corrections_never_accumulate_and_wrap_angles(self):
        self.path.set_corrections([0., self.path.end/2, self.path.end],
                                  [[-5., 5., -5.], [5., -5., 5.], [-5., 5., -5.]])
        for s in np.linspace(0, self.path.end, 101):
            expected = self.path.angle_offset(s)
            self.assertLessEqual(abs(expected).max(), 5.+1e-12)
            np.testing.assert_allclose(self.path.measured_offset(self.path.rotation(s)), expected, atol=1e-10)
        with self.assertRaisesRegex(ValueError, 'within'):
            self.path.set_corrections([0., self.path.end], [[0., 0., 0.], [0., 5.01, 0.]])
        wrapped = build_reference(self.geometry, replace(self.cfg, tilt_azimuth_deg=179.), SweepOptions(fixed_roll_deg=179.), 1.)
        np.testing.assert_allclose(wrapped.measured_offset(wrapped.rotation_for_offset([0., 5., 5.])), [0., 5., 5.], atol=1e-10)

    def test_saved_reference_roundtrip_and_retired_artifacts(self):
        self.path.set_corrections([0., self.path.end/2, self.path.end],
                                  [[-2.5, 0., 5.], [0., 2.5, 0.], [2.5, -5., -2.5]])
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            dump(folder/'sweep_reference.json', self.path.record)
            report = {'path_kind': 'continuous_outer_front_sweep', 'planning_mode': 'simple',
                      'config': asdict(self.cfg), 'sweep_options': asdict(self.options),
                      'sweep_reference_file': fingerprint(folder/'sweep_reference.json')}
            loaded = load_sweep_reference(folder, report)
            for s in np.linspace(0., self.path.end, 31):
                np.testing.assert_allclose(loaded.rotation(s), self.path.rotation(s), atol=1e-12)
        bad = deepcopy(self.path.record)
        bad.pop('regional_orientation')
        with self.assertRaisesRegex(ValueError, 'retired'):
            SimplePath(bad, self.cfg)
        bad = deepcopy(self.path.record)
        bad['regional_orientation']['policy'] = 'regional_gravity_fixed_z_y_tool_x_v2'
        with self.assertRaisesRegex(ValueError, 'retired'):
            SimplePath(bad, self.cfg)
        with self.assertRaisesRegex(ValueError, 'three angles'):
            SimplePath(self.path.record, replace(self.cfg, tilt_deg=40.))

    def fake_scene(self, allowed):
        path = self.path
        class Robot:
            home = np.zeros(4)
            def solutions(self, pose):
                return [np.r_[pose[0, 3], path.measured_offset(pose[:3, :3])*.001]]
        class Checker:
            def check(self, q, clearance):
                valid = bool(allowed(q[0], q[1:]*1000))
                return SimpleNamespace(valid=valid, pair='' if valid else 'tool / tooth')
            def edge(self, a, b, clearance, step):
                return self.check((a+b)/2, clearance)
        return Robot(), Checker()

    def test_nominal_orientation_is_preferred_and_geometry_only_never_checks_collisions(self):
        robot, checker = self.fake_scene(lambda p, d: True)
        parameters = self.path.samples(.002)
        selected = solve_sweep(self.path, parameters, robot, checker, np.eye(4), self.cfg)
        self.assertEqual(self.path.record['regional_orientation']['search_mode'], 'nominal_fixed')
        np.testing.assert_allclose([n['angle_offset'] for n in selected], 0.)
        solve_sweep(self.path, parameters, robot, None, np.eye(4), replace(self.cfg, check_mode='geometry_only'))

    def test_collision_recovers_with_one_common_adjustment(self):
        robot, checker = self.fake_scene(lambda p, d: d[0] > 2.)
        selected = solve_sweep(self.path, self.path.samples(.002), robot, checker, np.eye(4), self.cfg)
        self.assertEqual(self.path.record['regional_orientation']['search_mode'], 'adjusted_fixed')
        self.assertGreater(self.path.offsets[0, 0], 2.)
        np.testing.assert_array_equal(self.path.offsets[0], self.path.offsets[-1])
        self.assertTrue(all(abs(n['angle_offset']).max() <= 5. for n in selected))

    def test_no_feasible_adjustment_fails_without_relaxing_bounds(self):
        robot, checker = self.fake_scene(lambda p, d: d[0] > 5.1)
        with self.assertRaisesRegex(SweepFailure, 'within_5_deg'):
            solve_sweep(self.path, self.path.samples(.003), robot, checker, np.eye(4), self.cfg)

    def test_local_recovery_stays_relative_to_original_anchor(self):
        def allowed(p, delta):
            return ((p >= .007 or delta[0] < -2.) and (p <= .016 or delta[0] > 2.))
        robot, checker = self.fake_scene(allowed)
        selected = solve_sweep(self.path, self.path.samples(.001), robot, checker, np.eye(4), self.cfg)
        self.assertEqual(self.path.record['regional_orientation']['search_mode'], 'bounded_local')
        self.assertTrue(all(abs(n['angle_offset']).max() <= 5. for n in selected))
        self.assertLess(selected[0]['angle_offset'][0], 0.)
        self.assertGreater(selected[-1]['angle_offset'][0], 0.)

    def test_actual_fk_five_degree_limit_is_enforced_even_in_diagnostic_mode(self):
        cfg = replace(self.cfg, orientation_audit='diagnostic')
        path = self.path
        parameters = path.samples(.002)
        teeth = [path.tooth((a+b)/2) for a,b in zip(parameters[:-1], parameters[1:])]
        trajectory = Trajectory(np.linspace(0., 10., len(parameters)), parameters[:, None],
                                ['trace']*len(teeth), teeth, ['progress'])
        class Robot:
            def fk(self, q, full=False):
                pose = target_at(path, q[0], -120., np.eye(4), cfg)
                pose[:3, :3] = path.rotation_for_offset([0., 0., 6.])
                return pose
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            dump(folder/'sweep_reference.json', path.record)
            report = {'path_kind': 'continuous_outer_front_sweep', 'planning_mode': 'simple',
                      'axis_convention': SIMPLE_AXIS_CONVENTION, 'tcp_frame_id': TCP_FRAME_ID,
                      'world_from_scan_m': np.eye(4), 'config': asdict(cfg),
                      'sweep_options': asdict(self.options),
                      'sweep_reference_file': fingerprint(folder/'sweep_reference.json'),
                      'motion_segments': [{'status': 'planned', 'reference_kind': 'continuous_outer_front_sweep',
                          'stroke_id': -1, 'teeth': [1, 2, 3, 4], 'parameters': parameters,
                          'rolls_deg': np.full(len(parameters), -120.),
                          'trace_start_waypoint': 0, 'trace_end_waypoint': len(parameters)-1}]}
            dump(folder/'planning_report.json', report)
            with self.assertRaisesRegex(RuntimeError, 'Independent trace audit'):
                audit_trace(folder, trajectory, Robot())
            audit = json.loads((folder/'trace_audit.json').read_text())
            self.assertFalse(audit['regional_angle_bounds_passed'])

    def test_cli_uses_three_angles_and_rejects_retired_two_angles(self):
        from .__main__ import main
        with redirect_stdout(io.StringIO()) as stream:
            self.assertEqual(main(['--dry-run', '--planning-mode', 'simple',
                                   '--pose-angles-deg', '35', '-20', '-120']), 0)
        self.assertEqual(json.loads(stream.getvalue())['config']['tilt_deg'], 35.)
        with self.assertRaisesRegex(ValueError, 'retired'):
            Config(planning_mode='simple', simple_angle_deg=-80.)
        with self.assertRaises(SystemExit), redirect_stdout(io.StringIO()):
            main(['--dry-run', '--planning-mode', 'simple', '--simple-angles-deg', '-75', '-33'])


if __name__ == '__main__':
    unittest.main()
