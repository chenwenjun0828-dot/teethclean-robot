"""Local surface orientation, bounded recovery, saved references and FK audit."""
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
from .pose_search.constraints import axis_convention, pose_at, target_at
from .pose_search.continuous import SweepFailure, solve_sweep
from .simple import LocalSurfacePath
from .test_sweep import build_reference, straight_geometry


class LocalSurfacePlanningTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Config(planning_mode='local_surface', tilt_deg=50., tilt_azimuth_deg=0.)
        self.options = SweepOptions(fixed_roll_deg=50.)
        self.geometry = straight_geometry()
        self.path = build_reference(self.geometry, self.cfg, self.options, 1.)

    def test_requested_50_0_50_matches_local_preview_formula_and_preserves_geometry(self):
        world = np.eye(4)
        world[:3, :3] = Rotation.from_euler('xyz', [20., -30., 15.], degrees=True).as_matrix()
        world[:3, 3] = [.2, -.4, .6]
        path = build_reference(self.geometry, self.cfg, self.options, 1., world=world)
        original = build_reference(self.geometry, replace(self.cfg, planning_mode='continuous'), self.options, 1.)
        for actual, expected in zip(path.record['parts'], original.record['parts']):
            self.assertEqual(actual.keys(), expected.keys())
            for key in actual:
                np.testing.assert_equal(actual[key], expected[key])
        self.assertGreater(np.linalg.norm(path.rotation(0.)-path.rotation(path.end)), .1)
        for s in np.linspace(0., path.end, 31):
            expected = pose_at(path.evaluate(s), path.evaluate(s, 1), path.normal(s), 50., world, self.cfg)
            np.testing.assert_allclose(target_at(path, s, 50., world, self.cfg), expected, atol=1e-12)
            np.testing.assert_allclose(path.measured_offset(path.rotation(s), s), 0., atol=1e-10)

    def test_corrections_stay_relative_to_each_local_surface_without_accumulating(self):
        self.path.set_corrections([0., self.path.end/2, self.path.end],
                                  [[-5., 5., -5.], [5., -5., 5.], [-5., 5., -5.]])
        for s in np.linspace(0., self.path.end, 101):
            expected = self.path.angle_offset(s)
            np.testing.assert_allclose(self.path.measured_offset(self.path.rotation(s), s), expected, atol=1e-10)
            self.assertLessEqual(abs(expected).max(), 5.+1e-12)
        with self.assertRaisesRegex(ValueError, 'within'):
            self.path.set_corrections([0., self.path.end], [[0., 0., 0.], [0., 5.01, 0.]])

    def fake_scene(self, allowed):
        path = self.path
        start = path.evaluate(0.)[0]

        class Robot:
            home = np.zeros(4)

            def solutions(self, pose):
                s = np.clip(pose[0, 3]-start, 0., path.end)
                return [np.r_[pose[0, 3], path.measured_offset(pose[:3, :3], s)*.001]]

        class Checker:
            def check(self, q, clearance):
                valid = bool(allowed(q[0], q[1:]*1000))
                return SimpleNamespace(valid=valid, pair='' if valid else 'tool / tooth')

            def edge(self, a, b, clearance, step):
                return self.check((a+b)/2, clearance)

        return Robot(), Checker()

    def test_nominal_common_and_varying_corrections_use_local_frames(self):
        cases = [(lambda p, d: True, 'nominal_fixed'),
                 (lambda p, d: d[0] > 2., 'adjusted_fixed'),
                 (lambda p, d: (p >= .007 or d[0] < -2.) and (p <= .016 or d[0] > 2.),
                  'bounded_local')]
        for allowed, expected_mode in cases:
            with self.subTest(mode=expected_mode):
                robot, checker = self.fake_scene(allowed)
                parameters = self.path.samples(.001)
                with redirect_stdout(io.StringIO()):
                    selected = solve_sweep(self.path, parameters, robot, checker, np.eye(4), self.cfg, 50.)
                self.assertEqual(self.path.orientation['search_mode'], expected_mode)
                for s, node in zip(parameters, selected):
                    np.testing.assert_allclose(node['pose'], target_at(self.path, s, 50., np.eye(4), self.cfg), atol=1e-12)
                    self.assertLessEqual(abs(node['angle_offset']).max(), 5.)

    def test_unreachable_bounds_fail_and_geometry_only_never_uses_checker(self):
        robot, checker = self.fake_scene(lambda p, d: d[0] > 5.1)
        with redirect_stdout(io.StringIO()), self.assertRaisesRegex(SweepFailure, 'no_local_surface_pose_within_5_deg'):
            solve_sweep(self.path, self.path.samples(.003), robot, checker, np.eye(4), self.cfg, 50.)
        with redirect_stdout(io.StringIO()):
            selected = solve_sweep(self.path, self.path.samples(.003), robot, None, np.eye(4),
                                   replace(self.cfg, check_mode='geometry_only'), 50.)
        self.assertTrue(all(np.max(abs(n['angle_offset'])) == 0. for n in selected))

    def report(self, folder):
        dump(folder/'sweep_reference.json', self.path.record)
        return {'path_kind': 'continuous_outer_front_sweep', 'planning_mode': 'local_surface',
                'axis_convention': axis_convention(self.cfg), 'tcp_frame_id': TCP_FRAME_ID,
                'world_from_scan_m': np.eye(4), 'config': asdict(self.cfg),
                'sweep_options': asdict(self.options),
                'sweep_reference_file': fingerprint(folder/'sweep_reference.json')}

    def test_saved_corrections_roundtrip_and_mode_policy_angles_and_gravity_are_checked(self):
        self.path.set_corrections([0., self.path.end], [[-2.5, 0., 5.], [2.5, -5., -2.5]])
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            report = self.report(folder)
            loaded = load_sweep_reference(folder, report)
            self.assertIsInstance(loaded, LocalSurfacePath)
            for s in np.linspace(0., self.path.end, 31):
                np.testing.assert_allclose(loaded.rotation(s), self.path.rotation(s), atol=1e-12)
            for key, value in [('policy', 'retired'), ('base_angles_deg', [49., 0., 50.]),
                               ('gravity_world', [0., 1., 0.])]:
                bad = deepcopy(self.path.record)
                bad['local_surface_orientation'][key] = value
                with self.subTest(key=key), self.assertRaises(ValueError):
                    LocalSurfacePath(bad, self.cfg)
            report['planning_mode'] = 'simple'
            with self.assertRaisesRegex(ValueError, 'Planning mode differs'):
                load_sweep_reference(folder, report)

    def test_fk_audit_uses_progress_local_frame_and_enforces_five_degrees_in_diagnostic_mode(self):
        self.cfg = replace(self.cfg, orientation_audit='diagnostic')
        parameters = self.path.samples(.002)
        teeth = [self.path.tooth((a+b)/2) for a, b in zip(parameters[:-1], parameters[1:])]
        trajectory = Trajectory(np.linspace(0., 10., len(parameters)), parameters[:, None],
                                ['trace']*len(teeth), teeth, ['progress'])
        path, cfg = self.path, self.cfg

        class Robot:
            correction = 0.

            def fk(self, q, full=False):
                pose = target_at(path, q[0], 50., np.eye(4), cfg)
                pose[:3, :3] = path.rotation_for_offset([0., 0., self.correction], q[0])
                return pose

        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            report = self.report(folder)
            report['motion_segments'] = [{'status': 'planned', 'reference_kind': report['path_kind'],
                'stroke_id': -1, 'teeth': [1, 2, 3, 4], 'parameters': parameters,
                'rolls_deg': np.full(len(parameters), 50.), 'trace_start_waypoint': 0,
                'trace_end_waypoint': len(parameters)-1}]
            dump(folder/'planning_report.json', report)
            robot = Robot()
            audit = audit_trace(folder, trajectory, robot)
            self.assertTrue(audit['local_surface_angle_bounds_passed'])
            self.assertLess(max(audit['maximum_local_surface_angle_offsets_deg']), 1e-6)
            robot.correction = 6.
            with self.assertRaisesRegex(RuntimeError, 'Independent trace audit'):
                audit_trace(folder, trajectory, robot)
            self.assertFalse(json.loads((folder/'trace_audit.json').read_text())['local_surface_angle_bounds_passed'])

    def test_cli_and_pipeline_accept_new_mode_and_three_angles(self):
        from .__main__ import main
        from pipeline.run import run_pipeline
        with redirect_stdout(io.StringIO()) as stream:
            self.assertEqual(main(['--dry-run', '--planning-mode', 'local_surface',
                                   '--pose-angles-deg', '50', '0', '50']), 0)
        result = json.loads(stream.getvalue())
        self.assertEqual(result['planning_mode'], 'local_surface')
        self.assertEqual(result['sweep_options']['fixed_roll_deg'], 50.)
        result = run_pipeline(dry_run=True, planning_mode='local_surface', pose_angles_deg=[50., 0., 50.])
        self.assertEqual(result['planning_mode'], 'local_surface')
        self.assertEqual(result['fixed_roll_deg'], 50.)


if __name__ == '__main__':
    unittest.main()
