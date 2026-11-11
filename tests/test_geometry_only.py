"""Collision-free geometry-only planning: it verifies reachability and nothing more.

These tests pin the safety boundary of ``check_mode=geometry_only``: no
collision query may happen, no clearance may be reported, and the result may
never become an executable plan.
"""
from contextlib import redirect_stdout
from dataclasses import asdict
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from planning.config import Config, CHECK_MODES, GEOMETRY_ONLY
from planning.infrastructure.collision import Check, CollisionChecker, NoCollisionChecker
from planning.infrastructure.io import dump, fingerprint
from planning.infrastructure.robot import Robot
from planning.motion_planning import validation as validation_module
from planning.pose_search.continuous import (SweepFailure, candidate_nodes,
                                             solve_sweep)
from planning.infrastructure.transfer import final_retreat, initial_approach, _edge_valid
from pipeline.simulation import create_simulation


class GeometryOnlyConfigTests(unittest.TestCase):
    def test_default_is_full_and_only_the_two_documented_modes_exist(self):
        self.assertEqual(Config(planning_mode='continuous').check_mode, 'full')
        self.assertFalse(Config(planning_mode='continuous').geometry_only)
        self.assertEqual(set(CHECK_MODES), {'full', 'geometry_only'})
        self.assertTrue(Config(planning_mode='continuous', check_mode=GEOMETRY_ONLY).geometry_only)
        for value in ('collision_free', 'no-collision', '', None, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                Config(planning_mode='continuous', check_mode=value)

    def test_saved_configuration_restores_the_mode_and_old_files_default_to_full(self):
        saved = asdict(Config(planning_mode='continuous', check_mode=GEOMETRY_ONLY))
        self.assertEqual(Config.from_saved(saved).check_mode, GEOMETRY_ONLY)
        legacy = asdict(Config(planning_mode='continuous'))
        legacy.pop('check_mode')
        self.assertEqual(Config.from_saved(legacy).check_mode, 'full')

    def test_motion_description_discloses_the_disabled_clearance(self):
        full = Config(planning_mode='continuous').motion_description()['contact_policy']
        geometry = Config(planning_mode='continuous', check_mode=GEOMETRY_ONLY).motion_description()['contact_policy']
        self.assertIn('Full-tool collision rejection remains enabled', full)
        self.assertIn('Collision checking is disabled', geometry)
        self.assertIn('never execution-ready', geometry)


class NoCollisionCheckerTests(unittest.TestCase):
    def test_stub_never_reports_a_clearance_and_never_fails(self):
        robot = SimpleNamespace(model=object())
        checker = NoCollisionChecker(robot)
        for check in (checker.check(np.zeros(6)), checker.edge(np.zeros(6), np.ones(6), .001)):
            self.assertTrue(check.valid)
            self.assertTrue(np.isnan(check.clearance_m))
        self.assertEqual(checker.calls, 2)
        self.assertEqual(checker.mode, 'geometry_only')
        self.assertEqual(CollisionChecker.mode, 'full')


class _SpyChecker(NoCollisionChecker):
    """Records every query so a test can prove collision checking was skipped."""

    def __init__(self, robot):
        super().__init__(robot)
        self.queried = 0

    def check(self, q, clearance_m=0.):
        self.queried += 1
        return super().check(q, clearance_m)

    def edge(self, a, b, clearance_m, step_rad=.004):
        self.queried += 1
        return super().edge(a, b, clearance_m, step_rad)


class _BlockingChecker:
    """A real full-mode checker look-alike that collides everywhere."""

    mode = 'full'
    tool_center_tcp = np.array([0., 0., .04])

    def __init__(self):
        self.calls = 0

    def check(self, q, clearance_m=0.):
        self.calls += 1
        return Check(False, -1., 'tooth / tool')

    def edge(self, a, b, clearance_m, step_rad=.004):
        self.calls += 1
        return Check(False, -1., 'tooth / tool')


def _two_layer_path():
    """Minimal path object exposing the three methods candidate_nodes needs."""
    pose = np.eye(4)
    return SimpleNamespace(normal=lambda s: np.array([0., 0., 1.]),
                           tooth=lambda s: 1, part_index=lambda s: 0,
                           evaluate=lambda s, n=0: np.zeros(3),
                           parameters=[0., .001],
                           _pose=pose)


class SolveSweepTests(unittest.TestCase):
    class _Robot:
        home = np.zeros(6)

        def solutions(self, target, seed=None):
            return [np.zeros(6), np.full(6, .01)]

    def test_geometry_only_accepts_a_connected_chain_without_any_collision_query(self):
        robot = self._Robot()
        checker = _SpyChecker(SimpleNamespace(model=object()))
        cfg = Config(planning_mode='continuous', check_mode=GEOMETRY_ONLY)
        path = _two_layer_path()
        with patch('planning.pose_search.continuous.target_at', return_value=np.eye(4)), \
                redirect_stdout(io.StringIO()):
            selected = solve_sweep(path, [0., .001], robot, checker, np.eye(4), cfg)
        self.assertEqual(len(selected), 2)
        self.assertEqual(checker.queried, 0)

    def test_full_mode_would_have_rejected_the_same_geometry(self):
        robot = self._Robot()
        checker = _BlockingChecker()
        cfg = Config(planning_mode='continuous')
        path = _two_layer_path()
        with patch('planning.pose_search.continuous.target_at', return_value=np.eye(4)), \
                redirect_stdout(io.StringIO()):
            with self.assertRaises(SweepFailure):
                solve_sweep(path, [0., .001], robot, checker, np.eye(4), cfg)
        self.assertGreater(checker.calls, 0)

    def test_geometry_only_still_enforces_the_joint_step_limit(self):
        class JumpingRobot:
            """Second layer only offers a far-away IK branch: the chain must break."""
            home = np.zeros(6)

            def __init__(self):
                self.layer = 0

            def solutions(self, target, seed=None):
                self.layer += 1
                return [np.zeros(6)] if self.layer == 1 else [np.full(6, .5)]

        cfg = Config(planning_mode='continuous', check_mode=GEOMETRY_ONLY, joint_step_rad=.01)
        checker = _SpyChecker(SimpleNamespace(model=object()))
        path = _two_layer_path()
        with patch('planning.pose_search.continuous.target_at', return_value=np.eye(4)), \
                redirect_stdout(io.StringIO()):
            with self.assertRaises(SweepFailure) as failed:
                solve_sweep(path, [0., .001], JumpingRobot(), checker, np.eye(4), cfg)
        self.assertEqual(failed.exception.details['reason'], 'no_continuous_ik_path')
        self.assertEqual(checker.queried, 0)


class TransferTests(unittest.TestCase):
    def test_geometry_only_makes_every_free_space_edge_valid(self):
        cfg = Config(planning_mode='continuous', check_mode=GEOMETRY_ONLY)
        predicate = _edge_valid(cfg, _BlockingChecker())
        self.assertTrue(predicate(np.zeros(6), np.ones(6)))
        self.assertFalse(_edge_valid(Config(planning_mode='continuous'), _BlockingChecker())(np.zeros(6), np.ones(6)))

    def test_direct_approach_is_taken_when_collision_checking_is_off(self):
        robot = SimpleNamespace(home=np.zeros(6), fk=lambda q: np.eye(4),
                                lower=-np.ones(6), upper=np.ones(6))
        result = initial_approach(robot, _BlockingChecker(), Config(planning_mode='continuous', check_mode=GEOMETRY_ONLY),
                                  np.full(6, .01), np.array([0., 0., 1.]))
        self.assertEqual(result['method'], 'direct_initial_approach')
        with patch('planning.infrastructure.transfer.straight_tcp_motion',
                   return_value=[np.zeros(6), np.full(6, .001), np.full(6, .002)]):
            retreat = final_retreat(robot, _BlockingChecker(),
                                    Config(planning_mode='continuous', check_mode=GEOMETRY_ONLY), np.zeros(6),
                                    np.array([0., 0., 1.]))
        # Geometry-only keeps the full shortcut: the last node is reached directly.
        self.assertEqual(len(retreat), 1)
        np.testing.assert_array_equal(retreat[0], np.full(6, .002))


class MotionAuditTests(unittest.TestCase):
    def test_audit_reports_no_clearance_instead_of_a_fake_number(self):
        from planning.infrastructure.audit import audit_motion
        from planning.motion_planning.fixed_path import build
        from planning.test_fixed_path import CartesianRobot
        robot = CartesianRobot()
        q = np.c_[np.arange(4, dtype=float)*.001, np.zeros(4)]
        cfg = Config(planning_mode='continuous', speed_mm_s=10.)
        with redirect_stdout(io.StringIO()):
            plan = build(q, ['trace']*3, [1]*3, robot, cfg)
        full = audit_motion(plan, robot, SimpleNamespace(check=lambda q, c: Check(True, .003, '')),
                            cfg)
        self.assertTrue(full['collision_checked'])
        self.assertAlmostEqual(full['minimum_clearance_mm'], 3.)
        geometry = audit_motion(plan, robot, _BlockingChecker(), Config(planning_mode='continuous', check_mode=GEOMETRY_ONLY,
                                                                       speed_mm_s=10.))
        self.assertFalse(geometry['collision_checked'])
        self.assertIsNone(geometry['minimum_clearance_mm'])
        self.assertEqual(geometry['samples'], 0)


class ValidationTests(unittest.TestCase):
    """The geometry-only branch must stop before dynamics and stay non-executable."""

    @classmethod
    def setUpClass(cls):
        cls.model, cls.data = create_simulation()
        cls.robot = Robot(cls.model, cls.data)

    def _write_plan(self, directory):
        from planning.motion_planning.fixed_path import build
        from planning.test_sweep import SweepAuditTests
        directory = Path(directory)
        SweepAuditTests().setup_reference(directory, Config(planning_mode='continuous', check_mode=GEOMETRY_ONLY))
        q = self.robot.home.copy()
        with redirect_stdout(io.StringIO()):
            plan = build([q, q+.004, q+.008], ['trace', 'trace'], [1, 1], self.robot,
                         Config(planning_mode='continuous', check_mode=GEOMETRY_ONLY))
        plan.save(directory/'trajectory.npz')
        report = json.loads((directory/'planning_report.json').read_text(encoding='utf-8'))
        report.update(trajectory_file=fingerprint(directory/'trajectory.npz'),
                      planes_file=fingerprint(directory/'tooth_planes.json'),
                      geometry_file=fingerprint(directory/'geometry.json'),
                      per_tooth={'1': {'retained_length_mm': 5., 'source_length_mm': 5.}},
                      all_selected_teeth_have_motion=True, all_selected_meet_coverage=True)
        dump(directory/'planning_report.json', report)
        return plan, report

    def test_geometry_only_validation_never_runs_dynamics_and_never_marks_ready(self):
        cfg = Config(planning_mode='continuous', check_mode=GEOMETRY_ONLY)
        with tempfile.TemporaryDirectory() as temporary, \
                patch.object(validation_module, 'audit_trace',
                             return_value={'passed': True, 'actual_time_interpolant': True}), \
                patch('planning.infrastructure.audit.audit_motion',
                      return_value={'passed': True, 'collision_checked': False,
                                    'minimum_clearance_mm': None, 'samples': 0,
                                    'max_trace_tcp_speed_mm_s': 0., 'max_free_tcp_speed_mm_s': 0.}), \
                patch('control.position.execute') as execute:
            plan, report = self._write_plan(temporary)
            _, result = validation_module.validate(Path(temporary), plan, report,
                                                   self.robot, _BlockingChecker(), cfg)
        execute.assert_not_called()
        self.assertEqual(result['status'], 'geometry_only_verified')
        self.assertFalse(result['execution_ready'])
        self.assertFalse(result['fast_dynamics_verified'])
        self.assertFalse(result['collision_checking_enabled'])
        self.assertIsNone(result['minimum_clearance_mm'])
        self.assertFalse(result['clearance_measured'])
        self.assertNotIn('fast_execution_summary', result)


class ExecutionBoundaryTests(unittest.TestCase):
    def test_load_verified_refuses_a_geometry_only_plan_before_touching_the_scene(self):
        from planning.motion_planning.execution import load_verified
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            dump(directory/'planning_report.json',
                 {'planner': 'planning', 'check_mode': GEOMETRY_ONLY,
                  'collision_checking_enabled': False, 'execution_ready': False})
            dump(directory/'request.json', {})
            with self.assertRaisesRegex(ValueError, 'geometry_only'):
                load_verified(directory)

    def test_a_geometry_only_report_cannot_be_mistaken_for_a_full_one(self):
        from planning.motion_planning.execution import load_verified
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            # Even if every other gate were satisfied, the disabled clearance stops it.
            dump(directory/'planning_report.json',
                 {'planner': 'planning', 'check_mode': 'full',
                  'collision_checking_enabled': False, 'execution_ready': True})
            dump(directory/'request.json', {})
            with self.assertRaisesRegex(ValueError, 'geometry_only'):
                load_verified(directory)


class StandaloneCliTests(unittest.TestCase):
    def test_geometry_only_rejects_execute_and_plain_geometry_only(self):
        from planning.__main__ import main
        for argv, message in ((['--check-mode', 'geometry_only', '--execute',
                                '--out', 'unused'], 'never produces an executable plan'),
                              (['--check-mode', 'geometry_only', '--geometry-only',
                                '--out', 'unused'], 'cannot be combined')):
            with self.subTest(argv=argv), self.assertRaises(SystemExit) as stopped:
                main(argv)
            self.assertEqual(stopped.exception.code, 2)

    def test_geometry_only_is_passed_to_the_planner_and_returned_in_the_summary(self):
        from planning.__main__ import main
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()) as stream, \
                patch('planning.__main__.run', return_value=(None, object(),
                      {'status': 'geometry_only_verified', 'execution_ready': False,
                       'check_mode': GEOMETRY_ONLY}, None)) as run:
            code = main(['--check-mode', 'geometry_only', '--out', str(Path(temporary)/'plan')])
        self.assertEqual(code, 0)
        self.assertEqual(run.call_args.args[2].check_mode, GEOMETRY_ONLY)
        self.assertEqual(json.loads(stream.getvalue())['check_mode'], GEOMETRY_ONLY)

    def test_a_failed_geometry_only_run_does_not_look_like_success(self):
        from planning.__main__ import main
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()), \
                patch('planning.__main__.run', return_value=(None, None,
                      {'status': 'no_continuous_outer_front_sweep', 'execution_ready': False}, None)):
            code = main(['--check-mode', 'geometry_only', '--out', str(Path(temporary)/'plan')])
        self.assertEqual(code, 2)


class PipelineWiringTests(unittest.TestCase):
    def test_pipeline_forwards_the_mode_and_never_runs_dynamics(self):
        from pipeline.tracking import run_tracking
        with self.assertRaises(ValueError):
            run_tracking('unused', check_mode='bogus', output_dir=Path('x'), model_data=(1, 2))

    def test_dry_run_reports_the_requested_check_mode(self):
        from pipeline.run import run_pipeline
        report = run_pipeline(dry_run=True, check_mode=GEOMETRY_ONLY)
        self.assertEqual(report['check_mode'], GEOMETRY_ONLY)
        self.assertFalse(report['collision_checking_enabled'])
        self.assertTrue(run_pipeline(dry_run=True)['collision_checking_enabled'])
        with patch('pipeline.run.create_simulation') as scene:
            with self.assertRaises(ValueError):
                run_pipeline(check_mode='none')
            scene.assert_not_called()

    def test_cli_accepts_the_mode_and_rejects_unknown_values(self):
        from pipeline.run import main
        with redirect_stdout(io.StringIO()), \
                patch('pipeline.run.run_pipeline', return_value={'status': 'completed'}) as run:
            self.assertEqual(main(['--check-mode', 'geometry_only', '--no-output']), 0)
            self.assertEqual(run.call_args.kwargs['check_mode'], GEOMETRY_ONLY)
        with redirect_stdout(io.StringIO()), \
                patch('pipeline.run.run_pipeline', return_value={'status': 'geometry_only_verified',
                                                                 'check_mode': GEOMETRY_ONLY}):
            self.assertEqual(main(['--check-mode', 'geometry_only', '--no-output']), 0)
        from contextlib import redirect_stderr
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as stopped:
            main(['--check-mode', 'skip'])
        self.assertEqual(stopped.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
