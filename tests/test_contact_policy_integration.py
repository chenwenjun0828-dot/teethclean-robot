"""Contact configuration propagation and saved-plan trust boundaries."""
from contextlib import redirect_stdout
from copy import deepcopy
from dataclasses import asdict, replace
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from planning.config import Config, SweepOptions
from planning.infrastructure.io import dump, fingerprint
from planning.motion_planning.execution import load_verified, validate_motion_config
from planning.pose_search.constraints import axis_convention, TCP_FRAME_ID


class ContactConfigurationTests(unittest.TestCase):
    def test_contact_defaults_and_adjustable_nonworking_clearance(self):
        cfg = Config(motion_mode='contact')
        policy = cfg.tool_contact_policy()
        self.assertEqual(policy.working_segment_radius_mm, 1.)
        self.assertEqual(policy.contact_tolerance_mm, .05)
        self.assertEqual(policy.nonworking_clearance_mm, 0.)
        self.assertEqual(Config(standoff_mm=0., nonworking_clearance_mm=.08)
                         .tool_contact_policy().nonworking_clearance_mm, .08)
        self.assertIsNone(Config().tool_contact_policy())

    def test_explicit_contact_policy_allows_positive_reference_offset(self):
        cfg = Config(motion_mode='contact', standoff_mm=.14)
        self.assertEqual(cfg.standoff_mm, .14)
        self.assertEqual(cfg.tool_contact_policy().nonworking_clearance_mm, 0.)
        self.assertEqual(Config.from_saved(asdict(cfg)), cfg)
        with self.assertRaises(ValueError):
            Config(motion_mode='non-contact', standoff_mm=.14, clearance_mm=.15)

    def test_local_search_requires_boolean_and_restores_legacy_default(self):
        self.assertFalse(Config(local_angle_search=False).local_angle_search)
        saved = asdict(Config())
        saved.pop('local_angle_search')
        self.assertTrue(Config.from_saved(saved).local_angle_search)
        for value in (0, 1, None, 'false'):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'must be a boolean'):
                Config(local_angle_search=value)

    def test_legacy_zero_offset_does_not_acquire_contact_permission(self):
        saved = asdict(Config(standoff_mm=0.))
        for key in ('contact_tolerance_mm', 'working_segment_radius_mm', 'nonworking_clearance_mm'):
            saved.pop(key)
        cfg = Config.from_saved(saved)
        self.assertEqual(cfg.standoff_mm, 0.)
        self.assertIsNone(cfg.tool_contact_policy())

    def test_geometry_only_never_advertises_verified_contact(self):
        self.assertFalse(Config(standoff_mm=0., check_mode='geometry_only')
                         .motion_description()['working_contact_enabled'])

    def test_invalid_numeric_contact_parameters(self):
        for name in ('working_segment_radius_mm', 'contact_tolerance_mm', 'nonworking_clearance_mm'):
            for value in (True, -.01, float('nan'), float('inf'), '0'):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    Config(**{name: value})
        with self.assertRaises(ValueError):
            Config(working_segment_radius_mm=0.)

    def test_planning_cli_propagates_user_contact_values(self):
        from planning.__main__ import main
        with redirect_stdout(io.StringIO()) as stream:
            code = main(['--dry-run', '--motion-mode', 'contact',
                         '--contact-tolerance-mm', '.05', '--working-segment-radius-mm', '1.2',
                         '--nonworking-clearance-mm', '.08', '--no-local-angle-search'])
        self.assertEqual(code, 0)
        cfg = Config(**json.loads(stream.getvalue())['config'])
        self.assertFalse(cfg.local_angle_search)
        self.assertEqual(asdict(cfg.tool_contact_policy()), {
            'contact_tolerance_mm': .05, 'working_segment_radius_mm': 1.2,
            'nonworking_clearance_mm': .08})

    def test_pipeline_cli_accepts_the_same_values(self):
        from pipeline.run import build_argparser
        args = build_argparser().parse_args(['--offset-mm', '0',
            '--working-segment-radius-mm', '1.2', '--contact-tolerance-mm', '.05',
            '--nonworking-clearance-mm', '.08', '--no-local-angle-search'])
        self.assertFalse(args.local_angle_search)
        self.assertEqual((args.offset_mm, args.working_segment_radius_mm,
                          args.contact_tolerance_mm, args.nonworking_clearance_mm), (0., 1.2, .05, .08))


class SavedContactPlanTests(unittest.TestCase):
    def records(self):
        cfg = Config(motion_mode='contact')
        record = {'path_kind': 'continuous_outer_front_sweep', 'planning_mode': 'simple',
                  'region': cfg.region, 'config': asdict(cfg), 'sweep_options': asdict(SweepOptions())}
        return cfg, deepcopy(record), deepcopy(record), {'config': asdict(cfg)}

    def test_geometry_policy_cannot_change_at_load(self):
        for name, value in (('working_segment_radius_mm', 2.), ('contact_tolerance_mm', .1),
                            ('nonworking_clearance_mm', .15)):
            cfg, request, report, geometry = self.records()
            geometry['config'][name] = value
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, 'Working contact configuration'):
                validate_motion_config(request, report, geometry)

    def write_minimal_plan(self, folder):
        cfg, request, report, geometry = self.records()
        dump(folder/'geometry.json', geometry)
        dump(folder/'tooth_planes.json', {})
        dump(folder/'trajectory.npz', {})
        request.update({key: fingerprint(folder/'geometry.json')
                        for key in ('scene', 'robot_model', 'oral_settings')})
        report.update(planner='planning', execution_ready=True, check_mode='full',
                      collision_checking_enabled=True, axis_convention=axis_convention(cfg),
                      tcp_frame_id=TCP_FRAME_ID, trace_audit={'passed': True},
                      planes_file=fingerprint(folder/'tooth_planes.json'),
                      geometry_file=fingerprint(folder/'geometry.json'),
                      trajectory_file=fingerprint(folder/'trajectory.npz'))
        return request, report

    def test_no_contact_plan_loads_without_partition_identity(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            request, report = self.write_minimal_plan(folder)
            dump(folder/'request.json', request)
            dump(folder/'planning_report.json', report)
            with self.assertRaisesRegex(ValueError, 'partition manifest is missing'):
                load_verified(folder)


    def test_tampered_partition_is_rejected_before_scene_creation(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            request, report = self.write_minimal_plan(folder)
            dump(folder/'tool_contact_manifest.json', {'working_faces': [1]})
            for record in (request, report):
                record['tool_contact_manifest_file'] = fingerprint(folder/'tool_contact_manifest.json')
            dump(folder/'tool_contact_manifest.json', {'working_faces': [1, 2]})
            dump(folder/'request.json', request)
            dump(folder/'planning_report.json', report)
            with self.assertRaisesRegex(ValueError, 'partition manifest changed'):
                load_verified(folder)

    def test_ordinary_motion_audit_does_not_certify_working_contact(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            request, report = self.write_minimal_plan(folder)
            dump(folder/'tool_contact_manifest.json', {})
            for record in (request, report):
                record['tool_contact_manifest_file'] = fingerprint(folder/'tool_contact_manifest.json')
            report['motion_audit'] = {'passed': True}
            dump(folder/'request.json', request)
            dump(folder/'planning_report.json', report)
            with self.assertRaisesRegex(ValueError, 'Working contact trajectory verification is missing'):
                load_verified(folder)


class ContactExecutionMeasurements(unittest.TestCase):
    def test_runtime_keeps_negative_work_gap_separate_from_forbidden_clearance(self):
        from planning.infrastructure.collision import Check, CollisionChecker
        from planning.infrastructure.robot import Robot
        from planning.infrastructure.trajectory import Trajectory
        from pipeline.simulation import create_simulation
        from control.position.controller import execute
        model, data = create_simulation()
        robot = Robot(model, data)
        checker = CollisionChecker(robot, contact_policy=Config(standoff_mm=0.).tool_contact_policy())
        q = robot.home.copy()
        plan = Trajectory([0., .02], [q, q], ['transfer'], [0], robot.joint_names)
        with tempfile.TemporaryDirectory() as temporary, patch.object(
                checker, 'check', return_value=Check(True, .001, 'forbidden part / jaw', -.00003, True)):
            report = execute(model, data, plan, checker, temporary, headless=True)
        self.assertEqual(report['status'], 'completed')
        self.assertTrue(report['working_contact_verified'])
        self.assertAlmostEqual(report['minimum_actual_clearance_mm'], 1.)
        self.assertAlmostEqual(report['minimum_actual_working_gap_mm'], -.03)
        self.assertEqual(report['tool_contact_policy']['nonworking_clearance_mm'], 0.)


class InitialNodeCollisionSearchTests(unittest.TestCase):
    def node(self, tag, q=0.):
        return {'tag': tag, 'q': np.array([q, 0., 0., 0., 0., 0.]), 'pose': np.eye(4)}

    def test_invalid_start_does_not_query_downstream_geometry(self):
        from planning.pose_search.continuous import SweepFailure, whole_path_search
        nodes = [[self.node('bad')], [self.node('downstream')]]
        checked = []
        def reject_start(node):
            checked.append(node['tag'])
            return node['tag'] != 'bad', 'collision'
        with self.assertRaises(SweepFailure):
            whole_path_search(nodes, Config(), lambda a, b: self.fail('unexpected edge query'),
                              np.zeros(6), reject_start)
        self.assertEqual(checked, ['bad'])

    def test_alternative_start_still_checks_all_returned_nodes_and_edges(self):
        from planning.pose_search.continuous import whole_path_search
        nodes = [[self.node('bad'), self.node('good', .01)], [self.node('downstream')]]
        checked, edges = [], []
        def node_valid(node):
            checked.append(node['tag'])
            return node['tag'] != 'bad', 'collision'
        def edge_valid(a, b):
            edges.append((a['tag'], b['tag']))
            return True, ''
        selected = whole_path_search(nodes, Config(), edge_valid, np.zeros(6), node_valid)
        self.assertEqual([node['tag'] for node in selected], ['good', 'downstream'])
        self.assertEqual(checked, ['bad', 'good', 'downstream'])
        self.assertEqual(edges, [('good', 'downstream')])

    def test_impossible_nominal_probe_skips_whole_path_construction(self):
        from types import SimpleNamespace
        from planning.pose_search.continuous import SweepFailure
        from planning.simple import solve_sweep
        from planning.test_sweep import build_reference, straight_geometry
        cfg = Config()
        path = build_reference(straight_geometry(), cfg, SweepOptions(), 1.)
        robot = SimpleNamespace(home=np.zeros(6), solutions=lambda pose: [np.zeros(6)])
        checker = SimpleNamespace(check=lambda q, clearance: SimpleNamespace(valid=False))
        with patch('planning.pose_search.continuous.whole_path_search') as search:
            with redirect_stdout(io.StringIO()), self.assertRaises(SweepFailure):
                solve_sweep(path, path.samples(.001), robot, checker, np.eye(4), cfg)
        search.assert_not_called()

    def test_local_search_can_stop_after_common_candidates_fail(self):
        from planning.test_simple import SimplePlanningTests
        from planning.simple import solve_sweep
        from planning.pose_search.continuous import SweepFailure
        fixture = SimplePlanningTests()
        fixture.setUp()
        def allowed(position, delta):
            return ((position >= .007 or delta[0] < -2.)
                    and (position <= .016 or delta[0] > 2.))
        robot, checker = fixture.fake_scene(allowed)
        cfg = replace(fixture.cfg, local_angle_search=False)
        with redirect_stdout(io.StringIO()) as output:
            with self.assertRaisesRegex(SweepFailure, 'no_common_angle_correction_within_5_deg') as raised:
                solve_sweep(fixture.path, fixture.path.samples(.001), robot, checker, np.eye(4), cfg)
        self.assertFalse(raised.exception.details['local_angle_search'])
        self.assertEqual(raised.exception.details['fixed_angle_candidates_tested'], 125)
        self.assertIn('local angle search disabled', output.getvalue())
        self.assertNotIn('checking local corrections', output.getvalue())

    def test_probe_stops_after_one_branch_but_whole_path_keeps_all_branches(self):
        from planning.test_simple import SimplePlanningTests
        from planning.simple import solve_sweep
        from planning.pose_search.continuous import whole_path_search
        fixture = SimplePlanningTests(); fixture.setUp()
        robot, checker = fixture.fake_scene(lambda position, delta: True)
        original_solutions = robot.solutions
        robot.solutions = lambda pose: original_solutions(pose)*8
        checker.check = unittest.mock.Mock(wraps=checker.check)
        parameters = fixture.path.samples(.001)
        def search(nodes, cfg, edge_valid, home, node_valid):
            self.assertEqual(checker.check.call_count, min(17, len(parameters)))
            self.assertTrue(all(len(bank) == 8 for bank in nodes))
            return whole_path_search(nodes, cfg, edge_valid, home, node_valid)
        with patch('planning.pose_search.continuous.whole_path_search', side_effect=search):
            with redirect_stdout(io.StringIO()):
                selected = solve_sweep(fixture.path, parameters, robot, checker, np.eye(4), fixture.cfg)
        self.assertEqual(len(selected), len(parameters))

    def test_dense_infeasible_layer_stops_before_downstream_node_checks(self):
        from planning.test_simple import SimplePlanningTests
        from planning.simple import solve_sweep
        from planning.pose_search.continuous import whole_path_search, SweepFailure
        fixture = SimplePlanningTests(); fixture.setUp()
        parameters = fixture.path.samples(.001)
        probes = np.unique(np.linspace(0, len(parameters)-1, min(17, len(parameters))).astype(int))
        layer = next(i for i in range(len(parameters)) if i not in probes)
        blocked_position = fixture.path.position(parameters[layer], fixture.cfg.standoff_mm*.001)[0]
        robot, checker = fixture.fake_scene(lambda position, delta: abs(position-blocked_position) > 1e-10)
        original_solutions = robot.solutions
        robot.solutions = lambda pose: original_solutions(pose)*8
        queries, attempted = [], []
        original_check = checker.check
        def check(q, clearance):
            queries.append(q[0])
            return original_check(q, clearance)
        checker.check = check
        def search(nodes, cfg, edge_valid, home, node_valid):
            queries.clear()
            try:
                return whole_path_search(nodes, cfg, edge_valid, home, node_valid)
            except SweepFailure as error:
                attempted.append((error.details, queries.copy()))
                raise
        cfg = replace(fixture.cfg, local_angle_search=False)
        with patch('planning.pose_search.continuous.whole_path_search', side_effect=search):
            with redirect_stdout(io.StringIO()), self.assertRaises(SweepFailure):
                solve_sweep(fixture.path, parameters, robot, checker, np.eye(4), cfg)
        self.assertEqual(len(attempted), 1)
        details, positions = attempted[0]
        self.assertEqual(details['layer'], layer)
        self.assertEqual(details['reason'], 'no_collision_free_ik')
        self.assertLessEqual(max(positions), blocked_position+1e-10)


if __name__ == '__main__':
    unittest.main()
