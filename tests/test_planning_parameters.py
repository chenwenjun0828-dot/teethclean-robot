"""Parameter files must reach every planning command without changing its checks."""
import argparse
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import asdict
import importlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from pipeline.parameters import add_parameter_file, parse_parameters
from pipeline.simulation import PROJECT_ROOT
from planning import __main__ as planning_cli
from planning.config import Config
from planning.infrastructure.io import Inputs
from planning.motion_planning import execution, timing
from planning.test_view import sample_trajectory
from planning.visualization import pose_angles, preview, segment_map, sweep_preview, view


class PlanningParameterTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.file = self.root/'params.json'

    def save(self, values):
        self.file.write_text(json.dumps(values), encoding='utf-8')
        return ['--params', str(self.file)]

    def schema(self):
        parser = argparse.ArgumentParser()
        add_parameter_file(parser)
        parser.add_argument('--plan', type=Path, required=True)
        parser.add_argument('--out', type=Path, required=True)
        parser.add_argument('--camera', type=int, choices=range(12), default=10)
        parser.add_argument('--speed', type=view.positive, default=1.)
        parser.add_argument('--paused', action=argparse.BooleanOptionalAction, default=False)
        start = parser.add_mutually_exclusive_group()
        start.add_argument('--time', type=float)
        start.add_argument('--waypoint', type=int)
        return parser

    def test_required_options_types_and_cli_precedence(self):
        parser = self.schema()
        args = parse_parameters(parser, self.save({
            'plan': 'outputs/a plan', 'out': 'outputs/b', 'camera': 4, 'speed': 2.5,
            'paused': True, 'waypoint': 3})+['--out', 'cli-output', '--no-paused', '--speed', '3'])
        self.assertEqual(args.plan, PROJECT_ROOT/'outputs/a plan')
        self.assertEqual(args.out, Path('cli-output'))
        self.assertEqual((args.camera, args.speed, args.paused, args.waypoint), (4, 3., False, 3))
        self.assertTrue(all(action.required for action in parser._actions if action.dest in ('plan', 'out')))
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_parameters(parser, [])

    def test_invalid_json_options_fail_before_runtime(self):
        invalid = [
            {'camera': True}, {'camera': 1.5}, {'camera': '2'}, {'camera': 12},
            {'waypoint': False}, {'speed': 0}, {'speed': -1}, {'speed': 'nan'},
            {'paused': 'false'}, {'plan': None}, {'out': None}, {'port': 8765},
        ]
        for values in invalid:
            with self.subTest(values=values), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_parameters(self.schema(), self.save({'plan': 'a', 'out': 'b', **values}))

    def test_cross_source_time_waypoint_conflicts_are_rejected(self):
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_parameters(self.schema(), self.save({'plan': 'a', 'out': 'b', 'time': 1.})+
                             ['--waypoint', '2'])

    def test_planning_sample_matches_current_parameters_and_boolean_overrides(self):
        args = parse_parameters(planning_cli.build_argparser(), ['--params', 'config/planning_outer_front.json',
            '--pose-angles-deg', '35', '-20', '-141', '--no-headless', '--check-mode', 'geometry_only'])
        self.assertEqual(args.pose_angles_deg, [35., -20., -141.])
        self.assertEqual(args.check_mode, 'geometry_only')
        self.assertFalse(args.headless)
        self.assertFalse(args.execute)
        self.assertEqual(args.standoff_mm, 1.8)
        self.assertEqual(args.out, PROJECT_ROOT/'outputs/planning/outer_front')
        self.assertEqual(args.perception_result, PROJECT_ROOT/'outputs/tutorial/manual/regions/outer_front')
        args = parse_parameters(planning_cli.build_argparser(), self.save({'no_output': True})+['--save-output'])
        self.assertFalse(args.no_output)

    def test_prepared_region_and_jaw_are_not_silently_overridden(self):
        prepared = SimpleNamespace(region='inner_front', inputs=Inputs.defaults('mandibular'))
        with patch.object(planning_cli.PreparedRegion, 'load', return_value=prepared), \
                patch.object(planning_cli, 'run') as run:
            for values in ({}, {'region': 'inner_front'}):
                with self.subTest(values=values), redirect_stdout(io.StringIO()) as stream:
                    self.assertEqual(planning_cli.main(self.save({
                        'perception_result': 'prepared', 'dry_run': True, **values})), 0)
                self.assertEqual(json.loads(stream.getvalue())['region'], 'inner_front')
            for values in ({'region': 'outer_front'}, {'jaw': 'maxillary'}):
                with self.subTest(values=values), redirect_stdout(io.StringIO()) as stream:
                    self.assertEqual(planning_cli.main(self.save({
                        'perception_result': 'prepared', 'dry_run': True, **values})), 1)
                self.assertIn('differs from prepared perception', stream.getvalue())
            run.assert_not_called()

    def test_main_rejects_file_execution_conflicts_and_unknown_fields_before_planning(self):
        for values in ({'execute': True, 'dry_run': True}, {'execute': True, 'check_mode': 'geometry_only'},
                       {'offset_mm': 1.8}, {'simple_angles_deg': [-75, -33]}):
            with self.subTest(values=values), patch.object(planning_cli, 'run') as run, \
                    redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                planning_cli.main(self.save(values))
            run.assert_not_called()

    def test_angle_viewer_file_and_browser_override_reach_runtime(self):
        server = MagicMock(server_port=8765)
        server.serve_forever.side_effect = KeyboardInterrupt
        with patch.object(pose_angles, 'PosePreview') as create, \
                patch.object(pose_angles, 'create_server', return_value=server), \
                patch.object(pose_angles.webbrowser, 'open') as browser, redirect_stdout(io.StringIO()):
            self.assertEqual(pose_angles.main(['--params', 'config/planning_pose_angles.json',
                                              '--no-browser', '--port', '8766']), 0)
            self.assertEqual(create.call_args.kwargs['angles'], [6.37, 13.26, -.4])
            self.assertEqual(create.call_args.kwargs['standoff_mm'], 1.8)
            self.assertEqual(create.call_args.kwargs['jaw_opening_deg'], 20.)
            browser.assert_not_called()
            self.assertEqual(pose_angles.main(self.save({'no_browser': True})+['--browser']), 0)
            browser.assert_called_once()

    def test_viewer_required_plan_and_custom_types_reach_runtime(self):
        scene = (SimpleNamespace(vis=SimpleNamespace(map=SimpleNamespace(znear=1.))),
                 object(), object(), {}, {}, np.eye(4), sample_trajectory(), {})
        with patch.object(view, 'load_scene', return_value=scene) as load, \
                patch.object(view, 'make_camera') as camera, patch.object(view, 'show') as show, \
                redirect_stdout(io.StringIO()):
            self.assertEqual(view.main(['--params', 'config/planning_view.json', '--no-paused',
                                        '--speed', '2', '--waypoint', '2']), 0)
            load.assert_called_once_with(PROJECT_ROOT/'outputs/planning/outer_front', jaw_opening_deg=None)
            self.assertEqual(camera.call_args.args[2], 10)
            self.assertEqual(show.call_args.args[-1], 2.)
            self.assertFalse(show.call_args.args[-2].paused)
            self.assertEqual(show.call_args.args[-2].speed, 2.)

    def test_all_diagnostic_commands_accept_file_only_required_options(self):
        for module, filename in ((preview, 'planning_preview.json'),
                                 (sweep_preview, 'planning_preview.json'),
                                 (segment_map, 'planning_segment_map.json')):
            with self.subTest(module=module.__name__), patch.object(module, 'render', return_value=self.root/'figure.png') as render, \
                    redirect_stdout(io.StringIO()):
                module.main(['--params', 'config/'+filename])
                self.assertEqual(render.call_args.args[0], PROJECT_ROOT/'outputs/planning/outer_front')
                if module is segment_map:
                    self.assertEqual(render.call_args.args[1], PROJECT_ROOT/'outputs/planning/segment_map')

    def test_retiming_file_supplies_both_required_paths_and_cli_duration(self):
        report = {'fast_dynamics_verified': True, 'phase_durations_s': {}, 'motion': {'moving_internal_waypoints': 5}}
        with patch.object(timing, 'run', return_value=(SimpleNamespace(duration=80.), report)) as run, \
                redirect_stdout(io.StringIO()):
            self.assertEqual(timing.main(['--params', 'config/planning_retime.json', '--duration', '80']), 0)
            self.assertEqual(run.call_args.args, (PROJECT_ROOT/'outputs/planning/outer_front',
                                                 PROJECT_ROOT/'outputs/planning/outer_front_90s', 80.))
            self.assertEqual(run.call_args.kwargs['trace_speed_mm_s'], 1.)

    def test_execution_original_timing_file_does_not_trigger_retiming(self):
        saved = {'path_kind': 'continuous_outer_front_sweep', 'timing_method': next(iter(execution.SAVED_SPLINE_METHODS)),
                 'fast_dynamics_verified': True, 'fast_execution_summary': {'status': 'completed'}}
        (self.root/'request.json').write_text(json.dumps({'config': asdict(Config())}), encoding='utf-8')
        (self.root/'planning_report.json').write_text(json.dumps(saved), encoding='utf-8')
        scene = (object(), object(), SimpleNamespace(duration=10.), object())
        with patch.object(execution, 'load_verified', return_value=scene) as load, \
                patch.object(execution, 'prepare_timed_plan', return_value=self.root) as retime, \
                patch('control.position.execute') as execute, redirect_stdout(io.StringIO()):
            flags = ['--params', 'config/planning_execute.json', '--plan', str(self.root)]
            self.assertEqual(execution.main(flags), 0)
            load.assert_called_once_with(self.root, jaw_opening_deg=None)
            retime.assert_not_called()
            execute.assert_not_called()
            self.assertEqual(execution.main(flags+['--no-original-timing', '--duration', '80']), 0)
            self.assertEqual(retime.call_args.args, (self.root, 80.))

    def test_compatibility_modules_share_the_configured_implementations(self):
        for name, implementation in (('view', view), ('preview', preview), ('sweep_preview', sweep_preview),
                                     ('segment_map', segment_map), ('execute', execution), ('retime', timing)):
            with self.subTest(name=name):
                self.assertIs(importlib.import_module('planning.'+name).main, implementation.main)


if __name__ == '__main__':
    unittest.main()
