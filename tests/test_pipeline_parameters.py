"""File-based run parameters, CLI precedence and validation before simulation."""
from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from pipeline.parameters import parse_parameters
from pipeline.run import build_argparser, main
from pipeline.simulation import PROJECT_ROOT


class PipelineParameterTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.file = Path(temporary.name)/'run.json'

    def save(self, values):
        self.file.write_text(json.dumps(values), encoding='utf-8')
        return ['--params', str(self.file)]

    def parse(self, values, *overrides):
        return parse_parameters(build_argparser(), self.save(values)+list(overrides))

    def test_sample_matches_current_six_region_command_and_reaches_runtime(self):
        with patch('pipeline.run.run_pipeline', return_value={'status': 'completed'}) as run, \
                redirect_stdout(io.StringIO()):
            self.assertEqual(main(['--params', 'config/pipeline_outer_front.json', '--headless']), 0)
        args, options = run.call_args
        self.assertEqual(args[:4], ('mandibular', None,
            PROJECT_ROOT/'config/global_camera_registration.json', None))
        self.assertEqual(args[5:7], (1.8, 1.))
        self.assertTrue(args[8])
        expected = dict(region='outer_front', pose_angles_deg=[6.37, 13.26, -.4], jaw_opening_deg=20.,
                        planning_mode='simple',
                        joint_step_rad=1., max_orientation_step_deg=30.,
                        sweep_trim_mm=1., pose_spacing_mm=.25,
                        duration_s=None, transfer_speed_mm_s=150.,
                        orientation_audit='strict', registration_method='automatic')
        for name, value in expected.items():
            self.assertEqual(options[name], value, name)

    def test_cli_overrides_file_including_boolean_and_tuple(self):
        args = self.parse({'duration': 90, 'speed_mm_s': 1, 'headless': True,
                           'pose_angles_deg': [55, -20, -140]},
                          '--duration', '80', '--speed-mm-s', '1.2', '--no-headless',
                          '--pose-angles-deg', '50', '-10', '-135')
        self.assertEqual((args.duration, args.speed_mm_s, args.headless), (80., 1.2, False))
        self.assertEqual(args.pose_angles_deg, [50., -10., -135.])
        self.assertEqual(args.offset_mm, 1.5)

    def test_planning_mode_file_cli_precedence_and_runtime(self):
        selected = self.parse({'planning_mode': 'simple', 'pose_angles_deg': [35, -20, -120]})
        self.assertEqual(selected.planning_mode, 'simple')
        self.assertEqual(selected.pose_angles_deg, [35., -20., -120.])
        self.assertEqual(self.parse({'planning_mode': 'simple'},
                                   '--planning-mode', 'continuous').planning_mode, 'continuous')
        with patch('pipeline.run.run_pipeline', return_value={'status': 'completed'}) as run, \
                redirect_stdout(io.StringIO()):
            self.assertEqual(main(self.save({'planning_mode': 'simple', 'pose_angles_deg': [35, -20, -120]})), 0)
        self.assertEqual(run.call_args.kwargs['planning_mode'], 'simple')
        self.assertEqual(run.call_args.kwargs['pose_angles_deg'], [35., -20., -120.])
        with self.assertRaises(SystemExit):
            self.parse({'planning_mode': 'simple', 'simple_angles_deg': [-75, -33]})

    def test_omitted_parameters_and_old_cli_keep_defaults(self):
        args = self.parse({})
        original = parse_parameters(build_argparser(), [])
        for name, value in vars(original).items():
            if name != 'params':
                self.assertEqual(getattr(args, name), value)
        args = parse_parameters(build_argparser(), ['--headless', '--duration', '90'])
        self.assertEqual((args.headless, args.duration), (True, 90.))

    def test_file_paths_are_project_relative_and_accept_absolute_and_spaces(self):
        absolute = self.file.parent/'a scan.ply'
        args = self.parse({'config': 'config/global_camera_registration.json',
                           'scan': str(absolute), 'output': 'outputs/pipeline/my run'})
        self.assertEqual(args.config, PROJECT_ROOT/'config/global_camera_registration.json')
        self.assertEqual(args.scan, absolute)
        self.assertEqual(args.output, PROJECT_ROOT/'outputs/pipeline/my run')
        args = self.parse({'output': 'outputs/pipeline/file'}, '--output', 'cli-output')
        self.assertEqual(args.output, Path('cli-output'))

    def test_invalid_types_choices_and_unknown_fields_stop_before_runtime(self):
        invalid = [[], {'typo': 1}, {'params': 'another.json'}, {'duration': True},
                   {'duration': '90'}, {'duration': 10**400}, {'headless': 'false'}, {'headless': 0},
                   {'speed_mm_s': None}, {'scan': 1}, {'output': ''}, {'jaw': 'both'},
                   {'path_source': 'extracted'}, {'sweep_max_trim_mm': 3}, {'region': 'middle'},
                   {'speed_profile': 'constant'}, {'planning_mode': 'unknown'}, {'pose_angles_deg': [55, -20]},
                   {'pose_angles_deg': [55, False, -140]}, {'pose_angles_deg': '55 -20 -140'}]
        with patch('pipeline.run.run_pipeline') as run:
            for values in invalid:
                with self.subTest(values=values), redirect_stderr(io.StringIO()), \
                        self.assertRaises(SystemExit) as stopped:
                    main(self.save(values))
                self.assertEqual(stopped.exception.code, 2)
            run.assert_not_called()

    def test_optional_null_values_restore_unset_parameters(self):
        args = self.parse({'scan': None, 'duration': None, 'pose_angles_deg': None})
        self.assertIsNone(args.scan)
        self.assertIsNone(args.duration)
        self.assertIsNone(args.pose_angles_deg)

    def test_malformed_duplicate_nonfinite_and_missing_files_are_rejected(self):
        for text in ('{', '{"duration": 90, "duration": 80}',
                     '{"duration": NaN}', '{"duration": Infinity}', '{"duration": 1e999}'):
            with self.subTest(text=text), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.file.write_text(text, encoding='utf-8')
                parse_parameters(build_argparser(), ['--params', str(self.file)])
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_parameters(build_argparser(), ['--params', str(self.file.parent/'missing.json')])

    def test_windows_utf8_bom_is_accepted(self):
        self.file.write_text('{"duration": 90}', encoding='utf-8-sig')
        args = parse_parameters(build_argparser(), ['--params', str(self.file)])
        self.assertEqual(args.duration, 90.)

    def test_mutually_exclusive_values_are_checked_across_sources(self):
        cases = [({'no_output': True, 'dry_run': True}, []),
                 ({'no_output': True}, ['--dry-run']),
                 ({'mouth_forward_mm': 0}, ['--mouth-distance-mm', '20']),
                 ({'mouth_forward_mm': 45, 'mouth_distance_mm': 20}, [])]
        for values, flags in cases:
            with self.subTest(values=values), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.parse(values, *flags)

    def test_algorithm_constraints_still_fail_before_output_or_simulation(self):
        for values, overrides in [({'duration': -1}, []),
                                  ({'pose_angles_deg': [55, -20, -140]}, ['--fixed-roll-deg', '0'])]:
            with self.subTest(values=values), patch('pipeline.run.create_simulation') as scene, \
                    redirect_stderr(io.StringIO()):
                self.assertEqual(main(self.save(values)+['--dry-run']+overrides), 1)
                scene.assert_not_called()

    def test_real_sample_dry_run_does_not_simulate_or_create_output(self):
        output = self.file.parent/'should-not-exist'
        with patch('pipeline.run.create_simulation') as scene, redirect_stdout(io.StringIO()) as stream:
            result = main(['--params', 'config/pipeline_outer_front.json', '--dry-run', '--output', str(output)])
        self.assertEqual(result, 0)
        self.assertFalse(output.exists())
        scene.assert_not_called()
        report = json.loads(stream.getvalue())
        self.assertIsNone(report['target_duration_s'])
        self.assertEqual(report['pose_spacing_mm'], .25)
        self.assertEqual(report['registration_method'], 'automatic')


if __name__ == '__main__':
    unittest.main()
