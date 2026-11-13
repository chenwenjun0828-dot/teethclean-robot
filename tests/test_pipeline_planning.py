"""Fixed-angle continuous planning and the planning/control boundary."""
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import asdict
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from pipeline.run import main, run_pipeline
from pipeline.tracking import run_tracking
from planning.config import Config, SweepOptions
from planning.infrastructure.io import dump, fingerprint, Inputs
from planning.infrastructure.trajectory import Trajectory
from planning.test_sweep import build_reference
from planning.test_sweep import straight_geometry, prepared_region


class PipelinePlanningModeTests(unittest.TestCase):
    def test_pipeline_preserves_fixed_angles_and_reports_with_and_without_output(self):
        continuity = {'continuous_trace_blocks': 1, 'transfers_inside_sweep': 0}
        path_kind = 'continuous_outer_front_sweep'
        for no_output in (False, True):
            with self.subTest(no_output=no_output), tempfile.TemporaryDirectory() as directory, \
                    patch('pipeline.run.create_simulation', return_value=(SimpleNamespace(nbody=0), object())), \
                    patch('pipeline.run.perceive_gingival_line', return_value=SimpleNamespace(
                        registration_dir=Path('registration'), topology_path=Path('line.obj'))), \
                    patch('pipeline.run.run_tracking', return_value={
                        'status': 'completed', 'duration_s': 2., 'planning_status': path_kind,
                        'path_kind': path_kind, 'continuity': continuity}) as tracking:
                output = None if no_output else Path(directory)/'run'
                report = run_pipeline(output_dir=output, no_output=no_output,
                    registration_method='automatic', pose_angles_deg=[35., -60., 10.], joint_step_rad=.3,
                    max_orientation_step_deg=30., sweep_trim_mm=2., duration_s=90.,
                    pose_spacing_mm=.04)
                self.assertEqual(tracking.call_args.kwargs['planning_mode'], 'simple')
                self.assertEqual(tracking.call_args.kwargs['fixed_roll_deg'], 10.)
                self.assertEqual(report['fixed_roll_deg'], 10.)
                self.assertEqual(tracking.call_args.kwargs['joint_step_rad'], .3)
                self.assertEqual(report['joint_step_rad'], .3)
                self.assertEqual(tracking.call_args.kwargs['pose_spacing_mm'], .04)
                self.assertEqual(report['pose_spacing_mm'], .04)
                for key, value in {'max_orientation_step_deg': 30., 'sweep_trim_mm': 2.,
}.items():
                    self.assertEqual(tracking.call_args.kwargs[key], value)
                    self.assertEqual(report[key], value)
                self.assertEqual(tracking.call_args.kwargs['duration_s'], 90.)
                self.assertEqual(tracking.call_args.kwargs['jaw_opening_deg'], report['jaw_opening']['opening_deg'])
                self.assertEqual(report['target_duration_s'], 90.)
                self.assertEqual(report['stages']['retiming']['target_duration_s'], 90.)
                self.assertEqual(tracking.call_args.kwargs['tilt_deg'], 35.)
                self.assertEqual(tracking.call_args.kwargs['tilt_azimuth_deg'], -60.)
                self.assertEqual((report['tilt_deg'], report['tilt_azimuth_deg']), (35., -60.))
                self.assertEqual(report['planning_mode'], 'simple')
                self.assertEqual(report['stages']['planning']['mode'], 'simple')
                self.assertEqual(report['stages']['planning']['path_kind'], path_kind)
                self.assertEqual(report['stages']['planning']['continuity'], continuity)
                actual_output = tracking.call_args.args[7].parent
                if no_output:
                    self.assertFalse(actual_output.exists())
                else:
                    saved = json.loads((output/'pipeline_report.json').read_text())
                    self.assertEqual(saved['planning_mode'], 'simple')

    def test_regional_fixed_cli_is_default_and_unknown_mode_is_rejected(self):
        with redirect_stdout(io.StringIO()), \
                patch('pipeline.run.run_pipeline', return_value={'status': 'completed'}) as run:
            self.assertEqual(main(['--no-output']), 0)
            self.assertEqual(run.call_args.kwargs['planning_mode'], 'simple')
            self.assertEqual(run.call_args.args[5], 1.5)
            self.assertTrue(run.call_args.kwargs['no_output'])
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as stopped:
            main(['--planning-mode', 'unknown'])
        self.assertEqual(stopped.exception.code, 2)

    def test_dry_run_reports_regional_fixed_and_default_flip(self):
        self.assertEqual(run_pipeline(dry_run=True)['planning_mode'], 'simple')
        self.assertEqual(run_pipeline(dry_run=True)['fixed_roll_deg'], 0.)
        with patch('pipeline.run.create_simulation') as scene:
            with self.assertRaises(ValueError):
                run_pipeline(planning_mode='segmented')
            scene.assert_not_called()

    def test_fixed_roll_cli_dry_run_and_invalid_requests(self):
        with redirect_stdout(io.StringIO()), \
                patch('pipeline.run.run_pipeline', return_value={'status': 'completed'}) as run:
            self.assertEqual(main(['--fixed-roll-deg', '10', '--no-output']), 0)
            self.assertEqual(run.call_args.kwargs['fixed_roll_deg'], 10.)
        self.assertEqual(run_pipeline(dry_run=True, fixed_roll_deg=10.)['fixed_roll_deg'], 10.)
        with patch('pipeline.run.create_simulation') as scene:
            for kwargs in ({'fixed_roll_deg': float('nan')}, {'fixed_roll_deg': 181.}):
                with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                    run_pipeline(**kwargs)
            scene.assert_not_called()

    def test_simple_mode_reaches_tracking_with_and_without_persistent_output(self):
        for no_output in (False, True):
            with self.subTest(no_output=no_output), tempfile.TemporaryDirectory() as directory, \
                    patch('pipeline.run.create_simulation', return_value=(SimpleNamespace(nbody=0), object())), \
                    patch('pipeline.run.perceive_gingival_line', return_value=SimpleNamespace(
                        registration_dir=Path('registration'), topology_path=Path('line.obj'))), \
                    patch('pipeline.run.run_tracking', return_value={
                        'status': 'completed', 'planning_status': 'continuous_outer_front_sweep'}) as tracking:
                report = run_pipeline(output_dir=None if no_output else Path(directory)/'run',
                                      no_output=no_output, registration_method='automatic', planning_mode='simple',
                                      pose_angles_deg=[35., -20., -120.])
                self.assertEqual(tracking.call_args.kwargs['planning_mode'], 'simple')
                self.assertEqual(tracking.call_args.kwargs['tilt_deg'], 35.)
                self.assertEqual(tracking.call_args.kwargs['tilt_azimuth_deg'], -20.)
                self.assertEqual(tracking.call_args.kwargs['fixed_roll_deg'], -120.)
                self.assertEqual(report['planning_mode'], 'simple')
                self.assertEqual(report['stages']['planning']['mode'], 'simple')

    def test_joint_step_cli_and_invalid_requests(self):
        with redirect_stdout(io.StringIO()), \
                patch('pipeline.run.run_pipeline', return_value={'status': 'completed'}) as run:
            self.assertEqual(main(['--fixed-roll-deg', '10', '--joint-step-rad', '.3', '--no-output']), 0)
            self.assertEqual(run.call_args.kwargs['joint_step_rad'], .3)
        self.assertEqual(run_pipeline(dry_run=True, joint_step_rad=.3)['joint_step_rad'], .3)
        with patch('pipeline.run.create_simulation') as scene:
            for value in (0., -.1, float('nan'), float('inf'), True):
                with self.subTest(value=value), self.assertRaises(ValueError):
                    run_pipeline(joint_step_rad=value)
            scene.assert_not_called()

    def test_tilt_cli_dry_run_and_invalid_requests(self):
        with redirect_stdout(io.StringIO()), \
                patch('pipeline.run.run_pipeline', return_value={'status': 'completed'}) as run:
            self.assertEqual(main(['--tilt-deg', '35', '--tilt-azimuth-deg', '-60', '--no-output']), 0)
            self.assertEqual(run.call_args.kwargs['tilt_deg'], 35.)
            self.assertEqual(run.call_args.kwargs['tilt_azimuth_deg'], -60.)
        report = run_pipeline(dry_run=True, tilt_deg=35., tilt_azimuth_deg=-60.)
        self.assertEqual((report['tilt_deg'], report['tilt_azimuth_deg']), (35., -60.))
        with patch('pipeline.run.create_simulation') as scene:
            for kwargs in ({'tilt_deg': -181.}, {'tilt_deg': 181.},
                           {'tilt_azimuth_deg': float('nan')}, {'tilt_azimuth_deg': 181.},
                           {'tilt_azimuth_deg': True}):
                with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                    run_pipeline(**kwargs)
            scene.assert_not_called()

    def test_trim_and_orientation_cli_dry_run_and_invalid_requests(self):
        with redirect_stdout(io.StringIO()), \
                patch('pipeline.run.run_pipeline', return_value={'status': 'completed'}) as run:
            self.assertEqual(main(['--max-orientation-step-deg', '30', '--sweep-trim-mm', '2',
                                   '--dry-run']), 0)
            self.assertEqual(run.call_args.kwargs['max_orientation_step_deg'], 30.)
            self.assertEqual(run.call_args.kwargs['sweep_trim_mm'], 2.)
        report = run_pipeline(dry_run=True, max_orientation_step_deg=30.,
                              sweep_trim_mm=2.)
        self.assertEqual(report['max_orientation_step_deg'], 30.)
        self.assertEqual(report['sweep_trim_mm'], 2.)
        defaults = run_pipeline(dry_run=True)
        self.assertEqual(defaults['max_orientation_step_deg'], Config().max_orientation_step_deg)
        self.assertEqual(defaults['sweep_trim_mm'], 1.)
        with tempfile.TemporaryDirectory() as directory, patch('pipeline.run.create_simulation') as scene:
            output = Path(directory)/'run'
            for kwargs in ({'max_orientation_step_deg': 0}, {'max_orientation_step_deg': float('nan')},
                           {'max_orientation_step_deg': True}, {'sweep_trim_mm': 0},
                           {'sweep_trim_mm': float('inf')}):
                with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                    run_pipeline(output_dir=output, **kwargs)
                self.assertFalse(output.exists())
            scene.assert_not_called()

    def test_standalone_planning_accepts_same_tuning_without_denser_sampling(self):
        from planning.__main__ import main as planning_main
        output = io.StringIO()
        with redirect_stdout(output):
            self.assertEqual(planning_main(['--dry-run', '--max-orientation-step-deg', '30',
                                           '--sweep-trim-mm', '2']), 0)
        report = json.loads(output.getvalue())
        self.assertEqual(report['config']['max_orientation_step_deg'], 30.)
        self.assertEqual(report['config']['spacing_mm'], .25)
        self.assertEqual(report['sweep_options']['trim_mm'], 2.)

    def test_duration_cli_dry_run_and_rejection_before_creating_output(self):
        with redirect_stdout(io.StringIO()), \
                patch('pipeline.run.run_pipeline', return_value={'status': 'completed'}) as run:
            self.assertEqual(main(['--duration', '90', '--dry-run']), 0)
            self.assertEqual(run.call_args.kwargs['duration_s'], 90.)
        self.assertEqual(run_pipeline(dry_run=True, duration_s=90.)['target_duration_s'], 90.)
        self.assertIsNone(run_pipeline(dry_run=True)['target_duration_s'])
        with tempfile.TemporaryDirectory() as directory, patch('pipeline.run.create_simulation') as scene:
            output = Path(directory)/'run'
            for value in (0., -1., float('nan'), float('inf'), True):
                with self.subTest(value=value), self.assertRaises(ValueError):
                    run_pipeline(output_dir=output, duration_s=value)
                self.assertFalse(output.exists())
            scene.assert_not_called()


class TrackingPlanningTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.registration = self.root/'registration'
        self.registration.mkdir()
        dump(self.registration/'registration_report.json', {'config': {
            'mandibular_mesh': str(Inputs.defaults('mandibular').scan), 'mandibular_line': 'unused/line.obj'}})
        self.prepared = prepared_region()
        self.output = self.root/'tracking'
        self.model_data = (object(), object())
        self.world = np.eye(4)
        self.world[:3, 3] = [.1, .2, .3]
        bundle = SimpleNamespace(camera_name='global', world_from_camera=np.eye(4),
                                 world_from_scan=self.world)
        self.enterContext(patch('pipeline.tracking.load_registered_paths', return_value=bundle))
        self.enterContext(patch('pipeline.tracking.camera_world_transform_m', return_value=np.eye(4)))
        self.planner = self.enterContext(patch('pipeline.tracking.plan', side_effect=self.planned))
        self.execute = self.enterContext(patch('pipeline.tracking.execute', return_value={
            'status': 'completed', 'max_rotation_error_deg': .1, 'duration_s': 2.}))
        self.hidden_transfer = False

    def planned(self, inputs, output, cfg, **kwargs):
        output.mkdir(parents=True, exist_ok=True)
        options = kwargs['sweep_options']
        path = build_reference(straight_geometry(), cfg, options, options.trim_mm)
        parameters = path.samples(.001)
        phases = ['transfer']+['trace']*(len(parameters)-1)
        if self.hidden_transfer:
            phases[len(phases)//2] = 'transfer'
        teeth = [path.tooth(0.)]+[path.tooth((a+b)/2)
                                 for a, b in zip(parameters[:-1], parameters[1:])]
        trajectory = Trajectory(np.arange(len(parameters)+1, dtype=float),
                                np.r_[-.001, parameters][:, None], phases, teeth, ['progress'])
        if kwargs.get('duration_s') is not None:
            trajectory.times *= kwargs['duration_s']/trajectory.duration
        dump(output/'sweep_reference.json', path.record)
        report = {'execution_ready': True, 'status': 'continuous_outer_front_sweep',
                  'planning_mode': cfg.planning_mode,
                  'path_kind': 'continuous_outer_front_sweep', 'coverage_fraction': .6,
                  'config': asdict(cfg), 'sweep_options': asdict(options),
                  'sweep_reference_file': fingerprint(output/'sweep_reference.json'),
                  'motion_segments': [{'status': 'planned', 'reference_kind': 'continuous_outer_front_sweep',
                      'parameters': parameters, 'rolls_deg': np.full(len(parameters), options.fixed_roll_deg or 0.),
                      'teeth': [1, 2, 3, 4], 'trace_start_waypoint': 1,
                      'trace_end_waypoint': len(parameters)}]}
        if kwargs.get('duration_s') is not None:
            execution = {'status': 'completed', 'duration_s': trajectory.duration,
                         'max_rotation_error_deg': .1}
            report.update(timing_method='fixed_joint_cubic_toppra', fast_dynamics_verified=True,
                          fast_execution_summary=execution)
            dump(output/'execution_report.json', execution)
        return None, trajectory, report, (*self.model_data, object())

    def track(self, **kwargs):
        return run_tracking(self.registration, output_dir=self.output,
                            model_data=self.model_data, headless=True,
                            perception_result=self.prepared, **kwargs)

    def test_default_regional_fixed_mode_preserves_frames_speed_and_one_sweep(self):
        result = self.track()
        self.assertIsInstance(self.planner.call_args.kwargs['sweep_options'], SweepOptions)
        np.testing.assert_array_equal(self.planner.call_args.kwargs['world_from_scan'], self.world)
        self.assertIs(self.planner.call_args.kwargs['model_data'], self.model_data)
        self.assertEqual(self.planner.call_args.args[2].speed_mm_s, 15.)
        self.assertEqual(self.planner.call_args.args[2].standoff_mm, 1.5)
        self.assertEqual(result['planning_mode'], 'simple')
        self.assertEqual(result['continuity']['continuous_trace_blocks'], 1)
        self.assertEqual(result['continuity']['transfers_inside_sweep'], 0)
        self.assertEqual(result['continuity']['retreats_inside_sweep'], 0)
        self.execute.assert_called_once()
        self.assertTrue(self.execute.call_args.kwargs['headless'])

    def test_segmented_api_is_removed(self):
        with self.assertRaises(ValueError):
            self.track(planning_mode='segmented')
        self.planner.assert_not_called()
        self.execute.assert_not_called()

    def test_simple_mode_survives_saved_reference_and_execution(self):
        result = self.track(planning_mode='simple', pose_angles_deg=[35., -20., -120.])
        self.assertEqual(self.planner.call_args.args[2].planning_mode, 'simple')
        self.assertEqual(self.planner.call_args.args[2].tilt_deg, 35.)
        self.assertEqual(self.planner.call_args.args[2].tilt_azimuth_deg, -20.)
        self.assertEqual(result['pose_angles_deg'], [35., -20., -120.])
        self.assertEqual(result['planning_mode'], 'simple')
        self.assertEqual(result['continuity']['continuous_trace_blocks'], 1)
        self.execute.assert_called_once()

    def test_wrong_mode_result_never_reaches_control(self):
        def wrong_mode(*args, **kwargs):
            geometry, plan, report, scene = self.planned(*args, **kwargs)
            report['planning_mode'] = 'continuous'
            return geometry, plan, report, scene
        self.planner.side_effect = wrong_mode
        with self.assertRaisesRegex(ValueError, 'different planning mode'):
            self.track(planning_mode='simple')
        self.execute.assert_not_called()

    def test_fixed_roll_reaches_continuous_planner_and_execution_contract(self):
        result = self.track(fixed_roll_deg=10., joint_step_rad=.3,
                            tilt_deg=35., tilt_azimuth_deg=-60.)
        self.assertEqual(self.planner.call_args.kwargs['sweep_options'].fixed_roll_deg, 10.)
        self.assertEqual(self.planner.call_args.args[2].joint_step_rad, .3)
        self.assertEqual(result['joint_step_rad'], .3)
        cfg = self.planner.call_args.args[2]
        self.assertEqual((cfg.tilt_deg, cfg.tilt_azimuth_deg), (35., -60.))
        self.assertEqual((result['tilt_deg'], result['tilt_azimuth_deg']), (35., -60.))
        self.assertEqual(result['continuity']['continuous_trace_blocks'], 1)
        self.execute.assert_called_once()

    def test_trim_and_orientation_reach_planner_with_fixed_sampling(self):
        result = self.track(max_orientation_step_deg=30., sweep_trim_mm=2.)
        cfg = self.planner.call_args.args[2]
        options = self.planner.call_args.kwargs['sweep_options']
        self.assertEqual(cfg.max_orientation_step_deg, 30.)
        self.assertEqual(cfg.spacing_mm, .25)
        self.assertEqual(options.trim_mm, 2.)
        self.assertEqual(result['max_orientation_step_deg'], 30.)
        self.assertEqual(result['sweep_trim_mm'], 2.)
        self.assertEqual(result['continuity']['continuous_trace_blocks'], 1)
        self.execute.assert_called_once()

    def test_pose_density_is_forwarded_without_changing_geometry_sampling(self):
        result = self.track(pose_spacing_mm=.025)
        cfg = self.planner.call_args.args[2]
        self.assertEqual(cfg.pose_spacing_mm, .025)
        self.assertEqual(cfg.spacing_mm, .25)
        self.assertEqual(result['pose_spacing_mm'], .025)

    def test_duration_is_planned_once_and_headless_reuses_the_dynamics_run(self):
        result = self.track(duration_s=90., jaw_opening_deg=20.)
        self.assertEqual(self.planner.call_args.args[1], self.output)
        self.assertEqual(self.planner.call_args.kwargs['duration_s'], 90.)
        self.assertEqual(self.planner.call_args.kwargs['jaw_opening_deg'], 20.)
        self.execute.assert_not_called()
        self.assertEqual(result['duration_s'], 90.)
        self.assertEqual(result['target_duration_s'], 90.)
        self.assertEqual(result['execution_plan_dir'], str(self.output.resolve()))
        self.assertEqual(result['source_plan_dir'], str(self.output.resolve()))
        self.assertEqual(result['continuity']['continuous_trace_blocks'], 1)
        self.assertFalse(self.output.with_name('tracking_source').exists())

    def test_missing_dynamics_or_wrong_duration_never_starts_execution(self):
        for wrong_duration in (False, True):
            def invalid(*args, **kwargs):
                geometry, plan, report, scene = self.planned(*args, **kwargs)
                if wrong_duration:
                    plan.times *= 2
                else:
                    report['fast_dynamics_verified'] = False
                return geometry, plan, report, scene
            self.planner.side_effect = invalid
            with self.assertRaisesRegex(ValueError, 'has not passed execution validation'):
                self.track(duration_s=90.)
            self.execute.assert_not_called()

    def test_invalid_duration_never_starts_planning(self):
        for value in (0., -1., float('nan'), float('inf'), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.track(duration_s=value)
        self.planner.assert_not_called()
        self.execute.assert_not_called()

    def test_failed_or_unaudited_continuous_plan_never_starts_control_or_falls_back(self):
        for trajectory, report in ((None, {'status': 'no_continuous_outer_front_sweep',
                                          'failure': {'reason': 'no_reachable_principal_source'}}),
                                   (object(), {'execution_ready': False,
                                               'status': 'independent_audit_failed'})):
            with self.subTest(report=report):
                self.planner.reset_mock()
                self.planner.side_effect = None
                self.planner.return_value = (None, trajectory, report, None)
                with self.assertRaisesRegex(ValueError, report.get('failure', {}).get('reason') or report['status']):
                    self.track()
                self.planner.assert_called_once()
                self.execute.assert_not_called()

    def test_continuous_mode_rejects_segmented_result_and_hidden_transfers(self):
        self.planner.side_effect = None
        self.planner.return_value = (None, object(), {'execution_ready': True}, None)
        with self.assertRaisesRegex(ValueError, 'different path kind'):
            self.track()
        self.execute.assert_not_called()
        self.planner.side_effect = self.planned
        self.hidden_transfer = True
        with self.assertRaisesRegex(ValueError, 'Retreat/transfer/re-approach'):
            self.track()
        self.execute.assert_not_called()

    def test_candidate_failure_reasons_survive_without_planning_artifacts(self):
        self.planner.side_effect = None
        self.planner.return_value = (None, None, {'status': 'no_continuous_outer_front_sweep',
            'attempts': [{'reason': 'no_continuous_ik_path'},
                         {'reason': 'Insufficient retained source coverage for tooth 2'}]}, None)
        with self.assertRaisesRegex(ValueError, 'no_continuous_ik_path; Insufficient retained source coverage'):
            self.track()
        self.planner.assert_called_once()
        self.execute.assert_not_called()


if __name__ == '__main__':
    unittest.main()
