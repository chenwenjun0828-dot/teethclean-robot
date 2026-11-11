"""Fixed local angles, regional IK selection and saved trajectory integrity."""
from contextlib import redirect_stdout, redirect_stderr
from dataclasses import asdict, replace
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from .config import Config, SweepOptions, continuous_path_kind, resolve_pose_angles
from perception.regions.geometry import Geometry, Stroke
from .infrastructure.io import dump, fingerprint, Inputs
from .test_sweep import prepared_region, build_reference
from .infrastructure.trajectory import Trajectory
from .motion_planning.audit import audit_trace
from .motion_planning.contract import load_sweep_reference, validate_sweep_contract
from .motion_planning.execution import validate_motion_config
from .pose_search.constraints import AXIS_CONVENTION, TCP_FRAME_ID, target_at
from .pose_search.continuous import solve_sweep, SweepFailure


def regional_geometry(region, count=5):
    teeth = list(range(11, 11+count))
    strokes = []
    for i, tooth in enumerate(teeth):
        parameters = np.linspace(i/count, (i+1)/count, 21)
        points = np.c_[np.zeros((21, 2)), -.006*count*parameters]
        strokes.append(Stroke(i, 0, tooth, region, points, np.tile([0., 0., -1.], (21, 1)),
                              parameters, np.arange(21)))
    curve = {'knots': [0]*4+[1]*4,
             'coefficients_m': np.c_[np.zeros((4, 2)), -np.linspace(0., .006*count, 4)]}
    planes = {tooth: {'valid': True, 'normal_scan': [0., 1., 0.]} for tooth in teeth}
    return Geometry(strokes, planes, {'region': region, 'selected_teeth': teeth,
                    'regions': {region: teeth}, 'curves': [curve], 'preserve_stroke_order': True},
                    np.empty((0, 3)), np.empty((0, 3), int), np.empty(0, int))


def saved_reference(directory, region, count=5):
    cfg = Config(planning_mode='continuous', region=region, tilt_deg=55., tilt_azimuth_deg=-20.)
    options = SweepOptions(fixed_roll_deg=-141.)
    path = build_reference(regional_geometry(region, count), cfg, options, 1.)
    parameters = np.unique(np.r_[path.samples(.001), path.ends])
    trajectory = Trajectory(np.linspace(0., 10., len(parameters)), parameters[:, None],
        ['trace']*(len(parameters)-1),
        [path.tooth((a+b)/2) for a, b in zip(parameters[:-1], parameters[1:])], ['progress'])
    dump(directory/'sweep_reference.json', path.record)
    report = {'region': region, 'selected_teeth': path.record['teeth'],
              'path_kind': continuous_path_kind(region), 'config': asdict(cfg),
              'sweep_options': asdict(options), 'axis_convention': AXIS_CONVENTION,
              'tcp_frame_id': TCP_FRAME_ID, 'world_from_scan_m': np.eye(4),
              'sweep_reference_file': fingerprint(directory/'sweep_reference.json'),
              'motion_segments': [{'status': 'planned', 'reference_kind': continuous_path_kind(region),
                  'stroke_id': -1, 'teeth': path.record['teeth'], 'parameters': parameters,
                  'rolls_deg': np.full(len(parameters), -141.), 'trace_start_waypoint': 0,
                  'trace_end_waypoint': len(parameters)-1}]}
    dump(directory/'planning_report.json', report)
    dump(directory/'geometry.json', {'strokes': []})
    dump(directory/'tooth_planes.json', {})
    return cfg, path, trajectory, report


class RegionConfigurationTests(unittest.TestCase):
    def test_fixed_angle_tuple_is_explicit_and_validated(self):
        self.assertEqual(resolve_pose_angles([55., -20., -141.]), (55., -20., -141.))
        self.assertEqual(resolve_pose_angles(), (15., 0., 0.))
        for values in ([], [55, 0], [55, 0, 0, 0], [True, 0, 0], [55, np.nan, 0],
                       [181, 0, 0], [-181, 0, 0], [55, 181, 0], [55, 0, -181], 'abc', 2):
            with self.subTest(values=values), self.assertRaises(ValueError):
                resolve_pose_angles(values)
        with self.assertRaisesRegex(ValueError, 'individual angle'):
            resolve_pose_angles([55, -20, -141], fixed_roll_deg=10.)
        with self.assertRaises(ValueError):
            Config(planning_mode='continuous', region='both')
        legacy = asdict(Config(planning_mode='continuous'))
        legacy.pop('region')
        self.assertEqual(Config(**legacy).region, 'outer_front')

    def test_planning_cli_routes_region_and_three_angles(self):
        from .__main__ import main
        for region in ('outer_left', 'outer_front', 'outer_right'):
            with self.subTest(region=region), redirect_stdout(io.StringIO()), \
                    tempfile.TemporaryDirectory() as directory, \
                    patch('planning.__main__.run', return_value=(None, object(), {}, None)) as run:
                self.assertEqual(main(['--region', region, '--pose-angles-deg', '55', '-20', '-141',
                                       '--out', str(Path(directory)/'plan')]), 0)
                cfg, options = run.call_args.args[2], run.call_args.args[6]
                self.assertEqual((cfg.region, cfg.tilt_deg, cfg.tilt_azimuth_deg,
                                  options.fixed_roll_deg), (region, 55., -20., -141.))
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()), \
                patch('planning.__main__.run', return_value=(None, object(), {}, None)) as run:
            self.assertEqual(main(['--jaw-opening-deg', '20', '--out', str(Path(directory)/'plan')]), 0)
            self.assertEqual(run.call_args.kwargs['jaw_opening_deg'], 20.)
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            main(['--region', 'outer_left', '--segmented', '--no-output'])

    def test_planning_cli_rejects_out_of_range_jaw_opening(self):
        from .__main__ import main
        with redirect_stdout(io.StringIO()), patch('planning.__main__.run') as run:
            self.assertEqual(main(['--jaw-opening-deg', '41', '--dry-run']), 1)
            run.assert_not_called()

    def test_pipeline_preserves_region_and_angles_through_temporary_execution(self):
        from pipeline.run import run_pipeline, main
        with redirect_stdout(io.StringIO()), \
                patch('pipeline.run.run_pipeline', return_value={'status': 'completed'}) as run:
            self.assertEqual(main(['--region', 'outer_right', '--pose-angles-deg', '55', '-20', '-141',
                                   '--dry-run']), 0)
            self.assertEqual(run.call_args.kwargs['pose_angles_deg'], [55., -20., -141.])
            self.assertEqual(run.call_args.kwargs['region'], 'outer_right')
        for region in ('outer_left', 'outer_right'):
            result = run_pipeline(region=region, pose_angles_deg=(55, -20, -141), dry_run=True)
            self.assertEqual((result['region'], result['tilt_deg'], result['tilt_azimuth_deg'],
                              result['fixed_roll_deg']), (region, 55., -20., -141.))
            with patch('pipeline.run.create_simulation', return_value=(SimpleNamespace(nbody=0), object())), \
                    patch('pipeline.run.perceive_gingival_line', return_value=SimpleNamespace(
                        registration_dir=Path('registration'), topology_path=Path('topology.obj'))), \
                    patch('pipeline.run.run_tracking', return_value={'status': 'completed'}) as track:
                result = run_pipeline(region=region, pose_angles_deg=(55, -20, -141), no_output=True)
                self.assertEqual(result['region'], region)
                self.assertEqual(track.call_args.kwargs['region'], region)
                self.assertEqual(track.call_args.kwargs['fixed_roll_deg'], -141.)
                self.assertEqual(track.call_args.kwargs['tilt_deg'], 55.)
                self.assertFalse(track.call_args.args[7].exists())
        with patch('pipeline.run.create_simulation') as scene:
            for kwargs in ({'region': 'invalid'}, {'pose_angles_deg': [55, 0, 181]}):
                with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                    run_pipeline(**kwargs)
            scene.assert_not_called()


class RegionalSearchTests(unittest.TestCase):
    def test_side_strokes_follow_anterior_to_posterior_progress(self):
        for region in ('outer_left', 'outer_right'):
            geometry = regional_geometry(region)
            path = build_reference(regional_geometry(region), Config(planning_mode='continuous', region=region), SweepOptions(), 1.)
            self.assertLess(path.evaluate(path.end)[2], path.evaluate(0.)[2])
            self.assertEqual(path.record['teeth'], [11, 12, 13, 14, 15])
            self.assertEqual(path.record['synthetic_bridge_length_mm'], 0.)

    def test_fixed_angles_reach_side_source_and_global_search_without_roll_fallback(self):
        for region in ('outer_left', 'outer_right'):
            cfg = Config(planning_mode='continuous', region=region, tilt_deg=55., tilt_azimuth_deg=-20.)
            options = SweepOptions(fixed_roll_deg=-141.)
            geometry, targets = regional_geometry(region), []
            class Robot:
                home = np.zeros(1)
                def solutions(self, target):
                    targets.append(target)
                    return [np.array([-target[2, 3]*10])]
            class Checker:
                tool_center_tcp = np.array([-.01, 0., 0.])
                blocked = False
                def check(self, q, clearance):
                    return SimpleNamespace(valid=not self.blocked, pair='tool/tooth')
                def edge(self, a, b, clearance, step):
                    return SimpleNamespace(valid=not self.blocked, pair='tool/tooth')
            robot, checker = Robot(), Checker()
            with redirect_stdout(io.StringIO()):
                path = build_reference(geometry, cfg, options, 1.)
                parameters = path.samples(.001)
                selected = solve_sweep(path, parameters, robot, checker, np.eye(4), cfg,
                                       fixed_roll_deg=-141.)
            for s, node in zip(parameters, selected):
                self.assertEqual(node['roll'], -141.)
                np.testing.assert_allclose(node['pose'], target_at(path, s, -141., np.eye(4), cfg))
            checker.blocked = True
            with redirect_stdout(io.StringIO()), self.assertRaises(SweepFailure):
                solve_sweep(path, parameters, robot, checker, np.eye(4), cfg, fixed_roll_deg=-141.)


class RegionalAuditTests(unittest.TestCase):
    def test_regional_preview_entries_use_the_continuous_renderer(self):
        from .visualization.preview import render as preview
        from .visualization.segment_map import render as segment_map
        for region in ('outer_left', 'outer_right'):
            with tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary)
                cfg = Config(planning_mode='continuous', region=region)
                dump(directory/'geometry.json', {'config': asdict(cfg)})
                dump(directory/'tooth_planes.json', {})
                dump(directory/'planning_report.json', {'path_kind': continuous_path_kind(region)})
                with patch('planning.visualization.sweep_preview.render', return_value='rendered') as render:
                    self.assertEqual(preview(directory), 'rendered')
                    render.assert_called_once_with(directory)
                with patch('planning.visualization.sweep_preview.render', return_value='rendered') as render:
                    self.assertEqual(segment_map(directory, directory/'preview'), 'rendered')
                    render.assert_called_once_with(directory, directory/'preview')

    def test_tracking_executes_only_the_selected_validated_region(self):
        from pipeline.tracking import run_tracking
        for region in ('outer_left', 'outer_right'):
            for hidden_transfer in (False, True):
                with self.subTest(region=region, hidden_transfer=hidden_transfer), \
                        tempfile.TemporaryDirectory() as temporary:
                    root = Path(temporary)
                    registration = root/'registration'
                    registration.mkdir()
                    dump(registration/'registration_report.json', {'config': {
                        'mandibular_mesh': str(Inputs.defaults('mandibular').scan), 'mandibular_line': 'unused.csv'}})
                    prepared = prepared_region(regional_geometry(region), Config(planning_mode='continuous', region=region))
                    bundle = SimpleNamespace(camera_name='global', world_from_camera=np.eye(4),
                                             world_from_scan=np.eye(4))
                    def planned(inputs, output, cfg, **kwargs):
                        output.mkdir()
                        self.assertEqual(cfg.region, region)
                        self.assertEqual((cfg.tilt_deg, cfg.tilt_azimuth_deg,
                                          kwargs['sweep_options'].fixed_roll_deg), (55., -20., -141.))
                        _, _, trajectory, report = saved_reference(output, region)
                        report.update(execution_ready=True, coverage_fraction=.6,
                                      status=continuous_path_kind(region))
                        if hidden_transfer:
                            trajectory.phases[2] = 'transfer'
                        return None, trajectory, report, (object(), object(), object())
                    with patch('pipeline.tracking.load_registered_paths', return_value=bundle), \
                            patch('pipeline.tracking.camera_world_transform_m', return_value=np.eye(4)), \
                            patch('pipeline.tracking.plan', side_effect=planned), \
                            patch('pipeline.tracking.execute', return_value={
                                'status': 'completed', 'max_rotation_error_deg': .1}) as execute, \
                            redirect_stdout(io.StringIO()):
                        if hidden_transfer:
                            with self.assertRaisesRegex(ValueError, 'Retreat/transfer'):
                                run_tracking(registration, region=region, pose_angles_deg=[55, -20, -141],
                                             output_dir=root/'tracking', model_data=(object(), object()),
                                             planning_mode='continuous', perception_result=prepared)
                            execute.assert_not_called()
                        else:
                            result = run_tracking(registration, region=region, pose_angles_deg=[55, -20, -141],
                                                  output_dir=root/'tracking', model_data=(object(), object()),
                                                  planning_mode='continuous', perception_result=prepared)
                            execute.assert_called_once()
                            self.assertEqual(result['region'], region)
                            self.assertEqual(result['pose_angles_deg'], [55., -20., -141.])
                            self.assertTrue(result['continuity']['all_selected_teeth'])

    def test_one_and_five_tooth_contracts_and_full_fk_accept_selected_regions(self):
        for region in ('outer_left', 'outer_right'):
            for count in (1, 5):
                with self.subTest(region=region, count=count), tempfile.TemporaryDirectory() as temporary:
                    directory = Path(temporary)
                    cfg, path, trajectory, report = saved_reference(directory, region, count)
                    class Robot:
                        roll = -141.
                        def fk(self, q, full=False):
                            return target_at(path, q[0], self.roll, np.eye(4), cfg)
                    robot = Robot()
                    contract = validate_sweep_contract(trajectory, report, path)
                    self.assertTrue(contract['all_selected_teeth'])
                    self.assertNotIn('all_four_teeth', contract)
                    self.assertTrue(audit_trace(directory, trajectory, robot)['passed'])
                    robot.roll += 5.
                    with self.assertRaisesRegex(RuntimeError, 'Independent trace audit'):
                        audit_trace(directory, trajectory, robot)

    def test_changed_region_and_missing_tooth_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            _, path, trajectory, report = saved_reference(directory, 'outer_left')
            report['region'] = 'outer_right'
            with self.assertRaisesRegex(ValueError, 'region'):
                validate_sweep_contract(trajectory, report, path)
            with self.assertRaisesRegex(ValueError, 'Unsupported'):
                load_sweep_reference(directory, report)
            report['region'] = 'outer_left'
            report['motion_segments'][0]['teeth'].pop()
            with self.assertRaisesRegex(ValueError, 'selected region teeth'):
                validate_sweep_contract(trajectory, report, path)

    def test_saved_region_or_roll_cannot_be_changed_before_execution(self):
        cfg = asdict(Config(planning_mode='continuous', region='outer_left'))
        saved = {'config': cfg, 'region': 'outer_left', 'path_kind': 'continuous_outer_left_sweep',
                 'sweep_options': {'fixed_roll_deg': -141.}}
        self.assertEqual(validate_motion_config(saved, saved, saved).region, 'outer_left')
        with self.assertRaisesRegex(ValueError, 'Selected region'):
            validate_motion_config(saved, saved, {**saved, 'region': 'outer_right'})
        with self.assertRaisesRegex(ValueError, 'fixed roll'):
            validate_motion_config(saved, {**saved, 'sweep_options': {'fixed_roll_deg': 0.}}, saved)


if __name__ == '__main__':
    unittest.main()
