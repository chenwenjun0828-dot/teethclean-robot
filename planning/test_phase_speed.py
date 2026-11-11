"""Continuous defaults, phase allocation and validated execution routing."""
from contextlib import redirect_stdout
from contextlib import ExitStack
from dataclasses import asdict
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch, Mock

import numpy as np

from .config import Config, SweepOptions
from .infrastructure.trajectory import Trajectory
from .motion_planning.fixed_path import build, phase_durations
from .test_fixed_path import CartesianRobot
from .test_sweep import prepared_region


class ContinuousDefaultTests(unittest.TestCase):
    def setUp(self):
        self.enterContext(patch('planning.__main__.prepare_region', return_value=prepared_region()))
        self.enterContext(patch('planning.__main__.resolve_regional_dataset', return_value=None))

    def test_python_api_defaults_and_none_both_consume_prepared_reference(self):
        from .pipeline import run
        from .infrastructure.io import Inputs
        prepared = prepared_region()
        model = SimpleNamespace(body=lambda _: SimpleNamespace(id=0), body_pos=np.zeros((1, 3)))
        with ExitStack() as stack, tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            replacements = {
                'planning.pipeline.fingerprint': {'sha256': 'test'},
                'pipeline.simulation.create_simulation': (model, object()),
                'planning.pipeline.body_transform': np.eye(4),
                'planning.pipeline.Robot': object(),
                'planning.pipeline.CollisionChecker': SimpleNamespace(calls=0),
            }
            for name, value in replacements.items():
                stack.enter_context(patch(name, return_value=value))
            stack.enter_context(patch('planning.pipeline.validate', side_effect=lambda output, plan, report, *args: (plan, report)))
            continuous = stack.enter_context(patch('planning.pipeline.plan_sweep', return_value=(None, {})))
            run(prepared, Path(temp)/'continuous')
            continuous.assert_called_once()
            self.assertIsInstance(continuous.call_args.args[-1], SweepOptions)
            self.assertEqual(continuous.call_args.args[-1].fixed_roll_deg, 0.)
            run(prepared, Path(temp)/'explicit_none', sweep_options=None)
            self.assertEqual(continuous.call_count, 2)
            self.assertEqual(continuous.call_args.args[-1].fixed_roll_deg, 0.)

    def test_cli_uses_fixed_continuous_planning(self):
        from .__main__ import main
        for flags, roll in (([], 0.), (['--pose-angles-deg', '55', '-20', '-141'], -141.)):
            with self.subTest(flags=flags), tempfile.TemporaryDirectory() as temp, \
                    redirect_stdout(io.StringIO()), \
                    patch('planning.__main__.run', return_value=(None, object(), {}, None)) as run:
                self.assertEqual(main(['--out', str(Path(temp)/'plan'), *flags]), 0)
                options = run.call_args.args[6]
                self.assertIsInstance(options, SweepOptions)
                self.assertEqual(options.fixed_roll_deg, roll)

    def test_geometry_only_keeps_working_with_continuous_default(self):
        from .__main__ import main
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()), \
                patch('planning.__main__.run', return_value=(None, None, {}, None)) as run:
            self.assertEqual(main(['--geometry-only', '--out', str(Path(temp)/'geometry')]), 0)
            self.assertTrue(run.call_args.args[3])
            self.assertEqual(run.call_args.args[6].fixed_roll_deg, 0.)

    def test_planning_run_applies_and_records_selected_jaw_opening(self):
        from .pipeline import run
        from .infrastructure.io import Inputs
        prepared = prepared_region()
        model = SimpleNamespace(body=lambda _: SimpleNamespace(id=0),
                                body_pos=np.zeros((1, 3)), body_quat=np.array([[1., 0., 0., 0.]]))
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()), \
                patch('planning.pipeline.fingerprint', return_value={'path': 'test', 'sha256': 'test'}), \
                patch('pipeline.simulation.create_simulation', return_value=(model, object())) as create, \
                patch('planning.pipeline.body_transform', return_value=np.eye(4)), \
                patch('planning.pipeline.Robot', return_value=SimpleNamespace(model=model)), \
                patch('planning.pipeline.CollisionChecker', return_value=SimpleNamespace(calls=0)), \
                patch('planning.pipeline.plan_sweep', return_value=(None, {'status': 'failed'})) as plan_sweep, \
                patch('planning.pipeline.validate', side_effect=lambda output, plan, report, *args: (plan, report)):
            output = Path(temporary)/'plan'
            run(prepared, output, jaw_opening_deg=20.)
            self.assertEqual(create.call_args.kwargs['jaw_settings'].opening_deg, 20.)
            self.assertEqual(plan_sweep.call_args.kwargs['jaw_opening_deg'], 20.)
            saved = json.loads((output/'planning_report.json').read_text(encoding='utf-8'))
            self.assertEqual(saved['jaw_opening_deg'], 20.)
            request = json.loads((output/'request.json').read_text(encoding='utf-8'))
            self.assertEqual(request['jaw_opening_deg'], 20.)


class PhaseTimingTests(unittest.TestCase):
    def test_spare_time_slows_trace_without_slowing_free_space(self):
        robot = CartesianRobot()
        q = np.c_[np.arange(6, dtype=float)*.001, np.zeros(6)]
        phases = ['transfer', 'ingress', 'trace', 'trace', 'trace']
        cfg = Config(speed_mm_s=.7, transfer_speed_mm_s=150., joint_speed_rad_s=2.2,
                     joint_accel_rad_s2=12., orientation_speed_deg_s=200.)
        with redirect_stdout(io.StringIO()):
            plan = build(q, phases, [1]*5, robot, cfg)
            stretched = build(q, phases, [1]*5, robot, cfg, 90., fixed_trace=plan.constant_trace)
        self.assertAlmostEqual(stretched.duration, 90.)
        np.testing.assert_allclose(np.diff(stretched.times)[:2], np.diff(plan.times)[:2])
        self.assertGreater(phase_durations(stretched)['trace'], phase_durations(plan)['trace'])
        np.testing.assert_array_equal(plan.joints, stretched.joints)
        np.testing.assert_array_equal(plan.constant_trace.knots, stretched.constant_trace.knots)
        self.assertGreater(stretched.sample(stretched.times[3])['dq'][0], 0.)
        self.assertLess(stretched.limits()['max_joint_accel_rad_s2'], cfg.joint_accel_rad_s2+1e-8)
        with self.assertRaisesRegex(RuntimeError, 'target is'):
            build(q, phases, [1]*5, robot, cfg, plan.duration/2)


class ExecutionTimingTests(unittest.TestCase):
    def test_failed_retime_never_starts_execution(self):
        from .motion_planning.execution import main
        with redirect_stdout(io.StringIO()), \
                patch('planning.motion_planning.execution.prepare_timed_plan', side_effect=RuntimeError('rate limit')), \
                patch('control.position.execute') as execute:
            self.assertEqual(main(['--plan', 'unused', '--headless']), 1)
            execute.assert_not_called()

    def test_cache_reuses_only_matching_source_and_profile(self):
        from .motion_planning.execution import prepare_timed_plan
        from .infrastructure.io import dump, fingerprint
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            root = Path(temp)/'source'
            root.mkdir()
            dump(root/'planning_report.json', {'duration_s': 500.})
            (root/'trajectory.npz').write_bytes(b'fixture')

            def create(source, output, duration, **options):
                output.mkdir()
                dump(output/'planning_report.json', {
                    'retime_source': {'trajectory': fingerprint(source/'trajectory.npz'),
                        'report': fingerprint(source/'planning_report.json'), 'duration_s': 500.},
                    'timing_profile': {'duration_s': duration, 'trace_speed_mm_s': options.get('trace_speed_mm_s', 1.),
                        'transfer_speed_mm_s': 150., 'jaw_opening_deg': 20., 'version': 4},
                    'fast_dynamics_verified': True, 'execution_ready': True, 'duration_s': duration})

            with patch('planning.motion_planning.execution.load_verified'), \
                    patch('planning.motion_planning.timing.run', side_effect=create) as retime:
                result = prepare_timed_plan(root, jaw_opening_deg=20.)
                self.assertEqual(prepare_timed_plan(root, jaw_opening_deg=20.), result)
                retime.assert_called_once()
                report = json.loads((result/'planning_report.json').read_text())
                report['fast_dynamics_verified'] = False
                dump(result/'planning_report.json', report)
                with self.assertRaisesRegex(ValueError, 'incomplete or stale'):
                    prepare_timed_plan(root, jaw_opening_deg=20.)
                different = prepare_timed_plan(root, jaw_opening_deg=20., trace_speed_mm_s=1.2)
                self.assertNotEqual(different, result)
                (root/'trajectory.npz').write_bytes(b'changed')
                changed = prepare_timed_plan(root, jaw_opening_deg=20.)
                self.assertNotEqual(changed, result)

    def test_execution_defaults_to_timed_plan_and_original_is_explicit(self):
        from .motion_planning.execution import main
        for original in (False, True):
            with self.subTest(original=original), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                source, fast = root/'source', root/'fast'
                for directory in (source, fast):
                    directory.mkdir()
                    (directory/'request.json').write_text(json.dumps({'config': asdict(Config())}))
                    (directory/'planning_report.json').write_text(json.dumps({'path_kind': 'continuous_outer_front_sweep'}))
                plan = SimpleNamespace(duration=90.)
                with redirect_stdout(io.StringIO()), \
                        patch('planning.motion_planning.execution.prepare_timed_plan', return_value=fast) as prepare, \
                        patch('planning.motion_planning.execution.load_verified', return_value=(None, None, plan, None)) as load, \
                        patch('control.position.execute', return_value={'status': 'completed'}) as execute:
                    flags = ['--original-timing'] if original else []
                    self.assertEqual(main(['--plan', str(source), '--headless', '--jaw-opening-deg', '20', *flags]), 0)
                    self.assertEqual(load.call_args.args[0], source if original else fast)
                    self.assertEqual(load.call_args.kwargs['jaw_opening_deg'], 20.)
                    self.assertEqual(execute.call_args.args[4], source if original else fast)
                    if original:
                        prepare.assert_not_called()
                    else:
                        self.assertEqual(prepare.call_args.args[1], 90.)
                        self.assertEqual(prepare.call_args.kwargs['trace_speed_mm_s'], 1.)


if __name__ == '__main__':
    unittest.main()
