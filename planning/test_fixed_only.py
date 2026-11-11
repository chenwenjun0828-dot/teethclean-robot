"""The public planner exposes only fixed-angle continuous motion."""
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import asdict
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from .config import Config, SweepOptions
from .infrastructure.io import dump
from .motion_planning.continuous import plan_sweep
from .motion_planning.contract import validate_sweep_contract
from .motion_planning.execution import validate_motion_config
from .test_sweep import straight_geometry
from planning.test_sweep import build_reference


class FixedOnlyTests(unittest.TestCase):
    def test_retired_cli_options_are_rejected_before_planning(self):
        from .__main__ import main
        options = (['--segmented'], ['--continuous'], ['--roll-step-deg', '30'],
                   ['--sweep-roll-step-deg', '10'], ['--y-axis-deviation-deg', '60'],
                   ['--y-axis-offset-deg', '10'], ['--spin-speed-deg-s', '60'],
                   ['--spin-accel-deg-s2', '120'], ['--speed-profile', 'constant'])
        for flags in options:
            with self.subTest(flags=flags), redirect_stderr(io.StringIO()), \
                    patch('planning.__main__.run') as run:
                with self.assertRaises(SystemExit) as stopped:
                    main(['--dry-run', *flags])
                self.assertEqual(stopped.exception.code, 2)
                run.assert_not_called()

    def test_saved_fixed_angle_metadata_is_readable_and_retired_strategies_fail(self):
        cfg = asdict(Config(planning_mode='continuous'))
        cfg['roll_step_deg'] = 30.
        options = {**asdict(SweepOptions(fixed_roll_deg=-141.)),
                   'roll_step_deg': 10., 'roll_policy': None}
        saved = {'path_kind': 'continuous_outer_front_sweep', 'config': cfg, 'sweep_options': options}
        self.assertEqual(validate_motion_config(saved, saved, saved).tilt_deg, 15.)
        self.assertEqual(SweepOptions.from_saved(options).fixed_roll_deg, -141.)
        for overrides in ({'path_kind': 'segmented_middle'},
                          {'sweep_options': {**options, 'fixed_roll_deg': None}},
                          {'sweep_options': {**options, 'roll_policy': {'max_deviation_deg': 60.}}}):
            retired = {**saved, **overrides}
            with self.subTest(overrides=overrides), self.assertRaisesRegex(ValueError, 'Only fixed-angle'):
                validate_motion_config(retired, retired, retired)

    def test_generated_sweep_preserves_selected_teeth_and_fixed_roll_contract(self):
        geometry = straight_geometry()
        geometry.report['strokes'] = []
        model = SimpleNamespace(body=lambda _: SimpleNamespace(id=0),
                                body_pos=np.zeros((1, 3)), body_quat=np.array([[1., 0., 0., 0.]]))
        class Robot:
            joint_names = ['progress']
            home = np.array([-.001])
            initial_qpos = home.copy()
            data = None
            def fk(self, q, full=False):
                pose = np.eye(4)
                pose[0, 3] = q[0]
                return pose
            def jacobian(self, q):
                return np.array([[1.], [0.], [0.], [0.], [0.], [0.]])
        robot = Robot()
        robot.model = model
        checker = SimpleNamespace(check=lambda *args: SimpleNamespace(valid=True),
                                  calls=0, dental_transforms={})
        def selected(path, parameters, *args):
            return [{'q': np.array([s]), 'roll': -141., 'pose': robot.fk([s])} for s in parameters]
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()):
            output = Path(temporary)
            dump(output/'geometry.json', {})
            dump(output/'tooth_planes.json', {})
            with patch('planning.motion_planning.continuous.body_transform', return_value=np.eye(4)), \
                    patch('planning.motion_planning.continuous.solve_sweep', side_effect=selected), \
                    patch('planning.motion_planning.continuous.final_retreat', return_value=[np.array([.05])]), \
                    patch('planning.motion_planning.continuous.initial_approach', return_value={
                        'segments': [('transfer', [np.array([0.])])], 'method': 'direct_initial_approach'}):
                cfg = Config(planning_mode='continuous')
                options = SweepOptions(fixed_roll_deg=-141.)
                reference = build_reference(geometry, cfg, options, options.trim_mm)
                trajectory, report = plan_sweep(geometry, robot, checker, np.eye(4), cfg, output,
                                                options, reference=reference)
            from .motion_planning.contract import load_sweep_reference
            self.assertIsNotNone(trajectory)
            self.assertIsNotNone(trajectory.constant_trace)
            self.assertEqual(report['adaptive_trace_nodes_added'], 0)
            self.assertEqual(len(trajectory.times), report['input_pose_count']+2)
            import json
            pose_report = json.loads((output/'pose_search_report.json').read_text(encoding='utf-8'))
            self.assertEqual(pose_report['pose_count'], report['input_pose_count'])
            self.assertEqual(pose_report['pose_spacing_mm'], .05)
            self.assertEqual(np.load(output/'pose_parameters_m.npy').shape, (report['input_pose_count'],))
            self.assertEqual(report['pose_mode'], 'fixed_angles')
            self.assertEqual(report['motion_segments'][0]['teeth'], [1, 2, 3, 4])
            result = validate_sweep_contract(trajectory, report, load_sweep_reference(output, report))
            self.assertTrue(result['all_selected_teeth'])


if __name__ == '__main__':
    unittest.main()
