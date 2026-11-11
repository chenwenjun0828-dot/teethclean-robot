"""Offset selection, independent zero-offset auditing and saved-plan integrity."""
import json
import tempfile
import unittest
import io
from contextlib import redirect_stdout
from dataclasses import asdict, replace
from pathlib import Path
from unittest.mock import patch

import numpy as np

from .config import Config, SweepOptions
from .execute import validate_motion_config
from .pose_search.constraints import AXIS_CONVENTION, TCP_FRAME_ID, pose_at, tool_rotation


class MotionModeTests(unittest.TestCase):
    def test_defaults_legacy_and_serialized_modes(self):
        for cfg, mode, offset in ((Config(planning_mode='continuous'), 'non-contact', 1.5),
                                 (Config(planning_mode='continuous', motion_mode='contact'), 'contact', 0.),
                                 (Config(planning_mode='continuous', standoff_mm=0.), 'contact', 0.),
                                 (Config(planning_mode='continuous', standoff_mm=2.), 'non-contact', 2.)):
            self.assertEqual((cfg.motion_mode, cfg.standoff_mm), (mode, offset))
            self.assertEqual(asdict(Config(**json.loads(json.dumps(asdict(cfg))))), asdict(cfg))
            self.assertEqual(replace(cfg, speed_mm_s=15.).motion_mode, mode)
            self.assertEqual(cfg.clearance_mm, .15)
        legacy = asdict(Config(planning_mode='continuous'))
        legacy.pop('motion_mode')
        self.assertEqual(Config(**legacy).motion_mode, 'non-contact')

    def test_conflicting_modes_and_invalid_offsets_are_rejected(self):
        for kwargs in ({'motion_mode': 'contact', 'standoff_mm': 1.5},
                       {'motion_mode': 'non-contact', 'standoff_mm': 0.},
                       {'motion_mode': 'unknown'}, {'standoff_mm': -.1},
                       {'standoff_mm': float('nan')}, {'standoff_mm': float('inf')},
                       {'standoff_mm': False}, {'motion_mode': 'contact', 'clearance_mm': 0.}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Config(planning_mode='continuous', **kwargs)

    def test_contact_changes_only_position_under_world_transform(self):
        from scipy.spatial.transform import Rotation
        world = np.eye(4)
        world[:3, :3] = Rotation.from_euler('xyz', [.4, -.3, .7]).as_matrix()
        world[:3, 3] = [.2, .3, .4]
        point = np.array([.01, .02, .03])
        n = np.array([0., 0., 1.])
        contact = pose_at(point, [1., 0., .2], n, 30., world, Config(planning_mode='continuous', motion_mode='contact'))
        lifted = pose_at(point, [1., 0., .2], n, 30., world, Config(planning_mode='continuous'))
        np.testing.assert_allclose(contact[:3, 3], world[:3, :3] @ point + world[:3, 3])
        np.testing.assert_allclose(lifted[:3, 3] - contact[:3, 3], world[:3, :3] @ n * .0015)
        np.testing.assert_array_equal(contact[:3, :3], lifted[:3, :3])

    def test_cli_routes_both_modes_and_rejects_conflicting_arguments(self):
        from .__main__ import main
        with tempfile.TemporaryDirectory() as temp:
            for extra, expected in (([], 1.5), (['--motion-mode', 'non-contact'], 1.5),
                                    (['--motion-mode', 'contact'], 0.), (['--standoff-mm', '0'], 0.)):
                with patch('planning.__main__.run', return_value=(None, None, {'status': 'geometry_only'}, None)) as run:
                    self.assertEqual(main(['--out', str(Path(temp) / 'plan'), '--geometry-only', *extra]), 0)
                    self.assertEqual(run.call_args.args[2].standoff_mm, expected)
            with patch('planning.__main__.run') as run:
                self.assertEqual(main(['--out', str(Path(temp) / 'plan'), '--motion-mode', 'contact',
                                       '--standoff-mm', '1.5']), 1)
                run.assert_not_called()

    def test_saved_mode_cannot_be_switched_without_replanning(self):
        legacy = {'path_kind': 'continuous_outer_front_sweep', 'sweep_options': asdict(SweepOptions()),
                  'config': asdict(Config(planning_mode='continuous'))}
        legacy['config'].pop('motion_mode')
        self.assertEqual(validate_motion_config(legacy, legacy, legacy).standoff_mm, 1.5)
        cfg = {**legacy, 'config': asdict(Config(planning_mode='continuous', motion_mode='contact'))}
        self.assertEqual(validate_motion_config(cfg, cfg, cfg).standoff_mm, 0.)
        with self.assertRaisesRegex(ValueError, 'Request/report'):
            validate_motion_config(legacy, cfg, legacy)
        with self.assertRaisesRegex(ValueError, 'validated geometry'):
            validate_motion_config(cfg, cfg, legacy)
        with self.assertRaisesRegex(ValueError, 'metadata'):
            validate_motion_config({**cfg, 'motion_mode': 'non-contact'}, cfg, cfg)

    def test_zero_offset_audit_rejects_old_lifted_trajectory(self):
        from planning.infrastructure.io import dump
        from planning.infrastructure.trajectory import Trajectory
        from planning.motion_planning.audit import audit_trace
        from .test_sweep import SweepAuditTests

        class Robot:
            offset = 0.
            def fk(self, q, full=False):
                pose = np.eye(4)
                pose[:3, :3] = tool_rotation(path.evaluate(q[0], 1), path.normal(q[0]))
                pose[:3, 3] = path.position(q[0], self.offset)
                return pose

        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            path, plan, _ = SweepAuditTests().setup_reference(directory, Config(planning_mode='continuous', motion_mode='contact'))
            robot = Robot()
            self.assertTrue(audit_trace(directory, plan, robot)['passed'])
            robot.offset = .0015
            with self.assertRaisesRegex(RuntimeError, 'Independent trace audit'):
                audit_trace(directory, plan, robot)

    def test_zero_offset_does_not_bypass_full_tool_collision_checks(self):
        from types import SimpleNamespace
        from planning.pose_search.continuous import solve_sweep, SweepFailure
        from .test_sweep import straight_geometry, build_reference
        cfg = Config(planning_mode='continuous', motion_mode='contact')
        robot = SimpleNamespace(home=np.zeros(6), solutions=lambda target: [np.zeros(6)])
        clearances = []
        def reject(q, clearance):
            clearances.append(clearance)
            return SimpleNamespace(valid=False, pair='tool/tooth')
        checker = SimpleNamespace(tool_center_tcp=np.array([-.1, 0., 0.]), check=reject)
        path = build_reference(straight_geometry(), cfg, SweepOptions())
        with redirect_stdout(io.StringIO()), self.assertRaises(SweepFailure):
            solve_sweep(path, path.samples(.001), robot, checker, np.eye(4), cfg)
        self.assertTrue(clearances)
        np.testing.assert_allclose(clearances, .00015)


if __name__ == '__main__':
    unittest.main()
