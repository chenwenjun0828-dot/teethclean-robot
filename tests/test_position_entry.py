"""Position execution must accept each current mode and retain shared validation."""
from dataclasses import asdict
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from control.position import load_verified
from planning.config import Config
from planning.pose_search.constraints import TCP_FRAME_ID, axis_convention


class PositionEntryTests(unittest.TestCase):
    def write_report(self, directory, mode, **overrides):
        cfg = Config(planning_mode=mode)
        report = {'config': asdict(cfg), 'axis_convention': axis_convention(cfg),
                  'tcp_frame_id': TCP_FRAME_ID, **overrides}
        (Path(directory) / 'planning_report.json').write_text(
            json.dumps(report), encoding='utf-8')

    def test_current_modes_reach_shared_plan_verification(self):
        for mode in ('simple', 'continuous'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                self.write_report(directory, mode)
                with patch('planning.motion_planning.execution.load_verified') as shared:
                    self.assertIs(load_verified(directory), shared.return_value)
                    shared.assert_called_once_with(directory)

    def test_wrong_mode_axes_and_retired_tcp_are_rejected(self):
        for overrides in (
            {'axis_convention': axis_convention(Config(planning_mode='continuous'))},
            {'tcp_frame_id': 'retired_tcp'},
        ):
            with self.subTest(overrides=overrides), tempfile.TemporaryDirectory() as directory:
                self.write_report(directory, 'simple', **overrides)
                with patch('planning.motion_planning.execution.load_verified') as shared:
                    with self.assertRaisesRegex(ValueError, 'Retired TCP frame'):
                        load_verified(directory)
                    shared.assert_not_called()

    def test_shared_verification_failure_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            self.write_report(directory, 'simple')
            with patch('planning.motion_planning.execution.load_verified',
                       side_effect=ValueError('Trajectory lacks a matching validation fingerprint')):
                with self.assertRaisesRegex(ValueError, 'matching validation fingerprint'):
                    load_verified(directory)


if __name__ == '__main__':
    unittest.main()
