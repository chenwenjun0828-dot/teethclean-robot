"""The supported force CLI executes current trajectories and preserves failures."""
import contextlib
import io
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from control.force.runner import main


class ForceCLITests(unittest.TestCase):
    def report(self, passed=False):
        return {'status': 'completed', 'force_control_passed': passed, 'stop_reason': '',
                'recorded_duration_s': 60., 'retained_path_progress_fraction': 1.,
                'peak_compensated_force_N': 1., 'force_tracking_passed': True,
                'scanned_surface_contact_passed': passed}

    def test_default_runs_trajectory_and_preserves_failed_surface_validation(self):
        output = io.StringIO()
        with patch('control.force.force_trajectory.run', return_value=self.report()) as run:
            with contextlib.redirect_stdout(output):
                result = main(['--source', 'current/tracking', '--out', 'new/force'])
        self.assertEqual(result, 2)
        run.assert_called_once()
        args, kwargs = run.call_args
        self.assertEqual(args[:2], (Path('current/tracking'), Path('new/force')))
        self.assertEqual(args[2].target_force_N, 1.)
        self.assertEqual(kwargs['contact_schedule'], 'auto')
        summary = json.loads(output.getvalue())
        self.assertTrue(summary['force_tracking_passed'])
        self.assertFalse(summary['scanned_surface_contact_passed'])
        self.assertFalse(summary['force_control_passed'])

    def test_explicit_trajectory_keeps_segment_options_and_success_exit_code(self):
        with patch('control.force.force_trajectory.run', return_value=self.report(True)) as run:
            with contextlib.redirect_stdout(io.StringIO()):
                result = main(['--source', 'current/tracking', '--out', 'new/force',
                               '--mode', 'trajectory', '--contact-schedule', 'segments',
                               '--segment-speed-mm-s', '2:1.5', '--segment-roll-deg', '2:-10',
                               '--viewer'])
        self.assertEqual(result, 0)
        args, kwargs = run.call_args
        self.assertEqual(args[5], {1: -10.})
        self.assertTrue(args[6])
        self.assertEqual(kwargs['segment_speeds_m_s'], {1: .0015})
        self.assertEqual(kwargs['contact_schedule'], 'segments')

    def test_missing_source_and_retired_probe_options_fail_before_execution(self):
        for argv, message in [(['--out', 'new/force'], '--source'),
                              (['--source', 'current/tracking', '--out', 'new/force',
                                '--mode', 'probe'], 'invalid choice'),
                              (['--source', 'current/tracking', '--out', 'new/force',
                                '--sweep-mm', '.1'], 'unrecognized arguments')]:
            with self.subTest(argv=argv):
                error = io.StringIO()
                with patch('control.force.force_trajectory.run') as run:
                    with contextlib.redirect_stderr(error), self.assertRaises(SystemExit) as stopped:
                        main(argv)
                self.assertEqual(stopped.exception.code, 2)
                self.assertIn(message, error.getvalue())
                run.assert_not_called()

    def test_invalid_limits_fail_before_execution(self):
        with patch('control.force.force_trajectory.run') as run:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as stopped:
                main(['--source', 'current/tracking', '--out', 'new/force', '--force-limit-n', '.5'])
        self.assertEqual(stopped.exception.code, 2)
        run.assert_not_called()


if __name__ == '__main__':
    unittest.main()
