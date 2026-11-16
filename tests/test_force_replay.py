"""Saved force playback uses recorded poses and signals without running physics."""
import csv
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from control.force.force_history import CHANNELS
from control.force.replay import Playback, load_recording
from pipeline.simulation import DENTAL_MODEL_PATH
from planning.infrastructure.io import fingerprint


JOINTS = ('ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
          'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint')


class ForceReplayTests(unittest.TestCase):
    def test_controls_are_wall_clock_based_and_restart_at_contact(self):
        p = Playback(31., 40., 31.)
        p.advance(1.)
        self.assertEqual(p.time_s, 32.)
        p.key(ord(' '))
        p.advance(5.)
        self.assertEqual(p.time_s, 32.)
        p.key(ord(' '))
        p.key(ord('-'))
        self.assertEqual(p.speed, .5)
        p.advance(2.)
        self.assertEqual(p.time_s, 33.)
        p.key(ord('+'))
        self.assertEqual(p.speed, 1.)
        p.key(ord('H'))
        self.assertTrue(p.show_sensor)
        p.advance(30.)
        self.assertEqual(p.time_s, 40.)
        self.assertTrue(p.paused)
        p.key(ord('R'))
        self.assertEqual(p.time_s, 31.)
        self.assertFalse(p.paused)

    def test_recording_loads_saved_pose_force_and_contact_without_writing(self):
        with tempfile.TemporaryDirectory() as folder:
            run = Path(folder)
            report = {'status': 'completed', 'stage_passed': True, 'experiment_stage': 'hold',
                      'source_scene': {'sha256': fingerprint(DENTAL_MODEL_PATH)['sha256']}}
            (run/'force_trajectory_report.json').write_text(json.dumps(report))
            with (run/'trajectory_force.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=['time_s', 'phase', *JOINTS, *CHANNELS])
                writer.writeheader()
                for t, phase in [(0., 'transfer'), (1., 'trace'), (2., 'release')]:
                    row = {'time_s': t, 'phase': phase}
                    row.update({name: t for name in JOINTS})
                    row.update({name: .5 for name in CHANNELS})
                    writer.writerow(row)
            with (run/'trajectory_contacts.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=['time_s', 'x_m', 'y_m', 'z_m'])
                writer.writeheader()
                writer.writerow({'time_s': 1., 'x_m': .1, 'y_m': .2, 'z_m': .3})
            before = {p.name: p.read_bytes() for p in run.iterdir()}
            recording = load_recording(run)
            self.assertEqual(recording.index_at(.5), 1)
            self.assertEqual(recording.trace_start, 1.)
            np.testing.assert_allclose(recording.joints[1], np.ones(6))
            np.testing.assert_allclose(recording.contacts_at(1), [[.1, .2, .3]])
            self.assertEqual(before, {p.name: p.read_bytes() for p in run.iterdir()})
            report['experiment_stage'] = 'short'
            (run/'force_trajectory_report.json').write_text(json.dumps(report))
            self.assertEqual(load_recording(run).report['experiment_stage'], 'short')
            (run/'trajectory_contacts.csv').unlink()
            with self.assertRaises(FileNotFoundError):
                load_recording(run)
            report['stage_passed'] = False
            (run/'force_trajectory_report.json').write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, 'passed hold or short'):
                load_recording(run)


if __name__ == '__main__':
    unittest.main()
