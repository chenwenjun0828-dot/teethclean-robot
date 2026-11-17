"""Frame interpretation must change coordinates exactly once, without altering evidence."""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import shutil
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from perception.coordinate_conversion.handeye_frames import select_handeye_chain
from perception.coordinate_conversion.correct_session import correct, validate_session
from perception.regions import PreparedRegion

ROOT = Path(__file__).resolve().parents[1]
SESSION = ROOT / 'assets/perception/sessions/20261113_220209'
CALIBRATION = ROOT / 'assets/calibration/ur7e_d405/20261113'


class HandeyeFrameTests(unittest.TestCase):
    def test_forward_optical_point_does_not_receive_ros_axis_rotation_by_default(self):
        calibration = {'T_wrist3_camera_color_frame_m': np.eye(4).tolist()}
        axis = np.array([[0., 0., 1., 0.], [-1., 0., 0., 0.],
                         [0., -1., 0., 0.], [0., 0., 0., 1.]])
        sdk = dict(T_color_optical_depth_optical_m=np.eye(4).tolist(),
                   T_color_frame_depth_optical_ros_convention_m=axis.tolist())
        h, d, meta = select_handeye_chain(calibration, sdk)
        np.testing.assert_allclose(h @ d @ [0, 0, .2, 1], [0, 0, .2, 1])
        self.assertEqual(meta['mode'], 'color_optical')
        self.assertFalse(meta['ros_optical_to_stream_rotation_applied'])
        _, legacy, old_meta = select_handeye_chain(calibration, sdk, 'declared')
        np.testing.assert_allclose(legacy @ [0, 0, .2, 1], [.2, 0, 0, 1])
        self.assertTrue(old_meta['ros_optical_to_stream_rotation_applied'])
        self.assertAlmostEqual(np.rad2deg(Rotation.from_matrix(legacy[:3, :3]).magnitude()), 120.)
        self.assertEqual(calibration['T_wrist3_camera_color_frame_m'], np.eye(4).tolist())
        with self.assertRaises(ValueError):
            select_handeye_chain(calibration, sdk, 'unknown')

    def test_stable_session_uses_optical_chain_and_consistent_directions(self):
        manifest = validate_session(SESSION)
        self.assertEqual(manifest['handeye_frame_interpretation']['mode'], 'color_optical')
        sdk = json.loads((CALIBRATION / 'camera_extrinsics.json').read_text())
        inventory = json.loads((CALIBRATION / 'handeye_inventory.json').read_text())
        chosen = next(c for c in inventory['calibrations'] if str(c['label']) == '25')
        h, d, _ = select_handeye_chain(chosen, sdk)
        b = np.load(SESSION / 'transforms/T_controller_base_wrist3_nominal_m.npy')
        for jaw in manifest['jaws']:
            ds = np.load(SESSION / jaw / 'T_depth_optical_scan_m.npy')
            np.testing.assert_allclose(np.load(SESSION / jaw / 'T_controller_base_scan_nominal_m.npy'),
                                       b @ h @ d @ ds, atol=1e-12)
        self.assertFalse((SESSION / 'transforms/T_wrist3_color_frame_m.npy').exists())

    def test_replay_is_idempotent_and_rejects_corrupt_manifest(self):
        # A sibling retains source paths; TemporaryDirectory cleans only its explicit tree.
        with tempfile.TemporaryDirectory(prefix='.frame-test-', dir=SESSION.parent) as tmp:
            test_session = Path(tmp)
            shutil.copytree(SESSION, test_session, dirs_exist_ok=True)
            before = {p.relative_to(test_session): hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in test_session.rglob('*') if p.is_file()}
            backup = ROOT / 'outputs/perception/frame_correction/test_unused_backup'
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertFalse(correct(test_session, backup))
            after = {p.relative_to(test_session): hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in test_session.rglob('*') if p.is_file()}
            self.assertEqual(before, after)
            target = test_session / 'mandibular/regions/outer_left/points_controller_base_nominal_m.csv'
            target.write_text('0,0,0\n')
            with self.assertRaisesRegex(ValueError, 'fingerprint'):
                correct(test_session, backup)

    def test_prepared_region_resave_retains_effective_frame(self):
        prepared = PreparedRegion.load(SESSION / 'mandibular/regions/outer_left')
        self.assertEqual(prepared.handeye_frame_interpretation['mode'], 'color_optical')
        with tempfile.TemporaryDirectory(prefix='.region-resave-', dir=SESSION.parent) as tmp:
            prepared.save(tmp, diagnostics=False)
            restored = PreparedRegion.load(tmp)
            self.assertEqual(restored.handeye_frame_interpretation, prepared.handeye_frame_interpretation)


if __name__ == '__main__':
    unittest.main()
