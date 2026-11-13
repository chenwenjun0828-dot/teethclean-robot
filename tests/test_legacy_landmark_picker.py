"""Compatibility picker preserves ordering, coordinates and cleanup."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np
import open3d as o3d
import cv2

from perception.registration import legacy_landmark_picker as picker


class LegacyLandmarkPickerTests(unittest.TestCase):
    def setUp(self):
        self.points = np.arange(24, dtype=float).reshape(8, 3)
        self.scan = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.points))
        self.camera = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.points + [0, 0, 100]))

    def test_camera_then_scan_preserves_order_and_original_coordinates(self):
        with patch.object(picker, '_pick_cloud', side_effect=[list(range(6)), list(reversed(range(6)))]) as pick, \
             contextlib.redirect_stdout(io.StringIO()):
            source, target, si, ti = picker.pick_pairs_legacy(self.scan, self.camera)
        self.assertIs(pick.call_args_list[0].args[0], self.camera)
        self.assertIs(pick.call_args_list[1].args[0], self.scan)
        np.testing.assert_array_equal(source, self.points[si])
        np.testing.assert_array_equal(target, self.points[ti] + [0, 0, 100])
        self.assertEqual(si, list(reversed(range(6))))
        self.assertEqual(ti, list(range(6)))

    def test_incomplete_or_mismatched_selections_fail(self):
        for selections, error in (([[]], 'incomplete'),
                                  ([list(range(6)), list(range(5))], 'counts differ'),
                                  ([[0]*6, list(range(6))], 'duplicate')):
            with self.subTest(error=error), \
                 patch.object(picker, '_pick_cloud', side_effect=selections), \
                 contextlib.redirect_stdout(io.StringIO()), \
                 self.assertRaisesRegex(ValueError, error):
                picker.pick_pairs_legacy(self.scan, self.camera)

    def test_render_error_destroys_window(self):
        viewer = Mock()
        viewer.run.side_effect = RuntimeError('render failed')
        with patch.object(o3d.visualization, 'VisualizerWithEditing', return_value=viewer), \
             self.assertRaisesRegex(RuntimeError, 'render failed'):
            picker._pick_cloud(self.camera, 'test', camera=True)
        viewer.destroy_window.assert_called_once()

    def test_camera_reference_passed_to_scan_and_saved(self):
        image = np.zeros((40, 50, 3), dtype=np.uint8)
        def pick(cloud, title, *, camera=False, reference=None):
            if camera:
                reference['image'] = image
            return list(range(6))
        with tempfile.TemporaryDirectory() as tmp, \
             patch.object(picker, '_pick_cloud', side_effect=pick), \
             patch.object(cv2, 'imwrite', return_value=True) as save, \
             patch.object(picker, '_reference_window', return_value=contextlib.nullcontext()), \
             contextlib.redirect_stdout(io.StringIO()):
            reference_path = Path(tmp) / 'reference.png'
            picker.pick_pairs_legacy(self.scan, self.camera, reference_path=reference_path)
            checkpoint = json.loads(reference_path.with_suffix('.json').read_text())
            self.assertEqual(checkpoint['camera_indices'], list(range(6)))
        self.assertEqual(save.call_args.args[0], str(reference_path))
        self.assertIs(save.call_args.args[1], image)

    def test_scan_uses_native_event_loop_without_opencv_gui(self):
        viewer = Mock()
        viewer.get_picked_points.return_value = [0, 1]
        image = np.zeros((40, 50, 3), dtype=np.uint8)
        with patch.object(o3d.visualization, 'VisualizerWithEditing', return_value=viewer), \
             patch.object(cv2, 'imshow') as show, patch.object(cv2, 'waitKey') as pump:
            result = picker._pick_cloud(self.scan, 'scan', reference={'image': image})
        self.assertEqual(result, [0, 1])
        viewer.run.assert_called_once()
        pump.assert_not_called()
        show.assert_not_called()
        viewer.destroy_window.assert_called_once()

    def test_resume_skips_camera_selection_and_checks_fingerprints(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            cv2.imwrite(str(root / 'reference.png'), np.zeros((10, 10, 3), np.uint8))
            checkpoint = dict(camera_points_sha256=picker._cloud_hash(self.camera),
                              scan_points_sha256=picker._cloud_hash(self.scan),
                              camera_indices=list(range(6)), reference_image='reference.png')
            path = root / 'checkpoint.json'
            path.write_text(json.dumps(checkpoint))
            with patch.object(picker, '_pick_cloud', return_value=list(range(6))) as pick, \
                 patch.object(picker, '_reference_window', return_value=contextlib.nullcontext()), \
                 contextlib.redirect_stdout(io.StringIO()):
                result = picker.pick_pairs_legacy(self.scan, self.camera, camera_landmarks=path)
                pick.assert_called_once()
                self.assertIs(pick.call_args.args[0], self.scan)
                self.assertEqual(result[3], list(range(6)))
                checkpoint['camera_points_sha256'] = 'wrong'
                path.write_text(json.dumps(checkpoint))
                with self.assertRaisesRegex(ValueError, 'does not match'):
                    picker.pick_pairs_legacy(self.scan, self.camera, camera_landmarks=path)


if __name__ == '__main__':
    unittest.main()
