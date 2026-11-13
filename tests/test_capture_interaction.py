"""Capture interaction tests without a camera or graphical desktop."""
from copy import deepcopy
import contextlib
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from perception.real import _capture
from perception.acquisition import realsense


class CaptureInteractionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.args = SimpleNamespace(config=None, serial=None, count=1,
                                    out=Path(temporary.name) / 'capture',
                                    jaw='mandibular', select_roi=True, preview=True)
        self.frames = [SimpleNamespace(colors=np.full((40, 50, 3), value),
                                       depth_m=np.full((40, 50), .1), depth_scale_m=.0001)
                       for value in (.2, .4, .6)]
        self.camera = Mock()
        self.camera.read.side_effect = self.frames
        self.cv = Mock()
        self.cv.getWindowProperty.return_value = 1
        self.cv.selectROI.return_value = (5, 6, 20, 15)
        self.saved = []

    def run_capture(self, keys):
        self.cv.waitKey.side_effect = keys
        def save(directory, frame, jaw, config):
            self.saved.append((frame, deepcopy(config)))
            return {'capture_id': 'test', 'valid_points': 300}
        with patch.dict('sys.modules', {'cv2': self.cv}), \
             patch.object(realsense, 'RealSenseCamera') as factory, \
             patch('perception.real.save_real_capture', side_effect=save), \
             contextlib.redirect_stdout(io.StringIO()):
            factory.return_value.__enter__.return_value = self.camera
            return _capture(self.args)

    def test_live_refresh_then_freeze_select_and_save_same_frame(self):
        self.assertEqual(self.run_capture([-1, ord('s')]), 0)
        self.assertEqual(self.camera.read.call_count, 2)
        self.assertIs(self.saved[0][0], self.frames[1])
        self.assertEqual(self.saved[0][1]['roi'], [5, 6, 25, 21])
        frozen = self.cv.selectROI.call_args.args[1]
        np.testing.assert_array_equal(frozen, np.full((40, 50, 3), 102, np.uint8))
        # Pixels outside the ROI remain at their original brightness.
        shown = self.cv.imshow.call_args_list[0].args[1]
        np.testing.assert_array_equal(shown, np.full((40, 50, 3), 51, np.uint8))
        self.cv.destroyAllWindows.assert_called_once()

    def test_cancel_roi_returns_to_live_preview_and_uses_new_frame(self):
        self.args.preview = False  # --select-roi alone must also preview.
        self.cv.selectROI.side_effect = [(0, 0, 0, 0), (5, 6, 20, 15)]
        self.assertEqual(self.run_capture([ord('s'), -1, 32]), 0)
        self.assertEqual(self.cv.imshow.call_count, 3)
        self.assertEqual(self.cv.selectROI.call_count, 2)
        self.assertIs(self.saved[0][0], self.frames[2])

    def test_quit_or_close_does_not_save(self):
        for key, visible in ((ord('q'), 1), (27, 1), (-1, 0)):
            with self.subTest(key=key):
                self.camera.read.side_effect = self.frames
                self.cv.getWindowProperty.return_value = visible
                with self.assertRaisesRegex(RuntimeError, 'Capture cancelled'):
                    self.run_capture([key])
                self.assertEqual(self.saved, [])
        self.cv.selectROI.assert_not_called()

    def test_noninteractive_capture_remains_immediate(self):
        self.args.preview = self.args.select_roi = False
        self.assertEqual(self.run_capture([]), 0)
        self.assertIs(self.saved[0][0], self.frames[0])
        self.cv.imshow.assert_not_called()
        self.cv.selectROI.assert_not_called()

    def test_multiple_frames_reuse_roi_and_preview_each_frame(self):
        self.args.count = 2
        self.assertEqual(self.run_capture([ord('s'), ord('s')]), 0)
        self.cv.selectROI.assert_called_once()
        self.assertEqual(len(self.saved), 2)
        self.assertIs(self.saved[1][0], self.frames[1])
        self.assertEqual(self.saved[1][1]['roi'], [5, 6, 25, 21])

    def test_preview_without_roi_saves_on_space(self):
        self.args.select_roi = False
        self.assertEqual(self.run_capture([32]), 0)
        self.cv.selectROI.assert_not_called()
        self.assertIs(self.saved[0][0], self.frames[0])


if __name__ == '__main__':
    unittest.main()
