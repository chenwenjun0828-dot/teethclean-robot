import unittest
from unittest.mock import patch
import numpy as np
import open3d as o3d
try:
    from perception.registration.landmark_selection import PairSelection, nearest_depth_pixel
except ImportError:
    PairSelection = nearest_depth_pixel = None


class PairedLandmarkTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(PairSelection, "Paired selection state is missing")
        self.scan = np.arange(24, dtype=float).reshape(8, 3)
        self.camera = self.scan + [10, -20, 150]
        self.state = PairSelection(self.scan, self.camera, minimum_pairs=6)

    def test_camera_then_scan_commits_one_pair_and_preserves_original_coordinates(self):
        self.assertEqual(self.state.next_side, "camera")
        self.state.select("camera", 7)
        self.assertEqual(self.state.next_side, "scan")
        self.assertEqual(self.state.pair_count, 0)
        self.state.select("scan", 2)
        self.assertEqual(self.state.camera_indices, [7])
        self.assertEqual(self.state.scan_indices, [2])
        self.assertEqual(self.state.next_side, "camera")
        for i in range(5):
            self.state.select("camera", i)
            self.state.select("scan", [0, 1, 3, 4, 5][i])
        source, target, source_indices, target_indices = self.state.result()
        np.testing.assert_array_equal(source[0], self.scan[2])
        np.testing.assert_array_equal(target[0], self.camera[7])
        self.assertEqual(source_indices, [2, 0, 1, 3, 4, 5])
        self.assertEqual(target_indices, [7, 0, 1, 2, 3, 4])
        self.scan[:] = 999
        self.assertNotEqual(self.state.scan_points[0, 0], 999)

    def test_out_of_order_duplicate_and_invalid_indices_do_not_change_state(self):
        with self.assertRaises(ValueError):
            self.state.select("scan", 0)
        self.state.select("camera", 0)
        with self.assertRaises(ValueError):
            self.state.select("camera", 1)
        with self.assertRaises(ValueError):
            self.state.select("scan", 99)
        self.state.select("scan", 0)
        with self.assertRaises(ValueError):
            self.state.select("camera", 0)
        for index in (-1, 100, .5, True):
            with self.assertRaises(ValueError):
                self.state.select("camera", index)
        self.assertEqual(self.state.pair_count, 1)

    def test_undo_pending_then_whole_pair_and_reset(self):
        self.state.select("camera", 0); self.state.select("scan", 1)
        self.state.select("camera", 2)
        self.state.undo()
        self.assertEqual(self.state.pair_count, 1)
        self.assertIsNone(self.state.pending_camera)
        self.state.undo()
        self.assertEqual(self.state.pair_count, 0)
        self.assertEqual(self.state.scan_indices, [])
        self.state.undo()
        self.state.select("camera", 3)
        self.state.clear()
        self.assertEqual(self.state.next_side, "camera")

    def test_finish_requires_six_complete_pairs_and_no_pending_point(self):
        with self.assertRaises(ValueError):
            self.state.result()
        for i in range(6):
            self.state.select("camera", i); self.state.select("scan", i)
        self.assertTrue(self.state.can_finish)
        self.state.select("camera", 7)
        self.assertFalse(self.state.can_finish)
        with self.assertRaises(ValueError):
            self.state.result()
        self.state.undo()
        self.assertTrue(self.state.can_finish)

    def test_invalid_clouds_and_background_depth(self):
        for invalid in (np.empty((0, 3)), np.zeros((3, 2)), np.full((8, 3), np.nan)):
            with self.assertRaises(ValueError):
                PairSelection(invalid, self.camera)
        image = np.ones((10, 12), float)
        self.assertIsNone(nearest_depth_pixel(image, 4, 4))
        image[4, 4] = np.nan; image[4, 5] = .5; image[4, 7] = .4
        self.assertEqual(nearest_depth_pixel(image, 4, 4), (5, 4, .5))
        self.assertIsNone(nearest_depth_pixel(image, -1, 4))
        self.assertIsNone(nearest_depth_pixel(image, 12, 4))
        self.assertIsNone(nearest_depth_pixel(image, 0, 0, radius=1))

    def test_registration_entry_preserves_scan_camera_return_contract(self):
        import perception.registration as registration
        scan = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.scan))
        camera = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.camera))
        expected = (self.scan[:6], self.camera[:6], list(range(6)), list(range(6)))
        with patch("perception.registration.landmark_picker.pick_pairs", return_value=expected) as picker:
            result = registration.pick_landmark_pairs(scan, camera, 6)
        picker.assert_called_once_with(scan, camera, 6)
        np.testing.assert_array_equal(result[0], expected[0])
        np.testing.assert_array_equal(result[1], expected[1])

    def test_number_overlay_projects_with_nonzero_viewport_origin(self):
        import perception.registration.landmark_selection as selection
        self.assertTrue(hasattr(selection, "project_label_pixel"), "Number overlay projection is missing")
        project = selection.project_label_pixel
        self.assertEqual(project([0, 0, 0], np.eye(4), np.eye(4), (700, 90, 600, 500)),
                         (1000, 340))
        self.assertEqual(project([.5, -.5, 0], np.eye(4), np.eye(4), (700, 90, 600, 500)),
                         (1150, 465))
        self.assertIsNone(project([2, 0, 0], np.eye(4), np.eye(4), (700, 90, 600, 500)))

    def test_minimized_layout_keeps_last_valid_viewports(self):
        from types import SimpleNamespace
        from perception.registration.landmark_picker import PairedLandmarkWindow
        picker = PairedLandmarkWindow.__new__(PairedLandmarkWindow)
        picker.window = SimpleNamespace(content_rect=SimpleNamespace(width=0, height=0))
        error = None
        try:
            picker._on_layout(None)
        except Exception as exc:
            error = exc
        self.assertIsNone(error, "A minimized window must not resize scene cameras to 1x1")


if __name__ == "__main__":
    unittest.main()
