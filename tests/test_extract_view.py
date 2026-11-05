"""The extraction CLI viewer is optional and does not affect the API."""
import sys
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import open3d as o3d

from perception.gingival_extraction import extract as extraction


class ExtractionViewTests(unittest.TestCase):
    def test_overlay_contains_colored_mesh_and_connected_line(self):
        mesh = o3d.geometry.TriangleMesh.create_box()
        mesh.paint_uniform_color([0.8, 0.5, 0.4])
        points = np.array([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.]])
        edges = [(0, 1), (1, 2)]
        viewer = MagicMock()
        viewer.create_window.return_value = True

        with patch.object(o3d.visualization, 'Visualizer', return_value=viewer):
            extraction.show_3d_overlay(mesh, points, edges)

        geometries = [call.args[0] for call in viewer.add_geometry.call_args_list]
        self.assertIs(geometries[0], mesh)
        self.assertIsInstance(geometries[1], o3d.geometry.LineSet)
        self.assertIsInstance(geometries[2], o3d.geometry.PointCloud)
        np.testing.assert_array_equal(np.asarray(geometries[1].lines), edges)
        np.testing.assert_array_equal(np.asarray(geometries[1].points), points)
        viewer.run.assert_called_once()
        viewer.destroy_window.assert_called_once()

    def test_cli_opens_view_by_default_and_no_view_skips_it(self):
        result = {'points_mm': np.zeros((2, 3)), 'edges': [(0, 1)]}
        mesh = o3d.geometry.TriangleMesh.create_box()
        for extra_args, expected_calls, surface, cleanup in (
                ([], 1, 'outer', True), (['--no-view'], 0, 'outer', True),
                (['--no-view', '--surface', 'all'], 0, 'all', True),
                (['--no-view', '--no-cleanup'], 0, 'outer', False)):
            with self.subTest(extra_args=extra_args), \
                    patch.object(sys, 'argv', ['extract.py', '--input', 'scan.ply', *extra_args]), \
                    patch.object(extraction, 'extract_gingival_line', return_value=result) as extract, \
                    patch.object(extraction, 'read_mesh', return_value=mesh), \
                    patch.object(extraction, 'show_3d_overlay') as show:
                extraction.main()
                self.assertEqual(show.call_count, expected_calls)
                self.assertEqual(extract.call_args.kwargs['surface'], surface)
                self.assertEqual(extract.call_args.kwargs['cleanup'], cleanup)


if __name__ == '__main__':
    unittest.main()
