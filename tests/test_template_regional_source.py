"""Generated six-region candidates stay on actual mesh routes and retain provenance."""
from pathlib import Path
import unittest

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from perception.regional_paths import load_regional_paths, REGION_ORDER
from perception.regions.build_template_source import surface_path


class TemplateRegionalSourceTests(unittest.TestCase):
    def test_surface_route_erases_revisited_vertices_without_gap_connectors(self):
        vertices = np.column_stack((np.arange(7.), np.zeros(7), np.zeros(7)))
        edges = np.column_stack((np.arange(6), np.arange(1, 7)))
        points, ids, length = surface_path(vertices, edges, vertices[:, [0, 2]],
                                          np.zeros(7), [0, 4, 2, 6])
        np.testing.assert_array_equal(ids, np.arange(7))
        np.testing.assert_array_equal(points, vertices)
        self.assertEqual(length, 6.)

    def test_disconnected_surface_is_rejected(self):
        vertices = np.column_stack((np.arange(4.), np.zeros(4), np.zeros(4)))
        with self.assertRaisesRegex(ValueError, 'disconnected'):
            surface_path(vertices, np.array([[0, 1], [2, 3]]), vertices[:, [0, 2]],
                         np.zeros(4), [0, 3])

    def test_upper_source_matches_scan_and_remains_review_required(self):
        root = Path(__file__).resolve().parents[1]
        dataset = load_regional_paths(root / 'assets/regional_paths/maxillary')
        self.assertEqual(dataset.jaw, 'maxillary')
        self.assertFalse(dataset.metadata['region_boundaries_approved'])
        self.assertFalse(dataset.metadata['robot_execution_allowed'])
        self.assertEqual([c.region for c in dataset.chains], list(REGION_ORDER))
        mesh = o3d.io.read_triangle_mesh(str(dataset.mesh_path))
        tree = cKDTree(np.asarray(mesh.vertices))
        for chain in dataset.chains:
            with self.subTest(region=chain.region):
                self.assertGreaterEqual(len(chain.scan_points_mm), 4)
                self.assertLess(tree.query(chain.scan_points_mm)[0].max(), 1e-5)
                self.assertLess(np.linalg.norm(np.diff(chain.scan_points_mm, axis=0), axis=1).max(), 1.5)


if __name__ == '__main__':
    unittest.main()
