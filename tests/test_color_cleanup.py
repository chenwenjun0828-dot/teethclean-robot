"""Separate a supported crown from long scan-rim artifacts and color specks."""
import unittest

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from perception.gingival_extraction.cleanup import clean_color_regions


def color_region_fixture():
    x, y = np.meshgrid(np.linspace(-10, 10, 41), np.linspace(-10, 10, 41))
    vertices = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    faces = []
    for row in range(40):
        for col in range(40):
            a = row*41+col
            faces.extend(((a, a+1, a+41), (a+1, a+42, a+41)))
    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(faces))
    # A crown has a broad interior but also touches the left scan boundary.
    crown = (vertices[:, 0] <= -1) & (np.abs(vertices[:, 1]) <= 4)
    # This long white strip has substantial area but hugs the opposite boundary.
    rim = vertices[:, 0] >= 9
    scores = np.ones(len(vertices))
    scores[crown | rim] = 0
    tooth_speck = np.argmin(np.linalg.norm(vertices-[2, 0, 0], axis=1))
    gum_speck = np.argmin(np.linalg.norm(vertices-[-5, 0, 0], axis=1))
    scores[tooth_speck] = 0
    scores[gum_speck] = 1
    return mesh, scores, crown


class ColorCleanupTests(unittest.TestCase):
    def test_removes_long_rim_and_specks_preserves_crown_touching_boundary(self):
        mesh, scores, crown = color_region_fixture()
        original = scores.copy()
        cleaned, report = clean_color_regions(mesh, scores, .5)
        np.testing.assert_array_equal(cleaned < .5, crown)
        np.testing.assert_array_equal(scores, original)
        self.assertEqual(report['filled_small_gum_islands'], 1)
        self.assertEqual(report['removed_tooth_components'], 2)
        self.assertEqual({region['reason'] for region in report['removed_tooth_regions']},
                         {'small_tooth_color_island', 'scan_boundary_color_strip'})

    def test_rotated_translated_mesh_has_same_cleanup(self):
        mesh, scores, crown = color_region_fixture()
        rotation = Rotation.from_euler('xyz', [.6, -.4, 1.2]).as_matrix()
        mesh.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices) @ rotation.T + [100, -80, 30])
        cleaned, _ = clean_color_regions(mesh, scores, .5)
        np.testing.assert_array_equal(cleaned < .5, crown)

    def test_closed_crown_without_open_boundary_is_preserved(self):
        mesh = o3d.geometry.TriangleMesh.create_sphere(radius=3)
        cleaned, report = clean_color_regions(mesh, np.zeros(len(mesh.vertices)), .5)
        self.assertTrue(np.all(cleaned < .5))
        self.assertEqual(report['removed_tooth_components'], 0)


if __name__ == '__main__':
    unittest.main()
