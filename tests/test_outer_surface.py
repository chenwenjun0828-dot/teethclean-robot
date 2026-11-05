"""Known inside/outside geometry, independent of a scan's coordinate axes."""
import unittest

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from perception.gingival_extraction.outer_surface import filter_outer_segments


def dental_arch_fixture():
    # An open torus models a U-shaped arch with known buccal and lingual sides.
    theta = np.linspace(-2.0, 2.0, 81)
    phi = np.linspace(0, 2*np.pi, 24, endpoint=False)
    radial = 25 + 3*np.cos(phi)
    vertices = np.stack(np.broadcast_arrays(
        radial[None, :] * np.cos(theta[:, None]),
        radial[None, :] * np.sin(theta[:, None]),
        4*np.sin(phi)[None, :]), axis=-1).reshape(-1, 3)
    faces = []
    for i in range(len(theta)-1):
        for j in range(len(phi)):
            a, b = i*24+j, (i+1)*24+j
            c, d = i*24+(j+1)%24, (i+1)*24+(j+1)%24
            faces.extend(((a, b, c), (b, d, c)))
    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(faces))
    outer = [(vertices[i*24], vertices[(i+1)*24]) for i in range(10, 70)]
    inner = [(vertices[i*24+12], vertices[(i+1)*24+12]) for i in range(10, 70)]
    return mesh, np.zeros(len(vertices)), outer, inner


class OuterSurfaceTests(unittest.TestCase):
    def test_keeps_outer_removes_inner_and_rejects_bridging_edges(self):
        mesh, scores, outer, inner = dental_arch_fixture()
        crossing = [(outer[20][0], inner[20][1])]
        result, report = filter_outer_segments(mesh, scores, .5, outer+inner+crossing)
        np.testing.assert_allclose(result, outer)
        self.assertEqual(report['removed_segments'], len(inner)+1)

    def test_rotation_translation_and_reversed_winding_preserve_selection(self):
        mesh, scores, outer, inner = dental_arch_fixture()
        rotation = Rotation.from_euler('xyz', [.7, -.2, 1.1]).as_matrix()
        shift = np.array([80., -22., 143.])
        mesh.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices) @ rotation.T + shift)
        mesh.triangles = o3d.utility.Vector3iVector(np.asarray(mesh.triangles)[:, ::-1].copy())
        transformed_outer = [(a @ rotation.T+shift, b @ rotation.T+shift) for a, b in outer]
        transformed_inner = [(a @ rotation.T+shift, b @ rotation.T+shift) for a, b in inner]
        result, report = filter_outer_segments(mesh, scores, .5, transformed_outer+transformed_inner)
        np.testing.assert_allclose(result, transformed_outer)
        self.assertEqual(report['normal_orientation_sign'], -1.0)

    def test_ambiguous_geometry_fails_instead_of_returning_all_surfaces(self):
        mesh = o3d.geometry.TriangleMesh.create_sphere()
        with self.assertRaisesRegex(ValueError, 'occlusal plane'):
            filter_outer_segments(mesh, np.zeros(len(mesh.vertices)), .5, [])


if __name__ == '__main__':
    unittest.main()
