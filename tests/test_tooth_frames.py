"""Geometric contracts of tooth frames, without installed dependencies beyond the project."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from perception.gingival_line import RegisteredPaths
from planning.tooth_frames import (ToothFrames, _tooth_mask, define_tooth_frames,
    estimate_surface_normals, export_tooth_frames, registered_tooth_frames,
    tooth_frame_axes)
from planning.tooth_frames import _tooth_view_geometry, _tooth_view_camera, ToothSurfacePointSelector


class ToothFrameTests(unittest.TestCase):
    def test_vertex_selection_uses_original_mesh_ids_and_rejects_gum_and_invalid_ids(self):
        mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector([[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]]),
            o3d.utility.Vector3iVector([[0, 1, 2], [0, 2, 3]]))
        mask = np.array([False, True, True, True])
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'mesh.ply'
            o3d.io.write_triangle_mesh(str(source), mesh)
            with patch('planning.tooth_frames._tooth_mask', return_value=mask), \
                 patch('perception.gingival_extraction.outer_surface.filter_outer_segments',
                       return_value=([], {'normal_orientation_sign': 1.})):
                selector = ToothSurfacePointSelector(source, 'mandibular')
            frame, report = selector.frame(3)
            np.testing.assert_array_equal(frame.origins[0], np.asarray(mesh.vertices)[3])
            self.assertEqual(report['mesh_vertex_index'], 3)
            self.assertEqual(report['transform_name'], 'T_scan_tooth')
            np.testing.assert_allclose(frame.rotations[0, :, 2], [0, 0, 1])
            with self.assertRaisesRegex(ValueError, 'labelled gingiva'):
                selector.frame(0)
            for index in (-1, 4, 1.5, True, None):
                with self.assertRaisesRegex(ValueError, 'must be an integer'):
                    selector.frame(index)

    def test_view_preserves_scan_geometry_and_uses_an_orthogonal_camera(self):
        mesh = o3d.geometry.TriangleMesh.create_box()
        original = np.asarray(mesh.vertices).copy()
        frames = define_tooth_frames([[.5, .5, 1.], [.6, .5, 1.]],
                                    [[0, 0, 1], [0, 0, 1]], 'mandibular')
        geometry, endpoints = _tooth_view_geometry(mesh, frames, 0, 2.)
        np.testing.assert_allclose(endpoints, frames.origins[0] + 2. * .55 * 1.16 * frames.rotations[0].T)
        for i, (_, arrow, _) in enumerate(geometry[3:]):
            relative = np.asarray(arrow.vertices) - frames.origins[0]
            along_axis = relative @ frames.rotations[0, :, i]
            self.assertAlmostEqual(along_axis.min(), 0., places=10)
            self.assertAlmostEqual(along_axis.max(), 1.1, places=10)
        center, eye, up = _tooth_view_camera(frames, 0, 2.)
        self.assertAlmostEqual(np.dot(eye - center, up), 0., places=12)
        self.assertAlmostEqual(np.linalg.norm(up), 1., places=12)
        self.assertGreater(np.dot(eye-center, frames.rotations[0, :, 2]), 0.)
        np.testing.assert_array_equal(mesh.vertices, original)
        with self.assertRaises(ValueError):
            _tooth_view_geometry(mesh, frames, 10)

    def test_axes_follow_surface_and_jaw_polarity_with_original_origins(self):
        points = np.array([[7., 3., 2.], [1., 5., 9.]])
        normals = np.array([[1., .2, 0.], [0., .3, 1.]])
        lower = define_tooth_frames(points, normals, 'mandibular')
        upper = define_tooth_frames(points, normals, 'maxillary')
        np.testing.assert_array_equal(lower.origins, points)
        z = normals / np.linalg.norm(normals, axis=1)[:, None]
        np.testing.assert_allclose(lower.rotations[:, :, 2], z)
        np.testing.assert_allclose(lower.rotations[:, :, 0], -upper.rotations[:, :, 0])
        np.testing.assert_allclose(lower.rotations[:, :, 1], -upper.rotations[:, :, 1])
        self.assertTrue((lower.rotations[:, 1, 0] < 0).all())
        np.testing.assert_allclose(np.linalg.det(lower.rotations), 1.)
        np.testing.assert_allclose(lower.rotations.transpose(0, 2, 1) @ lower.rotations,
                                   np.tile(np.eye(3), (2, 1, 1)), atol=1e-15)
        points[:] = 0
        self.assertNotEqual(lower.origins[0, 0], 0)

    def test_rigid_equivariance_with_per_point_reference(self):
        normal = np.array([[1., .2, .1], [.1, .2, 1.]])
        reference = np.array([[0., -1., 0.], [0., -1., .1]])
        r = Rotation.from_euler('xyz', [32, 47, -20], degrees=True).as_matrix()
        np.testing.assert_allclose(tooth_frame_axes(normal @ r.T, reference @ r.T),
                                   r @ tooth_frame_axes(normal, reference), atol=1e-14)

    def test_bad_normals_parallel_reference_and_improper_rotation_rejected(self):
        for normal, direction in [([[0., 0., 0.]], [0, 1, 0]),
                                  ([[0., 1., 0.]], [0, -1, 0]),
                                  ([[0., 0., 1.]], [0, 0, 0]),
                                  ([[np.nan, 0., 1.]], [0, 1, 0])]:
            with self.assertRaises(ValueError):
                tooth_frame_axes(normal, direction)
        with self.assertRaises(ValueError):
            define_tooth_frames([[0, 0, 0]], [[1, 0, 0]], 'unknown')
        with self.assertRaises(ValueError):
            ToothFrames([[0, 0, 0]], [np.diag([1, 1, -1])])
        with self.assertRaises(ValueError):
            define_tooth_frames([[0, 0, 0], [1, 0, 0]], [[1, 0, 0]], 'mandibular')

    def test_scan_camera_world_conversions_rotate_axes_and_convert_translation_once(self):
        scan = define_tooth_frames([[10., 20., 30.], [11., 22., 33.]],
                                   [[0, 0, 1], [1, 0, 0]], 'mandibular')
        camera_from_scan = np.eye(4)
        camera_from_scan[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
        camera_from_scan[:3, 3] = [100, 200, 300]
        world_from_camera = np.eye(4)
        world_from_camera[:3, :3] = Rotation.from_euler('x', 25, degrees=True).as_matrix()
        world_from_camera[:3, 3] = [.2, -.1, .8]
        camera = scan.transformed(camera_from_scan, 'camera_cv', 'mm')
        world = camera.transformed(world_from_camera, 'world', 'm')
        scaled = camera_from_scan.copy()
        scaled[:3, 3] /= 1000
        direct = scan.transformed(world_from_camera @ scaled, 'world', 'm')
        np.testing.assert_allclose(world.transforms, direct.transforms, atol=1e-15)
        np.testing.assert_allclose(camera.origins[0], [80, 210, 330])
        np.testing.assert_array_equal(scan.origins[0], [10, 20, 30])
        np.testing.assert_allclose(world.rotations,
                                   world_from_camera[:3, :3] @ camera_from_scan[:3, :3] @ scan.rotations)
        # A local 1-mm displacement and the corresponding world 0.001-m
        # displacement must describe exactly the same physical point.
        camera_position = camera.transforms[0] @ [1, 0, 0, 1]
        world_position = world.transforms[0] @ [.001, 0, 0, 1]
        np.testing.assert_allclose(world_position[:3],
            world_from_camera[:3, :3] @ (camera_position[:3] / 1000) + world_from_camera[:3, 3])

    def test_closest_triangle_normals_exclude_gum_and_handle_reversed_winding(self):
        mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector([[0, 0, 0], [2, 0, 0], [0, 2, 0],
                                       [0, 0, .02], [0, 2, .02], [2, 0, .02]]),
            o3d.utility.Vector3iVector([[0, 1, 2], [3, 4, 5]]))
        query = np.array([[.4, .4, .015], [.7, .2, .01]])
        mask = np.array([True, True, True, False, False, False])
        normal, distances = estimate_surface_normals(mesh, query, mask, 1)
        np.testing.assert_allclose(normal, [[0, 0, 1], [0, 0, 1]])
        np.testing.assert_allclose(distances, [.015, .01], atol=1e-7)
        mesh.triangles = o3d.utility.Vector3iVector(np.asarray(mesh.triangles)[:, ::-1])
        reversed_normal, _ = estimate_surface_normals(mesh, query, mask, -1)
        np.testing.assert_allclose(reversed_normal, normal)
        with self.assertRaisesRegex(ValueError, 'too far'):
            estimate_surface_normals(mesh, [[.4, .4, 5]], mask, -1)
        with self.assertRaisesRegex(ValueError, 'No tooth'):
            estimate_surface_normals(mesh, query, np.zeros(6, bool), -1)

    def test_registration_adapter_and_exports_keep_original_indices(self):
        points = np.array([[4., 2., 1.], [9., 3., 7.]])
        scan = define_tooth_frames(points, [[1, 0, 0], [0, 0, 1]], 'mandibular')
        world_from_scan = np.eye(4)
        world_from_scan[:3, 3] = [.1, .2, .3]
        world_points = points / 1000 + [.1, .2, .3]
        paths = RegisteredPaths(world_points, [world_points], np.eye(4), world_from_scan,
                                points, 'mandibular', 'camera', Path('registration'), Path('topology.obj'))
        with patch('planning.tooth_frames.build_tooth_frames', return_value=(scan, np.zeros(2), {})):
            result = registered_tooth_frames(paths, Path('scan.ply'))
            np.testing.assert_allclose(result.world.origins, world_points)
            np.testing.assert_allclose(result.camera.origins, world_points * 1000)
            with tempfile.TemporaryDirectory() as tmp:
                export_tooth_frames(result, tmp, point_index=1)
                report = json.loads((Path(tmp) / 'tooth_frames_report.json').read_text())
                self.assertEqual(report['frames']['world']['units'], 'm')
                np.testing.assert_allclose(np.load(Path(tmp) / 'T_world_tooth.npy'), result.world.transforms)
                csv = np.genfromtxt(Path(tmp) / 'tooth_frames_scan.csv', delimiter=',', names=True)
                np.testing.assert_array_equal(csv['point_index'], [0, 1])
                np.testing.assert_array_equal(csv['Xx'], scan.rotations[:, 0, 0])
            paths.world_points += .01
            with self.assertRaisesRegex(ValueError, 'disagree'):
                registered_tooth_frames(paths, Path('scan.ply'))

    def test_wrong_label_geometry_is_rejected(self):
        mesh = o3d.geometry.TriangleMesh.create_box()
        labelled = o3d.geometry.TriangleMesh(mesh)
        labelled.paint_uniform_color([.95, .95, .90])
        labelled.translate([.01, 0, 0])
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'labels.ply'
            o3d.io.write_triangle_mesh(str(path), labelled)
            with self.assertRaisesRegex(ValueError, 'does not match'):
                _tooth_mask(mesh, path)


if __name__ == '__main__':
    unittest.main()
