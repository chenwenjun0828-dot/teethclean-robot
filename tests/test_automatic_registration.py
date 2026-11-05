"""Geometry, output contracts, and pipeline checks for landmark-free registration."""
from pathlib import Path
import json
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np
import open3d as o3d

from perception.registration import AutomaticRegistrationConfig, register_clouds_automatic
from perception.registration.automatic import preprocess_for_fpfh
from perception.registration.main import main
from perception.registration.registration_math import transform_points
from perception.registration.registration_workflow import register_capture, resolve_registration_method
from pipeline.run import run_pipeline
from tests.test_global_registration import asymmetric_cloud


def synthetic_pair(outliers=False):
    scan = asymmetric_cloud()
    points = np.asarray(scan.points)
    truth = np.eye(4)
    truth[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz([.7, -.4, 1.2])
    truth[:3, 3] = [10., -7., 150.]
    local = transform_points(points[points[:, 2] > -5], truth)
    if outliers:
        local = np.vstack((local, np.random.default_rng(42).normal(size=(12, 3))*10 + 600))
    camera = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(local))
    camera.paint_uniform_color([.7, .4, .2])
    return scan, camera, truth


class AutomaticRegistrationTests(unittest.TestCase):
    def test_unknown_pose_partial_cloud_and_outliers_return_scan_to_camera(self):
        scan, camera, truth = synthetic_pair(outliers=True)
        original_scan = np.asarray(scan.points).copy()
        original_camera = np.asarray(camera.points).copy()
        original_colors = np.asarray(camera.colors).copy()
        estimate, metrics = register_clouds_automatic(scan, camera)
        np.testing.assert_allclose(estimate, truth, atol=.01)
        self.assertGreater(metrics['fitness'], .95)
        self.assertLess(metrics['inlier_rmse_mm'], .1)
        self.assertLess(metrics['preprocessing']['camera']['cleaned_points'], len(original_camera))
        self.assertEqual(metrics['optimization_direction'], 'camera_to_scan')
        self.assertEqual(metrics['fitness_source_frame'], 'camera_cv')
        initial = np.asarray(metrics['T_camera_scan_initial'])
        self.assertLess(np.linalg.norm(initial[:3, 3] - truth[:3, 3]), 2.)
        # Transform must place the complete scan in the observed camera frame.
        np.testing.assert_allclose(transform_points(original_scan, estimate),
                                   transform_points(original_scan, truth), atol=.02)
        np.testing.assert_array_equal(scan.points, original_scan)
        np.testing.assert_array_equal(camera.points, original_camera)
        np.testing.assert_array_equal(camera.colors, original_colors)
        self.assertFalse(scan.has_normals())
        self.assertFalse(camera.has_normals())
        json.dumps(metrics, allow_nan=False)

    def test_empty_nonfinite_degenerate_and_overdownsampled_clouds_fail(self):
        scan, camera, _ = synthetic_pair()
        invalid = [np.empty((0, 3)), np.full((300, 3), np.nan), np.zeros((300, 3)),
                   np.column_stack((np.arange(300), np.zeros((300, 2))))]
        for points in invalid:
            cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
            with self.subTest(shape=points.shape), self.assertRaises(ValueError):
                register_clouds_automatic(scan, cloud)
        with self.assertRaisesRegex(ValueError, 'too few points'):
            register_clouds_automatic(scan, camera, AutomaticRegistrationConfig(voxel_mm=1000.))

    def test_parameter_validation_and_voxel_relative_defaults(self):
        for values in ({'voxel_mm': 0}, {'voxel_mm': float('nan')},
                       {'voxel_mm': True}, {'outlier_neighbors': 1},
                       {'minimum_points': 2}, {'icp_iterations': 1.5},
                       {'ransac_confidence': 1.1}, {'edge_length_ratio': -1},
                       {'seed': -1}, {'seed': 2147483648}, {'refinement_voxel_mm': -1},
                       {'normal_radius_mm': 3., 'fpfh_radius_mm': 1.}, {'unknown_key': 2}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                AutomaticRegistrationConfig.from_mapping(values)
        with self.assertRaises(ValueError):
            AutomaticRegistrationConfig.from_mapping([])
        options = AutomaticRegistrationConfig(voxel_mm=1.)
        values = options.effective()
        self.assertEqual(values['normal_radius_mm'], 2.)
        self.assertEqual(values['fpfh_radius_mm'], 5.)
        self.assertEqual(values['ransac_distance_mm'], 1.5)
        self.assertEqual(values['refinement_voxel_mm'], .5)

    def test_ransac_or_icp_without_correspondences_is_rejected(self):
        scan, camera, _ = synthetic_pair()
        failed = o3d.pipelines.registration.RegistrationResult()
        with patch('open3d.pipelines.registration.registration_ransac_based_on_feature_matching',
                   return_value=failed), self.assertRaisesRegex(ValueError, 'RANSAC'):
            register_clouds_automatic(scan, camera)
        with patch('open3d.pipelines.registration.registration_icp', return_value=failed), \
                self.assertRaisesRegex(ValueError, 'ICP'):
            register_clouds_automatic(scan, camera)

    def test_empty_fpfh_features_report_invalid_radii(self):
        scan, _, _ = synthetic_pair()
        options = AutomaticRegistrationConfig(normal_radius_mm=.0001, fpfh_radius_mm=.0002)
        with self.assertRaisesRegex(ValueError, 'FPFH'):
            preprocess_for_fpfh(scan, options)

    def test_standalone_ply_pcd_cli_saves_scan_to_camera_and_refuses_overwrite(self):
        scan, camera, truth = synthetic_pair()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scan_path, camera_path = root / 'scan.ply', root / 'camera.pcd'
            self.assertTrue(o3d.io.write_point_cloud(str(scan_path), scan))
            self.assertTrue(o3d.io.write_point_cloud(str(camera_path), camera))
            settings = root / 'settings.json'
            settings.write_text(json.dumps({'voxel_mm': .8, 'seed': 0}), encoding='utf-8')
            out = root / 'result'
            argv = ['--scan', str(scan_path), '--camera', str(camera_path), '--out', str(out),
                    '--settings', str(settings), '--voxel-mm', '.5', '--no-view']
            with patch('open3d.visualization.draw_geometries') as viewer:
                self.assertEqual(main(argv), 0)
                viewer.assert_not_called()
            estimated = np.load(out / 'T_camera_scan_estimated.npy')
            np.testing.assert_allclose(estimated, truth, atol=.01)
            np.testing.assert_allclose(np.loadtxt(out / 'T_camera_scan_estimated.txt'), estimated, atol=1e-9)
            registered = o3d.io.read_point_cloud(str(out / 'scan_registered_camera_mm.ply'))
            np.testing.assert_allclose(registered.points, transform_points(np.asarray(scan.points), estimated))
            report = json.loads((out / 'registration_report.json').read_text(encoding='utf-8'))
            self.assertEqual(report['source_frame'], 'oral_scan')
            self.assertEqual(report['target_frame'], 'camera_cv')
            self.assertEqual(report['automatic_registration']['voxel_mm'], .5)
            self.assertEqual(report['landmark_pairs'], 0)
            self.assertGreater((out / 'registration_overlay.png').stat().st_size, 1000)
            self.assertFalse((out / 'landmark_correspondences.json').exists())
            saved = (out / 'T_camera_scan_estimated.npy').read_bytes()
            self.assertEqual(main(argv), 1)
            self.assertEqual((out / 'T_camera_scan_estimated.npy').read_bytes(), saved)

    def test_method_selection_and_incompatible_landmark_input(self):
        self.assertEqual(resolve_registration_method({}), 'manual')
        self.assertEqual(resolve_registration_method({'registration_method': 'automatic'}), 'automatic')
        self.assertEqual(resolve_registration_method({'registration_method': 'automatic'}, 'manual'), 'manual')
        with self.assertRaises(ValueError):
            resolve_registration_method({}, 'unknown')
        provider = Mock()
        with self.assertRaisesRegex(ValueError, 'landmark provider'):
            run_pipeline(registration_method='automatic', landmark_provider=provider)
        provider.assert_not_called()
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError, 'manual landmarks'):
                register_capture('missing_capture', None, 'mandibular', Path(tmp) / 'result',
                                 landmarks='unused.json', registration_method='automatic')
            self.assertFalse((Path(tmp) / 'result').exists())


class AutomaticPipelineTests(unittest.TestCase):
    def test_default_automatic_pipeline_completes_with_45_degree_orientation_filter(self):
        with tempfile.TemporaryDirectory() as tmp, \
                patch('perception.registration.registration_workflow.pick_landmark_pairs',
                      side_effect=AssertionError('automatic mode opened the picker')):
            root = Path(tmp)/'filtered'
            report = run_pipeline(output_dir=root, registration_method='automatic',
                                  headless=True, speed_mm_s=20.)
            self.assertEqual(report['status'], 'completed')
            tracking = json.loads((root/'tracking/tracking_report.json').read_text(encoding='utf-8'))
            selection = tracking['orientation_filter']
            self.assertEqual(selection['max_step_deg'], 45.)
            self.assertGreater(selection['skipped_orientation_count'], 0)
            tcp = np.load(root/'tracking/T_world_tcp.npy')
            tooth = np.load(root/'tracking/T_world_tooth.npy')
            raw = np.load(root/'tracking/requested_T_world_tcp.npy')
            kept = selection['retained_sample_indices']
            np.testing.assert_array_equal(tcp[:, :3, 3], raw[:, :3, 3])
            np.testing.assert_array_equal(tcp[kept, :3, :3], tooth[kept, :3, :3])
            self.assertEqual(len(tcp), selection['source_samples'])
            self.assertGreater(tracking['motion_ik_samples'], len(tcp))
            self.assertLess(tracking['trace_max_mm'], 2.)
            self.assertLess(tracking['trace_rotation_max_deg'], 1.)

    def test_automatic_pipeline_exports_targets_and_rejects_unreachable_tooth_pose(self):
        with tempfile.TemporaryDirectory() as tmp, \
                patch('perception.registration.registration_workflow.pick_landmark_pairs',
                      side_effect=AssertionError('automatic mode opened the picker')):
            root = Path(tmp) / 'run'
            with self.assertRaisesRegex(ValueError, 'IK target unreachable'):
                run_pipeline(output_dir=root, registration_method='automatic',
                             headless=True, speed_mm_s=20., max_orientation_step_deg=None)
            report = json.loads((root / 'pipeline_report.json').read_text(encoding='utf-8'))
            self.assertEqual(report['status'], 'failed')
            self.assertEqual(report['registration_method'], 'automatic')
            self.assertEqual(report['stages']['registration']['registration_method'], 'automatic')
            result = json.loads((root / 'registration/registration_report.json').read_text(encoding='utf-8'))
            self.assertEqual(result['landmark_origin'], 'not_used')
            self.assertEqual(result['landmark_pairs'], 0)
            self.assertEqual(result['transform_name'], 'T_camera_scan')
            self.assertGreater(result['fitness'], .8)
            self.assertFalse((root / 'registration/landmark_correspondences.json').exists())
            estimate = np.load(root / 'registration/T_camera_scan_estimated.npy')
            # This truth file is read only after registration, as an independent oracle.
            truth = np.load(root / 'capture/evaluation/T_camera_scan_true_mandibular.npy')
            self.assertLess(np.linalg.norm(estimate[:3, 3] - truth[:3, 3]), .5)
            cosine = np.clip((np.trace(estimate[:3, :3] @ truth[:3, :3].T) - 1)/2, -1, 1)
            self.assertLess(np.degrees(np.arccos(cosine)), .5)
            tcp = np.load(root / 'tracking/T_world_tcp.npy')
            tooth = np.load(root / 'tracking/T_world_tooth.npy')
            np.testing.assert_allclose(tcp, tooth, atol=1e-12)
            tracking = json.loads((root / 'tracking/tracking_report.json').read_text(encoding='utf-8'))
            self.assertEqual(tracking['simulated_time_s'], 0.)
            self.assertFalse((root / 'tracking/joint_waypoints_rad.csv').exists())
            # Exercise config-based method selection separately from the explicit override.
            second = register_capture(root / 'capture', root / 'registration_config.json',
                                      'mandibular', root / 'registration_from_config')
            config_result = json.loads((second / 'registration_report.json').read_text(encoding='utf-8'))
            self.assertEqual(config_result['registration_method'], 'automatic')

    def test_automatic_failure_stops_before_planning_and_control(self):
        with tempfile.TemporaryDirectory() as tmp, \
                patch('perception.registration.automatic.register_clouds_automatic',
                      side_effect=ValueError('RANSAC found no valid registration')), \
                patch('pipeline.run.run_tracking') as tracking:
            root = Path(tmp) / 'failed'
            with self.assertRaisesRegex(ValueError, 'RANSAC'):
                run_pipeline(output_dir=root, registration_method='automatic', headless=True)
            tracking.assert_not_called()
            report = json.loads((root / 'pipeline_report.json').read_text(encoding='utf-8'))
            self.assertEqual(report['status'], 'failed')
            self.assertEqual(report['stage'], 'registration')
            self.assertFalse((root / 'registration/T_camera_scan_estimated.npy').exists())


if __name__ == '__main__':
    unittest.main()
