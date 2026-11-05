"""Cross-module checks for the colored-scan gingival pipeline."""
from pathlib import Path
import json
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from perception.frames import scan_to_camera_true_mm
from perception.gingival_extraction import extract_gingival_line
from perception.gingival_line import read_topology
from perception.registration.registration_math import transform_points
from pipeline.run import run_pipeline


ROOT = Path(__file__).resolve().parents[1]


class ModularPipelineTests(unittest.TestCase):
    def test_extraction_matches_existing_case_outputs_without_crosstooth(self):
        for jaw in ('mandibular', 'maxillary'):
            with self.subTest(jaw=jaw), tempfile.TemporaryDirectory() as tmp:
                scan = ROOT / 'models' / '2026615-gch' / f'2026615-gch病例-{jaw}.ply'
                result = extract_gingival_line(scan, Path(tmp), jaw, surface='all', cleanup=False)
                expected = ROOT / 'assets' / 'perception' / jaw / 'gingival_line_segments.obj'
                points, edges = read_topology(result['topology_obj'])
                reference, reference_edges = read_topology(expected)
                np.testing.assert_array_equal(points, reference)
                self.assertEqual(edges, reference_edges)
                self.assertEqual(len(result['points_mm']), len(points))

    def test_default_extraction_saves_filtered_outer_topology_for_both_jaws(self):
        for jaw in ('mandibular', 'maxillary'):
            with self.subTest(jaw=jaw), tempfile.TemporaryDirectory() as tmp:
                scan = ROOT / 'models' / '2026615-gch' / f'2026615-gch病例-{jaw}.ply'
                result = extract_gingival_line(scan, Path(tmp), jaw)
                points, edges = read_topology(result['topology_obj'])
                reference, _ = read_topology(ROOT / 'assets' / 'perception' / jaw / 'gingival_line_segments.obj')
                self.assertEqual(result['surface'], 'outer')
                self.assertGreater(len(points), len(reference)*.2)
                self.assertLess(len(points), len(reference)*.75)
                np.testing.assert_allclose(points, result['points_mm'], atol=1e-6)
                self.assertEqual(edges, result['edges'])
                report = json.loads((Path(tmp) / 'gingival_surface_filter.json').read_text())
                self.assertGreater(report['removed_segments'], 0)
                self.assertEqual(report['surface'], 'outer')
                cleanup = json.loads((Path(tmp) / 'gingival_cleanup.json').read_text())
                self.assertTrue(cleanup['enabled'])
                reasons = {region['reason'] for region in cleanup['removed_tooth_regions']}
                self.assertIn('scan_boundary_color_strip', reasons)
                self.assertIn('small_tooth_color_island', reasons)

    def test_failure_before_registration_never_starts_control(self):
        with tempfile.TemporaryDirectory() as tmp, \
                patch('perception.workflow.extract_gingival_line', side_effect=ValueError('no line')), \
                patch('pipeline.run.run_tracking') as control:
            with self.assertRaisesRegex(ValueError, 'no line'):
                run_pipeline(output_dir=Path(tmp) / 'run')
            control.assert_not_called()
            report = json.loads((Path(tmp) / 'run' / 'pipeline_report.json').read_text())
            self.assertEqual(report['status'], 'failed')
            self.assertEqual(report['stage'], 'extraction')

    def test_full_pipeline_exports_aligned_targets_before_unreachable_pose_stops_motion(self):
        # Simulation truth supplies test correspondences only. Production uses the picker.
        def fixture_landmarks(model, data, metadata, scan, output_dir):
            mesh = o3d.io.read_triangle_mesh(str(scan))
            vertices = np.asarray(mesh.vertices)
            source = vertices[np.random.default_rng(2026).choice(len(vertices), 10, replace=False)]
            target = transform_points(source, scan_to_camera_true_mm(
                data, 'mandibular', 'global_dental_camera'))
            path = output_dir / 'fixture_landmarks.json'
            path.write_text(json.dumps({
                'capture_id': metadata['capture_id'],
                'source_hashes': metadata['source_hashes'],
                'source_scan_mm': source.tolist(),
                'target_camera_mm': target.tolist(),
                'origin': 'simulation_test_fixture',
            }), encoding='utf-8')
            return path

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'run'
            # The existing mount supported a fixed tool orientation. Exact tooth alignment
            # now exposes unreachable poses; no target may be skipped or silently rotated.
            with self.assertRaisesRegex(ValueError, 'IK target unreachable'):
                run_pipeline(output_dir=root, headless=True,
                             speed_mm_s=20., landmark_provider=fixture_landmarks,
                             max_orientation_step_deg=None)
            report = json.loads((root / 'pipeline_report.json').read_text())
            self.assertEqual(report['status'], 'failed')
            self.assertFalse(report['dental_collision_validated'])
            poses = np.loadtxt(root / 'tracking' / 'target_poses_world_m_rad.csv',
                               delimiter=',', skiprows=1)
            self.assertEqual(poses.shape[1], 6)
            self.assertTrue(np.isfinite(poses).all())
            self.assertFalse((root / 'tracking' / 'joint_waypoints_rad.csv').exists())
            expected = np.asarray(json.loads((root / 'tracking' / 'tracking_report.json').read_text())
                                  ['orientation_world'])
            np.testing.assert_allclose(Rotation.from_euler('xyz', poses[:, 3:]).as_matrix(),
                                       expected, atol=1e-6)
            tooth = np.load(root / 'tracking' / 'T_world_tooth.npy')
            tcp = np.load(root / 'tracking' / 'T_world_tcp.npy')
            np.testing.assert_allclose(tcp, tooth, atol=1e-12)
            tracking = json.loads((root / 'tracking' / 'tracking_report.json').read_text())
            self.assertEqual(tracking['tool_pose'], 'tooth_aligned')
            self.assertFalse(tracking['fixed_orientation'])
            self.assertEqual(tracking['simulated_time_s'], 0.)
            self.assertEqual(len((root / 'tracking' / 'tcp_tracking.csv').read_text().splitlines()), 1)


if __name__ == '__main__':
    unittest.main()
