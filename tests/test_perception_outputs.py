"""Coordinate-chain, provenance and public-entrypoint regression tests."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from perception.artifacts import export_coordinates, load_extrinsics
from perception.frames import validate_transform


class PerceptionOutputTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.scan_points_mm = np.array([[1., 2., 3.], [4., 6., 9.]])
        self.line = self.root / 'line.csv'
        np.savetxt(self.line, self.scan_points_mm, delimiter=',', header='x,y,z', comments='')
        self.camera_scan_mm = np.array([[0., -1., 0., 30.], [1., 0., 0., -20.],
                                        [0., 0., 1., 100.], [0., 0., 0., 1.]])
        self.base_camera = np.array([[1., 0., 0., .7], [0., 0., -1., -.2],
                                     [0., 1., 0., .4], [0., 0., 0., 1.]])

    def extrinsics(self, status='measured', **overrides):
        payload = dict(T_base_camera=self.base_camera.tolist(), translation_unit='m', status=status)
        payload.update(overrides)
        path = self.root / 'extrinsics.json'
        path.write_text(json.dumps(payload))
        return path

    def test_noncommuting_chain_units_points_and_original_bytes(self):
        path = self.extrinsics()
        external = load_extrinsics(path)
        # Changing the source after load must not change the archived snapshot.
        original = path.read_bytes()
        path.write_text('{}')
        out = self.root / 'run'
        manifest = export_coordinates(out, self.camera_scan_mm, line=self.line, extrinsics=external)
        camera_points_m = (self.scan_points_mm @ self.camera_scan_mm[:3, :3].T
                           + self.camera_scan_mm[:3, 3]) / 1000.
        expected = camera_points_m @ self.base_camera[:3, :3].T + self.base_camera[:3, 3]
        actual = np.loadtxt(out / 'paths/line_base_m.csv', delimiter=',')
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        camera_scan_m = self.camera_scan_mm.copy()
        camera_scan_m[:3, 3] /= 1000.
        np.testing.assert_allclose(np.load(out / 'transforms/T_base_scan_m.npy'),
                                   self.base_camera @ camera_scan_m)
        self.assertEqual((out / 'transforms/extrinsics_used.json').read_bytes(), original)
        self.assertEqual(manifest['external_transform']['sha256'], external['sha256'])
        self.assertFalse(manifest['placeholder'])
        self.assertIn('line_source', manifest)

    def test_placeholder_names_headers_and_no_measured_products(self):
        external = load_extrinsics(self.extrinsics('placeholder'))
        out = self.root / 'placeholder'
        manifest = export_coordinates(out, self.camera_scan_mm, extrinsics=external, line=self.line)
        self.assertTrue(manifest['placeholder'])
        self.assertTrue((out / 'transforms/T_base_scan_placeholder_m.npy').exists())
        self.assertIn('status=placeholder', (out / 'paths/line_base_placeholder_m.csv').read_text())
        self.assertFalse((out / 'paths/line_base_m.csv').exists())
        self.assertFalse(manifest['robot_execution_allowed'])

    def test_camera_only_and_no_overwrite(self):
        out = self.root / 'camera_only'
        manifest = export_coordinates(out, self.camera_scan_mm, line=self.line)
        self.assertFalse(manifest['base_coordinates_available'])
        self.assertIsNone(manifest['target_frame'])
        self.assertFalse(any('base' in name for name in manifest['files']))
        with self.assertRaises(FileExistsError):
            export_coordinates(out, self.camera_scan_mm, line=self.line)

    def test_invalid_extrinsics_rejected(self):
        invalid = [dict(translation_unit='mm'), dict(status='unknown'),
                   dict(T_base_camera=np.eye(3).tolist()),
                   dict(T_base_camera=np.diag([2, 1, 1, 1]).tolist()),
                   dict(T_base_camera=np.diag([-1, 1, 1, 1]).tolist()),
                   dict(T_base_camera=np.full((4,4), float('nan')).tolist()),
                   dict(T_base_camera=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,1,1]])]
        for payload in invalid:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                load_extrinsics(self.extrinsics(**payload))

    def test_simulation_world_chain(self):
        out = self.root / 'world'
        manifest = export_coordinates(out, self.camera_scan_mm, line=self.line,
                                      world_from_camera_m=self.base_camera)
        self.assertEqual(manifest['target_frame'], 'world')
        self.assertEqual(manifest['external_transform_status'], 'simulation')
        self.assertFalse(manifest['base_coordinates_available'])
        matrix = np.load(out / 'transforms/T_world_scan_m.npy')
        validate_transform(matrix)
        self.assertTrue((out / 'paths/line_world_m.csv').exists())

    def test_simulation_entry_preserves_workflow_regions(self):
        from perception.simulation import run_simulation
        def workflow(model, data, jaw, scan, config, out, **kwargs):
            self.assertEqual(kwargs['registration_method'], 'automatic')
            capture, registration = out / 'capture', out / 'registration'
            capture.mkdir(); registration.mkdir()
            (capture / 'capture_metadata.json').write_text(json.dumps(
                dict(T_world_camera_cv_m=self.base_camera.tolist())))
            np.save(registration / 'T_camera_scan_estimated.npy', self.camera_scan_mm)
            (out / 'registration_config.json').write_text(json.dumps({jaw + '_line': str(self.line)}))
            return SimpleNamespace(capture_dir=capture, registration_dir=registration,
                                   regions_manifest=out / 'regions/perception_regions.json')
        with patch('pipeline.simulation.create_simulation', return_value=(None,None)), \
             patch('perception.workflow.perceive_gingival_line', side_effect=workflow):
            out = run_simulation(self.root / 'simulation')
        report = json.loads((out / 'run_report.json').read_text())
        self.assertEqual(report['status'], 'completed')
        self.assertIn('regions_manifest', report)
        self.assertTrue((out / 'transforms/T_world_scan_m.npy').exists())

    def test_real_primary_entry_forwards_to_shared_implementation(self):
        from perception.real import main
        with patch('perception.registration.manual_real.run_manual', return_value=0) as run:
            self.assertEqual(main(['manual', '--source', 'sample', '--extrinsics', 'example.json']), 0)
        args = run.call_args.args[0]
        self.assertEqual(args.extrinsics, Path('example.json'))
        self.assertEqual(args.source, 'sample')
