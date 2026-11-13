"""Offline tests for saved PLY normalization and manual/camera workflow wiring."""
import json
import contextlib
import io
import sys
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import open3d as o3d

from perception.registration import manual_real as manual


class RealEntryTests(unittest.TestCase):
    def test_devices_and_manual_dispatch_without_legacy_calibration(self):
        from perception.real import main
        with patch.dict(sys.modules, {'perception.calibration': None}), \
             patch('perception.acquisition.realsense.list_devices', return_value=[]), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main(['devices']), 0)
            with patch.object(manual, 'run_manual', return_value=0) as run:
                self.assertEqual(main(['manual', '--source', 'camera']), 0)
                self.assertEqual(run.call_args.args[0].source, 'camera')

    def test_legacy_calibrate_reports_missing_module(self):
        from perception.real import main
        with tempfile.TemporaryDirectory() as tmp, \
             patch.dict(sys.modules, {'perception.calibration': None}), \
             contextlib.redirect_stderr(io.StringIO()) as errors:
            result = main(['calibrate', '--samples', 'unused.json',
                           '--out', str(Path(tmp) / 'calibration.json')])
            self.assertEqual(result, 1)
            self.assertIn('Legacy calibrate requires perception.calibration', errors.getvalue())
            self.assertFalse((Path(tmp) / 'calibration.json').exists())


class ManualRealTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.points = np.random.default_rng(2).uniform(-15, 15, (1500, 3))
        self.scan = self.save('scan.ply', self.points)
        self.offset = np.array([3., -2., 120.])
        self.camera = self.save('camera.ply', self.points + self.offset)

    def save(self, name, points):
        path = self.root / name
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(str(path), cloud)
        return path

    def pairs(self, *args, **kwargs):
        indices = [0, 10, 20, 30, 40, 50]
        return self.points[indices], (self.points + self.offset)[indices], indices, indices

    def test_metre_opengl_conversion_and_roi(self):
        raw = (self.points + self.offset) * [1, -1, -1] / 1000
        path = self.save('raw.ply', raw)
        cloud, conversion = manual.prepare_cloud(path, units='m', frame='opengl',
            roi=(-20, -20, 90, 20, 20, 150))
        np.testing.assert_allclose(cloud.points, self.points + self.offset)
        np.testing.assert_equal(np.diag(conversion), [1000, -1000, -1000, 1])
        with self.assertRaises(ValueError):
            manual.prepare_cloud(path, units='mm', frame='cv', roi=(1, 1, 1, 2, 2, 2))

    def test_manual_registration_direction_line_and_landmark_reuse(self):
        line = self.root / 'line.csv'
        np.savetxt(line, self.points[:8], delimiter=',', header='x,y,z', comments='')
        with patch.object(manual, 'pick_landmark_pairs', side_effect=self.pairs):
            out = manual.register_manual_clouds(self.camera, self.scan, self.root / 'run',
                units='mm', frame='cv', line=line)
        estimate = np.load(out / 'T_camera_scan_estimated.npy')
        np.testing.assert_allclose(estimate[:3, 3], self.offset, atol=1e-5)
        np.testing.assert_allclose(np.loadtxt(out / 'line_camera_mm.csv', delimiter=',', skiprows=1),
                                   self.points[:8] + self.offset, atol=1e-5)
        with patch.object(manual, 'pick_landmark_pairs') as picker:
            old_pairs = json.loads((out / 'landmarks.json').read_text())
            old_pairs['roi_sha256'] = old_pairs.pop('camera_sha256')
            (self.root / 'legacy.json').write_text(json.dumps(old_pairs))
            manual.register_manual_clouds(self.camera, self.scan, self.root / 'replay',
                units='mm', frame='cv', landmarks=self.root / 'legacy.json')
            picker.assert_not_called()
        pairs = json.loads((out / 'landmarks.json').read_text())
        pairs['scan_sha256'] = 'wrong'
        (self.root / 'bad.json').write_text(json.dumps(pairs))
        with self.assertRaisesRegex(ValueError, 'do not match'):
            manual.register_manual_clouds(self.camera, self.scan, self.root / 'bad',
                units='mm', frame='cv', landmarks=self.root / 'bad.json')

    def test_failed_icp_keeps_landmarks_but_no_final_transform(self):
        with patch.object(manual, 'pick_landmark_pairs', side_effect=self.pairs), \
             patch.object(manual, 'refine_manual', side_effect=ValueError('no matches')):
            with self.assertRaises(ValueError):
                manual.register_manual_clouds(self.camera, self.scan, self.root / 'failed',
                    units='mm', frame='cv')
        out = self.root / 'failed'
        self.assertTrue((out / 'landmarks.json').exists())
        self.assertFalse((out / 'T_camera_scan_estimated.npy').exists())
        self.assertEqual(json.loads((out / 'registration_report.json').read_text())['status'], 'failed')

    def test_legacy_picker_cli_runs_same_registration_and_exports(self):
        with patch('perception.registration.legacy_landmark_picker.pick_pairs_legacy',
                   side_effect=self.pairs) as picker:
            result = manual.main(['--source', 'ply', '--camera', str(self.camera),
                '--scan', str(self.scan), '--units', 'mm', '--frame', 'cv',
                '--picker', 'legacy', '--out', str(self.root / 'legacy_picker')])
        self.assertEqual(result, 0)
        picker.assert_called_once()
        out = self.root / 'legacy_picker'
        np.testing.assert_allclose(np.load(out / 'transforms/T_camera_scan_m.npy')[:3, 3],
                                   self.offset / 1000, atol=1e-8)
        report = json.loads((out / 'registration/registration_report.json').read_text())
        self.assertEqual(report['landmark_picker'], 'legacy')

    def test_external_matrix_reaches_run_report_and_preserves_old_command(self):
        line = self.root / 'line.csv'
        np.savetxt(line, self.points[:8], delimiter=',', header='x,y,z', comments='')
        matrix = np.eye(4)
        matrix[:3,3] = [.5, -.2, .1]
        extrinsics = self.root / 'external.json'
        extrinsics.write_text(json.dumps(dict(T_base_camera=matrix.tolist(),
            translation_unit='m', status='measured')))
        out = self.root / 'export_run'
        with patch.object(manual, 'pick_landmark_pairs', side_effect=self.pairs):
            code = manual.main(['--source','ply','--camera',str(self.camera),'--scan',str(self.scan),
                '--units','mm','--frame','cv','--line',str(line),'--extrinsics',str(extrinsics),
                '--out',str(out)])
        self.assertEqual(code, 0)
        report = json.loads((out / 'run_report.json').read_text())
        self.assertEqual(report['coordinate_outputs']['external_transform_status'], 'measured')
        np.testing.assert_allclose(np.loadtxt(out/'paths/line_base_m.csv', delimiter=','),
            (self.points[:8]+self.offset)/1000+matrix[:3,3], atol=1e-8)
        original_report = (out / 'run_report.json').read_bytes()
        self.assertEqual(manual.main(['--out',str(out),'--prepare-only']), 1)
        self.assertEqual((out / 'run_report.json').read_bytes(), original_report)

    def test_camera_source_calls_capture_before_registration(self):
        def capture(argv):
            out = Path(argv[argv.index('--out') + 1])
            out.mkdir()
            (out / 'camera_mm.ply').write_bytes(self.camera.read_bytes())
            return 0
        with patch('perception.real.main', side_effect=capture) as acquire, \
             patch.object(manual, 'pick_landmark_pairs', side_effect=self.pairs):
            result = manual.main(['--source', 'camera', '--scan', str(self.scan),
                                  '--out', str(self.root / 'capture_run')])
        self.assertEqual(result, 0)
        self.assertIn('--select-roi', acquire.call_args.args[0])

    def test_ply_chooser_and_cancelled_camera(self):
        with patch.object(manual, 'choose_ply', return_value=self.camera) as choose:
            result = manual.main(['--source', 'ply', '--units', 'mm', '--frame', 'cv',
                '--scan', str(self.scan), '--out', str(self.root / 'choose'), '--prepare-only'])
            choose.assert_called_once()
            self.assertEqual(result, 0)
        with patch('perception.real.main', return_value=1), \
             patch.object(manual, 'register_manual_clouds') as register:
            self.assertEqual(manual.main(['--source', 'camera', '--out', str(self.root / 'cancel')]), 1)
            register.assert_not_called()

    def test_saved_original_frame_can_be_recropped_for_maxillary_without_camera(self):
        import cv2
        from perception.acquisition.capture import RGBDFrame, save_real_capture
        from perception.real import load_real_config
        y, x = np.indices((30, 40))
        points = np.stack([x * .2, y * .2, np.full(x.shape, 100.)], axis=2)
        frame = RGBDFrame(np.zeros((30, 40, 3), np.uint8),
            np.full((30, 40), 1000, np.uint16), .0001, points,
            np.full(points.shape, .5), np.ones(x.shape, bool), {},
            {'host_received_at_utc': '2026-11-13T14:02:09Z', 'camera_serial': 'TEST-D405'})
        source = self.root / 'original_capture'
        original = save_real_capture(source, frame, 'mandibular', load_real_config())
        original_bytes = (source / 'capture_metadata.json').read_bytes()
        out = self.root / 'upper_recrop'
        with patch.object(cv2, 'selectROI', return_value=(2, 3, 20, 20)), \
             patch.object(cv2, 'destroyAllWindows'), \
             patch('perception.acquisition.realsense.RealSenseCamera', side_effect=AssertionError('No hardware needed')):
            code = manual.main(['--source', 'capture', '--capture', str(source),
                '--jaw', 'maxillary', '--scan', str(self.scan), '--out', str(out), '--prepare-only'])
        self.assertEqual(code, 0)
        metadata = json.loads((out / 'capture/capture_metadata.json').read_text())
        self.assertEqual(metadata['jaw'], 'maxillary')
        self.assertEqual(metadata['parent_capture_jaw'], 'mandibular')
        self.assertEqual(metadata['roi'], [2, 3, 22, 23])
        self.assertEqual(metadata['acquisition_id'], original['acquisition_id'])
        self.assertNotEqual(metadata['capture_id'], original['capture_id'])
        self.assertEqual(metadata['host_received_at_utc'], frame.metadata['host_received_at_utc'])
        self.assertEqual((source / 'capture_metadata.json').read_bytes(), original_bytes)
