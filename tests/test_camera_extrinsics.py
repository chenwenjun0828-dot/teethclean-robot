"""Check SDK storage order, transform direction, axis mapping and exports."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from perception.acquisition.camera_extrinsics import build_report, save_report, read_camera_extrinsics


class CameraExtrinsicsTests(unittest.TestCase):
    def report(self):
        rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        translation = [.01, -.02, .03]
        return build_report(SimpleNamespace(rotation=rotation.flatten(order='F'),
                            translation=translation), dict(name='D405', serial='TEST'))

    def test_column_major_direction_inverse_and_ros_axes(self):
        report = self.report()
        transform = np.array(report['T_color_optical_depth_optical_m'])
        point = [.1, .2, .3, 1.]
        np.testing.assert_allclose(transform @ point, [-.19, .08, .33, 1.])
        inverse = np.array(report['T_depth_optical_color_optical_m'])
        np.testing.assert_allclose(inverse @ transform, np.eye(4), atol=1e-12)
        candidate = np.array(report['T_color_frame_depth_optical_ros_convention_m'])
        np.testing.assert_allclose(candidate @ point, [.33, .19, -.08, 1.])
        self.assertFalse(report['base_coordinates_available'])
        self.assertIn('not_verified', report['ros_frame_status'])

    def test_invalid_sdk_rotation_rejected(self):
        with self.assertRaises(ValueError):
            build_report(SimpleNamespace(rotation=[2.] * 9, translation=[0.] * 3), {})

    def test_read_checks_sdk_point_transform_and_releases_device(self):
        transform = np.array(self.report()['T_color_optical_depth_optical_m'])
        extrinsics = SimpleNamespace(rotation=transform[:3, :3].flatten(order='F'),
                                     translation=transform[:3, 3])
        camera = Mock()
        camera.device_info = {'name': 'D405', 'serial': 'TEST'}
        camera.depth_profile.get_extrinsics_to.return_value = extrinsics
        camera.rs.rs2_transform_point_to_point.side_effect = (
            lambda ext, point: transform[:3, :3] @ point + transform[:3, 3])
        with patch('perception.acquisition.camera_extrinsics.RealSenseCamera') as factory, \
             patch('perception.acquisition.camera_extrinsics._intrinsics', return_value={}):
            factory.return_value.__enter__.return_value = camera
            report = read_camera_extrinsics(serial='TEST')
            self.assertEqual(factory.call_args.args[0]['serial'], 'TEST')
            self.assertEqual(factory.call_args.args[0]['warmup_frames'], 0)
            factory.return_value.__exit__.assert_called_once()
        self.assertEqual(report['sdk_point_transform_check'], 'passed')

    def test_export_preserves_values_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'result'
            report = self.report()
            save_report(report, out)
            self.assertEqual(json.loads((out / 'camera_extrinsics.json').read_text()), report)
            key = 'T_color_frame_depth_optical_ros_convention_m'
            np.testing.assert_allclose(np.load(out / (key + '.npy')), report[key])
            np.testing.assert_allclose(np.loadtxt(out / (key + '.txt')), report[key])
            with self.assertRaises((ValueError, FileExistsError)):
                save_report(report, out)


if __name__ == '__main__':
    unittest.main()
