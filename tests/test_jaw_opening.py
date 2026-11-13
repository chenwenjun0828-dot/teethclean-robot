"""Lower-jaw geometry, collision, and capture/pipeline integration checks."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from perception.frames import scan_to_camera_true_mm, camera_points_to_world_m
from pipeline.oral import (JawOpening, JawSettings, MouthOpening, MouthSettings,
                           load_jaw_settings as load_settings,
                           save_jaw_settings as save_settings, jaw_overrides as with_overrides)
from pipeline.simulation import create_simulation


class JawOpeningTests(unittest.TestCase):
    def test_settings_roundtrip_overrides_and_invalid_input(self):
        settings = JawSettings(opening_deg=15., hinge_center_mm=[1., 10., -40.])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'jaw.json'
            save_settings(settings, path)
            self.assertEqual(load_settings(path), settings)
            self.assertEqual(with_overrides(load_settings(path), 0.).opening_deg, 0.)
        for angle in (-11., 41., float('nan'), float('inf'), True, '15'):
            with self.subTest(angle=angle), self.assertRaises(ValueError):
                JawSettings(opening_deg=angle)
        for center in ([0., 1.], [0., 1., float('inf')], None, ['bad', 1, 2]):
            with self.subTest(center=center), self.assertRaises(ValueError):
                JawSettings(hinge_center_mm=center)

    def test_absolute_updates_preserve_upper_robot_and_restore_reference(self):
        model, data = create_simulation(jaw_settings=JawSettings())
        jaw = JawOpening(model)
        original_pos, original_quat = model.body_pos.copy(), model.body_quat.copy()
        upper_pos = data.body('dental_maxillary').xpos.copy()
        upper_mat = data.body('dental_maxillary').xmat.copy()
        qpos, qvel, ctrl = data.qpos.copy(), data.qvel.copy(), data.ctrl.copy()
        front = np.array([.006, .002, .026])
        for angle in (15., 40., -10., 15., 0.):
            settings = JawSettings(opening_deg=angle)
            jaw.update(settings, data)
            r = Rotation.from_euler('x', angle, degrees=True).as_matrix()
            hinge = np.array(settings.hinge_center_mm)/1000.
            local = r @ front + model.body_pos[jaw.body_id]
            np.testing.assert_allclose(local, hinge+r@(front-hinge), atol=1e-12)
            if angle > 0:
                self.assertLess(local[1], front[1])
            np.testing.assert_allclose(data.body('dental_maxillary').xpos, upper_pos, atol=1e-12)
            np.testing.assert_allclose(data.body('dental_maxillary').xmat, upper_mat, atol=1e-12)
            np.testing.assert_array_equal(data.qpos, qpos)
            np.testing.assert_array_equal(data.qvel, qvel)
            np.testing.assert_array_equal(data.ctrl, ctrl)
        np.testing.assert_allclose(model.body_pos, original_pos, atol=1e-16)
        np.testing.assert_allclose(model.body_quat, original_quat, atol=1e-16)

    def test_moved_static_jaw_collision_is_detected_at_new_position(self):
        model = mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0"/>
        <worldbody><body name="dental_teeth_root">
        <body name="dental_mandibular"><geom name="tooth" type="sphere"
         size=".003" pos="0 0 .06"/></body>
        <body name="dental_maxillary"><geom type="sphere" size=".003" pos="0 .1 0"/></body>
        </body><body name="probe"><freejoint/><geom name="probe_geom" type="sphere" size=".003"/>
        </body></worldbody></mujoco>''')
        data = mujoco.MjData(model)
        data.qpos[:3] = Rotation.from_euler('x', 30., degrees=True).apply([0., 0., .06])
        jaw = JawOpening(model)
        jaw.update(JawSettings(hinge_center_mm=(0., 0., 0.)), data)
        self.assertEqual(data.ncon, 0)
        jaw.update(JawSettings(opening_deg=30., hinge_center_mm=(0., 0., 0.)), data)
        self.assertGreater(data.ncon, 0)
        pair = {model.geom('tooth').id, model.geom('probe_geom').id}
        self.assertTrue(any(set(contact.geom) == pair for contact in data.contact))
        jaw.update(JawSettings(hinge_center_mm=(0., 0., 0.)), data)
        self.assertEqual(data.ncon, 0)

    def test_opened_jaw_camera_mapping_and_mask_distance_follow_pose(self):
        model, data = create_simulation(jaw_settings=JawSettings(),
                                         mouth_settings=MouthSettings(distance_mm=25.))
        jaw, mouth = JawOpening(model), MouthOpening(model)
        settings = MouthSettings(distance_mm=25.)
        for angle in (0., 25., -10.):
            jaw.update(JawSettings(opening_deg=angle), data)
            mouth.refresh_position(settings, data)
            for name in ('mandibular', 'maxillary'):
                points = np.array([[6., 2., 26.], [0., 10., -30.]])
                truth = scan_to_camera_true_mm(data, name, 'global_dental_camera')
                world = camera_points_to_world_m(points@truth[:3, :3].T+truth[:3, 3],
                                                  data, 'global_dental_camera')
                body = data.body('dental_'+name)
                expected = points/1000.@body.xmat.reshape(3, 3).T+body.xpos
                np.testing.assert_allclose(world, expected, atol=1e-12)
            # Independently transform the actual geom vertices into the root frame.
            front = []
            root = data.body('dental_teeth_root')
            for name in ('mandibular', 'maxillary'):
                gid = model.geom('dental_'+name+'_visual').id
                mesh = int(model.geom_dataid[gid])
                start, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
                vertices = model.mesh_vert[start:start+count].astype(float)
                world = vertices@data.geom_xmat[gid].reshape(3, 3).T+data.geom_xpos[gid]
                local = (world-root.xpos)@root.xmat.reshape(3, 3)
                front.append(local[:, 2].max()*1000.)
            mask_z = model.body_pos[mouth.body_id, 2]*1000.
            self.assertAlmostEqual(mask_z-max(front), 25., places=7)

    def test_pipeline_uses_opened_scene_for_capture_and_tracking_and_records_settings(self):
        from pipeline.run import run_pipeline
        seen = []

        def perceive(model, data, *args, **kwargs):
            seen.append((model, data))
            relative = data.body('dental_teeth_root').xmat.reshape(3, 3).T @ (
                data.body('dental_mandibular').xmat.reshape(3, 3))
            np.testing.assert_allclose(relative,
                                       Rotation.from_euler('x', 15., degrees=True).as_matrix(), atol=1e-10)
            return SimpleNamespace(registration_dir=Path('unused'), topology_path=Path('unused'))

        def track(*args, model_data=None, **kwargs):
            self.assertIs(model_data[0], seen[0][0])
            self.assertIs(model_data[1], seen[0][1])
            return {'status': 'completed'}

        with tempfile.TemporaryDirectory() as directory, \
                patch('pipeline.run.perceive_gingival_line', side_effect=perceive), \
                patch('pipeline.run.run_tracking', side_effect=track):
            report = run_pipeline(output_dir=directory, jaw_opening_deg=15.,
                                  registration_method='automatic', headless=True)
            saved = json.loads((Path(directory)/'pipeline_report.json').read_text())
            self.assertEqual(report['jaw_opening']['opening_deg'], 15.)
            self.assertEqual(saved['jaw_opening']['opening_deg'], 15.)

    def test_custom_scene_without_jaws_loads_unless_opening_explicitly_requested(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'scene.xml'
            path.write_text('<mujoco><worldbody><geom type="sphere" size=".01"/></worldbody></mujoco>')
            model, _ = create_simulation(path)
            self.assertEqual(model.ngeom, 1)
            with self.assertRaisesRegex(ValueError, 'requires mandibular and maxillary'):
                create_simulation(path, jaw_settings=JawSettings(opening_deg=15.))


if __name__ == '__main__':
    unittest.main()
