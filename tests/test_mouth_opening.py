"""Geometry and actual-renderer regressions for the adjustable aperture."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import mujoco
import numpy as np

from pipeline.oral import (MouthOpening, MouthSettings, ROOT,
                           load_mouth_settings as load_settings,
                           save_mouth_settings as save_settings, face_geometry,
                           mouth_overrides as with_overrides)
from pipeline.simulation import create_simulation


class MouthOpeningTests(unittest.TestCase):
    def test_settings_roundtrip_and_invalid_sizes(self):
        settings = MouthSettings(width_mm=60., height_mm=50., center_mm=[2., 12., 50.], distance_mm=20.)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'mouth.json'
            save_settings(settings, path)
            self.assertEqual(load_settings(path), settings)
        for field, value in (('width_mm', 0.), ('height_mm', float('nan')),
                             ('width_mm', 111.), ('height_mm', 131.),
                             ('distance_mm', -1.), ('distance_mm', float('inf')),
                             ('distance_mm', True),
                             ('center_mm', [0., float('inf'), 45.])):
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                replace(settings, **{field: value})

    def test_distance_is_measured_from_actual_frontmost_jaw_in_rotated_scene(self):
        settings = MouthSettings(width_mm=70., height_mm=45., distance_mm=20.)
        model, data = create_simulation(mouth_settings=settings)
        mouth = MouthOpening(model)
        root = data.body('dental_teeth_root')
        rotation = root.xmat.reshape(3, 3)
        actual_fronts = []
        for jaw in ('mandibular', 'maxillary'):
            geom_id = model.geom('dental_'+jaw+'_visual').id
            mesh_id = int(model.geom_dataid[geom_id])
            start, count = int(model.mesh_vertadr[mesh_id]), int(model.mesh_vertnum[mesh_id])
            vertices = model.mesh_vert[start:start+count].astype(float)
            world = vertices @ data.geom_xmat[geom_id].reshape(3, 3).T + data.geom_xpos[geom_id]
            actual_fronts.append(((world-root.xpos) @ rotation)[:, 2].max())
        front_m = max(actual_fronts)
        plane_m = ((data.body('dental_mouth_opening').xpos-root.xpos) @ rotation)[2]
        self.assertAlmostEqual((plane_m-front_m)*1000., 20., places=8)
        initial_plane = data.body('dental_mouth_opening').xpos.copy()
        initial_vertices = model.mesh_vert.copy()
        initial_jaws = [data.geom_xpos[model.geom('dental_'+jaw+'_visual').id].copy()
                        for jaw in ('mandibular', 'maxillary')]
        mouth.update(replace(settings, distance_mm=35.), data)
        np.testing.assert_allclose(data.body('dental_mouth_opening').xpos-initial_plane,
                                   rotation[:, 2]*.015, atol=1e-10)
        np.testing.assert_array_equal(model.mesh_vert, initial_vertices)
        for jaw, original in zip(('mandibular', 'maxillary'), initial_jaws):
            np.testing.assert_array_equal(data.geom_xpos[model.geom('dental_'+jaw+'_visual').id], original)

    def test_distance_overrides_and_legacy_absolute_position(self):
        settings = MouthSettings(distance_mm=20.)
        self.assertEqual(with_overrides(settings, distance_mm=30.).distance_mm, 30.)
        legacy = with_overrides(settings, forward_mm=50.)
        self.assertIsNone(legacy.distance_mm)
        self.assertEqual(legacy.center_mm[2], 50.)
        with self.assertRaises(ValueError):
            with_overrides(settings, distance_mm=20., forward_mm=50.)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'old.json'
            path.write_text('{"width_mm": 70, "height_mm": 45, "center_mm": [0, 10, 45]}',
                            encoding='utf-8')
            self.assertIsNone(load_settings(path).distance_mm)

    def test_runtime_updates_respect_compiler_frame_and_preserve_robot(self):
        model, data = create_simulation()
        mouth = MouthOpening(model)
        initial_qpos, initial_ctrl = data.qpos.copy(), data.ctrl.copy()
        settings = MouthSettings(width_mm=70., height_mm=40., center_mm=(4., 13., 55.))
        mouth.update(settings, data)
        root = data.body('dental_teeth_root')
        expected = root.xpos + root.xmat.reshape(3, 3) @ (np.array(settings.center_mm)/1000.)
        np.testing.assert_allclose(data.body('dental_mouth_opening').xpos, expected, atol=1e-10)
        start = mouth.vertex_start
        vertices = model.mesh_vert[start:start+mouth.vertex_count]
        restored = vertices @ mouth.mesh_rotation.T + mouth.mesh_position
        np.testing.assert_allclose(restored, face_geometry(settings)[0], atol=1e-8)
        np.testing.assert_array_equal(data.qpos, initial_qpos)
        np.testing.assert_array_equal(data.ctrl, initial_ctrl)
        ids = [mouth.face_id, *mouth.rim_ids]
        self.assertTrue(np.all(model.geom_contype[ids] == 0))
        self.assertTrue(np.all(model.geom_conaffinity[ids] == 0))
        mouth.update(replace(settings, enabled=False), data)
        self.assertTrue(np.all(model.geom_rgba[ids, 3] == 0.))

    def test_actual_segmentation_sees_hole_and_independent_width_height_changes(self):
        # A coloured target behind the mask makes filled holes and stale GPU
        # geometry observable, independently of the dental scan's own gaps.
        assets = (ROOT/'assets'/'simulation'/'mouth_opening_assets.xml').as_posix()
        body = (ROOT/'assets'/'simulation'/'mouth_opening_body.xml').as_posix()
        xml = f'''<mujoco><visual><global offwidth="320" offheight="320"/></visual>
        <asset><include file="{assets}"/></asset><worldbody>
        <camera name="front" pos="0 .01 .25" fovy="45"/>
        <geom name="target" type="box" pos="0 .01 0" size=".055 .06 .001" rgba="1 0 0 1"/>
        <include file="{body}"/></worldbody></mujoco>'''
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mouth = MouthOpening(model, prefix='')
        target_id = model.geom('target').id
        masks = []
        for settings in (MouthSettings(), MouthSettings(width_mm=75.),
                          MouthSettings(height_mm=40.)):
            mouth.update(settings, data)
            # A new renderer uploads the updated mesh, as a new pipeline capture does.
            with mujoco.Renderer(model, 320, 320) as renderer:
                renderer.update_scene(data, camera='front')
                renderer.enable_segmentation_rendering()
                segmentation = renderer.render()
            mask = segmentation[:, :, 0] == target_id
            self.assertTrue(mask[160, 160], 'The aperture centre must remain genuinely empty')
            self.assertFalse(mask[160, 238], 'The surrounding face must occlude the target')
            y, x = np.nonzero(mask)
            masks.append((int(x.max()-x.min()), int(y.max()-y.min()), int(mask.sum())))
        self.assertGreater(masks[1][0], masks[0][0]*1.5)
        self.assertAlmostEqual(masks[1][1], masks[0][1], delta=2)
        self.assertAlmostEqual(masks[2][0], masks[0][0], delta=2)
        self.assertLess(masks[2][1], masks[0][1]*.6)


if __name__ == '__main__':
    unittest.main()
