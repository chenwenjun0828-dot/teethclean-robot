"""Preview geometry must agree with the actual planning and MuJoCo frames."""
import json
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import urlopen

import mujoco
import numpy as np

from .config import Config, SweepOptions
from .pose_search.constraints import target_at, tool_rotation
from .simple import orient_reference
from .visualization.pose_angles import PosePreview, create_server, tool_mesh_tcp
from .infrastructure.collision import mesh_from_geom


class PoseAnglePreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.preview = PosePreview(angles=(6.37, 13.26, -.4), jaw_opening_deg=20.)
        cls.server = create_server(cls.preview, 0)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.url = f'http://127.0.0.1:{cls.server.server_port}'

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()

    def test_both_modes_match_planner_targets_at_path_ends_and_midpoint(self):
        p = self.preview
        prepared = p.get_region('outer_front')
        for mode in ('simple', 'continuous'):
            cfg = Config(planning_mode=mode, tilt_deg=6.37, tilt_azimuth_deg=13.26, standoff_mm=1.8)
            reference = (orient_reference(prepared.reference, prepared.geometry, cfg,
                         SweepOptions(fixed_roll_deg=-.4), world=p.world) if mode == 'simple'
                         else prepared.reference)
            for station in (0., prepared.reference.end/2, prepared.reference.end):
                with self.subTest(mode=mode, station=station):
                    result = p.pose(region='outer_front', angles=(6.37, 13.26, -.4),
                                    standoff_mm=1.8, mode=mode, station_mm=station*1000)
                    np.testing.assert_allclose(result['world_from_tcp_m'],
                        target_at(reference, station, -.4, p.world, cfg), atol=1e-12)
                    self.assertFalse(result['execution_ready'])

    def test_display_coordinates_reconstruct_world_tcp(self):
        p = self.preview
        for region in ('outer_front', 'inner_front'):
            result = p.pose(region=region, angles=(35., -20., -141.), standoff_mm=0.)
            np.testing.assert_allclose(np.array(result['position_display_mm'])*.001+p.origin,
                                       np.array(result['world_from_tcp_m'])[:3, 3], atol=1e-12)

    def test_tool_mesh_rotates_about_actual_mujoco_tcp(self):
        p = self.preview
        tcp_vertices = np.asarray(tool_mesh_tcp(p.model, p.data).vertices)*.001
        data = mujoco.MjData(p.model)
        data.qpos[:] = p.data.qpos
        joint = p.model.joint('ur_wrist_3_joint').id
        data.qpos[p.model.jnt_qposadr[joint]] += .7
        mujoco.mj_forward(p.model, data)
        geom, site = p.model.geom('ur_end_scaler_geom').id, p.model.site('ur_scaler_tcp').id
        vertices, _ = mesh_from_geom(p.model, geom)
        expected = vertices @ data.geom_xmat[geom].reshape(3, 3).T+data.geom_xpos[geom]
        actual = tcp_vertices @ data.site_xmat[site].reshape(3, 3).T+data.site_xpos[site]
        np.testing.assert_allclose(actual, expected, atol=1e-10)

    def test_picked_point_preserves_angles_and_uses_local_surface(self):
        p = self.preview
        picked = [1., 2., 3., 0., 1., 0.]
        result = p.pose(region='outer_front', angles=(20., -30., 40.), standoff_mm=2., picked=picked)
        np.testing.assert_allclose(result['rotation_world'], tool_rotation(None, [0., 1., 0.], 20., 40., -30.))
        np.testing.assert_allclose(result['position_display_mm'], [1., 4., 3.], atol=1e-10)
        self.assertEqual(result['reference'], 'picked_surface')

    def test_invalid_inputs_fail_without_producing_a_pose(self):
        for values in ({'angles': (float('nan'), 0., 0.)}, {'angles': (181., 0., 0.)},
                       {'standoff_mm': -1.}, {'station_mm': -1.}, {'mode': 'retired'},
                       {'picked': [0., 0., 0., 0., 0., 0.]}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.preview.pose(**{'region': 'outer_front', 'angles': (0., 0., 0.),
                                     'standoff_mm': 1.8, **values})

    def test_local_server_serves_offline_modules_and_rejects_bad_pose(self):
        for path in ('/', '/app.js', '/vendor/three.module.js', '/vendor/OrbitControls.js'):
            with self.subTest(path=path), urlopen(self.url+path) as response:
                self.assertEqual(response.status, 200)
                self.assertGreater(len(response.read()), 0)
                if path.endswith('.js'):
                    self.assertIn('javascript', response.headers['Content-Type'])
        with urlopen(self.url+'/api/pose?angle=35&azimuth=-20&flip=12') as response:
            self.assertEqual(json.load(response)['pose_angles_deg'], [35., -20., 12.])
        with self.assertRaises(HTTPError) as caught:
            urlopen(self.url+'/api/pose?angle=nan')
        self.assertEqual(caught.exception.code, 400)


if __name__ == '__main__':
    unittest.main()
