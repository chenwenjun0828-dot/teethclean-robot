import unittest
from types import SimpleNamespace
import numpy as np
import mujoco
from perception.frames import (
    intrinsics_from_fovy, depth_to_points_mm, scan_to_camera_true_mm,
    camera_points_to_world_m, validate_transform,
)
from pipeline import simulation


class PerceptionFrameTests(unittest.TestCase):
    def test_depth_units_and_pixel_axes(self):
        points, colors = depth_to_points_mm(
            np.full((2, 2), 0.2), np.full((2, 2, 3), 255, dtype=np.uint8),
            {"fx": 100., "fy": 100., "cx": .5, "cy": .5},
            np.ones((2, 2), dtype=bool),
        )
        np.testing.assert_allclose(points, [[-1,-1,200],[1,-1,200],[-1,1,200],[1,1,200]])
        np.testing.assert_allclose(colors, 1.)

    def test_intrinsics_and_invalid_inputs(self):
        k = intrinsics_from_fovy(1280, 720, 30)
        self.assertAlmostEqual(k["fx"], 1343.5382907247958)
        self.assertEqual(k["cx"], 639.5)
        with self.assertRaises(ValueError):
            intrinsics_from_fovy(640, 0, 30)
        with self.assertRaises(ValueError):
            validate_transform(np.diag([2., 1., 1., 1.]))

    def test_empty_mask_and_invalid_depth(self):
        depth = np.array([[np.nan, 0.],[-1., .2]])
        points, _ = depth_to_points_mm(
            depth, np.zeros((2,2,3), np.uint8),
            intrinsics_from_fovy(2,2,30), np.zeros((2,2),bool),
        )
        self.assertEqual(points.shape, (0,3))
        with self.assertRaises(ValueError):
            depth_to_points_mm(depth, np.zeros((3,2,3)), intrinsics_from_fovy(2,2,30), np.ones((2,2)))

    def test_scan_camera_world_roundtrip_with_rotated_scene(self):
        model, data = simulation.create_simulation(simulation.DENTAL_MODEL_PATH)
        points = np.array([[32.,11.,-10.],[20.,-3.,4.]])
        for jaw in ("mandibular","maxillary"):
            truth = scan_to_camera_true_mm(data, jaw, "global_dental_camera")
            camera = points @ truth[:3,:3].T + truth[:3,3]
            world = camera_points_to_world_m(camera,data,"global_dental_camera")
            body = data.body("dental_"+jaw)
            expected = points / 1000 @ body.xmat.reshape(3,3).T + body.xpos
            np.testing.assert_allclose(world, expected, atol=1e-10)

    def test_actual_renderer_depth_plane_and_pixel_center(self):
        model = mujoco.MjModel.from_xml_string("""
        <mujoco><visual><global offwidth="160" offheight="120"/></visual>
        <worldbody><camera name="camera" fovy="30"/>
        <geom name="plane" type="box" pos="0 0 -0.205" size="1 1 .005"/>
        <geom name="marker" type="sphere" pos=".04 .025 -.18" size=".003"/>
        </worldbody></mujoco>""")
        data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        with mujoco.Renderer(model,120,160) as renderer:
            renderer.update_scene(data,camera="camera")
            renderer.enable_depth_rendering()
            depth=renderer.render().copy()
            renderer.enable_segmentation_rendering()
            seg=renderer.render()
        marker=seg[:,:,0]==model.geom("marker").id
        plane=seg[:,:,0]==model.geom("plane").id
        np.testing.assert_allclose(depth[plane],.2,atol=2e-5)
        y,x=np.nonzero(marker)
        k=intrinsics_from_fovy(160,120,30)
        self.assertLess(abs(x.mean()-(k["fx"]*.04/.18+k["cx"])),.75)
        self.assertLess(abs(y.mean()-(-k["fy"]*.025/.18+k["cy"])),.75)
