"""Focused checks for the active planning and control modules."""
import unittest

import mujoco
import numpy as np

from perception.gingival_line import ordered_chains, resample_path
from planning.ik import FixedPoseIK, rotation_error
from planning.trajectory import plan_tracking
from planning.validation import configure_collision_mode
from control.position import step_checked
from pipeline.simulation import create_simulation


class PlanningControlTests(unittest.TestCase):
    def test_disconnected_line_stays_as_separate_ordered_paths(self):
        points = np.array([[2, 0, 0], [0, 0, 0], [1, 0, 0],
                           [9, 0, 0], [10, 0, 0]], float)
        chains = ordered_chains(points, [(1, 2), (2, 0), (3, 4)])
        self.assertEqual(sorted(len(chain) for chain in chains), [2, 3])
        sampled = resample_path(chains[0] / 1000, 0.0002)
        np.testing.assert_allclose(sampled[[0, -1]], (chains[0] / 1000)[[0, -1]])

    def test_ik_solves_current_tcp_and_rejects_unreachable_path(self):
        model, data = create_simulation()
        ik = FixedPoseIK(model)
        position = data.site('ur_scaler_tcp').xpos.copy()
        rotation = data.site('ur_scaler_tcp').xmat.reshape(3, 3).copy()
        np.testing.assert_allclose(ik.solve(position, rotation, data.qpos), data.qpos, atol=1e-5)
        original = data.qpos.copy()
        with self.assertRaisesRegex(ValueError, 'IK|reachable'):
            plan_tracking(model, data, np.array([[10, 10, 10], [10.01, 10, 10]]), np.eye(3))
        np.testing.assert_array_equal(data.qpos, original)
        self.assertAlmostEqual(np.linalg.norm(rotation_error(
            np.diag([1., -1., -1.]), np.eye(3))), np.pi)

    def test_geometry_collision_mode_keeps_other_robot_geometries_enabled(self):
        model, _ = create_simulation()
        before = model.geom_contype.copy()
        configure_collision_mode(model, 'geometry')
        jaw_ids = [model.geom('dental_' + jaw + '_visual').id
                   for jaw in ('mandibular', 'maxillary')]
        other = np.setdiff1d(np.arange(model.ngeom), jaw_ids)
        np.testing.assert_array_equal(model.geom_contype[other], before[other])
        self.assertTrue(np.all(model.geom_contype[jaw_ids] == 0))

    def test_contact_inside_control_batch_is_detected(self):
        model = mujoco.MjModel.from_xml_string('''<mujoco>
          <option timestep="0.002" gravity="0 0 0"/>
          <worldbody>
            <geom name="obstacle" type="sphere" size="0.001" pos="0 0.0019 0" solref="0.1 1"/>
            <body name="ur_test" pos="-0.004 0 0">
              <joint type="slide" axis="1 0 0"/>
              <geom name="moving" type="sphere" size="0.001" mass="1" solref="0.1 1"/>
            </body>
          </worldbody></mujoco>''')
        data = mujoco.MjData(model)
        data.qvel[0] = 1
        mujoco.mj_forward(model, data)
        with self.assertRaisesRegex(RuntimeError, 'contact'):
            step_checked(model, data, 10)


if __name__ == '__main__':
    unittest.main()
