"""Regression for tool mount roll, downstream geometry and upstream invariants."""
import unittest

import mujoco
import numpy as np

from perception.session import DEFAULT_SESSION, load_session_robot_observation
from pipeline.simulation import body_transform, create_simulation
from planning.infrastructure.robot import Robot


class ToolMountTests(unittest.TestCase):
    def test_legacy_contact_fixture_compensation_preserves_tool_pose(self):
        model, data = create_simulation()
        robot = Robot(model, data)
        mount = model.body('ur_end_scaler_link').id
        tool = model.geom('ur_end_scaler_geom').id
        installed_quat = model.body_quat[mount].copy()
        for q in (robot.home.copy(), np.zeros(6),
                  np.array([-1.7250550766, -2.021367241, 2.117827707,
                            -2.323188962, -.4772823283, 1.545525781])):
            with self.subTest(joints=q.tolist()):
                model.body_quat[mount] = [0., 0., 0., 1.]
                old_tcp = robot.fk(q, full=True).copy()
                old_position = robot.verify_data.geom_xpos[tool].copy()
                old_rotation = robot.verify_data.geom_xmat[tool].copy()
                model.body_quat[mount] = installed_quat
                adapted = q.copy()
                adapted[5] -= np.pi
                np.testing.assert_allclose(robot.fk(adapted, full=True), old_tcp, atol=1e-12)
                np.testing.assert_allclose(robot.verify_data.geom_xpos[tool],
                                           old_position, atol=1e-12)
                np.testing.assert_allclose(robot.verify_data.geom_xmat[tool],
                                           old_rotation, atol=1e-12)

    def test_mount_roll_moves_tool_tcp_and_camera_together(self):
        model, data = create_simulation()
        mount = model.body('ur_end_scaler_link').id
        qadr = [model.jnt_qposadr[model.joint('ur_' + name).id] for name in
                load_session_robot_observation(DEFAULT_SESSION).joint_names]
        capture = load_session_robot_observation(DEFAULT_SESSION).joint_positions_rad
        installed_quat = model.body_quat[mount].copy()
        unchanged = ('ur_base', 'ur_wrist_3_link', 'ur_force_sensor_link',
                     'dental_mandibular', 'dental_maxillary')

        def downstream_poses():
            poses = {}
            for kind, name, positions, rotations in (
                    ('geom', 'ur_end_scaler_geom', data.geom_xpos, data.geom_xmat),
                    ('site', 'ur_scaler_tcp', data.site_xpos, data.site_xmat),
                    ('site', 'ur_scaler_force_site', data.site_xpos, data.site_xmat),
                    ('camera', 'ur_scaler_camera', data.cam_xpos, data.cam_xmat)):
                idx = getattr(model, kind)(name).id
                pose = np.eye(4)
                pose[:3, :3] = rotations[idx].reshape(3, 3)
                pose[:3, 3] = positions[idx]
                poses[name] = pose
            return poses

        for q in (data.qpos[qadr].copy(), capture, np.zeros(6)):
            with self.subTest(joints=q.tolist()):
                data.qpos[qadr] = q
                # Reconstruct the previous mounting in memory only.
                model.body_quat[mount] = [0., 0., 0., 1.]
                mujoco.mj_forward(model, data)
                before = downstream_poses()
                upstream = {name: body_transform(model, data, name) for name in unchanged}
                old_mount = body_transform(model, data, 'ur_end_scaler_link')
                model.body_quat[mount] = installed_quat
                mujoco.mj_forward(model, data)
                new_mount = body_transform(model, data, 'ur_end_scaler_link')
                sensor = upstream['ur_force_sensor_link']
                # The approved physical installation aligns the tool and sensor axes.
                np.testing.assert_allclose(sensor[:3, :3].T @ new_mount[:3, :3],
                                           np.eye(3), atol=1e-12)
                np.testing.assert_allclose(new_mount[:3, 3], old_mount[:3, 3], atol=1e-12)
                rotation = sensor[:3, :3] @ np.diag([-1., -1., 1.]) @ sensor[:3, :3].T
                for name, pose in downstream_poses().items():
                    expected_rotation = rotation @ before[name][:3, :3]
                    expected_position = (old_mount[:3, 3] +
                                         rotation @ (before[name][:3, 3] - old_mount[:3, 3]))
                    np.testing.assert_allclose(pose[:3, :3], expected_rotation, atol=1e-12)
                    np.testing.assert_allclose(pose[:3, 3], expected_position, atol=1e-12)
                for name in unchanged:
                    np.testing.assert_allclose(body_transform(model, data, name),
                                               upstream[name], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
