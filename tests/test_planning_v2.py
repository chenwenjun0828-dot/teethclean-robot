"""Behavioral regression tests for the rebuilt geometry-to-control contract."""
import tempfile
import unittest
from pathlib import Path
from dataclasses import replace
import numpy as np
import fcl
from scipy.spatial.transform import Rotation
from planning.config import Config
from planning.infrastructure.io import validate_transform
from planning.pose_search.constraints import tool_rotation, normal_angle, pose_at
from perception.regions.geometry import fit_curve
from planning.infrastructure.robot import Robot
from planning.infrastructure.collision import bvh, CollisionChecker
from planning.infrastructure.trajectory import Trajectory
from planning.motion_planning.fixed_path import build
from planning.pose_search.continuous import whole_path_search, SweepFailure
from pipeline.simulation import create_simulation
from control.position import GingivalTCPController, step_checked


class GeometryTests(unittest.TestCase):
    def test_fixed_tilt_holds_for_oblique_tangent(self):
        tangent = np.array([1., .2, .3]); tangent /= np.linalg.norm(tangent)
        for roll in (-170., 0., 37., 90.):
            r = tool_rotation(tangent, [0, 1, 0], 35., roll)
            np.testing.assert_allclose(r.T@r, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(r), 1.)
            self.assertAlmostEqual(normal_angle(r, [0, 1, 0]), 35.)

    def test_incompatible_tangent_is_rejected_not_projected(self):
        with self.assertRaises(ValueError):
            tool_rotation([0, 0, 1], [0, 0, 1])

    def test_pose_is_rigid_rotation_equivariant(self):
        rotation = Rotation.from_euler('xyz', [20, -40, 12], degrees=True).as_matrix()
        original = tool_rotation([1, 0, 0], [0, 1, 0], 35., -141., -20.)
        transformed = tool_rotation(rotation[:, 0], rotation[:, 1], 35., -141., -20., gravity=rotation@[0, 0, -1])
        np.testing.assert_allclose(transformed, rotation@original, atol=1e-12)

    def test_cubic_endpoints_corridor_and_arc_sampling(self):
        t = np.linspace(-1, 1, 45)
        p = np.c_[t*.005, .0003*t*t, .0002*np.sin(t*4)]
        sampled, tangent, u, record = fit_curve(p, .00015, .00025)
        np.testing.assert_allclose(sampled[[0, -1]], p[[0, -1]], atol=1e-12)
        self.assertEqual(record['degree'], 3)
        self.assertLessEqual(record['max_fit_error_mm'], .15)
        np.testing.assert_allclose(np.linalg.norm(tangent, axis=1), 1., atol=1e-12)
        increments = np.linalg.norm(np.diff(sampled, axis=0), axis=1)
        self.assertLess(increments.max()/increments.min(), 1.02)

    def test_short_chain_does_not_get_invented_cubic_support(self):
        with self.assertRaises(ValueError):
            fit_curve([[0, 0, 0], [.001, 0, 0], [.002, 0, 0]], .0001, .0002)

    def test_config_and_world_units_contract_reject_invalid_inputs(self):
        with self.assertRaises(ValueError): Config(standoff_mm=.1, clearance_mm=.2)
        with self.assertRaises(ValueError): Config(tilt_deg=181)

    def test_roll_cannot_change_the_x_axis_or_its_plane_angle(self):
        tangent = np.array([np.cos(.2), 0, np.sin(.2)])
        angles = [normal_angle(tool_rotation(tangent, [0, 1, 0], 35., roll), [0, 1, 0])
                  for roll in (-170., 0., 37., 90.)]
        np.testing.assert_allclose(angles, 35., atol=1e-12)

    def test_world_tcp_translation_follows_path_and_shaft_keeps_normal_tilt(self):
        world = np.eye(4)
        world[:3, :3] = Rotation.from_euler('xyz', [.3, .2, .6]).as_matrix()
        world[:3, 3] = [.2, -.4, .8]
        tangent = np.array([1., .1, .2]); tangent /= np.linalg.norm(tangent)
        normal = np.array([0., 0., 1.])
        first = pose_at(np.zeros(3), tangent, normal, 0, world, Config())
        second = pose_at(.001*tangent, tangent, normal, 0, world, Config())
        direction = second[:3, 3]-first[:3, 3]; direction /= np.linalg.norm(direction)
        np.testing.assert_allclose(direction, world[:3, :3]@tangent, atol=1e-12)
        self.assertAlmostEqual(normal_angle(first[:3, :3], world[:3, :3]@normal), 15.)

    def test_oblique_tangent_keeps_prescribed_normal_angle(self):
        tangent = [np.cos(np.deg2rad(60)), 0, np.sin(np.deg2rad(60))]
        pose = pose_at([0, 0, 0], tangent, [0, 1, 0], 0, np.eye(4), Config())
        self.assertAlmostEqual(normal_angle(pose[:3, :3], [0, 1, 0]), 15.)
        with self.assertRaises(ValueError): validate_transform(np.diag([1000, 1000, 1000, 1]))


class MotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model, cls.data = create_simulation()
        cls.robot = Robot(cls.model, cls.data)

    def test_inverse_branches_replay_random_forward_targets(self):
        rng = np.random.default_rng(13)
        for _ in range(20):
            q = rng.uniform(-2., 2., 6)
            target = self.robot.fk(q, full=True)
            solutions = self.robot.solutions(target, q)
            self.assertTrue(solutions)
            for candidate in solutions:
                np.testing.assert_allclose(self.robot.fk(candidate, full=True), target, atol=1e-8)

    def test_unreachable_target_cannot_produce_a_success(self):
        pose = np.eye(4); pose[:3, 3] = [10, 10, 10]
        self.assertEqual(self.robot.solutions(pose), [])

    def test_tcp_x_follows_real_shaft_and_y_is_marked_radial_direction(self):
        from planning.infrastructure.collision import mesh_from_geom
        model, data = create_simulation()
        body, geom, site = model.body('ur_end_scaler_link').id, model.geom('ur_end_scaler_geom').id, model.site('ur_scaler_tcp').id
        body_rotation = data.xmat[body].reshape(3, 3)
        vertices, _ = mesh_from_geom(model, geom)
        world_vertices = vertices@data.geom_xmat[geom].reshape(3, 3).T+data.geom_xpos[geom]
        local = (world_vertices-data.xpos[body])@body_rotation*1000
        ring = np.unique(np.round(local[(local[:, 2]>30)&(local[:, 2]<50)&(local[:, 0]<0)], 5), axis=0)
        _, singular, basis = np.linalg.svd(ring-ring.mean(0), full_matrices=False)
        axial = basis[-1]*np.sign(basis[-1, 2])
        actual = body_rotation.T@data.site_xmat[site].reshape(3, 3)
        self.assertLess(singular[-1], .001)
        self.assertGreater(actual[:, 0]@axial, 1-1e-10)
        self.assertLess(actual[0, 1], -.9)
        self.assertGreater(actual[2, 1], .3)
        np.testing.assert_allclose(actual[:, 2], [0, -1, 0], atol=1e-10)
        np.testing.assert_allclose(np.cross(actual[:, 0], actual[:, 1]), actual[:, 2], atol=1e-12)
        np.testing.assert_allclose(model.site_pos[site], [-.001155821, .000026541, .123973109], atol=1e-12)
        self.assertLess(np.linalg.norm(world_vertices-data.site_xpos[site], axis=1).min(), 1e-7)

    def test_old_distal_tip_frame_plan_is_rejected(self):
        import json
        from control.position import load_verified
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path/'planning_report.json').write_text(json.dumps({'execution_ready': True,
                'axis_convention': 'tcp_x_along_gingival_tangent'}), encoding='utf-8')
            (path/'request.json').write_text('{}', encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'Retired TCP frame'):
                load_verified(path)

    def test_branch_graph_rejects_missing_points_and_joint_jumps(self):
        node = lambda q: {'q': np.full(6, q), 'pose': np.eye(4), 'roll': 0.}
        for nodes in ([[node(0)], [], [node(.01)]], [[node(0)], [node(1.)]]):
            with self.assertRaises(SweepFailure):
                whole_path_search(nodes, Config(), lambda a, b: (True, ''), np.zeros(6))

    def test_dynamic_programming_avoids_greedy_dead_end(self):
        node = lambda q: {'q': np.full(6, q), 'pose': np.eye(4), 'roll': 0.}
        nodes = [[node(0), node(.2)], [node(.05), node(.25)], [node(.3)]]
        selected = whole_path_search(nodes, Config(joint_step_rad=.1), lambda a, b: (True, ''), np.zeros(6))
        np.testing.assert_allclose([item['q'][0] for item in selected], [.2, .25, .3])

    def test_timing_obeys_speed_acceleration_and_is_c2_at_nodes(self):
        cfg = Config()
        q = self.robot.home.copy()
        plan = build([q, q+.05, q-.03], ['trace', 'retreat'], [1, 1], self.robot, cfg)
        limits = plan.limits()
        self.assertLessEqual(limits['max_joint_speed_rad_s'], cfg.joint_speed_rad_s+1e-12)
        self.assertLessEqual(limits['max_joint_accel_rad_s2'], cfg.joint_accel_rad_s2+1e-12)
        for t in plan.times:
            np.testing.assert_allclose(plan.sample(t)['dq'], 0, atol=1e-12)
            np.testing.assert_allclose(plan.sample(t)['ddq'], 0, atol=1e-12)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'plan.npz'; plan.save(path)
            np.testing.assert_allclose(Trajectory.load(path).joints, plan.joints)

    def test_invalid_serialized_trajectory_rejected(self):
        with self.assertRaises(ValueError):
            Trajectory([0, 0], np.zeros((2, 6)), ['trace'], [1], self.robot.joint_names)

    def test_transfer_search_routes_around_obstacle_and_rejects_blocked_goal(self):
        from planning.infrastructure.transfer import search_transfer
        def clear(a, b):
            d = b-a
            t = np.clip(-np.dot(a, d)/max(np.dot(d, d), 1e-12), 0., 1.)
            return np.linalg.norm(a+t*d) > .3
        start, goal = np.array([-.8, 0.]), np.array([.8, 0.])
        path = search_transfer(start, goal, [-1., -1.], [1., 1.], clear)
        self.assertIsNotNone(path)
        np.testing.assert_array_equal(path[0], start)
        np.testing.assert_array_equal(path[-1], goal)
        self.assertTrue(all(clear(a, b) for a, b in zip(path[:-1], path[1:])))
        self.assertIsNone(search_transfer(start, np.zeros(2), [-1., -1.], [1., 1.], clear))

    def test_triangle_collision_detects_surface_intersection(self):
        mesh = bvh(np.array([[-1, -1, 0], [1, -1, 0], [0, 1, 0]]), [[0, 1, 2]])
        triangle = fcl.CollisionObject(mesh)
        sphere = fcl.CollisionObject(fcl.Sphere(.1), fcl.Transform(np.array([0., 0, .05])))
        self.assertGreater(fcl.collide(triangle, sphere), 0)
        sphere.setTranslation(np.array([0., 0, .3]))
        self.assertAlmostEqual(fcl.distance(triangle, sphere), .2, places=6)

    def test_new_control_contract_runs_without_teleporting_state(self):
        model, data = create_simulation(); robot = Robot(model, data); checker = CollisionChecker(robot)
        q = robot.home.copy(); end = q.copy(); end[0] += .003
        plan = Trajectory([0., .2], [q, end], ['transfer'], [0], robot.joint_names)
        controller = GingivalTCPController(model, plan, checker)
        before = data.qpos.copy()
        controller.update(data)
        np.testing.assert_array_equal(data.qpos, before)
        for _ in range(25):
            controller.update(data); step_checked(model, data)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertGreater(checker.calls, 0)


if __name__ == '__main__':
    unittest.main()
