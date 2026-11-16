"""Cartesian entry/return, interpolated geometry and saved free-motion contracts."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

import numpy as np

from planning.config import Config
from planning.infrastructure.collision import NoCollisionChecker
from planning.infrastructure.robot import Robot
from planning.infrastructure.trajectory import Trajectory
from scipy.spatial.transform import Rotation
from planning.infrastructure.io import dump, fingerprint
from planning.motion_planning.free_motion import (prepare_transfer, audit_transfers,
    retime_free, load_transfer_record, TRANSFER_POLICY, validate_transfer_record)
from pipeline.simulation import create_simulation


class CartesianTestRobot:
    """Analytical XYZ/rotation-Z fixture, including zero-length rotations."""
    home = np.zeros(4)
    lower = np.full(4, -4.)
    upper = np.full(4, 4.)
    joint_names = ['x', 'y', 'z', 'rz']

    def fk(self, q, full=False):
        pose = np.eye(4)
        pose[:3, 3] = q[:3]
        pose[:3, :3] = Rotation.from_euler('z', q[3]).as_matrix()
        return pose

    def solutions(self, pose, seed=None):
        return [np.r_[pose[:3, 3], Rotation.from_matrix(pose[:3, :3]).as_rotvec()[2]]]

    def jacobian(self, q):
        jacobian = np.zeros((6, 4))
        jacobian[:3, :3] = np.eye(3)
        jacobian[5, 3] = 1.
        return jacobian


class StraightTransferTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        model, data = create_simulation()
        cls.robot = Robot(model, data)
        cls.cfg = Config(check_mode='geometry_only')

    def test_actual_robot_straight_motion_and_exact_endpoint(self):
        robot, cfg = self.robot, self.cfg
        goal = robot.home + np.array([.03, -.02, .025, .01, -.01, .02])
        result = prepare_transfer(robot, NoCollisionChecker(robot), cfg, robot.home, goal)
        plan = Trajectory(np.r_[0., np.cumsum(result['dt'])], result['joints'],
                          ['transfer']*len(result['dt']), [0]*len(result['dt']),
                          robot.joint_names, result['coefficients'])
        reference = dict(result['reference'], start_waypoint=0,
                         end_waypoint=len(plan.joints)-1, phase='transfer')
        audit = audit_transfers(plan, robot, cfg, {'version': TRANSFER_POLICY, 'segments': [reference]},
                                require_return=False)
        self.assertTrue(audit['passed'])
        self.assertLessEqual(audit['segments'][0]['max_position_error_mm'], cfg.interpolation_position_mm)
        np.testing.assert_allclose(plan.joints[-1], goal, atol=1e-12)
        for t in (0., plan.duration):
            np.testing.assert_allclose(plan.sample(t)['dq'], 0., atol=1e-10)
            np.testing.assert_allclose(plan.sample(t)['ddq'], 0., atol=1e-9)
        self.assertTrue(all(np.linalg.norm(plan.sample(t)['dq']) > 1e-9 for t in plan.times[1:-1]))
        for t in plan.times[1:-1]:
            for field in ('q', 'dq', 'ddq'):
                np.testing.assert_allclose(plan.sample(t-1e-8)[field], plan.sample(t+1e-8)[field], atol=1e-6)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'trajectory.npz'
            plan.save(path)
            restored = Trajectory.load(path)
            for t in np.linspace(0., plan.duration, 31):
                for field in ('q', 'dq', 'ddq'):
                    np.testing.assert_allclose(restored.sample(t)[field], plan.sample(t)[field])

    def test_collision_failure_never_falls_back_to_joint_shortcut(self):
        checker = SimpleNamespace(check=lambda *args: SimpleNamespace(valid=False, pair='blocked'),
                                  edge=lambda *args: SimpleNamespace(valid=False, pair='blocked'))
        with self.assertRaisesRegex(RuntimeError, 'straight_transfer'):
            prepare_transfer(self.robot, checker, Config(), self.robot.home, self.robot.home+.01)

    def test_fixed_endpoint_branch_is_not_replaced_by_equivalent_pose(self):
        goal = self.robot.home.copy()
        goal[5] += 2*np.pi
        with self.assertRaisesRegex(RuntimeError, 'straight_transfer'):
            prepare_transfer(self.robot, NoCollisionChecker(self.robot), self.cfg, self.robot.home, goal)

    def test_zero_motion_remains_finite(self):
        result = prepare_transfer(self.robot, NoCollisionChecker(self.robot), self.cfg,
                                  self.robot.home, self.robot.home)
        self.assertTrue(np.isfinite(result['coefficients']).all())
        np.testing.assert_array_equal(result['joints'][0], result['joints'][-1])

    def test_reference_version_and_endpoint_tampering_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'transfer'):
            validate_transfer_record({'version': 'obsolete', 'segments': []})
        result = prepare_transfer(self.robot, NoCollisionChecker(self.robot), self.cfg,
                                  self.robot.home, self.robot.home+.01)
        plan = Trajectory(np.r_[0., np.cumsum(result['dt'])], result['joints'],
                          ['transfer']*len(result['dt']), [0]*len(result['dt']),
                          self.robot.joint_names, result['coefficients'])
        reference = dict(result['reference'], start_waypoint=0, end_waypoint=len(plan.joints)-1,
                         phase='transfer')
        reference['end_joints_rad'] = (plan.joints[-1]+.1).tolist()
        with self.assertRaisesRegex(ValueError, 'endpoint'):
            audit_transfers(plan, self.robot, self.cfg,
                            {'version': TRANSFER_POLICY, 'segments': [reference]}, require_return=False)

    def test_retiming_preserves_the_polynomial_geometry_and_rate_limits(self):
        robot, cfg = self.robot, self.cfg
        result = prepare_transfer(robot, NoCollisionChecker(robot), cfg, robot.home, robot.home+.03)
        slower = replace(cfg, transfer_speed_mm_s=.5, joint_speed_rad_s=.01)
        dt = retime_free(result['dt'], result['coefficients'], result['joints'], robot, slower)
        original = Trajectory(np.r_[0., np.cumsum(result['dt'])], result['joints'],
            ['transfer']*len(dt), [0]*len(dt), robot.joint_names, result['coefficients'])
        retimed = Trajectory(np.r_[0., np.cumsum(dt)], result['joints'], original.phases,
            original.teeth, robot.joint_names, result['coefficients'])
        self.assertGreater(retimed.duration, original.duration)
        for fraction in np.linspace(0, 1, 73):
            np.testing.assert_allclose(original.sample(original.duration*fraction)['q'],
                retimed.sample(retimed.duration*fraction)['q'], atol=1e-12)
        self.assertLessEqual(retimed.limits()['max_joint_speed_rad_s'], slower.joint_speed_rad_s)
        ref = dict(result['reference'], phase='transfer', start_waypoint=0, end_waypoint=len(dt))
        self.assertTrue(audit_transfers(retimed, robot, slower,
            {'version': TRANSFER_POLICY, 'segments': [ref]}, require_return=False)['passed'])

    def test_zero_length_half_turn_uses_orientation_interpolation(self):
        robot = CartesianTestRobot()
        goal = np.array([0., 0., 0., np.pi])
        result = prepare_transfer(robot, object(), self.cfg, robot.home, goal)
        self.assertTrue(result['audit']['passed'])
        np.testing.assert_allclose(result['joints'][:, :3], 0., atol=1e-12)
        self.assertGreater(result['dt'].sum(), 0.)
        np.testing.assert_array_equal(result['joints'][-1], goal)

    def test_audit_detects_bowed_interpolant_with_exact_line_waypoints(self):
        robot = CartesianTestRobot()
        q = np.array([[0., 0., 0., 0.], [.001, 0., 0., 0.], [.002, 0., 0., 0.]])
        v = np.zeros_like(q)
        v[1, 1] = .002
        coefficients = np.array([Trajectory._quintic_coefficients(a, b, va, vb,
            np.zeros(4), np.zeros(4), 1.) for a, b, va, vb in zip(q[:-1], q[1:], v[:-1], v[1:])])
        plan = Trajectory([0., 1., 2.], q, ['transfer']*2, [0]*2, robot.joint_names, coefficients)
        ref = {'start_pose_world_m': robot.fk(q[0]).tolist(), 'end_pose_world_m': robot.fk(q[-1]).tolist(),
            'start_joints_rad': q[0].tolist(), 'end_joints_rad': q[-1].tolist(),
            'orientation': 'shortest_slerp', 'clock': 'quintic_progress',
            'phase': 'transfer', 'start_waypoint': 0, 'end_waypoint': 2}
        with self.assertRaisesRegex(RuntimeError, 'geometry audit'):
            audit_transfers(plan, robot, self.cfg,
                {'version': TRANSFER_POLICY, 'segments': [ref]}, require_return=False)

    def test_finite_segment_rejects_small_backtracking_inside_position_tolerance(self):
        robot = CartesianTestRobot()
        q = np.zeros((3, 4))
        q[:, 0] = [0., -.00001, .0001]
        c = np.array([Trajectory._smoothstep_coefficients(a, b) for a, b in zip(q[:-1], q[1:])])
        plan = Trajectory([0., 1., 2.], q, ['transfer']*2, [0]*2, robot.joint_names, c)
        ref = {'start_pose_world_m': robot.fk(q[0]).tolist(), 'end_pose_world_m': robot.fk(q[-1]).tolist(),
            'start_joints_rad': q[0].tolist(), 'end_joints_rad': q[-1].tolist(),
            'orientation': 'shortest_slerp', 'clock': 'quintic_progress',
            'phase': 'transfer', 'start_waypoint': 0, 'end_waypoint': 2}
        audit = audit_transfers(plan, robot, self.cfg, {'version': TRANSFER_POLICY, 'segments': [ref]},
            require_return=False, raise_on_failure=False)
        self.assertLess(audit['segments'][0]['max_position_error_mm'], self.cfg.interpolation_position_mm)
        self.assertFalse(audit['segments'][0]['monotonic_finite_segment'])
        self.assertFalse(audit['passed'])

    def test_provenance_rejects_changed_reference_and_preserves_legacy_loading(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self.assertIsNone(load_transfer_record(directory, {'schema_version': 7}, {}))
            path = directory/'transfer_reference.json'
            dump(path, {'version': TRANSFER_POLICY, 'segments': []})
            metadata = {'schema_version': 8, 'transfer_policy': TRANSFER_POLICY,
                        'transfer_reference_file': fingerprint(path)}
            self.assertEqual(load_transfer_record(directory, metadata, metadata)['version'], TRANSFER_POLICY)
            dump(path, {'version': 'tampered', 'segments': []})
            with self.assertRaisesRegex(ValueError, 'reference changed'):
                load_transfer_record(directory, metadata, metadata)
            with self.assertRaisesRegex(ValueError, 'policy changed'):
                load_transfer_record(directory, metadata, {'schema_version': 8})


if __name__ == '__main__':
    unittest.main()
