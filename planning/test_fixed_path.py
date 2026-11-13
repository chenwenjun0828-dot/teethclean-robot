"""Fixed-path API, independent audits and legacy artifact playback."""
from dataclasses import replace
from contextlib import redirect_stdout
import io
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch
import mujoco
import numpy as np

from planning.config import Config
from planning.infrastructure.fixed_spline import JointSpline
from planning.infrastructure.trajectory import Trajectory
from planning.motion_planning.fixed_path import build, plan_pose_sequence


class CartesianRobot:
    """Two actual MuJoCo slide joints give an independently measurable curved TCP."""
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="robot_mount"><body><joint name="x" type="slide" axis="1 0 0"/>
          <geom size=".01" mass="1"/><body><joint name="y" type="slide" axis="0 1 0"/>
          <geom size=".01" mass="1"/><site name="tcp"/></body></body></body>
          </worldbody></mujoco>''')
        self.verify_data = mujoco.MjData(self.model)
        self.data = self.verify_data
        self.site = 0
        self.dadr = self.qadr = np.arange(2)
        self.joint_names = ['x', 'y']
        self.lower, self.upper = np.full(2, -1.), np.ones(2)
        self.home = np.zeros(2)
        self.initial_qpos = self.home.copy()

    def fk(self, q, full=False):
        self.data.qpos[:] = q
        mujoco.mj_forward(self.model, self.data)
        result = np.eye(4)
        result[:3, 3] = self.data.site_xpos[self.site]
        return result

    def jacobian(self, q):
        self.fk(q)
        a, b = np.zeros((3, 2)), np.zeros((3, 2))
        mujoco.mj_jacSite(self.model, self.data, a, b, self.site)
        return np.vstack((a, b))


class FixedPathTests(unittest.TestCase):
    def setUp(self):
        self.robot = CartesianRobot()
        self.cfg = Config(speed_mm_s=10.)
        self.q = np.array([[0, 0], [.0004, .0002], [.002, .0012], [.005, .002], [.01, .006]])
        self.checker = SimpleNamespace(check=lambda q, clearance:
            SimpleNamespace(valid=True, pair='', clearance_m=.01))

    def plan(self, duration=10.):
        return build(self.q, ['trace']*4, [1]*4, self.robot, self.cfg, duration)

    def test_passes_all_original_nodes_with_local_speed_timing(self):
        plan = self.plan()
        self.assertEqual(len(plan.times), len(self.q))
        self.assertEqual(plan.constant_trace.version, 2)
        for time, q in zip(plan.times, self.q):
            np.testing.assert_allclose(plan.sample(time)['q'], q, atol=1e-12)
        self.assertGreater(plan.limits()['moving_internal_waypoints'], 0)

    def test_version_one_clock_remains_readable_for_saved_playback(self):
        from planning.infrastructure.legacy_trajectory import ConstantTCPTrace
        geometry = JointSpline.build(self.q, self.robot)
        record = dict(geometry.record(), version=1, ramp_fraction=.02)
        legacy = ConstantTCPTrace.from_record(record)
        plan = Trajectory(legacy.fraction_at(legacy.node_s)*10., self.q,
                          ['trace']*4, [1]*4, self.robot.joint_names,
                          constant_trace=legacy)
        with tempfile.TemporaryDirectory() as temp:
            file = Path(temp)/'legacy.npz'
            plan.save(file)
            restored = Trajectory.load(file)
            self.assertEqual(restored.constant_trace.version, 1)
            for t in np.linspace(0., 10., 41):
                for key in ('q', 'dq', 'ddq'):
                    np.testing.assert_allclose(plan.sample(t)[key], restored.sample(t)[key], atol=1e-12)

    def test_derivatives_continuity_and_conservative_bounds(self):
        plan = self.plan()
        bounds = plan.limits()
        for t in np.linspace(.02, 9.98, 501):
            state = plan.sample(t)
            h = 1e-5
            before, after = plan.sample(t-h), plan.sample(t+h)
            np.testing.assert_allclose((after['q']-before['q'])/(2*h), state['dq'], atol=2e-10)
            np.testing.assert_allclose((after['dq']-before['dq'])/(2*h), state['ddq'], atol=2e-9)
            self.assertLessEqual(abs(state['dq']).max(), bounds['max_joint_speed_rad_s']+1e-10)
            self.assertLessEqual(abs(state['ddq']).max(), bounds['max_joint_accel_rad_s2']+1e-10)
        for t in np.r_[plan.times[1:-1], 1., 9.]:
            for name in ('q', 'dq', 'ddq'):
                np.testing.assert_allclose(plan.sample(t-1e-8)[name], plan.sample(t+1e-8)[name], atol=1e-8)
        for t in (0., plan.duration):
            np.testing.assert_allclose(plan.sample(t)['dq'], 0, atol=1e-12)
            np.testing.assert_allclose(plan.sample(t)['ddq'], 0, atol=1e-12)

    def test_round_trip_and_uniform_time_stretch_preserve_the_curve(self):
        plan = self.plan()
        with tempfile.TemporaryDirectory() as temp:
            file = Path(temp)/'trajectory.npz'
            plan.save(file)
            restored = Trajectory.load(file)
            for t in np.linspace(0, 10, 101):
                for key in ('q', 'dq', 'ddq'):
                    np.testing.assert_allclose(plan.sample(t)[key], restored.sample(t)[key], atol=1e-13)
            restored.times *= 2
            for t in np.linspace(0, 10, 101):
                np.testing.assert_allclose(plan.sample(t)['q'], restored.sample(2*t)['q'], atol=1e-13)
                np.testing.assert_allclose(plan.sample(t)['dq']/2, restored.sample(2*t)['dq'], atol=1e-13)
            legacy = Trajectory([0, 1], [[0, 0], [.01, .01]], ['trace'], [1], ['x', 'y'])
            legacy.save(file)
            self.assertIsNone(Trajectory.load(file).constant_trace)

    def test_infeasible_duration_repeated_positions_and_invalid_pose_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'target is'):
            self.plan(.01)
        with self.assertRaisesRegex(ValueError, 'distinct consecutive'):
            JointSpline.build(np.array([[0., 0.], [0., 0.]]), self.robot)
        poses = np.array([self.robot.fk(q) for q in self.q])
        poses[0, 0, 0] = 2
        with self.assertRaisesRegex(ValueError, 'orthonormal'):
            plan_pose_sequence(poses, self.robot, self.checker, self.cfg, joints=self.q)
        poses = np.array([self.robot.fk(q) for q in self.q])
        from planning.pose_search.continuous import SweepFailure
        with patch.object(self.robot, 'solutions', return_value=[], create=True), self.assertRaises(SweepFailure):
            plan_pose_sequence(poses, self.robot, self.checker, self.cfg)

    def test_public_api_reuses_ik_and_rejects_interior_collision_or_joint_overshoot(self):
        poses = np.array([self.robot.fk(q) for q in self.q])
        plan, audit = plan_pose_sequence(poses, self.robot, self.checker, self.cfg,
                                          joints=self.q, duration_s=10.)
        self.assertTrue(audit['passed'])
        self.assertEqual(audit['speed_profile']['mode'], 'local')
        self.checker.check = lambda q, clearance: SimpleNamespace(
            valid=not (.003 < q[0] < .004), pair='interior obstacle', clearance_m=.01)
        with self.assertRaisesRegex(RuntimeError, 'interior obstacle'):
            plan_pose_sequence(poses, self.robot, self.checker, self.cfg, joints=self.q, duration_s=10.)
        self.checker.check = lambda q, clearance: SimpleNamespace(valid=True, pair='', clearance_m=.01)
        # All nodes fit; the natural cubic overshoots the node maximum between them.
        q = np.array([[0, 0], [.01, .01], [.02, .01], [.03, 0]])
        self.robot.upper[1] = .0101
        with self.assertRaisesRegex(RuntimeError, 'joint limits'):
            plan_pose_sequence(np.array([self.robot.fk(x) for x in q]), self.robot,
                               self.checker, replace(self.cfg, speed_mm_s=20.), joints=q, duration_s=10.)

    def test_export_validation_runs_physics_once_and_marks_failure_unexecutable(self):
        from planning.motion_planning.validation import validate
        from planning.infrastructure.io import dump
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            out = Path(temp)
            dump(out/'geometry.json', {})
            dump(out/'tooth_planes.json', {})
            report = {'path_kind': 'continuous_outer_front_sweep',
                      'per_tooth': {'1': {'retained_length_mm': 1., 'source_length_mm': 2.}}}
            def physics(*args, **kwargs):
                result = {'status': 'completed', 'minimum_actual_clearance_mm': 1.}
                dump(out/'execution_report.json', result)
                return result
            with patch('planning.motion_planning.validation.audit_trace', return_value={'passed': True}) as geometry, \
                    patch('planning.motion_planning.validation.audit_motion', return_value={
                        'passed': True, 'max_trace_tcp_speed_mm_s': 1.,
                        'max_free_tcp_speed_mm_s': 0., 'minimum_clearance_mm': 1.}) as motion, \
                    patch('control.position.execute', side_effect=physics) as execute:
                plan, result = validate(out, self.plan(), report, self.robot, self.checker, self.cfg)
                self.assertIsNotNone(plan.constant_trace)
                self.assertTrue(result['execution_ready'])
                geometry.assert_called_once()
                motion.assert_called_once()
                execute.assert_called_once()
                geometry.side_effect = RuntimeError('curve deviates')
                execute.reset_mock()
                plan, result = validate(out, plan, result, self.robot, self.checker, self.cfg)
                self.assertIsNone(plan)
                self.assertFalse(result['execution_ready'])
                self.assertFalse(result['fast_dynamics_verified'])
                execute.assert_not_called()


if __name__ == '__main__':
    unittest.main()
