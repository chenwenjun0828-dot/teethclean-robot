"""Fixed-path local timing preserves nodes, C2 continuity and verified limits."""
from contextlib import redirect_stdout
from dataclasses import asdict, replace
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

from planning.config import Config
from planning.infrastructure.audit import audit_motion
from planning.infrastructure.fixed_spline import JointSpline
from planning.infrastructure.local_speed import LocalSpeedTrace
from planning.infrastructure.trajectory import Trajectory
from planning.motion_planning.fixed_path import build
from planning.test_fixed_path import CartesianRobot


class LocalSpeedTests(unittest.TestCase):
    def setUp(self):
        self.robot = CartesianRobot()
        self.cfg = Config(planning_mode='continuous', speed_mm_s=10., speed_profile='local')
        self.q = np.array([[0., 0.], [.0004, .0002], [.002, .0012], [.005, .002], [.01, .006]])
        self.checker = SimpleNamespace(check=lambda *args: SimpleNamespace(valid=True, pair='', clearance_m=.01))

    def build(self, duration=10., tail=False):
        q = np.r_[[[-.01, -.01]], self.q, [[.015, .008]]] if tail else self.q
        phases = ['transfer']+['trace']*4+['retreat'] if tail else ['trace']*4
        with redirect_stdout(io.StringIO()):
            return build(q, phases, [1]*len(phases), self.robot, self.cfg, duration)

    def test_local_clock_preserves_fixed_curve_and_every_input_node_with_fast_entry_exit(self):
        plan = self.build(tail=True)
        self.assertAlmostEqual(plan.duration, 10.)
        self.assertEqual(plan.constant_trace.version, 2)
        self.assertEqual(plan.constant_trace.start, 1)
        self.assertEqual(plan.constant_trace.end, 5)
        for time, q in zip(plan.times, plan.joints):
            np.testing.assert_allclose(plan.sample(time)['q'], q, atol=1e-11)
        trace = plan.constant_trace
        for fraction in np.linspace(0, 1, 131):
            t = plan.times[1]+fraction*(plan.times[5]-plan.times[1])
            np.testing.assert_allclose(plan.sample(t)['q'], trace.path(trace.clock(fraction)), atol=1e-12)
        audit = audit_motion(plan, self.robot, self.checker, self.cfg)
        self.assertTrue(audit['passed'])
        self.assertEqual(audit['speed_profile']['mode'], 'local')
        self.assertLessEqual(audit['max_trace_tcp_speed_mm_s'], self.cfg.speed_mm_s)

    def test_c2_clock_boundaries_stopped_endpoints_and_derivatives(self):
        plan = self.build()
        trace = plan.constant_trace
        for time in trace.clock_t[1:-1]*plan.duration:
            for key in ('q', 'dq', 'ddq'):
                np.testing.assert_allclose(plan.sample(time-1e-8)[key], plan.sample(time+1e-8)[key], atol=1e-7)
        for time in (0., plan.duration):
            np.testing.assert_allclose(plan.sample(time)['dq'], 0., atol=1e-12)
            np.testing.assert_allclose(plan.sample(time)['ddq'], 0., atol=1e-10)
        bounds = plan.limits()
        for time in np.linspace(.01, plan.duration-.01, 301):
            before, state, after = plan.sample(time-1e-6), plan.sample(time), plan.sample(time+1e-6)
            np.testing.assert_allclose((after['q']-before['q'])/2e-6, state['dq'], atol=1e-7)
            np.testing.assert_allclose((after['dq']-before['dq'])/2e-6, state['ddq'], atol=1e-6)
            self.assertLessEqual(abs(state['dq']).max(), bounds['max_joint_speed_rad_s']+1e-10)
            self.assertLessEqual(abs(state['ddq']).max(), bounds['max_joint_accel_rad_s2']+1e-10)

    def test_roundtrip_stretch_and_infeasible_target(self):
        plan = self.build()
        with tempfile.TemporaryDirectory() as temp:
            file = Path(temp)/'trajectory.npz'
            plan.save(file)
            restored = Trajectory.load(file)
            for time in np.linspace(0, 10, 41):
                for key in ('q', 'dq', 'ddq'):
                    np.testing.assert_allclose(plan.sample(time)[key], restored.sample(time)[key], atol=1e-12)
            restored.times *= 2
            for time in np.linspace(0, 10, 41):
                np.testing.assert_allclose(plan.sample(time)['q'], restored.sample(2*time)['q'], atol=1e-12)
                np.testing.assert_allclose(plan.sample(time)['ddq']/4, restored.sample(2*time)['ddq'], atol=1e-12)
        with self.assertRaisesRegex(RuntimeError, 'target is'):
            self.build(.001)

    def test_dense_path_with_small_tcp_motion_and_large_joint_changes(self):
        class ScaledRobot:
            def fk(self, q):
                pose = np.eye(4)
                pose[:2, 3] = q*np.array([1., .0001])
                return pose

            def jacobian(self, q):
                jacobian = np.zeros((6, 2))
                jacobian[0, 0], jacobian[1, 1] = 1., .0001
                return jacobian

        # Reproduce the large joint derivatives of densely sampled IK near a
        # poorly conditioned TCP Jacobian. Unit-interval ECOS timing fails here.
        parameter = np.linspace(0., 1., 1215)
        joints = np.c_[.04*parameter, np.sin(2*np.pi*150*parameter)]
        robot = ScaledRobot()
        cfg = replace(self.cfg, speed_mm_s=1., joint_speed_rad_s=2.2,
                      joint_accel_rad_s2=12., orientation_speed_deg_s=200.)
        fixed = JointSpline.build(joints, robot)
        timed = LocalSpeedTrace.retime(fixed, robot, cfg)
        np.testing.assert_array_equal(timed.path.c, fixed.path.c)
        np.testing.assert_array_equal(timed.joints, joints)
        duration = timed.minimum_duration_s
        self.assertGreaterEqual(duration, fixed.length*1000/cfg.speed_mm_s)
        velocity, acceleration = timed.rate_bounds(duration)
        self.assertLessEqual(velocity.max(), cfg.joint_speed_rad_s)
        self.assertLessEqual(acceleration.max(), cfg.joint_accel_rad_s2)
        self.assertLessEqual(np.degrees(velocity.sum(axis=1)).max(), cfg.orientation_speed_deg_s)
        for time, q in zip(timed.fraction_at_u(timed.knots)*duration, joints):
            np.testing.assert_allclose(timed.sample(time, duration)[0], q, atol=1e-9)
        for time in np.linspace(0., duration, 2001):
            q, dq, _ = timed.sample(time, duration)
            self.assertLessEqual(np.linalg.norm(robot.jacobian(q)[:3]@dq)*1000,
                                 cfg.speed_mm_s)

    def test_timing_failure_reports_solver_code_and_grid_size(self):
        trace = JointSpline.build(self.q, self.robot)
        solver = SimpleNamespace(
            compute_parameterization=lambda *args: (None, None, None),
            problem_data=SimpleNamespace(return_code=SimpleNamespace(name='FailUncontrollable')))
        with patch('toppra.algorithm.TOPPRA', return_value=solver):
            with self.assertRaisesRegex(RuntimeError, 'ECOS; 17 grid points; return code: FailUncontrollable'):
                LocalSpeedTrace.retime(trace, self.robot, self.cfg)

    def test_short_clock_cells_preserve_c2_without_absolute_position_cancellation(self):
        joints = np.c_[np.r_[0., .03+np.arange(101)*1e-7], np.zeros(102)]
        trace = JointSpline.build(joints, self.robot)

        def solver(*args, **kwargs):
            velocity = np.full(len(kwargs['gridpoints']), 10.)
            velocity[[0, -1]] = 0.
            return SimpleNamespace(compute_parameterization=lambda *args: (None, velocity, None))

        with patch('toppra.algorithm.TOPPRA', side_effect=solver):
            timed = LocalSpeedTrace.retime(trace, self.robot, self.cfg)
        np.testing.assert_array_equal(timed.path.c, trace.path.c)
        np.testing.assert_array_equal(timed.joints, joints)
        widths = np.diff(timed.clock_t)
        self.assertLess(widths.min(), 1e-6)
        acceleration = (timed.clock_c*(np.arange(6)*(np.arange(6)-1))).sum(1)/widths**2
        np.testing.assert_allclose(acceleration, 0., atol=1e-7)
        for time, q in zip(timed.fraction_at_u(timed.knots), joints):
            np.testing.assert_allclose(timed.sample(time, 1.)[0], q, atol=1e-10)

    def test_numerical_failure_retries_equivalent_units_with_identical_path_and_limits(self):
        import toppra
        trace = JointSpline.build(self.q, self.robot)
        real_solver = toppra.algorithm.TOPPRA
        failed = SimpleNamespace(
            compute_parameterization=lambda *args: (None, None, None),
            problem_data=SimpleNamespace(return_code=SimpleNamespace(name='FailUncontrollable')))
        attempts = []

        def solver(constraints, path, **options):
            attempts.append((path, options['gridpoints'].copy()))
            return failed if len(attempts) == 1 else real_solver(constraints, path, **options)

        with patch('toppra.algorithm.TOPPRA', side_effect=solver):
            timed = LocalSpeedTrace.retime(trace, self.robot, self.cfg)
        self.assertEqual(len(attempts), 2)
        np.testing.assert_allclose(attempts[0][1], attempts[1][1]*10)
        np.testing.assert_array_equal(timed.joints, trace.joints)
        np.testing.assert_array_equal(timed.path.c, trace.path.c)
        v, a = timed.rate_bounds(timed.minimum_duration_s)
        self.assertLessEqual(v.max(), self.cfg.joint_speed_rad_s)
        self.assertLessEqual(a.max(), self.cfg.joint_accel_rad_s2)

    def test_arc_inverse_roundoff_at_stopped_endpoints_preserves_clock_consistency(self):
        trace = JointSpline.build(self.q, self.robot)
        # Sub-femtometre polynomial roundoff can amplify when inverting a
        # clock whose endpoint velocity is zero.
        trace.inverse[0, 0] += 1e-18
        trace.inverse[-1, -1] -= 1e-16
        with redirect_stdout(io.StringIO()):
            plan = build(self.q, ['trace']*4, [1]*4, self.robot, self.cfg,
                         10., fixed_trace=trace)
        self.assertEqual(plan.duration, 10.)
        np.testing.assert_array_equal(plan.constant_trace.fraction_at([0., trace.length]), [0., 1.])
        for time, q in zip(plan.times, self.q):
            np.testing.assert_allclose(plan.sample(time)['q'], q, atol=1e-11)
        with tempfile.TemporaryDirectory() as temp:
            file = Path(temp)/'trajectory.npz'
            plan.save(file)
            restored = Trajectory.load(file)
            np.testing.assert_array_equal(restored.times, plan.times)
        changed = plan.times.copy()
        changed[2] += .001
        with self.assertRaisesRegex(ValueError, 'waypoint times differ'):
            Trajectory(changed, plan.joints, plan.phases, plan.teeth, plan.joint_names,
                       plan.coefficients, plan.mount_position_m, plan.constant_trace)

    def test_saved_plan_retiming_preserves_geometry_and_pose_exports(self):
        from planning.infrastructure.io import dump
        from planning.retime import run
        original = self.build(tail=True)
        self.checker.robot = self.robot
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()):
            source, output = Path(temp)/'source', Path(temp)/'retimed'
            source.mkdir()
            original.save(source/'trajectory.npz')
            dump(source/'request.json', {'config': asdict(self.cfg)})
            dump(source/'planning_report.json', {
                'config': asdict(self.cfg), 'path_kind': 'continuous_outer_front_sweep',
                'per_tooth': {'1': {'retained_length_mm': 1., 'source_length_mm': 2.}},
                'downsampling': {'applied': True}})
            dump(source/'geometry.json', {})
            dump(source/'tooth_planes.json', {})
            np.save(source/'pose_joints_rad.npy', self.q)

            def physics(*args, **kwargs):
                result = {'status': 'completed', 'minimum_actual_clearance_mm': 1.}
                dump(output/'execution_report.json', result)
                return result

            with patch('planning.motion_planning.execution.load_verified',
                       return_value=(None, None, original, self.checker)), \
                    patch('planning.motion_planning.validation.audit_trace', return_value={'passed': True}), \
                    patch('control.position.execute', side_effect=physics) as execute:
                retimed, report = run(source, output, 90.)
            self.assertEqual(retimed.duration, 90.)
            self.assertTrue(report['execution_ready'])
            self.assertEqual(report['timing_profile']['version'], 4)
            for name in ('joint_speed_rad_s', 'joint_accel_rad_s2', 'orientation_speed_deg_s'):
                self.assertEqual(report['config'][name], getattr(self.cfg, name))
            self.assertNotIn('sample_spacing_mm', report['timing_profile'])
            self.assertNotIn('downsampling', report)
            execute.assert_called_once()
            np.testing.assert_array_equal(original.joints, retimed.joints)
            np.testing.assert_array_equal(original.constant_trace.path.c, retimed.constant_trace.path.c)
            np.testing.assert_array_equal(np.load(output/'pose_joints_rad.npy'), self.q)
            restored = Trajectory.load(output/'trajectory.npz')
            for t in np.linspace(0., 90., 41):
                np.testing.assert_allclose(retimed.sample(t)['q'], restored.sample(t)['q'], atol=1e-12)

    def test_diagnostic_orientation_keeps_metrics_and_still_rejects_position_error(self):
        from planning.test_sweep import SweepAuditTests
        from planning.pose_search.constraints import tool_rotation
        from planning.motion_planning.audit import audit_trace
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)
            path, plan, report = SweepAuditTests().setup_reference(out, Config(planning_mode='continuous', orientation_audit='diagnostic'))
            class Robot:
                offset = 0.
                def fk(self, q, full=False):
                    pose = np.eye(4)
                    pose[:3, :3] = tool_rotation(path.evaluate(q[0], 1), path.normal(q[0]), 25.)
                    pose[:3, 3] = path.position(q[0], .0015)+[0, 0, self.offset]
                    return pose
            robot = Robot()
            audit = audit_trace(out, plan, robot)
            self.assertTrue(audit['passed'])
            self.assertFalse(audit['orientation_passed'])
            self.assertFalse(audit['orientation_enforced'])
            self.assertGreater(audit['rotation_error_deg'], 5.)
            robot.offset = .002
            with self.assertRaisesRegex(RuntimeError, 'Independent trace audit'):
                audit_trace(out, plan, robot)

    def test_cli_forwarding_and_old_saved_profile(self):
        from planning.__main__ import build_argparser
        from pipeline.run import main
        args = build_argparser().parse_args(['--dry-run', '--duration', '90', '--orientation-audit', 'diagnostic'])
        self.assertEqual((args.speed_profile, args.transfer_speed_mm_s), ('local', 150.))
        with redirect_stdout(io.StringIO()), patch('pipeline.run.run_pipeline', return_value={'status': 'dry_run'}) as run:
            main(['--dry-run', '--duration', '90', '--orientation-audit', 'diagnostic',
                  '--transfer-speed-mm-s', '120'])
            self.assertEqual(run.call_args.kwargs['orientation_audit'], 'diagnostic')
            self.assertEqual(run.call_args.kwargs['transfer_speed_mm_s'], 120.)
        saved = asdict(self.cfg)
        saved.pop('speed_profile')
        self.assertEqual(Config.from_saved(saved).speed_profile, 'local')
        saved['speed_profile'] = 'constant'
        self.assertEqual(Config.from_saved(saved).speed_profile, 'local')
        with self.assertRaisesRegex(ValueError, 'Only local TOPPRA'):
            Config(planning_mode='continuous', speed_profile='constant')


if __name__ == '__main__':
    unittest.main()
