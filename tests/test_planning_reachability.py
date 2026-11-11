"""Regression tests for runtime mounting and executed continuous trajectories."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from planning.config import Config
from planning.infrastructure.mount import apply_mount_position
from planning.pose_search.constraints import pose_at, normal_angle, tool_rotation
from planning.pose_search.continuous import whole_path_search
from planning.infrastructure.transfer import initial_approach
from planning.infrastructure.trajectory import Trajectory
from planning.motion_planning.fixed_path import build
from planning.infrastructure.audit import audit_trace
from pipeline.simulation import create_simulation
from planning.infrastructure.robot import Robot


class MountAndPoseTests(unittest.TestCase):
    def test_runtime_mount_moves_robot_without_moving_teeth_or_joint_state(self):
        model, data = create_simulation()
        joints = data.qpos.copy()
        teeth = data.xpos[model.body('dental_mandibular').id].copy()
        tcp = data.site_xpos[model.site('ur_scaler_tcp').id].copy()
        delta = np.array([.02, -.03, .01])
        position = model.body_pos[model.body('robot_mount').id].copy()+delta
        apply_mount_position(model, position, data)
        np.testing.assert_allclose(data.site_xpos[model.site('ur_scaler_tcp').id]-tcp, delta, atol=1e-12)
        np.testing.assert_array_equal(data.qpos, joints)
        np.testing.assert_array_equal(data.xpos[model.body('dental_mandibular').id], teeth)
        with self.assertRaises(ValueError): apply_mount_position(model, [0, np.nan, 0], data)

    def test_prescribed_tilt_and_invalid_boundaries(self):
        for angle in (5., 15., 30., 55.):
            result = pose_at(np.zeros(3), [1, 0, .2], [0, 1, 0], -141.,
                             np.eye(4), Config(tilt_deg=angle))
            self.assertAlmostEqual(normal_angle(result[:3, :3], [0, 1, 0]), angle)
        for angle in (-181., 181.):
            with self.assertRaises(ValueError):
                Config(tilt_deg=angle)

    def test_pose_graph_refuses_rotation_jump_even_with_close_joints(self):
        def node(q, angle):
            pose = np.eye(4); pose[:3,:3] = Rotation.from_euler('x',angle,degrees=True).as_matrix()
            return {'q': np.full(6,q), 'pose': pose, 'roll': angle}
        nodes = [[node(0,0),node(.1,30)], [node(.01,35)], [node(.02,40)]]
        selected = whole_path_search(nodes, Config(max_orientation_step_deg=10.),
                                     lambda a, b: (True, ''), np.zeros(6))
        self.assertEqual(selected[0]['roll'],30)

    def test_pose_graph_uses_alternative_when_best_edge_collides(self):
        def node(q):
            return {'q':np.full(6,q),'pose':np.eye(4),'roll':0}
        nodes=[[node(0.),node(.1)],[node(.01)],[node(.02)]]
        selected=whole_path_search(nodes,Config(),lambda a,b:(a['q'][0]!=0., 'collision'),np.zeros(6))
        self.assertAlmostEqual(selected[0]['q'][0],.1)

    def test_direct_initial_approach_needs_no_offset(self):
        start, end = np.zeros(6), np.ones(6)*.01
        robot = SimpleNamespace(home=start)
        checker = SimpleNamespace(edge=lambda *args: SimpleNamespace(valid=True,clearance_m=.002))
        result = initial_approach(robot,checker,Config(),end,np.array([0,0,1]))
        self.assertEqual(result['retreat_mm'],0)
        self.assertEqual(result['segments'][0][0],'transfer')

    def test_initial_approach_tries_smallest_valid_offset(self):
        start, end = np.zeros(6), np.ones(6)*.01
        robot = SimpleNamespace(fk=lambda q: np.eye(4), home=start, lower=-np.ones(6), upper=np.ones(6))
        checker = SimpleNamespace(edge=lambda a,b,*args: SimpleNamespace(
            valid=not np.array_equal(b,end),clearance_m=.002))
        def straight(robot,checker,q,target,cfg):
            if target[2,3] < .005-1e-10: return None
            return [q,q+.01]
        with patch('planning.infrastructure.transfer.straight_tcp_motion',side_effect=straight):
            result = initial_approach(robot,checker,Config(),end,np.array([0,0,1]))
        self.assertEqual(result['retreat_mm'],5.)


class ContinuousMotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        model,data=create_simulation(); cls.robot=Robot(model,data)

    def test_trace_velocity_is_nonzero_and_c2_across_unequal_intervals(self):
        q=self.robot.home.copy()
        points=[q,q+.01,q+.025,q+.04,q+.045]
        cfg=Config()
        plan=build(points,['trace','trace','trace','retreat'],[1]*4,self.robot,cfg)
        self.assertGreater(np.linalg.norm(plan.sample(plan.times[1])['dq']),1e-5)
        for time in plan.times[1:-1]:
            left,right=plan.sample(time-1e-7),plan.sample(time+1e-7)
            for field in ('q','dq','ddq'):
                np.testing.assert_allclose(left[field],right[field],atol=1e-6,rtol=0)
        np.testing.assert_allclose(plan.sample(plan.times[3])['dq'],0,atol=1e-10)
        limits=plan.limits()
        self.assertLessEqual(limits['max_joint_speed_rad_s'],cfg.joint_speed_rad_s)
        self.assertLessEqual(limits['max_joint_accel_rad_s2'],cfg.joint_accel_rad_s2)
        self.assertLessEqual(limits['tcp_angular_speed_bound_deg_s'],cfg.orientation_speed_deg_s)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'trajectory.npz'; plan.save(path); loaded=Trajectory.load(path)
            for t in np.linspace(0,plan.duration,41):
                for key in ('q','dq','ddq'): np.testing.assert_allclose(plan.sample(t)[key],loaded.sample(t)[key])
            np.testing.assert_allclose(plan.mount_position_m,loaded.mount_position_m)

    def test_free_transfer_uses_configured_cartesian_speed(self):
        from planning.test_fixed_path import CartesianRobot
        robot = CartesianRobot()
        cfg = dict(joint_speed_rad_s=2.2, joint_accel_rad_s2=12., orientation_speed_deg_s=200.)
        q = [[0., 0.], [.5, 0.], [.501, 0.]]
        slow = build(q, ['transfer', 'trace'], [0, 1], robot,
                     Config(transfer_speed_mm_s=20., **cfg))
        fast = build(q, ['transfer', 'trace'], [0, 1], robot,
                     Config(transfer_speed_mm_s=150., **cfg))
        self.assertGreater(slow.times[1], fast.times[1]*7.)

    def test_motion_audit_checks_executed_curve_interior(self):
        from planning.infrastructure.audit import audit_motion
        q=self.robot.home.copy()
        plan=build([q,q+.01,q+.02],['trace','trace'],[1,1],self.robot,Config())
        checker=SimpleNamespace(check=lambda candidate,clearance: SimpleNamespace(
            valid=not q[0]+.004<candidate[0]<q[0]+.006,clearance_m=.003,pair='synthetic obstacle'))
        with self.assertRaisesRegex(RuntimeError,'synthetic obstacle'):
            audit_motion(plan,self.robot,checker,Config())

    def test_corrupted_polynomial_endpoint_is_rejected(self):
        q=self.robot.home.copy(); plan=build([q,q+.01],['trace'],[1],self.robot,Config())
        coefficients=plan.coefficients.copy(); coefficients[0,0,0]+=.1
        with self.assertRaisesRegex(ValueError,'endpoints'):
            Trajectory(plan.times,plan.joints,plan.phases,plan.teeth,plan.joint_names,coefficients)

    def test_audit_detects_bowed_interpolant_when_waypoints_are_on_curve(self):
        from planning.test_sweep import SweepAuditTests
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            reference, source, _ = SweepAuditTests().setup_reference(path)
            q = np.c_[source.joints[:, 0], np.zeros(len(source.joints))]
            velocity = np.zeros_like(q)
            velocity[1, 1] = .005
            coefficients = np.array([Trajectory._quintic_coefficients(a, b, va, vb,
                np.zeros(2), np.zeros(2), dt)
                for a, b, va, vb, dt in zip(q[:-1], q[1:], velocity[:-1], velocity[1:], np.diff(source.times))])
            plan = Trajectory(source.times, q, source.phases, source.teeth, ['progress', 'bow'], coefficients)
            class FakeRobot:
                def fk(self, joints, full=False):
                    pose = np.eye(4)
                    pose[:3, :3] = tool_rotation(reference.evaluate(joints[0], 1), reference.normal(joints[0]))
                    pose[:3, 3] = reference.position(joints[0], .0015)+[0, joints[1], 0]
                    return pose
            with self.assertRaisesRegex(RuntimeError, 'Independent trace audit'):
                audit_trace(path, plan, FakeRobot())
            audit = json.loads((path/'trace_audit.json').read_text())
            self.assertGreater(audit['position_error_mm'], .1)
            self.assertTrue(audit['actual_time_interpolant'])

if __name__=='__main__': unittest.main()
