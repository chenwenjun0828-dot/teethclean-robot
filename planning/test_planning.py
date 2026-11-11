"""Behavior tests for independent normal-tilt planning."""
import unittest
import tempfile
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from perception.regions.surface import three_point_plane, surface_plane
from .pose_search.constraints import tool_rotation, normal_angle, surface_frame, pose_at
from .config import Config


class ThreePointTests(unittest.TestCase):
    def test_plane_uses_three_actual_widely_separated_points(self):
        p = np.array([[x,y,.003] for x in np.linspace(-.004,.004,15) for y in np.linspace(-.006,.006,18)])
        r = three_point_plane(p,[0,0,1],.0001)
        self.assertTrue(r['valid'])
        np.testing.assert_allclose(r['normal_scan'],[0,0,1],atol=1e-12)
        np.testing.assert_array_equal(r['landmarks_scan_m'],p[r['landmark_support_indices']])
        self.assertGreater(r['triangle_area_mm2'],45)
        self.assertLess(r['rms_mm'],1e-10)

    def test_collinear_duplicate_or_nonfinite_support_is_rejected(self):
        for points in (np.zeros((3,3)), np.c_[np.arange(10),np.zeros((10,2))], [[0,0,0],[1,1,1],[np.nan,0,1]]):
            with self.assertRaises(ValueError): three_point_plane(points,[0,0,1],.001)

    def test_nonplanar_tooth_is_not_silently_accepted(self):
        rng = np.random.default_rng(2)
        p = rng.uniform(-.005,.005,(100,3))
        self.assertFalse(three_point_plane(p,[0,0,1],.00001)['valid'])

    def test_rigid_transform_preserves_plane(self):
        p = np.array([[0,0,0],[.01,0,0],[0,.02,0],[.003,.004,0]])
        R = Rotation.from_euler('xyz',[30,20,-40],degrees=True).as_matrix()
        r = three_point_plane(p@R.T+[.2,.3,-.1],R[:,2],.0001)
        np.testing.assert_allclose(r['normal_scan'],R[:,2],atol=1e-12)


class SurfacePlaneTests(unittest.TestCase):
    def test_curved_patch_fits_all_points_instead_of_three_extremes(self):
        p = np.array([[x, y, 20*(x*x+y*y)]
                      for x in np.linspace(-.004, .004, 15)
                      for y in np.linspace(-.006, .006, 18)])
        fitted = surface_plane(p, [0, 0, 1], .001, np.ones(len(p)))
        old = three_point_plane(p, [0, 0, 1], .001)
        np.testing.assert_allclose(fitted['normal_scan'], [0, 0, 1], atol=1e-12)
        self.assertLess(fitted['rms_mm'], old['rms_mm'])
        display = fitted['plane_vertices_scan_m']
        np.testing.assert_allclose((display-fitted['center_scan_m'])@fitted['normal_scan'], 0, atol=1e-12)

    def test_area_weighting_is_invariant_to_duplicate_subdivision_samples(self):
        p = np.array([[0,0,0], [.01,0,.001], [0,.01,0], [.01,.01,.002], [.004,.005,.0002]])
        weights = np.array([1., 2., 3., 4., 5.])
        base = surface_plane(p, [0,0,1], .01, weights)
        subdivided = surface_plane(np.repeat(p, [9,1,1,1,1], axis=0), [0,0,1], .01,
                                   np.r_[np.full(9, 1/9), weights[1:]])
        np.testing.assert_allclose(base['normal_scan'], subdivided['normal_scan'], atol=1e-12)
        np.testing.assert_allclose(base['center_scan_m'], subdivided['center_scan_m'], atol=1e-12)
        transform = Rotation.from_euler('xyz', [30,20,-40], degrees=True).as_matrix()
        moved = surface_plane(p@transform.T+[.2,.3,-.1], transform[:,2], .01, weights)
        np.testing.assert_allclose(moved['normal_scan'], transform@base['normal_scan'], atol=1e-12)

    def test_invalid_and_collinear_area_support_is_rejected(self):
        p = np.array([[0,0,0], [.01,0,0], [0,.01,0]])
        for weights in ([1,0,1], [1,np.nan,1], [1,1]):
            with self.assertRaises(ValueError):
                surface_plane(p, [0,0,1], .001, weights)
        with self.assertRaises(ValueError):
            surface_plane(np.c_[np.arange(10), np.zeros((10,2))], [0,0,1], .001, np.ones(10))


class PoseTests(unittest.TestCase):
    def test_normal_tilt_stays_exact_for_all_rolls_and_oblique_tangents(self):
        n = np.array([0.,1.,0.]); tangent = [1,.4,.8]
        for roll in np.arange(-180,180,15):
            R = tool_rotation(tangent,n,15,roll)
            self.assertAlmostEqual(normal_angle(R,n),15,places=10)
            self.assertLess(R[1,0],0)
            np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(R),1.)

    def test_frame_and_pose_transform_equivariance(self):
        R = Rotation.from_euler('xyz',[.4,-.7,.9]).as_matrix()
        base = tool_rotation([1,0,.2],[0,1,0],15,37)
        np.testing.assert_allclose(tool_rotation(R@[1,0,.2],R[:,1],15,37,gravity=R@[0,0,-1]),R@base,atol=1e-12)
        world = np.eye(4); world[:3,:3]=R; world[:3,3]=[.3,.2,.5]
        pose = pose_at([.01,.02,.03],[1,0,.2],[0,0,1],37,world,Config(planning_mode='continuous'))
        np.testing.assert_allclose(pose[:3,3],R@[.01,.02,.0315]+world[:3,3],atol=1e-12)

    def test_degenerate_frame_rejected(self):
        with self.assertRaises(ValueError): surface_frame([0,0,1],[0,0,1])

    def test_angle_is_from_normal_not_surface(self):
        R=tool_rotation([1,0,0],[0,1,0])
        self.assertAlmostEqual(np.degrees(np.arcsin(abs(R[1,0]))),75.)
        changed=tool_rotation([1,0,0],[0,1,0],25.)
        self.assertAlmostEqual(normal_angle(changed,[0,1,0]),25.)

    def test_configuration_rejects_invalid_geometry_and_speed(self):
        for kwargs in ({'tilt_deg':181},{'standoff_mm':.1},
                       {'speed_mm_s':float('nan')}, {'transfer_speed_mm_s':0.}):
            with self.assertRaises(ValueError): Config(planning_mode='continuous', **kwargs)


class AuditTests(unittest.TestCase):
    def test_independent_audit_accepts_normal_tilt_and_rejects_wrong_angle(self):
        from planning.infrastructure.io import dump
        from planning.infrastructure.trajectory import Trajectory
        from planning.infrastructure.poses import TCP_FRAME_ID
        from planning.pose_search.constraints import AXIS_CONVENTION
        from planning.motion_planning.audit import audit_trace
        from dataclasses import asdict
        from .test_sweep import SweepAuditTests

        class StraightRobot:
            tilt=15.
            azimuth=-60.
            def fk(self,q,full=False):
                pose=np.eye(4)
                pose[:3,:3]=tool_rotation(path.evaluate(q[0],1),path.normal(q[0]),self.tilt,
                                         tilt_azimuth_deg=self.azimuth)
                pose[:3,3]=path.position(q[0],.0015)
                return pose

        with tempfile.TemporaryDirectory() as temp:
            directory=Path(temp)
            path,trajectory,report=SweepAuditTests().setup_reference(directory,Config(planning_mode='continuous', tilt_azimuth_deg=-60.))
            robot=StraightRobot()
            audit=audit_trace(directory,trajectory,robot)
            self.assertTrue(audit['passed'])
            self.assertLess(audit['normal_angle_error_deg'],1e-6)
            robot.tilt=30.
            with self.assertRaisesRegex(RuntimeError,'Independent trace audit'):
                audit_trace(directory,trajectory,robot)
            robot.tilt=15.
            robot.azimuth=0.
            audit=audit_trace(directory,trajectory,robot,raise_on_failure=False)
            self.assertFalse(audit['passed'])
            self.assertGreater(audit['normal_angle_error_deg'],10.)
            self.assertGreater(audit['frame_error_deg'],10.)

    def test_execution_loader_rejects_old_planner_and_changed_planes(self):
        from planning.infrastructure.io import dump,fingerprint
        from .motion_planning.execution import load_verified
        with tempfile.TemporaryDirectory() as temp:
            directory=Path(temp)
            dump(directory/'request.json',{})
            dump(directory/'planning_report.json',{'planner':'old-planner'})
            with self.assertRaisesRegex(ValueError,'Expected planning'):
                load_verified(directory)
            dump(directory/'tooth_planes.json',{'normal':[0,0,1]})
            saved=fingerprint(directory/'tooth_planes.json')
            dump(directory/'planning_report.json',{'planner':'planning','planes_file':saved})
            dump(directory/'tooth_planes.json',{'normal':[1,0,0]})
            with self.assertRaisesRegex(ValueError,'planes changed'):
                load_verified(directory)


if __name__ == '__main__': unittest.main()
