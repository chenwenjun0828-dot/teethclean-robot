import unittest
import numpy as np
import open3d as o3d
from perception.registration import register_clouds, coarse_from_landmarks, load_line_csv
from perception.registration.registration_math import estimate_rigid_transform, robust_rigid_transform, transform_points


def asymmetric_cloud():
    rng=np.random.default_rng(13)
    direction=rng.normal(size=(3500,3))
    direction/=np.linalg.norm(direction,axis=1)[:,None]
    points=direction*np.array([25.,17.,12.])
    points[:,2]+=3*np.sin(points[:,0]/8)+.01*points[:,0]*points[:,1]
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    return cloud


class GlobalRegistrationTests(unittest.TestCase):
    def test_kabsch_and_robust_math_recover_transform(self):
        rng=np.random.default_rng(4)
        source=rng.normal(size=(8,3))*15
        truth=np.eye(4)
        truth[:3,:3]=o3d.geometry.get_rotation_matrix_from_xyz([.1,-.08,.2])
        truth[:3,3]=[10,-7,150]
        target=transform_points(source,truth)
        np.testing.assert_allclose(estimate_rigid_transform(source,target),truth,atol=1e-10)
        target[-1]+=[12,-9,7]
        estimate,inliers,_=robust_rigid_transform(source,target,1.)
        self.assertEqual(inliers.sum(),7)
        np.testing.assert_allclose(estimate,truth,atol=1e-9)

    def test_partial_cloud_icp_returns_scan_to_camera(self):
        scan=asymmetric_cloud()
        points=np.asarray(scan.points)
        truth=np.eye(4)
        truth[:3,:3]=o3d.geometry.get_rotation_matrix_from_xyz([.1,-.08,.2])
        truth[:3,3]=[10,-7,150]
        camera=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(
            transform_points(points[points[:,2]>-5],truth)))
        initial=truth.copy();initial[:3,3]+=[.6,-.5,.4]
        estimate,metrics=register_clouds(scan,camera,initial,voxel_mm=.35)
        self.assertLess(np.linalg.norm(estimate[:3,3]-truth[:3,3]),.15)
        self.assertGreater(metrics["icp_fine_fitness"],.95)
        with self.assertRaises(ValueError):
            register_clouds(scan,camera,np.diag([2.,1.,1.,1.]))

    def test_landmarks_reject_wrong_count_and_too_few_inliers(self):
        source=np.random.default_rng(12).normal(size=(6,3))*10
        with self.assertRaises(ValueError):
            coarse_from_landmarks(source,source[:5])
        target=source.copy();target[-1]+=[50,50,50]
        with self.assertRaises(ValueError):
            coarse_from_landmarks(source,target)
        with self.assertRaises(ValueError):
            coarse_from_landmarks(np.zeros((6,3)),np.zeros((6,3)))

    def test_empty_cloud_does_not_produce_a_registration(self):
        with self.assertRaises(ValueError):
            register_clouds(o3d.geometry.PointCloud(),o3d.geometry.PointCloud(),np.eye(4))
