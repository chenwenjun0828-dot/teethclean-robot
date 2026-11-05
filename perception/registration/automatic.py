"""FPFH + RANSAC + point-to-plane ICP without landmarks or pose priors.

Both returned transforms map oral-scan millimetres to camera millimetres.
Optimization and fitness evaluation use the partial camera cloud as source.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from time import perf_counter

import numpy as np
import open3d as o3d

from ..frames import validate_transform


@dataclass(frozen=True)
class AutomaticRegistrationConfig:
    voxel_mm: float = 0.5
    normal_radius_mm: float | None = None
    fpfh_radius_mm: float | None = None
    ransac_distance_mm: float | None = None
    icp_distance_mm: float | None = None
    refinement_voxel_mm: float | None = None
    outlier_neighbors: int = 20
    outlier_std_ratio: float = 2.0
    minimum_points: int = 200
    ransac_iterations: int = 100000
    ransac_confidence: float = 0.999
    edge_length_ratio: float = 0.9
    icp_iterations: int = 50
    seed: int = 0

    def __post_init__(self):
        for name in ("voxel_mm", "outlier_std_ratio"):
            self._positive(name, getattr(self, name))
        for name in ("normal_radius_mm", "fpfh_radius_mm", "ransac_distance_mm",
                     "icp_distance_mm"):
            value = getattr(self, name)
            if value is not None:
                self._positive(name, value)
        if self.refinement_voxel_mm is not None:
            self._positive("refinement_voxel_mm", self.refinement_voxel_mm, allow_zero=True)
        for name, minimum in (("outlier_neighbors", 2), ("minimum_points", 3),
                              ("ransac_iterations", 1), ("icp_iterations", 1), ("seed", 0)):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
                raise ValueError(f"{name} must be an integer >= {minimum}")
        if self.seed > 2147483647:
            raise ValueError("seed must fit a signed 32-bit integer")
        for name in ("ransac_confidence", "edge_length_ratio"):
            value = getattr(self, name)
            self._positive(name, value)
            if value > 1:
                raise ValueError(f"{name} must be <= 1")
        if self.effective()["fpfh_radius_mm"] < self.effective()["normal_radius_mm"]:
            raise ValueError("FPFH radius must be >= normal radius")

    @staticmethod
    def _positive(name, value, allow_zero=False):
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not np.isfinite(value) or value < 0
                or (not allow_zero and value == 0)):
            raise ValueError(f"{name} must be {'nonnegative' if allow_zero else 'positive'} and finite")

    def effective(self):
        """Resolve radius defaults after voxel-size overrides, for reproducible reports."""
        values = asdict(self)
        for name, factor in (("normal_radius_mm", 2), ("fpfh_radius_mm", 5),
                             ("ransac_distance_mm", 1.5), ("icp_distance_mm", 1),
                             ("refinement_voxel_mm", 0.5)):
            if values[name] is None:
                values[name] = self.voxel_mm * factor
        return values

    @classmethod
    def from_mapping(cls, values):
        if not isinstance(values, dict):
            raise ValueError("automatic_registration settings must be a JSON object")
        try:
            return cls(**values)
        except TypeError as error:
            raise ValueError(f"Invalid automatic_registration settings: {error}") from error


def _check_cloud(cloud, name, minimum_points):
    points = np.asarray(cloud.points)
    if len(points) < minimum_points:
        raise ValueError(f"{name}: too few points ({len(points)} < {minimum_points})")
    if not np.isfinite(points).all():
        raise ValueError(f"{name}: point cloud contains non-finite coordinates")
    if np.linalg.matrix_rank(points - points.mean(axis=0)) < 2:
        raise ValueError(f"{name}: point cloud geometry is degenerate")


def _with_normals(cloud, voxel_mm, radius_mm, minimum_points, name):
    result = cloud.voxel_down_sample(voxel_mm) if voxel_mm > 0 else o3d.geometry.PointCloud(cloud)
    _check_cloud(result, name, minimum_points)
    # Do not inherit normals whose origin, scale, or orientation is unknown.
    result.normals = o3d.utility.Vector3dVector()
    result.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_mm, max_nn=30))
    return result


def preprocess_for_fpfh(cloud, options=None, name="cloud"):
    """Return cleaned full cloud, normal-bearing downsample, FPFH, and point counts."""
    options = options if options is not None else AutomaticRegistrationConfig()
    values = options.effective()
    _check_cloud(cloud, name, options.minimum_points)
    cleaned, _ = o3d.geometry.PointCloud(cloud).remove_statistical_outlier(
        nb_neighbors=min(options.outlier_neighbors, len(cloud.points) - 1),
        std_ratio=options.outlier_std_ratio,
    )
    _check_cloud(cleaned, f"{name} after outlier removal", options.minimum_points)
    downsampled = _with_normals(cleaned, options.voxel_mm, values["normal_radius_mm"],
                                options.minimum_points, f"{name} after downsampling")
    feature = o3d.pipelines.registration.compute_fpfh_feature(
        downsampled, o3d.geometry.KDTreeSearchParamHybrid(
            radius=values["fpfh_radius_mm"], max_nn=100))
    data = np.asarray(feature.data)
    if not np.isfinite(data).all() or not np.any(data):
        raise ValueError(f"{name}: FPFH features are empty or non-finite; check voxel/radii")
    counts = {"input_points": len(cloud.points), "cleaned_points": len(cleaned.points),
              "feature_points": len(downsampled.points)}
    return cleaned, downsampled, feature, counts


def _check_result(result, stage):
    validate_transform(result.transformation)
    if (not np.isfinite(result.fitness) or not np.isfinite(result.inlier_rmse)
            or result.fitness <= 0 or len(result.correspondence_set) < 3):
        raise ValueError(f"{stage} found insufficient correspondences; automatic registration failed")


def register_clouds_automatic(scan_cloud, camera_cloud, options=None):
    """Return (T_camera_scan, metrics) without an initial pose or selected landmarks.

    ``metrics['T_camera_scan_initial']`` is the RANSAC transform in the same
    scan-to-camera direction. Fitness/RMSE describe camera-to-scan matching.
    Inputs must use millimetres and are never modified.
    """
    options = options if options is not None else AutomaticRegistrationConfig()
    values = options.effective()
    started = perf_counter()
    camera, camera_down, camera_fpfh, camera_counts = preprocess_for_fpfh(
        camera_cloud, options, "camera")
    scan, scan_down, scan_fpfh, scan_counts = preprocess_for_fpfh(scan_cloud, options, "scan")
    registration = o3d.pipelines.registration
    o3d.utility.random.seed(options.seed)
    ransac = registration.registration_ransac_based_on_feature_matching(
        camera_down, scan_down, camera_fpfh, scan_fpfh, True,
        values["ransac_distance_mm"], registration.TransformationEstimationPointToPoint(False), 3,
        [registration.CorrespondenceCheckerBasedOnEdgeLength(options.edge_length_ratio),
         registration.CorrespondenceCheckerBasedOnDistance(values["ransac_distance_mm"])],
        registration.RANSACConvergenceCriteria(options.ransac_iterations, options.ransac_confidence),
    )
    _check_result(ransac, "RANSAC")
    initial = validate_transform(np.linalg.inv(ransac.transformation))
    camera_fine = _with_normals(camera, values["refinement_voxel_mm"],
                                values["normal_radius_mm"], options.minimum_points, "camera ICP")
    scan_fine = _with_normals(scan, values["refinement_voxel_mm"],
                              values["normal_radius_mm"], options.minimum_points, "scan ICP")
    icp = registration.registration_icp(
        camera_fine, scan_fine, values["icp_distance_mm"], ransac.transformation,
        registration.TransformationEstimationPointToPlane(),
        registration.ICPConvergenceCriteria(max_iteration=options.icp_iterations),
    )
    _check_result(icp, "ICP")
    transform = validate_transform(np.linalg.inv(icp.transformation))
    metrics = {
        "algorithm": "fpfh_ransac_point_to_plane_icp",
        "optimization_direction": "camera_to_scan", "icp_direction": "camera_to_scan",
        "fitness_source_frame": "camera_cv", "fitness_target_frame": "oral_scan",
        "fitness": float(icp.fitness), "inlier_rmse_mm": float(icp.inlier_rmse),
        "icp_fine_fitness": float(icp.fitness), "icp_fine_rmse_mm": float(icp.inlier_rmse),
        "ransac_fitness": float(ransac.fitness), "ransac_rmse_mm": float(ransac.inlier_rmse),
        "ransac_correspondences": len(ransac.correspondence_set),
        "icp_correspondences": len(icp.correspondence_set),
        "T_camera_scan_initial": initial.tolist(),
        "source_points": len(camera_fine.points), "target_points": len(scan_fine.points),
        "preprocessing": {"camera": camera_counts, "scan": scan_counts},
        "automatic_registration": values, "elapsed_s": perf_counter() - started,
    }
    return transform, metrics
