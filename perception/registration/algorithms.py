"""Landmark initialization and rigid point-to-plane ICP. No simulator truth input."""
from __future__ import annotations
from pathlib import Path
import numpy as np
import open3d as o3d
from ..frames import validate_transform
from ..global_camera import make_cloud
from .registration_math import robust_rigid_transform


def load_scan_cloud(mesh_paths,voxel_mm=0.0):
    points=[];colors=[]
    for path in mesh_paths:
        mesh=o3d.io.read_triangle_mesh(str(path))
        if not mesh.has_vertices() or not mesh.has_triangles():
            raise ValueError(f"Missing triangle mesh: {path}")
        vertices=np.asarray(mesh.vertices)
        if not np.isfinite(vertices).all():
            raise ValueError("Scan has non-finite vertices")
        points.append(vertices)
        colors.append(np.asarray(mesh.vertex_colors) if mesh.has_vertex_colors() else np.full(vertices.shape,.8))
    if not points:
        raise ValueError("No scan meshes supplied")
    cloud=make_cloud(np.vstack(points),np.vstack(colors))
    return cloud.voxel_down_sample(voxel_mm) if voxel_mm>0 else cloud


def load_line_csv(path):
    data=np.genfromtxt(path,delimiter=",",names=True,dtype=float,encoding="utf-8-sig")
    if not {"x","y","z"}.issubset(data.dtype.names or ()):
        raise ValueError(f"Reference line requires x,y,z columns: {path}")
    points=np.column_stack([np.atleast_1d(data[key]) for key in ("x","y","z")])
    if not len(points) or not np.isfinite(points).all():
        raise ValueError("Reference line must contain finite points")
    return points


def coarse_from_landmarks(source,target,minimum_points=6,reject_mm=2.0):
    source=np.asarray(source,float);target=np.asarray(target,float)
    if source.ndim!=2 or source.shape[1]!=3 or target.shape!=source.shape:
        raise ValueError("Landmark arrays must have matching Nx3 shapes")
    if len(source)<minimum_points:
        raise ValueError(f"At least {minimum_points} corresponding landmarks are required")
    transform,inliers,residuals=robust_rigid_transform(source,target,reject_mm)
    if int(inliers.sum())<minimum_points:
        raise ValueError(f"Only {int(inliers.sum())} landmark inliers; select at least {minimum_points} reliable pairs")
    return validate_transform(transform),inliers,residuals


def pick_landmark_pairs(scan_cloud,camera_cloud,minimum_points=6):
    from .landmark_picker import pick_pairs
    print("LEFT: camera cloud; RIGHT: scan. Shift+left click one LEFT point, then its RIGHT match.")
    print("Repeat for at least six pairs. Shift+right/Backspace: undo. Finish & Register or Q: finish.")
    return pick_pairs(scan_cloud,camera_cloud,minimum_points)


def prepare_cloud(cloud,voxel_mm,normal_radius_mm,minimum_points=200):
    result=o3d.geometry.PointCloud(cloud)
    if voxel_mm>0:
        result=result.voxel_down_sample(voxel_mm)
    if len(result.points)<minimum_points:
        raise ValueError(f"Too few points after preprocessing: {len(result.points)} < {minimum_points}")
    if not np.isfinite(np.asarray(result.points)).all():
        raise ValueError("Point cloud contains non-finite coordinates")
    result.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=max(normal_radius_mm,1.),max_nn=40))
    return result


def register_prepared(scan,camera,initial_transform,coarse_mm=8.,fine_mm=3.,iterations=80,direction="camera_to_scan"):
    initial_transform=validate_transform(initial_transform)
    if not 0<fine_mm<=coarse_mm or iterations<1:
        raise ValueError("Invalid ICP distances or iteration limit")
    if direction=="camera_to_scan":
        source,target,initial=camera,scan,np.linalg.inv(initial_transform)
    elif direction=="scan_to_camera":
        source,target,initial=scan,camera,initial_transform
    else:
        raise ValueError("Unknown ICP direction")
    estimation=o3d.pipelines.registration.TransformationEstimationPointToPlane()
    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=1e-7,relative_rmse=1e-7,max_iteration=int(iterations))
    coarse=o3d.pipelines.registration.registration_icp(source,target,coarse_mm,initial,estimation,criteria)
    fine=o3d.pipelines.registration.registration_icp(source,target,fine_mm,coarse.transformation,estimation,criteria)
    if fine.fitness<=0:
        raise ValueError("ICP found no correspondences")
    transform=np.asarray(fine.transformation)
    if direction=="camera_to_scan":
        transform=np.linalg.inv(transform)
    return validate_transform(transform),{
        "icp_direction":direction,"icp_coarse_fitness":float(coarse.fitness),
        "icp_coarse_rmse_mm":float(coarse.inlier_rmse),
        "icp_fine_fitness":float(fine.fitness),"icp_fine_rmse_mm":float(fine.inlier_rmse),
        "source_points":len(source.points),"target_points":len(target.points),
    }


def register_clouds(scan_cloud,camera_cloud,initial_transform,voxel_mm=.7,coarse_mm=8.,fine_mm=3.,iterations=80,direction="camera_to_scan",minimum_points=200):
    validate_transform(initial_transform)
    if voxel_mm<=0:
        raise ValueError("Voxel size must be positive")
    scan=prepare_cloud(scan_cloud,voxel_mm,voxel_mm*6,minimum_points)
    camera=prepare_cloud(camera_cloud,voxel_mm,voxel_mm*8,minimum_points)
    return register_prepared(scan,camera,initial_transform,coarse_mm,fine_mm,iterations,direction)
