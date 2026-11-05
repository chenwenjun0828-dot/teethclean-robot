"""Metric camera geometry. Public point arrays use millimetres in CV axes."""
from __future__ import annotations
import numpy as np

GL_TO_CV = np.diag([1., -1., -1.])


def validate_transform(transform):
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("Transform must be a finite 4x4 matrix")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6):
        raise ValueError("Transform must contain a proper rotation, without scale")
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError("Invalid homogeneous transform bottom row")
    return transform


def intrinsics_from_fovy(width, height, fovy_deg):
    if width <= 0 or height <= 0 or not np.isfinite(fovy_deg) or not 0 < fovy_deg < 180:
        raise ValueError("Invalid image size or camera field of view")
    focal = height / (2 * np.tan(np.deg2rad(fovy_deg) / 2))
    return {"fx": float(focal), "fy": float(focal), "cx": (width-1)/2, "cy": (height-1)/2}


def depth_to_points_mm(depth_m, rgb, intrinsics, valid_mask):
    depth = np.asarray(depth_m, dtype=float)
    rgb = np.asarray(rgb)
    mask = np.asarray(valid_mask, dtype=bool)
    if depth.ndim != 2 or rgb.shape != depth.shape + (3,) or mask.shape != depth.shape:
        raise ValueError("RGB, mask and depth shapes disagree")
    values = np.asarray([intrinsics[key] for key in ("fx","fy","cx","cy")],float)
    if not np.isfinite(values).all() or (values[:2] <= 0).any():
        raise ValueError("Invalid camera intrinsics")
    v,u = np.nonzero(mask & np.isfinite(depth) & (depth > 0))
    z = depth[v,u] * 1000
    points = np.column_stack(((u-values[2])*z/values[0], (v-values[3])*z/values[1], z))
    return points, rgb[v,u].astype(float)/255


def scan_to_camera_true_mm(data, jaw, camera_name):
    if jaw not in ("mandibular","maxillary"):
        raise ValueError("Unknown jaw")
    body = data.body("dental_"+jaw)
    camera = data.camera(camera_name)
    cv_from_world = GL_TO_CV @ camera.xmat.reshape(3,3).T
    result = np.eye(4)
    result[:3,:3] = cv_from_world @ body.xmat.reshape(3,3)
    result[:3,3] = 1000 * cv_from_world @ (body.xpos-camera.xpos)
    return validate_transform(result)


def camera_points_to_world_m(points_mm, data, camera_name):
    camera = data.camera(camera_name)
    world_from_cv = camera.xmat.reshape(3,3) @ GL_TO_CV
    return np.asarray(points_mm)/1000 @ world_from_cv.T + camera.xpos


def camera_world_transform_m(data, camera_name):
    camera = data.camera(camera_name)
    result = np.eye(4)
    result[:3,:3] = camera.xmat.reshape(3,3) @ GL_TO_CV
    result[:3,3] = camera.xpos
    return result
