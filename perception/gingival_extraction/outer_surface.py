"""Automatic buccal/labial filtering for a complete colored dental arch.

The model is fitted in scan coordinates; it does not assume an X/Y/Z jaw
orientation. It filters existing contour segments without moving or joining them.
"""
from __future__ import annotations

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


def filter_outer_segments(
    mesh: o3d.geometry.TriangleMesh,
    scores: np.ndarray,
    threshold: float,
    segments: list[tuple[np.ndarray, np.ndarray]],
) -> tuple[list[tuple[np.ndarray, np.ndarray]], dict]:
    """Keep contour segments outside the arch and with outward surface normals.

    PCA of tooth-like vertices estimates the occlusal plane. A fitted circle
    locates the inside of the U-shaped arch. Local radial quartiles describe
    the tooth band; the inner quartile provides a conservative inner bound,
    allowing the gum margin to lie inside the projected crown contour.
    """
    vertices = np.asarray(mesh.vertices)
    tooth_mask = np.asarray(scores) < threshold
    teeth = vertices[tooth_mask]
    if len(teeth) < 30:
        raise ValueError('Outer surface filtering needs enough tooth-like vertices')

    origin = teeth.mean(axis=0)
    eigenvalues, eigenvectors = np.linalg.eigh(np.cov(teeth.T))
    if eigenvalues[1] <= 1e-12 or eigenvalues[0] > 0.35 * eigenvalues[1]:
        raise ValueError('Could not estimate the occlusal plane for outer surface filtering')
    plane = eigenvectors[:, 1:]
    tooth_xy = (teeth - origin) @ plane
    circle_system = np.column_stack((2.0 * tooth_xy, np.ones(len(teeth))))
    solution, _, rank, _ = np.linalg.lstsq(
        circle_system, np.sum(tooth_xy**2, axis=1), rcond=None)
    if rank < 3:
        raise ValueError('Could not fit a dental arch for outer surface filtering')
    center = solution[:2]
    tooth_radial = tooth_xy - center
    tooth_radius = np.linalg.norm(tooth_radial, axis=1)
    if np.any(tooth_radius < 1e-8):
        raise ValueError('Dental arch center overlaps tooth geometry')
    tooth_angles = np.arctan2(tooth_radial[:, 1], tooth_radial[:, 0])
    # An isolated tooth or short scan fragment cannot identify inside/outside.
    angle_gaps = np.diff(np.r_[np.sort(tooth_angles), np.min(tooth_angles) + 2*np.pi])
    if 2*np.pi - angle_gaps.max() < np.pi / 2:
        raise ValueError('Outer surface filtering requires a complete or mostly complete dental arch')

    angles = np.linspace(-np.pi, np.pi, 180, endpoint=False)
    quartiles = np.full((len(angles), 2), np.nan)
    for i, angle in enumerate(angles):
        difference = np.arctan2(np.sin(tooth_angles-angle), np.cos(tooth_angles-angle))
        local = tooth_radius[np.abs(difference) <= np.deg2rad(6)]
        if len(local) >= 10:
            quartiles[i] = np.percentile(local, [25, 50])
    valid = np.isfinite(quartiles[:, 0])
    if np.count_nonzero(valid) < 20:
        raise ValueError('Insufficient dental arch coverage for outer surface filtering')

    def radial_boundary(query_angles: np.ndarray, column: int) -> np.ndarray:
        return np.interp(query_angles, angles[valid], quartiles[valid, column], period=2*np.pi)

    mesh.compute_vertex_normals()
    normals = np.asarray(mesh.vertex_normals)
    tooth_alignment = np.sum((normals[tooth_mask] @ plane) * tooth_radial, axis=1) / tooth_radius
    # Correct globally reversed triangle winding using both sides of the tooth band.
    evidence = tooth_alignment * (tooth_radius - radial_boundary(tooth_angles, 1))
    confidence = float(evidence.sum() / max(np.abs(evidence).sum(), 1e-12))
    if abs(confidence) < 0.1:
        raise ValueError('Ambiguous mesh normal orientation for outer surface filtering')
    normal_sign = 1.0 if confidence > 0 else -1.0

    report = {
        'surface': 'outer',
        'method': 'occlusal_pca_arch_radial_band_and_surface_normals',
        'occlusal_plane_normal': eigenvectors[:, 0].tolist(),
        'arch_center_mm': (origin + plane @ center).tolist(),
        'normal_orientation_sign': normal_sign,
        'normal_orientation_confidence': abs(confidence),
        'minimum_outward_normal_projection': 0.05,
        'radial_band_percentile': 25,
        'angular_window_degrees': 6,
        'input_segments': len(segments),
    }
    if not segments:
        report['kept_segments'] = 0
        report['removed_segments'] = 0
        return [], report

    ends = np.asarray(segments, dtype=float)
    samples = np.concatenate((ends, ends.mean(axis=1, keepdims=True)), axis=1)
    flat = samples.reshape(-1, 3)
    radial = (flat - origin) @ plane - center
    radius = np.linalg.norm(radial, axis=1)
    angle = np.arctan2(radial[:, 1], radial[:, 0])
    _, nearest = cKDTree(vertices).query(flat)
    alignment = normal_sign * np.sum((normals[nearest] @ plane) * radial, axis=1) / np.maximum(radius, 1e-12)
    outside = (radius >= radial_boundary(angle, 0)) & (alignment > 0.05)
    # Test endpoints AND midpoint: no edge is allowed to bridge a removed side.
    keep = outside.reshape(-1, 3).all(axis=1)
    filtered = [segment for segment, accepted in zip(segments, keep) if accepted]
    report['kept_segments'] = len(filtered)
    report['removed_segments'] = len(segments) - len(filtered)
    return filtered, report
