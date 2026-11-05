"""Rigid-registration math used by the dental coarse-registration workflow."""

from __future__ import annotations

from itertools import combinations

import numpy as np


def _as_points(points: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(points, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3)")
    if len(array) < 3:
        raise ValueError(f"{name} must contain at least 3 points")
    if not np.isfinite(array).all():
        raise ValueError(f"{name} contains NaN or infinity")
    return array


def estimate_rigid_transform(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Return T_target_source such that target ~= transform(source, T)."""
    source = _as_points(source, "source")
    target = _as_points(target, "target")
    if source.shape != target.shape:
        raise ValueError("source and target must have the same shape")

    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    source_zero = source - source_center
    target_zero = target - target_center

    covariance = source_zero.T @ target_zero
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T

    translation = target_center - rotation @ source_center
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    transform = np.asarray(transform, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if transform.shape != (4, 4):
        raise ValueError("transform must have shape (4, 4)")
    return points @ transform[:3, :3].T + transform[:3, 3]


def registration_errors(
    source: np.ndarray, target: np.ndarray, transform: np.ndarray
) -> np.ndarray:
    return np.linalg.norm(transform_points(source, transform) - target, axis=1)


def robust_rigid_transform(
    source: np.ndarray,
    target: np.ndarray,
    rejection_threshold_mm: float = 2.0,
    max_iterations: int = 5,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Estimate a rigid transform and iteratively reject landmark pairs whose
    residual exceeds rejection_threshold_mm.

    Returns transform, inlier mask, and residuals for all original pairs.
    """
    source = _as_points(source, "source")
    target = _as_points(target, "target")
    if source.shape != target.shape:
        raise ValueError("source and target must have the same shape")
    if rejection_threshold_mm <= 0:
        raise ValueError("rejection_threshold_mm must be positive")

    # Manual landmark sets are small, so exhaustively test three-point models.
    # This prevents one badly clicked cusp from pulling the initial fit away.
    best_inliers = None
    best_transform = None
    best_score = (-1, float("-inf"))
    for indices in combinations(range(len(source)), 3):
        subset = np.asarray(indices)
        centered = source[subset] - source[subset].mean(axis=0)
        if np.linalg.matrix_rank(centered) < 2:
            continue
        candidate = estimate_rigid_transform(source[subset], target[subset])
        candidate_residuals = registration_errors(source, target, candidate)
        candidate_inliers = candidate_residuals <= rejection_threshold_mm
        count = int(candidate_inliers.sum())
        if count < 3:
            continue
        rms = float(
            np.sqrt(np.mean(candidate_residuals[candidate_inliers] ** 2))
        )
        score = (count, -rms)
        if score > best_score:
            best_score = score
            best_inliers = candidate_inliers
            best_transform = candidate

    if best_inliers is None or best_transform is None:
        raise ValueError("Could not find a valid non-collinear landmark model")

    inliers = best_inliers
    transform = estimate_rigid_transform(source[inliers], target[inliers])

    for _ in range(max_iterations):
        residuals = registration_errors(source, target, transform)
        updated = residuals <= rejection_threshold_mm
        if updated.sum() < 3:
            break
        if np.array_equal(updated, inliers):
            inliers = updated
            break
        inliers = updated
        transform = estimate_rigid_transform(source[inliers], target[inliers])

    residuals = registration_errors(source, target, transform)
    return transform, inliers, residuals
