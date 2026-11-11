"""Withheld correspondences, configurable gates and static-scene repeatability."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ..frames import validate_transform
from ..global_camera import file_hash, load_capture, write_json
from .registration_math import transform_points


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def load_registration_result(directory):
    directory = Path(directory).resolve()
    report = read_json(directory / "registration_report.json")
    capture = load_capture(report["capture_dir"])["metadata"]
    if capture["capture_id"] != report["capture_id"] or capture["jaw"] != report["jaw"]:
        raise ValueError("Registration and capture identities disagree")
    if report.get("data_source") == "realsense":
        for key in ("camera_serial", "optical_frame"):
            if capture.get(key) != report.get(key):
                raise ValueError("Registration camera identity changed")
        if "T_camera_scan_estimated.npy" not in report.get("artifact_hashes", {}):
            raise ValueError("Real registration is missing its transform fingerprint")
    for jaw, digest in report["source_hashes"].items():
        if file_hash(report["config"][jaw + "_mesh"]) != digest:
            raise ValueError("Scan model changed since registration")
    for name, digest in report.get("artifact_hashes", {}).items():
        if Path(name).name != name or file_hash(directory / name) != digest:
            raise ValueError(f"Registration artifact integrity check failed: {name}")
    transform = validate_transform(np.load(directory / "T_camera_scan_estimated.npy", allow_pickle=False))
    return report, transform


def evaluate_checkpoints(transform, payload, report, fitting_pairs=None):
    if payload.get("capture_id") != report["capture_id"] or payload.get("source_hashes") != report["source_hashes"]:
        raise ValueError("Checkpoints belong to a different capture or scan")
    if payload.get("independent_of_registration") is not True:
        raise ValueError("Checkpoints must be declared independent of registration")
    source = np.asarray(payload["source_scan_mm"], float)
    target = np.asarray(payload["target_camera_mm"], float)
    if (source.ndim != 2 or source.shape[1] != 3 or len(source) < 3
            or source.shape != target.shape or not np.isfinite(source).all()
            or not np.isfinite(target).all()):
        raise ValueError("Checkpoints require at least three finite paired 3D points in mm")
    if (np.linalg.matrix_rank(source - source.mean(axis=0)) < 2
            or np.linalg.matrix_rank(target - target.mean(axis=0)) < 2):
        raise ValueError("Checkpoints must cover a noncollinear region")
    if fitting_pairs is not None:
        for points, key in ((source, "source_scan_mm"), (target, "target_camera_mm")):
            fitting = np.asarray(fitting_pairs[key], float)
            if np.any(np.linalg.norm(points[:, None] - fitting[None, :], axis=2) < 1e-6):
                raise ValueError("A withheld checkpoint reuses a registration landmark")
    predicted = transform_points(source, validate_transform(transform))
    errors = np.linalg.norm(predicted - target, axis=1)
    return {"count": len(source), "mean_mm": float(errors.mean()),
            "rms_mm": float(np.sqrt(np.mean(errors ** 2))),
            "p95_mm": float(np.percentile(errors, 95)), "max_mm": float(errors.max()),
            "errors_mm": errors.tolist(), "predicted_camera_mm": predicted.tolist(),
            "measurement_method": payload.get("measurement_method", "user_supplied_correspondences"),
            "independence": "user_declared_with_exact_fitting_point_reuse_check",
            "measurement_uncertainty_included": False}


def quality_assessment(metrics, checkpoints, limits):
    known = {"min_fitness", "max_inlier_rmse_mm", "max_checkpoint_p95_mm"}
    if set(limits) - known:
        raise ValueError(f"Unknown quality limits: {sorted(set(limits) - known)}")
    checks = {}
    for key, limit in limits.items():
        if limit is None:
            continue
        if isinstance(limit, bool) or not np.isfinite(limit) or limit <= 0 or (key == "min_fitness" and limit > 1):
            raise ValueError(f"Invalid quality limit: {key}")
        if key == "min_fitness":
            value = metrics.get("fitness", metrics.get("icp_fine_fitness"))
        elif key == "max_inlier_rmse_mm":
            value = metrics.get("inlier_rmse_mm", metrics.get("icp_fine_rmse_mm"))
        else:
            value = None if checkpoints is None else checkpoints["p95_mm"]
        passed = None if value is None else bool(value >= limit if key == "min_fitness" else value <= limit)
        checks[key] = {"value": value, "limit": limit, "passed": passed}
    if not checks:
        status = "not_configured"
    elif any(check["passed"] is False for check in checks.values()):
        status = "failed"
    elif any(check["passed"] is None for check in checks.values()):
        status = "not_evaluable"
    else:
        status = "passed_configured_limits"
    return {"status": status, "checks": checks, "tooth_identity_verified": False,
            "robot_execution_allowed": False}


def evaluate_registration(registration_dir, checkpoints_path=None, limits=None, output_path=None):
    directory = Path(registration_dir).resolve()
    report, transform = load_registration_result(directory)
    matrix_path = directory / "T_camera_scan_estimated.npy"
    checkpoints = None
    if checkpoints_path is not None:
        pairs_path = directory / "landmark_correspondences.json"
        pairs = read_json(pairs_path) if pairs_path.exists() else None
        checkpoints = evaluate_checkpoints(transform, read_json(checkpoints_path), report, pairs)
    limits = report.get("config", {}).get("quality", {}) if limits is None else limits
    result = {
        "schema_version": 1, "registration_dir": str(directory),
        "capture_id": report["capture_id"], "source_hashes": report["source_hashes"],
        "transform_sha256": file_hash(matrix_path),
        "checkpoint_file_sha256": file_hash(checkpoints_path) if checkpoints_path else None,
        "fitting": {key: report.get(key) for key in (
            "fitness", "inlier_rmse_mm", "icp_fine_fitness", "icp_fine_rmse_mm", "elapsed_s", "registration_wall_time_s")},
        "checkpoints": checkpoints, "quality": quality_assessment(report, checkpoints, limits),
        "robot_execution_allowed": False,
    }
    target = Path(output_path) if output_path is not None else directory / "validation_report.json"
    if target.exists():
        raise FileExistsError(f"Validation report already exists: {target}; choose a new --out")
    write_json(target, result)
    return result


def repeatability_report(registration_dirs, static_scene=False):
    if not static_scene:
        raise ValueError("Repeatability requires an explicitly declared static scene")
    directories = [Path(path).resolve() for path in registration_dirs]
    if len(directories) < 2 or len(set(directories)) != len(directories):
        raise ValueError("Provide at least two distinct registration result directories")
    loaded = [load_registration_result(path) for path in directories]
    reports = [item[0] for item in loaded]
    first = reports[0]
    identity_keys = ("jaw", "source_hashes", "camera_serial", "optical_frame", "units", "data_source")
    if first.get("data_source") != "realsense":
        raise ValueError("Static acquisition repeatability requires real-camera reports")
    for report in reports:
        if any(report.get(key) != first.get(key) for key in identity_keys):
            raise ValueError("Repeatability results use different scans, jaws, cameras or frames")
    transforms = [item[1] for item in loaded]
    translation = [float(np.linalg.norm(t[:3, 3] - transforms[0][:3, 3])) for t in transforms]
    rotation = [float(np.degrees(np.arccos(np.clip(
        (np.trace(t[:3, :3] @ transforms[0][:3, :3].T) - 1) / 2, -1, 1)))) for t in transforms]
    captures = len({report["capture_id"] for report in reports})
    acquisitions = len({report.get("acquisition_id", report["capture_id"]) for report in reports})
    return {"schema_version": 1, "static_scene_declared": True,
            "reference_registration": str(directories[0]), "runs": len(reports),
            "unique_captures": captures,
            "unique_acquisitions": acquisitions,
            "kind": "algorithm_or_preprocessing_repeatability" if acquisitions == 1 else "acquisition_and_algorithm_repeatability",
            "translation_from_first_mm": translation, "rotation_from_first_deg": rotation,
            "translation_rms_from_first_mm": float(np.sqrt(np.mean(np.square(translation)))),
            "rotation_rms_from_first_deg": float(np.sqrt(np.mean(np.square(rotation)))),
            "is_absolute_accuracy": False, "robot_execution_allowed": False}
