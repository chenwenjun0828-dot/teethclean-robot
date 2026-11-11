"""Offline eye-to-hand calibration from robot poses and rigid board poses."""
from __future__ import annotations

import json
from pathlib import Path
import uuid

import cv2
import numpy as np

from ..frames import validate_transform
from ..global_camera import file_hash


def _samples(payload):
    if (payload.get("units") != "m" or payload.get("optical_frame") != "depth"
            or not payload.get("camera_serial")):
        raise ValueError("Calibration requires units=m, optical_frame=depth and camera_serial")
    samples = payload["samples"]
    base_gripper = [validate_transform(item["T_base_gripper_m"]) for item in samples]
    camera_target = [validate_transform(item["T_camera_target_m"]) for item in samples]
    return base_gripper, camera_target


def _average_transform(transforms):
    result = np.eye(4)
    u, _, vt = np.linalg.svd(sum(item[:3, :3] for item in transforms))
    result[:3, :3] = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
    result[:3, 3] = np.mean([item[:3, 3] for item in transforms], axis=0)
    return validate_transform(result)


def _closure_errors(base_gripper, camera_target, base_camera, gripper_target):
    values = [np.linalg.inv(b) @ base_camera @ c for b, c in zip(base_gripper, camera_target)]
    translation = [float(np.linalg.norm(t[:3, 3] - gripper_target[:3, 3]) * 1000) for t in values]
    rotation = [float(np.degrees(np.arccos(np.clip(
        (np.trace(t[:3, :3] @ gripper_target[:3, :3].T) - 1) / 2, -1, 1)))) for t in values]
    return {"samples": len(values), "translation_residuals_mm": translation,
            "rotation_residuals_deg": rotation,
            "translation_rms_mm": float(np.sqrt(np.mean(np.square(translation)))),
            "rotation_rms_deg": float(np.sqrt(np.mean(np.square(rotation))))}


def calibrate_eye_to_hand(payload, validation=None):
    """Board must be rigid on the gripper; camera must be fixed to the base."""
    base_gripper, camera_target = _samples(payload)
    if len(base_gripper) < 3:
        raise ValueError("At least three poses with nonparallel rotations are required")
    axes = [cv2.Rodrigues(b[:3, :3] @ base_gripper[0][:3, :3].T)[0].reshape(3)
            for b in base_gripper[1:]]
    if np.linalg.matrix_rank(np.asarray(axes), tol=1e-3) < 2:
        raise ValueError("Calibration rotations are degenerate; use multiple nonparallel axes")
    # For eye-to-hand, OpenCV's gripper2base input receives base2gripper.
    gripper_base = [np.linalg.inv(value) for value in base_gripper]
    rotation, translation = cv2.calibrateHandEye(
        [b[:3, :3] for b in gripper_base], [b[:3, 3] for b in gripper_base],
        [c[:3, :3] for c in camera_target], [c[:3, 3] for c in camera_target],
        method=cv2.CALIB_HAND_EYE_PARK)
    base_camera = np.eye(4)
    base_camera[:3, :3], base_camera[:3, 3] = rotation, np.asarray(translation).reshape(3)
    validate_transform(base_camera)
    gripper_target = _average_transform([
        np.linalg.inv(b) @ base_camera @ c for b, c in zip(base_gripper, camera_target)])
    report = {
        "schema_version": 1, "calibration_id": uuid.uuid4().hex,
        "method": "opencv_eye_to_hand_park", "units": "m", "optical_frame": "depth",
        "camera_serial": payload["camera_serial"], "T_base_camera_m": base_camera.tolist(),
        "T_gripper_target_m": gripper_target.tolist(),
        "fitting": _closure_errors(base_gripper, camera_target, base_camera, gripper_target),
        "held_out_validation": None, "robot_execution_allowed": False,
    }
    if validation is not None:
        if (validation.get("camera_serial") != payload["camera_serial"]
                or validation.get("optical_frame") != payload["optical_frame"]):
            raise ValueError("Validation camera identity does not match calibration")
        vb, vc = _samples(validation)
        if not vb:
            raise ValueError("Validation sample list is empty")
        if any(np.allclose(b, trained, atol=1e-8, rtol=0) for b in vb for trained in base_gripper):
            raise ValueError("Held-out validation reuses a fitting pose")
        report["held_out_validation"] = _closure_errors(vb, vc, base_camera, gripper_target)
    return report


def load_base_calibration(path, capture_metadata):
    path = Path(path).resolve()
    payload = json.loads(path.read_text(encoding="utf-8-sig"))
    if payload.get("units") != "m":
        raise ValueError("Base calibration translation units must be metres")
    for key in ("camera_serial", "optical_frame"):
        if not payload.get(key) or payload[key] != capture_metadata.get(key):
            raise ValueError(f"Base calibration {key} does not match this capture")
    matrix = validate_transform(payload["T_base_camera_m"])
    provenance = {"path": str(path), "sha256": file_hash(path),
                  "calibration_id": payload.get("calibration_id"),
                  "camera_serial": payload["camera_serial"], "optical_frame": payload["optical_frame"],
                  "held_out_validation": payload.get("held_out_validation")}
    return matrix, provenance
