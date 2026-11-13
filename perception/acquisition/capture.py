"""Persist native depth-camera geometry without simulator dependencies."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import uuid

import numpy as np
from PIL import Image

from ..global_camera import (ensure_empty_directory, file_hash, make_cloud,
                             save_cloud, valid_depth_mask, write_json)


@dataclass
class RGBDFrame:
    rgb: np.ndarray
    depth_raw: np.ndarray
    depth_scale_m: float
    points_mm: np.ndarray
    colors: np.ndarray
    color_valid_mask: np.ndarray
    intrinsics: dict
    metadata: dict

    @property
    def depth_m(self):
        return self.depth_raw.astype(np.float64) * self.depth_scale_m


def depth_roi_mask(depth_m, config):
    mask = valid_depth_mask(depth_m, config["min_depth_mm"], config["max_depth_mm"])
    height, width = mask.shape
    roi = config.get("roi")
    if roi is None:
        roi = [0, 0, width, height]
    values = np.asarray(roi)
    if (values.shape != (4,) or not np.isfinite(values).all()
            or not np.equal(values, values.astype(int)).all()):
        raise ValueError("ROI must contain four integer depth-image coordinates")
    x0, y0, x1, y1 = map(int, roi)
    if not 0 <= x0 < x1 <= width or not 0 <= y0 < y1 <= height:
        raise ValueError("ROI is empty or outside the native depth image")
    region = np.zeros(mask.shape, bool)
    region[y0:y1, x0:x1] = True
    return mask & region, [x0, y0, x1, y1]


def save_real_capture(output_dir, frame: RGBDFrame, jaw, config):
    if jaw not in ("mandibular", "maxillary"):
        raise ValueError("Real captures must select one jaw; capture moving jaws separately")
    if frame.depth_raw.dtype != np.uint16 or frame.depth_raw.ndim != 2:
        raise ValueError("Raw depth must be a two-dimensional Z16 image")
    if not np.isfinite(frame.depth_scale_m) or frame.depth_scale_m <= 0:
        raise ValueError("Invalid measured depth scale")
    shape = frame.depth_raw.shape
    if (frame.points_mm.shape != shape + (3,) or frame.colors.shape != shape + (3,)
            or frame.color_valid_mask.shape != shape):
        raise ValueError("Point/color maps must use the native depth image grid")
    if (frame.rgb.ndim != 3 or frame.rgb.shape[2] != 3 or frame.rgb.dtype != np.uint8
            or not np.isfinite(frame.colors).all()
            or np.any((frame.colors < 0) | (frame.colors > 1))):
        raise ValueError("Invalid RGB image or normalized point colors")
    depth = frame.depth_m
    raw_mask = ((frame.depth_raw > 0) & np.isfinite(frame.points_mm).all(axis=2)
                & (frame.points_mm[:, :, 2] > 0))
    if not np.allclose(frame.points_mm[:, :, 2][raw_mask], depth[raw_mask] * 1000,
                       atol=.01, rtol=1e-5):
        raise ValueError("SDK point geometry and measured depth scale disagree")
    mask, roi = depth_roi_mask(depth, config)
    mask &= raw_mask
    minimum = config.get("capture_minimum_points", 200)
    neighbors = config.get("capture_outlier_neighbors", 20)
    ratio = config.get("capture_outlier_std_ratio", 2.)
    if (isinstance(minimum, bool) or not isinstance(minimum, int) or minimum < 3
            or isinstance(neighbors, bool) or not isinstance(neighbors, int)
            or neighbors < 0 or neighbors == 1 or not np.isfinite(ratio) or ratio <= 0):
        raise ValueError("Invalid capture minimum points or statistical filter settings")
    if int(mask.sum()) < minimum:
        raise ValueError("Too few dental points; check ROI, depth range, exposure and distance")
    cloud = make_cloud(frame.points_mm[mask], frame.colors[mask])
    if neighbors:
        cloud, indices = cloud.remove_statistical_outlier(
            nb_neighbors=min(neighbors, len(cloud.points) - 1), std_ratio=ratio)
        filtered = np.zeros(mask.size, bool)
        filtered[np.flatnonzero(mask)[np.asarray(indices, int)]] = True
        filtered = filtered.reshape(shape)
    else:
        filtered = mask.copy()
    if int(filtered.sum()) < minimum:
        raise ValueError("Too few dental points after capture outlier removal")
    out = ensure_empty_directory(output_dir)
    Image.fromarray(frame.rgb).save(out / "rgb.png")
    arrays = {"depth_raw": frame.depth_raw, "depth_m": depth,
              "points_camera_mm": frame.points_mm, "point_colors": frame.colors,
              "color_valid_mask": frame.color_valid_mask,
              "roi_mask": mask, "valid_mask": filtered}
    for name, array in arrays.items():
        np.save(out / (name + ".npy"), array, allow_pickle=False)
    save_cloud(out / "camera_raw_mm.ply", make_cloud(frame.points_mm[raw_mask], frame.colors[raw_mask]))
    save_cloud(out / "camera_roi_mm.ply", make_cloud(frame.points_mm[mask], frame.colors[mask]))
    save_cloud(out / "camera_mm.ply", cloud)
    preview = np.zeros(shape, np.uint8)
    preview[mask] = np.clip(255 * (depth[mask] * 1000 - config["min_depth_mm"])
                            / (config["max_depth_mm"] - config["min_depth_mm"]), 0, 255).astype(np.uint8)
    Image.fromarray(preview).save(out / "depth_preview.png")
    overlay = np.rint(frame.colors * 255).astype(np.uint8)
    overlay[~filtered] //= 4
    Image.fromarray(overlay).save(out / "selection_preview.png")
    files = sorted(path.name for path in out.iterdir() if path.is_file())
    capture_id = uuid.uuid4().hex
    metadata = {
        **frame.metadata, "schema_version": 1, "data_source": "realsense",
        "capture_id": capture_id, "acquisition_id": frame.metadata.get("acquisition_id", capture_id),
        "jaw": jaw, "units": "mm",
        "frame": "camera_cv", "optical_frame": "depth", "depth_unit": "m",
        "camera": "d405_depth_optical", "depth_scale_m": float(frame.depth_scale_m),
        "intrinsics": frame.intrinsics, "resolution": [shape[1], shape[0]],
        "mask_mode": "depth_roi", "roi": roi,
        "min_depth_mm": config["min_depth_mm"], "max_depth_mm": config["max_depth_mm"],
        "raw_valid_points": int(raw_mask.sum()), "roi_points": int(mask.sum()),
        "valid_points": len(cloud.points),
        "roi_depth_valid_fraction": float(mask.sum() / ((roi[2]-roi[0])*(roi[3]-roi[1]))),
        "source_hashes": {}, "reference_line_hashes": {},
        "saved_at_utc": datetime.now(timezone.utc).isoformat(), "config": config,
        "file_hashes": {name: file_hash(out / name) for name in files},
    }
    write_json(out / "capture_metadata.json", metadata)
    return metadata
