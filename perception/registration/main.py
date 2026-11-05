"""Automatically register an oral-scan PLY/PCD to a local camera PLY/PCD (mm)."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys

os.environ.setdefault("OMP_NUM_THREADS", "4")
import numpy as np
import open3d as o3d

from ..global_camera import ensure_empty_directory, file_hash, save_cloud, write_json
from .automatic import AutomaticRegistrationConfig, register_clouds_automatic
from .registration_workflow import save_overlay_png, save_registration_transforms


def build_argparser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scan", required=True, type=Path, help="Complete oral-scan point cloud in mm")
    parser.add_argument("--camera", required=True, type=Path, help="Local camera point cloud in mm")
    parser.add_argument("--out", required=True, type=Path, help="Empty result directory")
    parser.add_argument("--settings", type=Path, help="JSON object of automatic registration options")
    parser.add_argument("--no-view", action="store_true", help="Save results without opening the 3D viewer")
    for name, default in asdict(AutomaticRegistrationConfig()).items():
        parser.add_argument("--" + name.replace("_", "-"), dest=name,
                            type=int if isinstance(default, int) else float, default=None,
                            help=f"Override {name}; default {default if default is not None else 'derived from voxel_mm'}")
    return parser


def run_point_cloud_registration(scan_path, camera_path, output_dir, options=None, show_view=False):
    """Save T_camera_scan and the complete oral scan transformed to camera space."""
    scan_path, camera_path = Path(scan_path).resolve(), Path(camera_path).resolve()
    for path in (scan_path, camera_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    out = ensure_empty_directory(output_dir)
    scan = o3d.io.read_point_cloud(str(scan_path))
    camera = o3d.io.read_point_cloud(str(camera_path))
    estimate, metrics = register_clouds_automatic(scan, camera, options)
    save_registration_transforms(out, np.asarray(metrics["T_camera_scan_initial"]), estimate)
    registered = o3d.geometry.PointCloud(scan).transform(estimate)
    save_cloud(out / "scan_registered_camera_mm.ply", registered)
    write_json(out / "registration_report.json", {
        "input_kind": "point_cloud_pair", "registration_method": "automatic",
        "scan_path": str(scan_path), "camera_path": str(camera_path),
        "input_hashes": {"scan": file_hash(scan_path), "camera": file_hash(camera_path)},
        "transform_name": "T_camera_scan", "source_frame": "oral_scan", "target_frame": "camera_cv",
        "units": "mm", "landmark_origin": "not_used", "landmark_pairs": 0,
        "landmark_inliers": 0, **metrics,
    })
    save_overlay_png(out / "registration_overlay.png", camera, registered, {})
    print(f"Saved automatic registration to {out}")
    print(f"Camera-to-scan fitness={metrics['fitness']:.4f}, RMSE={metrics['inlier_rmse_mm']:.4f} mm")
    print("T_camera_scan (oral_scan_mm -> camera_cv_mm):")
    print(estimate)
    if show_view:
        o3d.visualization.draw_geometries(
            [camera, registered.paint_uniform_color([.2, .85, .3])],
            window_name="Automatic Dental Registration", width=1280, height=800)
    return out


def main(argv=None):
    args = build_argparser().parse_args(argv)
    try:
        values = json.loads(args.settings.read_text(encoding="utf-8-sig")) if args.settings else {}
        if not isinstance(values, dict):
            raise ValueError("Settings must be a JSON object")
        for name in asdict(AutomaticRegistrationConfig()):
            value = getattr(args, name)
            if value is not None:
                values[name] = value
        options = AutomaticRegistrationConfig.from_mapping(values)
        run_point_cloud_registration(args.scan, args.camera, args.out, options, show_view=not args.no_view)
    except (ValueError, RuntimeError, OSError) as error:
        print(f"Automatic registration failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
