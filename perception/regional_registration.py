"""Capture and register a frozen regional dataset using the existing workflow."""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from .frames import validate_transform
from .global_camera import capture_rgbd, save_capture, load_config, file_hash
from .registration import register_capture


@dataclass(frozen=True)
class RegionalRegistration:
    world_from_scan_m: np.ndarray
    capture_dir: Path
    registration_dir: Path
    report: dict


def register_regional_paths(model, data, dataset, output_dir):
    """Register scan to a NEW RGB-D capture, never using scan-body ground truth.

    The flat reference CSV is only the existing registration service's point
    export input. Chain connectivity remains in dataset.chains, never in CSV row
    adjacency. It does not supply correspondences or an initial pose.
    """
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    config = load_config()
    mesh_key, line_key = dataset.jaw + '_mesh', dataset.jaw + '_line'
    if file_hash(config[mesh_key]) != dataset.metadata['mesh_sha256']:
        raise ValueError('Configured registration mesh differs from regional source mesh')
    points = np.vstack([chain.scan_points_mm for chain in dataset.chains])
    line_path = output_dir / 'regional_reference_scan_mm.csv'
    np.savetxt(line_path, points, delimiter=',', header='x,y,z', comments='', fmt='%.10f')
    config = dict(config, registration_method='automatic')
    config[mesh_key] = str(dataset.mesh_path)
    config[line_key] = str(line_path)
    config_path = output_dir / 'registration_config.json'
    config_path.write_text(json.dumps(config, indent=2, ensure_ascii=False), encoding='utf-8')
    frame = capture_rgbd(model, data, config['camera'], config['width'], config['height'])
    capture_dir = output_dir / 'capture'
    metadata = save_capture(capture_dir, frame, model, data, dataset.jaw, config)
    registration_dir = register_capture(
        capture_dir, config_path, dataset.jaw, output_dir / 'registration',
        registration_method='automatic')
    estimate = validate_transform(np.load(registration_dir / 'T_camera_scan_estimated.npy'))
    estimate_m = estimate.copy()
    estimate_m[:3, 3] /= 1000.
    world_from_scan = validate_transform(np.asarray(metadata['T_world_camera_cv_m'])) @ estimate_m
    report = json.loads((registration_dir / 'registration_report.json').read_text(encoding='utf-8'))
    return RegionalRegistration(world_from_scan, capture_dir, registration_dir, report)
