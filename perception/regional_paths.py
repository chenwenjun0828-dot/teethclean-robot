"""Read frozen regional candidates; recover scan coordinates without re-registering."""
from __future__ import annotations

import csv
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np

from .frames import validate_transform

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASET = PROJECT_ROOT / 'assets/regional_paths/mandibular'
REGION_ORDER = ('outer_left', 'outer_front', 'outer_right',
                'inner_right', 'inner_front', 'inner_left')


@dataclass(frozen=True)
class RegionalPath:
    order: int
    region: str
    scan_points_mm: np.ndarray


@dataclass(frozen=True)
class RegionalPaths:
    jaw: str
    mesh_path: Path
    chains: tuple[RegionalPath, ...]
    metadata: dict


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_regional_paths(dataset=DEFAULT_DATASET):
    """Validate a bundled snapshot and preserve CSV chain/point direction exactly.

    T_camera_scan maps scan mm to the old D405 camera mm. Its inverse removes
    that historical rigid placement; no old camera pose is used in MuJoCo.
    """
    dataset = Path(dataset).resolve()
    metadata = json.loads((dataset / 'metadata.json').read_text(encoding='utf-8'))
    required = {'schema_version': 1, 'frame': 'd405_camera', 'unit': 'mm',
                'transform_name': 'T_camera_scan', 'transform_translation_unit': 'mm',
                'robot_execution_allowed': False}
    if any(metadata.get(k) != v for k, v in required.items()):
        raise ValueError('Unsupported regional dataset schema, frame, units or execution flag')
    if metadata.get('jaw') not in ('mandibular', 'maxillary'):
        raise ValueError('Unknown regional jaw')
    if metadata.get('region_order') != list(REGION_ORDER):
        raise ValueError('Regional order must match the six-region convention')
    for name in ('regional_chain_candidates_camera_mm.csv', 'T_camera_scan.txt'):
        if _sha256(dataset / name) != metadata.get('files_sha256', {}).get(name):
            raise ValueError(f'Regional input hash mismatch: {name}')
    mesh_path = PROJECT_ROOT / metadata['mesh_path']
    if not mesh_path.is_file():
        mesh_path = PROJECT_ROOT / f"assets/source_cad/teeth/{metadata['jaw']}_scan.ply"
    if _sha256(mesh_path) != metadata.get('mesh_sha256'):
        raise ValueError('Regional source mesh hash mismatch')
    transform = validate_transform(np.loadtxt(dataset / 'T_camera_scan.txt'))
    with (dataset / 'regional_chain_candidates_camera_mm.csv').open(
            newline='', encoding='utf-8-sig') as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != metadata['expected_points']:
        raise ValueError('Regional point count differs from metadata')
    chains = []
    cursor = 0
    for order, region in enumerate(REGION_ORDER, 1):
        selected = []
        while cursor < len(rows) and int(rows[cursor]['chain_order']) == order:
            row = rows[cursor]
            if (row['region'] != region or int(row['point_order']) != len(selected)
                    or row['robot_execution_allowed'] != 'False'
                    or row['simulation_only'] != 'True'):
                raise ValueError('Invalid region, point order or execution flag')
            selected.append([float(row[f'{a}_mm']) for a in 'xyz'])
            cursor += 1
        points = np.asarray(selected, dtype=float)
        if len(points) < 2 or not np.isfinite(points).all():
            raise ValueError(f'Region {region} requires at least two finite points')
        # Row-vector inverse of camera = R @ scan + t, all lengths in mm.
        scan = (points - transform[:3, 3]) @ transform[:3, :3]
        chains.append(RegionalPath(order, region, scan))
    if cursor != len(rows):
        raise ValueError('Unexpected or reordered regional chains')
    metadata = dict(metadata, dataset_dir=str(dataset))
    return RegionalPaths(metadata['jaw'], mesh_path, tuple(chains), metadata)


def scan_points_to_world_m(points_mm, world_from_scan_m):
    """Place scan-local mm in world m using an explicitly supplied rigid pose."""
    transform = validate_transform(world_from_scan_m)
    return np.asarray(points_mm) / 1000. @ transform[:3, :3].T + transform[:3, 3]


def evaluate_regional_alignment(dataset, estimated_pose, reference_pose):
    """Post-registration simulator evaluation only; never supplies registration input."""
    estimated_pose = validate_transform(estimated_pose)
    reference_pose = validate_transform(reference_pose)
    results, all_errors = [], []
    for chain in dataset.chains:
        estimated = scan_points_to_world_m(chain.scan_points_mm, estimated_pose)
        reference = scan_points_to_world_m(chain.scan_points_mm, reference_pose)
        errors = np.linalg.norm(estimated - reference, axis=1) * 1000.
        all_errors.extend(errors)
        results.append({'region': chain.region, 'points': len(errors),
                        'rms_mm': float(np.sqrt(np.mean(errors**2))),
                        'max_mm': float(errors.max())})
    relative = estimated_pose[:3, :3] @ reference_pose[:3, :3].T
    angle = np.arccos(np.clip((np.trace(relative) - 1.) / 2., -1., 1.))
    return {
        'method': 'corresponding regional points vs known simulator scan-body pose',
        'evaluation_only': True,
        'translation_error_mm': float(np.linalg.norm(
            estimated_pose[:3, 3] - reference_pose[:3, 3]) * 1000.),
        'rotation_error_deg': float(np.rad2deg(angle)),
        'path_rms_mm': float(np.sqrt(np.mean(np.asarray(all_errors)**2))),
        'path_max_mm': float(max(all_errors)), 'regions': results,
    }
