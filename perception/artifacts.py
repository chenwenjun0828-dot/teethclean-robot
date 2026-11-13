"""Shared coordinate exports for independent simulated and real perception runs."""
from __future__ import annotations

from datetime import datetime
import hashlib
import json
from pathlib import Path

import numpy as np

from .frames import validate_transform
from .global_camera import PROJECT_ROOT, ensure_empty_directory, file_hash, write_json
from .registration.algorithms import load_line_csv
from .registration.registration_math import transform_points


def default_output(source):
    if source not in ('real', 'simulation'):
        raise ValueError('Unknown perception source')
    return PROJECT_ROOT / 'outputs/perception' / source / datetime.now().strftime('%Y%m%d_%H%M%S_%f')


def load_extrinsics(path):
    """Read a snapshot of camera-CV to robot-base extrinsics in metres."""
    path = Path(path).resolve()
    raw = path.read_bytes()
    payload = json.loads(raw.decode('utf-8-sig'))
    if not isinstance(payload, dict):
        raise ValueError('Extrinsics must be a JSON object')
    if payload.get('translation_unit') != 'm':
        raise ValueError('Extrinsics translation_unit must be m')
    if payload.get('status') not in ('placeholder', 'measured'):
        raise ValueError('Extrinsics status must be placeholder or measured')
    if 'T_base_camera' not in payload:
        raise ValueError('Extrinsics requires T_base_camera (camera CV -> robot base)')
    matrix = validate_transform(payload['T_base_camera']).copy()
    return dict(matrix=matrix, payload=payload, raw=raw, path=str(path),
                sha256=hashlib.sha256(raw).hexdigest(), status=payload['status'])


def export_coordinates(run_dir, camera_from_scan_mm, *, line=None, extrinsics=None,
                       world_from_camera_m=None):
    """Export scan->camera and optional camera->target chains, always using metres.

    Placeholder base products have a filename suffix and CSV header marker.
    This function does not infer camera poses or attach an identity transform.
    """
    if extrinsics is not None and world_from_camera_m is not None:
        raise ValueError('Choose real base or simulated world output')
    camera_from_scan = validate_transform(camera_from_scan_mm).copy()
    camera_from_scan[:3, 3] /= 1000.
    target, external, status = None, None, 'not_provided'
    if extrinsics is not None:
        target, external, status = 'base', validate_transform(extrinsics['matrix']), extrinsics['status']
    elif world_from_camera_m is not None:
        target, external, status = 'world', validate_transform(world_from_camera_m), 'simulation'
    points_mm = load_line_csv(line) if line is not None else None
    line_source = (dict(path=str(Path(line).resolve()), sha256=file_hash(line))
                   if line is not None else None)
    root = Path(run_dir).resolve()
    transforms = ensure_empty_directory(root / 'transforms')
    paths = ensure_empty_directory(root / 'paths')
    files = {}

    def save_matrix(name, matrix):
        np.save(transforms / (name + '.npy'), matrix)
        np.savetxt(transforms / (name + '.txt'), matrix,
                   header=f'{name}; translation_unit=m; status={status}')
        files[name] = str((transforms / (name + '.npy')).relative_to(root))

    save_matrix('T_camera_scan_m', camera_from_scan)
    suffix = '_placeholder' if status == 'placeholder' else ''
    composed = None
    if target is not None:
        composed = validate_transform(external @ camera_from_scan)
        save_matrix(f'T_{target}_camera{suffix}_m', external)
        save_matrix(f'T_{target}_scan{suffix}_m', composed)
    if extrinsics is not None:
        (transforms / 'extrinsics_used.json').write_bytes(extrinsics['raw'])

    def save_points(name, points, frame, point_status):
        filename = paths / (name + '.csv')
        np.savetxt(filename, points, delimiter=',', header=(
            f'frame={frame}; units=m; status={point_status}\nx_m,y_m,z_m'))
        files[name] = str(filename.relative_to(root))

    if points_mm is not None:
        save_points('line_camera_m', transform_points(points_mm / 1000., camera_from_scan),
                    'camera_cv', 'registered')
        if composed is not None:
            save_points(f'line_{target}{suffix}_m', transform_points(points_mm / 1000., composed),
                        target, status)
    provenance = (dict(path=extrinsics['path'], sha256=extrinsics['sha256'],
                       copy='transforms/extrinsics_used.json') if extrinsics is not None else None)
    manifest = dict(source_frame='oral_scan', camera_frame='camera_cv', target_frame=target,
        translation_unit='m', external_transform_status=status, external_transform=provenance,
        base_coordinates_available=target == 'base', placeholder=status == 'placeholder',
        line_source=line_source, files=files, robot_execution_allowed=False)
    manifest['artifact_sha256'] = {name: file_hash(root / filename) for name, filename in files.items()}
    write_json(transforms / 'metadata.json', manifest)
    write_json(paths / 'metadata.json', manifest)
    return manifest
