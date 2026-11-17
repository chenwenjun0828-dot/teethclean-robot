"""Recompute a published session's frame interpretation from immutable scan geometry."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

import numpy as np

from perception.regions import PreparedRegion
from .convert_points import save_matrix
from .export_regions import export_jaw
from .handeye_frames import MODES, select_handeye_chain


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding='utf-8')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def entries(manifest):
    yield from manifest['calibration_sources'].values()
    yield from manifest['shared_transforms'].values()
    for group in manifest['jaws'].values():
        yield from group.get('shared_artifacts', {}).values()
        for region in group['regions'].values():
            yield from region['files'].values()
    yield from manifest.get('frame_correction_files', {}).values()


def validate_session(session):
    session = Path(session).resolve()
    manifest = read(session / 'planning_input_manifest.json')
    for entry in entries(manifest):
        if digest(session / entry['path']) != entry['sha256']:
            raise ValueError('Session fingerprint differs: ' + entry['path'])
    for jaw, group in manifest['jaws'].items():
        for region in group['regions'].values():
            prepared = PreparedRegion.load(session / region['files']['prepared_region']['path'])
            folder = prepared.output_dir
            with np.load(folder / 'continuous_gingival_line.npz') as local, np.load(
                    folder / 'region_controller_base_nominal.npz') as base:
                matrix = prepared.world_from_scan
                np.testing.assert_allclose(base['T_controller_base_scan_m'], matrix, atol=1e-12)
                points = local['points_scan_m'] @ matrix[:3, :3].T + matrix[:3, 3]
                np.testing.assert_allclose(base['points_m'], points, atol=1e-12)
                np.testing.assert_allclose(base['normals'], local['normals_scan'] @ matrix[:3, :3].T, atol=1e-12)
                tangents = np.array([prepared.reference.evaluate(s, derivative=1) for s in local['parameters_m']])
                tangents /= np.linalg.norm(tangents, axis=1, keepdims=True)
                np.testing.assert_allclose(base['tangents'], tangents @ matrix[:3, :3].T, atol=1e-12)
                np.testing.assert_allclose(np.loadtxt(folder / 'points_controller_base_nominal_m.csv', delimiter=','), points, atol=1e-12)
                np.testing.assert_allclose(np.loadtxt(folder / 'continuous_gingival_line_world_m.csv', delimiter=',', skiprows=1), points, atol=1e-12)
    return manifest


def _rebuild(session, manifest, mode):
    sources = manifest['calibration_sources']
    fk = read(session / sources['fk_report.json']['path'])
    sdk = read(session / sources['camera_extrinsics.json']['path'])
    inventory = read(session / sources['handeye_inventory.json']['path'])
    label = manifest.get('handeye_label', '25')
    selected = [c for c in inventory['calibrations'] if str(c['label']) == str(label)]
    if len(selected) != 1:
        raise ValueError('Select exactly one hand-eye calibration label')
    h, d, interpretation = select_handeye_chain(selected[0], sdk, mode)
    old_mode = manifest.get('handeye_frame_interpretation', {}).get('mode', 'declared')
    old_h, old_d, _ = select_handeye_chain(selected[0], sdk, old_mode)
    b = np.asarray(fk['T_controller_base_wrist3_nominal_m'])
    l = np.asarray(fk['T_base_link_wrist3_nominal_m'])
    transforms = session / 'transforms'
    # Validate old derived matrices against their recorded interpretation before replacing.
    for jaw in manifest['jaws']:
        folder = session / jaw
        ds = np.load(folder / 'T_depth_optical_scan_m.npy')
        np.testing.assert_allclose(np.load(folder / 'T_controller_base_scan_nominal_m.npy'),
                                   b @ old_h @ old_d @ ds, atol=1e-10, rtol=0)
    if old_mode == mode and 'handeye_frame_interpretation' in manifest:
        return None
    shared = {
        'T_controller_base_wrist3_nominal_m': b,
        'T_base_link_wrist3_nominal_m': l,
        interpretation['handeye_matrix_name']: h,
        interpretation['depth_matrix_name']: d,
        'T_controller_base_depth_optical_nominal_m': b @ h @ d,
        'T_base_link_depth_optical_nominal_m': l @ h @ d,
    }
    for name, matrix in shared.items():
        save_matrix(transforms, name, matrix)
    # Remove inactive, misleadingly named transform aliases from the active session only.
    for name in ('T_wrist3_color_frame_m', 'T_wrist3_color_optical_m',
                 'T_color_frame_depth_optical_ros_convention_m', 'T_color_optical_depth_optical_m'):
        if name not in shared:
            for suffix in ('.npy', '.txt'):
                (transforms / (name + suffix)).unlink(missing_ok=True)
    manifest['shared_transforms'] = {
        name: dict(path=f'transforms/{name}.npy', sha256=digest(transforms / (name + '.npy')))
        for name in shared}
    manifest['handeye_frame_interpretation'] = interpretation
    manifest['handeye_label'] = str(label)
    manifest['coordinate_revision'] = f'{mode}_v1'
    manifest['coordinate_revision_utc'] = datetime.now(timezone.utc).isoformat()
    changes = {}
    for jaw, group in manifest['jaws'].items():
        folder = session / jaw
        ds = np.load(folder / 'T_depth_optical_scan_m.npy')
        old_centers = []
        base_scan = b @ h @ d @ ds
        save_matrix(folder, 'T_controller_base_scan_nominal_m', base_scan)
        save_matrix(folder, 'T_base_link_scan_nominal_m', l @ h @ d @ ds)
        # Use a proper rotation for every PreparedRegion; translation is never altered.
        rigid = base_scan.copy()
        u, _, vt = np.linalg.svd(rigid[:3, :3])
        sign = np.diag([1., 1., np.linalg.det(u @ vt)])
        rigid[:3, :3] = u @ sign @ vt
        save_matrix(folder, 'T_controller_base_scan_nominal_rigid_m', rigid)
        write(folder / 'regional_transform_normalization.json', dict(
            reason='SDK floating-point rotation roundoff; nearest proper rotation for regional geometry',
            max_rotation_entry_change=float(np.max(np.abs(base_scan[:3, :3] - rigid[:3, :3]))),
            translation_unchanged=True, handeye_frame=mode))
        for region in group['regions'].values():
            region_path = session / region['files']['prepared_region']['path']
            record = read(region_path)
            region_folder = region_path.parent
            with np.load(region_folder / 'region_controller_base_nominal.npz') as previous:
                old_centers.append(previous['points_m'].copy())
            record['world_from_scan_m'] = rigid.tolist()
            record['handeye_frame_interpretation'] = interpretation
            write(region_path, record)
            with np.load(region_folder / 'continuous_gingival_line.npz') as local:
                points = local['points_scan_m'] @ rigid[:3, :3].T + rigid[:3, 3]
            np.savetxt(region_folder / 'continuous_gingival_line_world_m.csv', points,
                       delimiter=',', header='x_m,y_m,z_m', comments='')
            if (region_folder / 'world_from_scan_m.npy').exists():
                np.save(region_folder / 'world_from_scan_m.npy', rigid)
        changes[jaw] = dict(previous_region_mean_m=np.concatenate(old_centers).mean(axis=0).tolist())
    write(session / 'planning_input_manifest.json', manifest)  # export reads the interpretation
    summary = {jaw: export_jaw(session, jaw) for jaw in manifest['jaws']}
    write(session / 'regional_export_report.json', summary)
    for jaw, group in manifest['jaws'].items():
        points = [np.load(session / r['files']['geometry_npz']['path'])['points_m'] for r in group['regions'].values()]
        current = np.concatenate(points).mean(axis=0)
        changes[jaw]['corrected_region_mean_m'] = current.tolist()
        changes[jaw]['region_mean_shift_m'] = float(np.linalg.norm(current - changes[jaw]['previous_region_mean_m']))
        # Optional old world-space QA images must not survive a coordinate revision.
        for name in ('regions_overview.png', 'surface_qa.json'):
            (session / jaw / 'regions' / name).unlink(missing_ok=True)
    write(session / 'handeye_frame_interpretation.json', interpretation)
    write(session / 'frame_correction_report.json', dict(
        previous_mode=old_mode, mode=mode, jaws=changes, original_registration_unchanged=True,
        local_curve_geometry_unchanged=True, raw_calibration_modified=False,
        robot_execution_allowed=False))
    manifest['frame_correction_files'] = {
        name: dict(path=name, sha256=digest(session / name))
        for name in ('handeye_frame_interpretation.json', 'frame_correction_report.json')}
    for entry in entries(manifest):
        entry['sha256'] = digest(session / entry['path'])
    # Documentation may be edited separately; never retain an obsolete digest.
    manifest.pop('documentation_sha256', None)
    write(session / 'planning_input_manifest.json', manifest)
    validate_session(session)
    return changes


def correct(session, backup_dir, mode='color_optical'):
    session, backup = Path(session).resolve(), Path(backup_dir).resolve()
    if mode not in MODES:
        raise ValueError('Unknown hand-eye frame interpretation')
    if backup == session or backup.is_relative_to(session) or session.is_relative_to(backup):
        raise ValueError('Backup and session directories must be disjoint')
    manifest = validate_session(session)
    # Sibling staging preserves all existing relative source/calibration paths.
    prefix = f'.{session.name}-frame-'
    with tempfile.TemporaryDirectory(prefix=prefix, dir=session.parent) as temporary:
        staging = Path(temporary).resolve()
        if staging.parent != session.parent or not staging.name.startswith(prefix):
            raise ValueError('Staging directory escaped the intended session parent')
        shutil.copytree(session, staging, dirs_exist_ok=True)
        changes = _rebuild(staging, manifest, mode)
        if changes is None:
            print('Session already uses', mode, '; validated without rewriting.')
            return False
        if backup.exists():
            raise ValueError('Use a new backup directory')
        shutil.copytree(session, backup)
        old_files = {p.relative_to(session) for p in session.rglob('*') if p.is_file()}
        new_files = {p.relative_to(staging) for p in staging.rglob('*') if p.is_file()}
        shutil.copytree(staging, session, dirs_exist_ok=True)
        for relative in old_files - new_files:
            target = session / relative
            if not target.resolve().is_relative_to(session) or target.is_symlink():
                raise ValueError('Inactive artifact removal escaped session')
            target.unlink()
    validate_session(session)
    print('Corrected:', session, '\nOriginal backup:', backup)
    print(json.dumps(changes, indent=2))
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--backup-dir', type=Path, required=True)
    parser.add_argument('--handeye-frame', choices=MODES, default='color_optical')
    args = parser.parse_args()
    correct(args.session, args.backup_dir, args.handeye_frame)


if __name__ == '__main__':
    main()
