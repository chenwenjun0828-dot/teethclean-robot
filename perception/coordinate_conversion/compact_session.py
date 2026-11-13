"""Remove rebuildable diagnostics and share one validated mesh per jaw."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

from perception.regions import PreparedRegion


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compact(session):
    session = Path(session).resolve()
    manifest_path = session / 'planning_input_manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    folders, expected_meshes = [], {}
    for jaw, group in manifest['jaws'].items():
        shared = session / jaw / 'shared/region_mesh.npz'
        for name, record in group['regions'].items():
            folder = (session / record['files']['prepared_region']['path']).parent
            PreparedRegion.load(folder)  # Check all current artifacts before any removal.
            region = json.loads((folder/'perception_region.json').read_text())
            mesh = folder/'region_mesh.npz'
            source = mesh if mesh.exists() else shared
            mesh_hash = digest(source)
            if mesh_hash != region['files']['region_mesh.npz']:
                raise ValueError('Mesh fingerprint differs')
            if jaw in expected_meshes and expected_meshes[jaw] != mesh_hash:
                raise ValueError('Region meshes differ; cannot share a common jaw mesh')
            expected_meshes[jaw] = mesh_hash
            if shared.exists() and digest(shared) != digest(source):
                raise ValueError('Region meshes differ; cannot share a common jaw mesh')
            folders.append((jaw, name, folder, record, region, source, shared))
    # No recursive deletion: every candidate file must be below the explicit session.
    removed = 0
    for jaw, name, folder, record, region, source, shared in folders:
        shared.parent.mkdir(exist_ok=True)
        if not shared.exists():
            shutil.copyfile(source, shared)
        if digest(shared) != region['files']['region_mesh.npz']:
            raise ValueError('Shared jaw mesh fingerprint differs')
        region['artifact_paths'] = {'region_mesh.npz': '../../shared/region_mesh.npz'}
        (folder/'perception_region.json').write_text(json.dumps(region, indent=2), encoding='utf-8')
        keep = (set(region['files']) - {'region_mesh.npz'}) | {
            'perception_region.json', 'points_controller_base_nominal_m.csv',
            'region_controller_base_nominal.npz', 'base_coordinates.json',
            'continuous_gingival_line_world_m.csv'}
        candidates = [p for p in folder.rglob('*') if p.is_file() and p.relative_to(folder).as_posix() not in keep]
        if any(not p.resolve().is_relative_to(session) or p.is_symlink() for p in candidates):
            raise ValueError('Diagnostic cleanup escaped the session directory')
        for path in candidates:
            path.unlink()
            removed += 1
        teeth = folder/'teeth'
        if teeth.is_dir() and not any(teeth.iterdir()):
            teeth.rmdir()
        for entry in record['files'].values():
            entry['sha256'] = digest(session / entry['path'])
        PreparedRegion.load(folder)
    for jaw, group in manifest['jaws'].items():
        shared = session / jaw / 'shared/region_mesh.npz'
        group['shared_artifacts'] = {'region_mesh': dict(path=shared.relative_to(session).as_posix(), sha256=digest(shared))}
        preview = session/jaw/'regions/six_regions_controller_base_nominal_m.ply'
        if preview.exists():
            assert preview.resolve().is_relative_to(session)
            preview.unlink()
            removed += 1
    manifest['storage'] = 'compact_planning_only_shared_jaw_mesh'
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    files = [p for p in session.rglob('*') if p.is_file()]
    print('Compact session:', len(files), 'files,', round(sum(p.stat().st_size for p in files)/1048576, 2), 'MB;', removed, 'diagnostic/duplicate files removed')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    compact(parser.parse_args().session)


if __name__ == '__main__':
    main()
