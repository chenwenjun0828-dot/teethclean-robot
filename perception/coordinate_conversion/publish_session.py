"""Publish a validated planning snapshot into stable assets, without raw captures."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil

from perception.regions import PreparedRegion
from .export_regions import export_jaw


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding='utf-8')


def publish(conversion, output, calibration_output):
    conversion, output, calibration_output = (Path(p).resolve() for p in (conversion, output, calibration_output))
    for target in (output, calibration_output):
        if target.exists():
            raise ValueError(f'Use a new asset directory: {target}')
    manifest = read(conversion / 'planning_input_manifest.json')
    prepared = {}
    # Validate all input hashes and PreparedRegions before publishing anything.
    entries = list(manifest['calibration_sources'].values()) + list(manifest['shared_transforms'].values())
    for jaw, record in manifest['jaws'].items():
        prepared[jaw] = {}
        for name, region in record['regions'].items():
            entries.extend(region['files'].values())
            prepared[jaw][name] = PreparedRegion.load(conversion / region['files']['prepared_region']['path'])
    for item in entries:
        if hashlib.sha256((conversion / item['path']).read_bytes()).hexdigest() != item['sha256']:
            raise ValueError('Source artifact changed: ' + item['path'])
    output.mkdir(parents=True)
    calibration_output.mkdir(parents=True)

    def entry(path):
        return dict(path=Path(os.path.relpath(path, output)).as_posix(),
                    sha256=hashlib.sha256(path.read_bytes()).hexdigest())

    result = copy.deepcopy(manifest)
    result.update(status='stable_planning_input_snapshot', documentation='README.md',
                  raw_capture_included=False, source_conversion_id=conversion.name)
    result['calibration_sources'] = {}
    for name, item in manifest['calibration_sources'].items():
        if name in ('robot_joint_observation.json', 'fk_report.json'):
            destination = output / name
        else:
            destination = calibration_output / name
        shutil.copyfile(conversion / item['path'], destination)
        if name == 'robot_joint_observation.json':
            observation = read(destination)
            observation.update(path_base='manifest', fk_report='fk_report.json',
                camera_intrinsics_extrinsics_report=Path(os.path.relpath(
                    calibration_output / 'camera_extrinsics.json', output)).as_posix(),
                forward_kinematics_status='computed_nominal_factory_correction_not_applied')
            write(destination, observation)
        result['calibration_sources'][name] = entry(destination)
    result['shared_transforms'] = {}
    (output / 'transforms').mkdir()
    for path in conversion.glob('T_*.*'):
        if path.suffix in ('.npy', '.txt'):
            shutil.copyfile(path, output / 'transforms' / path.name)
    for name in manifest['shared_transforms']:
        result['shared_transforms'][name] = entry(output / 'transforms' / (name + '.npy'))
    regional_summary = {}
    for jaw, regions in prepared.items():
        jaw_folder = output / jaw
        jaw_folder.mkdir()
        for path in (conversion / jaw).glob('T_*.*'):
            if path.suffix in ('.npy', '.txt'):
                shutil.copyfile(path, jaw_folder / path.name)
        normalization = conversion / jaw / 'regional_transform_normalization.json'
        if normalization.exists():
            shutil.copyfile(normalization, jaw_folder / normalization.name)
        for name, region in regions.items():
            # Reserializing rewrites source references relative to the new asset location.
            region.save(jaw_folder / 'regions' / name, diagnostics=False)
        index = dict(schema_version=1, module='perception', jaw=jaw, path_base='manifest',
                     coordinate_frame='controller_base', region_order=list(regions),
                     cross_region_connections=False,
                     regions={name: f'{name}/perception_region.json' for name in regions})
        write(jaw_folder / 'regions/perception_regions.json', index)
        regional_summary[jaw] = export_jaw(output, jaw)
        for name, record in result['jaws'][jaw]['regions'].items():
            folder = jaw_folder / 'regions' / name
            for key, source in manifest['jaws'][jaw]['regions'][name]['files'].items():
                record['files'][key] = entry(folder / Path(source['path']).name)
            PreparedRegion.load(folder)
        for name in ('regions_overview.png', 'surface_qa.json'):
            source = conversion / jaw / 'regions' / name
            if source.exists():
                shutil.copyfile(source, jaw_folder / 'regions' / name)
    write(output / 'regional_export_report.json', regional_summary)
    result.pop('documentation_sha256', None)  # Added after the session README is finalized.
    write(output / 'planning_input_manifest.json', result)
    sdk = read(calibration_output / 'camera_extrinsics.json')
    write(calibration_output / 'provenance.json', dict(
        status='provided_handeye_and_device_sdk_extrinsics', camera_serial=sdk['device']['serial'],
        robot_effector_frame='wrist_3_link', tracking_base_frame='camera_color_frame',
        translation_unit='m', selected_handeye_label='25',
        robot_execution_allowed=False, sources=manifest['calibration_sources']))
    from .compact_session import compact
    compact(output)
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--conversion-dir', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--calibration-out', type=Path, required=True)
    args = parser.parse_args(argv)
    print('Published:', publish(args.conversion_dir, args.out, args.calibration_out))


if __name__ == '__main__':
    main()
