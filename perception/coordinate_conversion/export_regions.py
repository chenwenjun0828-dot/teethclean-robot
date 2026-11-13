"""Export existing PreparedRegion geometry with explicit controller-base coordinates."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from perception.regions import PreparedRegion, REGION_ORDER


def export_jaw(conversion, jaw):
    regions = conversion / jaw / 'regions'
    matrix_path = conversion / jaw / 'T_controller_base_scan_nominal_rigid_m.npy'
    if not matrix_path.exists():
        matrix_path = conversion / jaw / 'T_controller_base_scan_nominal_m.npy'
    matrix = np.load(matrix_path)
    summary = dict(frame='controller_base', units='m', status='nominal_kinematics',
                   robot_execution_allowed=False, cross_region_connections=False,
                   regions={})
    for name in REGION_ORDER:
        folder = regions / name
        prepared = PreparedRegion.load(folder)
        np.testing.assert_allclose(prepared.world_from_scan, matrix, atol=1e-12)
        with np.load(folder / 'continuous_gingival_line.npz') as data:
            source, normals = data['points_scan_m'], data['normals_scan']
            stations = data['parameters_m']
        points = source @ matrix[:3, :3].T + matrix[:3, 3]
        base_normals = normals @ matrix[:3, :3].T
        tangents = np.array([prepared.reference.evaluate(s, derivative=1) for s in stations])
        tangents /= np.linalg.norm(tangents, axis=1, keepdims=True)
        base_tangents = tangents @ matrix[:3, :3].T
        teeth = np.array([prepared.reference.tooth(s) for s in stations], dtype=int)
        np.testing.assert_allclose(points, np.loadtxt(folder / 'continuous_gingival_line_world_m.csv',
                                                    delimiter=',', skiprows=1), atol=1e-12)
        np.testing.assert_allclose(np.linalg.norm(base_normals, axis=1), 1., atol=1e-6)
        inverse = np.linalg.inv(matrix)
        np.testing.assert_allclose(points @ inverse[:3, :3].T + inverse[:3, 3], source, atol=1e-12)
        np.savetxt(folder / 'points_controller_base_nominal_m.csv', points, delimiter=',',
                   header='frame=controller_base; units=m; nominal kinematics; region='+name+'\nx_m,y_m,z_m')
        np.savez_compressed(folder / 'region_controller_base_nominal.npz',
            points_m=points, normals=base_normals, tangents=base_tangents,
            parameters_m=stations, tooth_ids=teeth, T_controller_base_scan_m=matrix)
        np.save(folder / 'T_controller_base_scan_nominal_m.npy', matrix)
        record = dict(region=name, frame='controller_base', position_unit='m',
            status='nominal_kinematics_not_independently_validated',
            world_field_interpretation='PreparedRegion world_from_scan uses controller_base as world',
            source_boundaries_approved=prepared.geometry.report.get('source_metadata', {}).get('region_boundaries_approved', False),
            robot_execution_allowed=False, cross_region_connections=False,
            points=len(points), retained_length_mm=prepared.reference.record['sweep_length_mm'],
            endpoint_trim_mm=prepared.reference.record['trim_mm'], trim_scope='region_endpoints_only',
            points_file='points_controller_base_nominal_m.csv',
            geometry_file='region_controller_base_nominal.npz',
            transform_source='../../' + matrix_path.name,
            transform_sha256=hashlib.sha256(matrix_path.read_bytes()).hexdigest(),
            note='Different regions are independent. No TCP offset or executable robot poses included.')
        (folder / 'base_coordinates.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
        summary['regions'][name] = dict(directory=str(folder.relative_to(conversion)),
            points=len(points), retained_length_mm=record['retained_length_mm'])
        print(jaw, name, len(points), 'points,', round(record['retained_length_mm'], 3), 'mm')
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--conversion-dir', type=Path, required=True)
    parser.add_argument('--jaw', choices=('mandibular', 'maxillary'), action='append')
    args = parser.parse_args(argv)
    conversion = args.conversion_dir.resolve()
    summary = {jaw: export_jaw(conversion, jaw) for jaw in (args.jaw or ('mandibular', 'maxillary'))}
    (conversion / 'regional_export_report.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
